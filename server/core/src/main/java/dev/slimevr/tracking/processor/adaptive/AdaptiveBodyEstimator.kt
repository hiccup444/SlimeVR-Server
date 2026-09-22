package dev.slimevr.tracking.processor.adaptive

import dev.slimevr.config.AdaptiveTrackingConfig
import dev.slimevr.tracking.processor.skeleton.HumanSkeleton
import dev.slimevr.tracking.trackers.Tracker
import io.github.axisangles.ktmath.Quaternion
import io.github.axisangles.ktmath.Vector3
import java.nio.file.Paths
import java.util.concurrent.CompletableFuture
import kotlin.math.abs
import kotlin.math.atan2

data class TrackerDriftDiagnostic(val trackerId: Int, val biasRadians: Float, val residual: DriftResidual?, val predictedRateRadiansPerSecond: Double? = null, val poseConfidence: GlobalPoseConfidence? = null)

/** Coordinates evidence-gated calibration separately from immediate pose constraints. */
class AdaptiveBodyEstimator(private val config: AdaptiveTrackingConfig) : AutoCloseable {
	private val absoluteArms = AbsoluteArmYawEstimator(config)
	private var calibration: CalibrationLearner? = null
	private fun learner(): CalibrationLearner = calibration ?: CalibrationLearner(CalibrationStore(Paths.get(config.calibrationDirectory))).also { calibration = it }

	fun clearLearnedCalibration(): CompletableFuture<Boolean> = learner().clearLearned()

	override fun close() {
		calibration?.close()
		calibration = null
		reset()
	}
	private data class Reference(val yaw: Float, val headPosition: Vector3, val witnesses: List<Quaternion>, val context: Long)
	private class State(val tracker: Tracker) {
		val residual = ResidualTracker()
		val corrector = AdaptiveYawCorrector()
		var reference: Reference? = null
		var lastYaw: Float? = null
		var lastTime: Long? = null
	}
	private val states = mutableMapOf<Int, State>()
	private var nextContext = 0L
	private var footDiagnostics: List<TrackerDriftDiagnostic> = emptyList()
	val diagnostics: List<TrackerDriftDiagnostic> get() = footDiagnostics + absoluteArms.diagnostics

	fun updateAbsoluteConstraints(skeleton: HumanSkeleton, now: Long) = absoluteArms.update(skeleton, now, ::observeCalibration)

	fun reset() {
		absoluteArms.reset()
		calibration?.resetTransient()
		states.values.forEach { it.tracker.adaptiveYawBiasRadians = 0f }
		states.clear()
		footDiagnostics = emptyList()
	}

	fun update(skeleton: HumanSkeleton, now: Long) {
		if (!config.temperatureLearningEnabled) calibration?.resetTransient()
		if (!config.yawCorrectionEnabled || skeleton.getPauseTracking() || skeleton.stayAlignedConfig.enabled || skeleton.localizer.getEnabled()) {
			reset()
			return
		}
		val head = skeleton.headTracker
		val results = mutableListOf<TrackerDriftDiagnostic>()
		fun foot(tracker: Tracker?, thigh: Tracker?, ankle: Tracker?, contact: FootContactSnapshot) {
			if (tracker == null) return
			val old = states[tracker.id]
			if (old != null && old.tracker !== tracker) {
				old.tracker.adaptiveYawBiasRadians = 0f
				states.remove(tracker.id)
			}
			val state = states.getOrPut(tracker.id) { State(tracker) }
			val measured = tracker.getRotationWithoutAdaptive()
			val yaw = heading(measured)
			val delta = state.lastTime?.let { now - it }
			val speed = if (yaw != null && state.lastYaw != null && delta != null && delta in 1..250_000_000L) abs(wrapYaw(yaw - state.lastYaw!!)) / (delta * 1e-9f) else null
			state.lastTime = now
			state.lastYaw = yaw
			val witnesses = listOfNotNull(head, thigh, ankle, skeleton.hipTracker)
			val valid = !tracker.resetsHandler.isDriftCompensationActive &&
				tracker.isImu() &&
				contact.state == FootContactState.PLANTED &&
				contact.weight >= 0.6f &&
				head?.hasPosition == true &&
				thigh != null &&
				ankle != null &&
				available(tracker, now) &&
				witnesses.all { available(it, now) } &&
				yaw != null &&
				speed != null &&
				speed < Math.toRadians(0.5) &&
				finite(head.position) &&
				tracker.hasAcceleration &&
				tracker.lastAccelerationUpdateNanos?.let { now - it in 0..250_000_000L } == true &&
				finite(tracker.getAcceleration()) &&
				tracker.getAcceleration().len() < 0.5f
			if (!valid) {
				observeCalibration(tracker, null, now)
				state.reference = null
				state.residual.reset()
				results.add(TrackerDriftDiagnostic(tracker.id, state.corrector.biasRadians, null))
				return
			}
			val reference = state.reference ?: Reference(yaw - state.corrector.biasRadians, head.position, witnesses.map { it.getRotationWithoutAdaptive() }, ++nextContext).also { state.reference = it }
			val stableWitnesses = reference.witnesses.size == witnesses.size &&
				witnesses.indices.all {
					reference.witnesses[it].angleToR(witnesses[it].getRotationWithoutAdaptive()) < Math.toRadians(1.0)
				}
			if (!stableWitnesses || (head.position - reference.headPosition).len() > 0.02f) {
				observeCalibration(tracker, null, now)
				state.reference = null
				state.residual.reset()
				results.add(TrackerDriftDiagnostic(tracker.id, state.corrector.biasRadians, null))
				return
			}
			val changes = witnesses.indices.map { reference.witnesses[it].angleToR(witnesses[it].getRotationWithoutAdaptive()).toDouble() }
			val poseConfidence = CalibrationPoseConfidence.estimate(
				listOf(tracker) + witnesses,
				listOf(head),
				changes,
				changes,
				((head.position - reference.headPosition).len() / 0.02f * 0.15f).coerceIn(0f, 1f),
				now,
				contact.weight,
			)
			val residual = state.residual.update(
				DriftEvidence(wrapYaw(yaw - reference.yaw), DriftEvidenceSource.PLANTED_CONTACT_WITH_STABLE_CHAIN, reference.context, poseConfidence.score, poseConfidence.score, 0.95f, witnesses.size, available = poseConfidence.learningEligible),
				now,
			)
			val predictedRate = observeCalibration(tracker, residual, now)
			// A mature temperature model compensates the residual filter's lag only while evidence remains eligible.
			val correctionResidual = if (predictedRate != null && residual.eligibleForLearning) residual.copy(filteredErrorRadians = wrapYaw(residual.filteredErrorRadians + (predictedRate * 2.0).toFloat())) else residual
			tracker.adaptiveYawBiasRadians = state.corrector.update(correctionResidual, now, config.yawCorrectionStrength)
			results.add(TrackerDriftDiagnostic(tracker.id, tracker.adaptiveYawBiasRadians, residual, predictedRate, poseConfidence))
		}
		foot(skeleton.leftFootTracker, skeleton.leftUpperLegTracker, skeleton.leftLowerLegTracker, skeleton.legTweaks.adaptiveLeftFoot.snapshot)
		foot(skeleton.rightFootTracker, skeleton.rightUpperLegTracker, skeleton.rightLowerLegTracker, skeleton.legTweaks.adaptiveRightFoot.snapshot)
		val present = results.map { it.trackerId }.toSet()
		states.keys.filter { it !in present }.forEach { states.remove(it)?.tracker?.adaptiveYawBiasRadians = 0f }
		footDiagnostics = results
	}

	private fun observeCalibration(tracker: Tracker, residual: DriftResidual?, now: Long): Double? {
		if (!config.temperatureLearningEnabled) return null
		val device = tracker.device ?: return null
		val hardware = device.hardwareIdentifier.takeIf { it.isNotBlank() && !it.equals("Unknown", true) } ?: return null
		val sensor = tracker.trackerNum
		val temperatureAge = tracker.lastTemperatureUpdateNanos?.let { now - it }
		val temperature = if (temperatureAge != null && temperatureAge in 0..30_000_000_000L) tracker.temperature?.toDouble() else null
		return learner().observe("${device.origin}:$hardware:$sensor", temperature, residual, now)
	}

	private fun available(tracker: Tracker, now: Long): Boolean {
		if (!tracker.status.sendData || !tracker.hasRotation) return false
		val age = tracker.lastRotationUpdateNanos?.let { now - it }
		if ((age == null && tracker.usesTimeout) || (age != null && age !in 0..250_000_000L)) return false
		return heading(tracker.getRotationWithoutAdaptive()) != null
	}
}

private fun finite(v: Vector3) = v.x.isFinite() && v.y.isFinite() && v.z.isFinite()
private fun heading(q: Quaternion): Float? {
	if (!q.w.isFinite() || !q.x.isFinite() || !q.y.isFinite() || !q.z.isFinite() || !q.lenSq().isFinite() || q.lenSq() <= 0f) return null
	val forward = q.unit().sandwich(Vector3(0f, 0f, 1f))
	if (forward.x * forward.x + forward.z * forward.z < 0.1f) return null
	return atan2(forward.x, forward.z)
}
