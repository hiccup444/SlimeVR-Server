package dev.slimevr.tracking.processor.adaptive

import dev.slimevr.config.AdaptiveTrackingConfig
import dev.slimevr.tracking.processor.skeleton.HumanSkeleton
import dev.slimevr.tracking.trackers.Tracker
import dev.slimevr.tracking.trackers.TrackerPosition
import io.github.axisangles.ktmath.Quaternion
import io.github.axisangles.ktmath.Vector3
import java.nio.file.Paths
import java.util.concurrent.CompletableFuture
import kotlin.math.abs
import kotlin.math.atan2

data class TrackerDriftDiagnostic(
	val trackerId: Int,
	val biasRadians: Float,
	val residual: DriftResidual?,
	val predictedRateRadiansPerSecond: Double? = null,
	val poseConfidence: GlobalPoseConfidence? = null,
	val correctionMode: String = "UNSPECIFIED",
	val canRecoverPreExistingBias: Boolean = false,
	val learning: YawLearningDiagnostic? = null,
	val historicalConfidenceMultiplier: Float = 1f,
	val holdoverActive: Boolean = false,
	val temperatureModelStatus: String = "UNAVAILABLE",
)

/** Coordinates evidence-gated calibration separately from immediate pose constraints. */
class AdaptiveBodyEstimator(private val config: AdaptiveTrackingConfig) : AutoCloseable {
	private val absoluteArms = AbsoluteArmYawEstimator(config)
	private var calibration: CalibrationLearner? = null
	private var reliability: TrackerReliabilityLearner? = null
	private fun reliabilityLearner(): TrackerReliabilityLearner = reliability ?: TrackerReliabilityLearner(
		TrackerReliabilityStore(Paths.get(config.calibrationDirectory)),
	).also { reliability = it }
	private fun learner(): CalibrationLearner = calibration ?: CalibrationLearner(CalibrationStore(Paths.get(config.calibrationDirectory))).also { calibration = it }

	fun clearLearnedCalibration(): CompletableFuture<Boolean> = learner().clearLearned()
		.thenCombine(reliabilityLearner().clearLearned()) { drift, history -> drift && history }

	/** Historical residuals affect pose fitting only; they cannot block their own calibration evidence. */
	fun applyReliability(frame: AdaptiveTelemetryFrame, trackers: List<Tracker>): AdaptiveTelemetryFrame {
		val byId = trackers.associateBy { it.id }
		return frame.copy(
			samples = frame.samples.map { sample ->
				val key = byId[sample.id]?.takeIf { it.isImu() }?.let(::hardwareKey) ?: return@map sample
				val multiplier = reliabilityLearner().multiplier(key)
				val confidence = sample.confidence ?: return@map sample
				if (multiplier >= 1f) {
					sample
				} else {
					sample.copy(
						confidence = confidence.copy(
							score = confidence.score * multiplier,
							reasons = confidence.reasons + "HISTORICAL_INDEPENDENT_RESIDUAL_PRIOR",
						),
					)
				}
			},
		)
	}

	override fun close() {
		calibration?.close()
		calibration = null
		reliability?.close()
		reliability = null
		reset()
	}
	private data class Reference(val yaw: Float, val headPosition: Vector3, val witnesses: List<Quaternion>, val witnessIds: List<Int>, val context: Long)
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
	private val learningStatistics = mutableMapOf<Int, YawLearningStatistics>()
	var diagnostics: List<TrackerDriftDiagnostic> = emptyList()
		private set

	fun updateAbsoluteConstraints(skeleton: HumanSkeleton, now: Long) {
		absoluteArms.update(skeleton, now, ::observeCalibration)
		val measured = (footDiagnostics + absoluteArms.diagnostics).associateBy { it.trackerId }
		val trackers = skeleton.allHumanBones.mapNotNull { it.attachedTracker }.filter { it.isImu() }.distinctBy { it.id }
		learningStatistics.keys.retainAll(trackers.map { it.id }.toSet())
		diagnostics = trackers.map { tracker ->
			val foot = tracker.trackerPosition == TrackerPosition.LEFT_FOOT || tracker.trackerPosition == TrackerPosition.RIGHT_FOOT
			val arm = tracker.trackerPosition == TrackerPosition.LEFT_UPPER_ARM || tracker.trackerPosition == TrackerPosition.RIGHT_UPPER_ARM
			val mode = when {
				foot -> "PLANTED_REFERENCE_INCREMENTAL_ONLY"
				arm -> "ABSOLUTE_ARM_REACH"
				else -> "NO_INDEPENDENT_YAW_MODEL"
			}
			val blocked = when {
				!config.yawCorrectionEnabled -> "YAW_CORRECTION_DISABLED"
				!config.yawCorrectionStrength.isFinite() || config.yawCorrectionStrength <= 0f -> "CORRECTION_STRENGTH_ZERO"
				skeleton.getPauseTracking() -> "TRACKING_PAUSED"
				skeleton.stayAlignedConfig.enabled -> "BLOCKED_BY_STAY_ALIGNED"
				skeleton.localizer.getEnabled() -> "BLOCKED_BY_LOCALIZER"
				arm && config.armCalibrationMode != "disabled" -> "BLOCKED_BY_ARM_CALIBRATION"
				tracker.resetsHandler.isDriftCompensationActive -> "BLOCKED_BY_LEGACY_DRIFT_COMPENSATION"
				!foot && !arm -> "NO_INDEPENDENT_YAW_MODEL"
				else -> null
			}
			val item = if (blocked == null) measured[tracker.id] else null
			val base = item ?: TrackerDriftDiagnostic(
				tracker.id,
				tracker.adaptiveYawBiasRadians,
				DriftResidual(0f, 0f, 0.0, false, blocked ?: "WAITING_FOR_EVIDENCE"),
			)
			val key = hardwareKey(tracker)
			if (key != null && config.yawCorrectionEnabled && (foot || arm)) {
				val residual = base.residual
				val independentlySupported = residual?.source == DriftEvidenceSource.ABSOLUTE_POSITION_CONSTRAINT && residual.eligibleForLearning
				reliabilityLearner().observe(
					key,
					residual?.let { abs(wrapYaw(it.filteredErrorRadians - base.biasRadians)).toDouble() } ?: 0.0,
					base.poseConfidence?.score?.toDouble() ?: 0.0,
					now,
					independentlySupported,
				)
			}
			base.copy(
				correctionMode = mode,
				canRecoverPreExistingBias = arm,
				historicalConfidenceMultiplier = if (key != null) reliability?.multiplier(key) ?: 1f else 1f,
				learning = learningStatistics.getOrPut(tracker.id) { YawLearningStatistics() }.observe(base.residual, now),
				temperatureModelStatus = temperatureModelStatus(tracker, now, base.predictedRateRadiansPerSecond),
			)
		}
	}

	fun reset() {
		absoluteArms.reset()
		calibration?.resetTransient()
		reliability?.resetTransient()
		states.values.forEach { it.tracker.adaptiveYawBiasRadians = 0f }
		states.clear()
		footDiagnostics = emptyList()
		diagnostics = emptyList()
		learningStatistics.clear()
	}

	fun update(skeleton: HumanSkeleton, now: Long) {
		if (!config.temperatureLearningEnabled) calibration?.resetTransient()
		if (!config.yawCorrectionEnabled || !config.yawCorrectionStrength.isFinite() || config.yawCorrectionStrength <= 0f || skeleton.getPauseTracking() || skeleton.stayAlignedConfig.enabled || skeleton.localizer.getEnabled()) {
			reset()
			return
		}
		val qualityFrame = skeleton.humanPoseManager.adaptiveMeasurementQuality.observe(skeleton, now)
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
			// A head turn does not rotate a planted foot. HMD position and quality
			// are checked separately; never use HMD yaw as a foot-heading reference.
			val witnesses = listOfNotNull(thigh, ankle, skeleton.hipTracker)
			val valid = !tracker.resetsHandler.isDriftCompensationActive &&
				tracker.isImu() &&
				contact.state == FootContactState.PLANTED &&
				contact.weight >= 0.6f &&
				head?.hasPosition == true &&
				thigh != null &&
				ankle != null &&
				available(tracker, now) &&
				available(head, now) &&
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
				if (tracker.resetsHandler.isDriftCompensationActive) {
					state.corrector.reset()
					tracker.adaptiveYawBiasRadians = 0f
				}
				val predictedRate = observeCalibration(tracker, null, now)
				state.reference = null
				state.residual.reset()
				val canPredict = tracker.isImu() && !tracker.resetsHandler.isDriftCompensationActive && available(tracker, now)
				tracker.adaptiveYawBiasRadians = state.corrector.predict(if (canPredict) predictedRate else null, now, config.yawCorrectionStrength, FOOT_HOLDOVER_LIMIT)
				val reason = when {
					tracker.resetsHandler.isDriftCompensationActive -> "BLOCKED_BY_LEGACY_DRIFT_COMPENSATION"
					contact.state != FootContactState.PLANTED -> "WAITING_FOR_PLANTED_FOOT"
					else -> "FOOT_EVIDENCE_UNAVAILABLE_OR_MOVING"
				}
				results.add(TrackerDriftDiagnostic(tracker.id, tracker.adaptiveYawBiasRadians, DriftResidual(0f, 0f, 0.0, false, reason), predictedRate, holdoverActive = state.corrector.predictionActive))
				return
			}
			val reference = state.reference ?: Reference(yaw - state.corrector.biasRadians, head.position, witnesses.map { it.getRotationWithoutAdaptive() }, witnesses.map { it.id }, ++nextContext).also { state.reference = it }
			val stableWitnesses = reference.witnessIds == witnesses.map { it.id } &&
				witnesses.indices.all {
					reference.witnesses[it].angleToR(witnesses[it].getRotationWithoutAdaptive()) < Math.toRadians(1.0)
				}
			if (!stableWitnesses || (head.position - reference.headPosition).len() > 0.02f) {
				val predictedRate = observeCalibration(tracker, null, now)
				state.reference = null
				state.residual.reset()
				tracker.adaptiveYawBiasRadians = state.corrector.predict(predictedRate, now, config.yawCorrectionStrength, FOOT_HOLDOVER_LIMIT)
				results.add(TrackerDriftDiagnostic(tracker.id, tracker.adaptiveYawBiasRadians, DriftResidual(0f, 0f, 0.0, false, "FOOT_REFERENCE_CONTEXT_CHANGED"), predictedRate, holdoverActive = state.corrector.predictionActive))
				return
			}
			val changes = witnesses.indices.map { reference.witnesses[it].angleToR(witnesses[it].getRotationWithoutAdaptive()).toDouble() }
			val poseConfidence = CalibrationPoseConfidence.estimate(
				listOf(tracker, head) + witnesses,
				listOf(head),
				changes,
				changes,
				((head.position - reference.headPosition).len() / 0.02f * 0.15f).coerceIn(0f, 1f),
				now,
				contact.weight,
				qualityFrame = qualityFrame,
			)
			val residual = state.residual.update(
				DriftEvidence(wrapYaw(yaw - reference.yaw), DriftEvidenceSource.PLANTED_CONTACT_WITH_STABLE_CHAIN, reference.context, poseConfidence.score, poseConfidence.score, 0.95f, witnesses.size, available = poseConfidence.learningEligible),
				now,
			)
			val predictedRate = observeCalibration(tracker, residual, now)
			// A mature temperature model compensates the residual filter's lag only while evidence remains eligible.
			val correctionResidual = if (predictedRate != null && residual.eligibleForLearning) residual.copy(filteredErrorRadians = wrapYaw(residual.filteredErrorRadians + (predictedRate * 2.0).toFloat())) else residual
			tracker.adaptiveYawBiasRadians = if (residual.eligibleForLearning) {
				state.corrector.update(correctionResidual, now, config.yawCorrectionStrength)
			} else {
				state.corrector.predict(predictedRate, now, config.yawCorrectionStrength, FOOT_HOLDOVER_LIMIT)
			}
			results.add(TrackerDriftDiagnostic(tracker.id, tracker.adaptiveYawBiasRadians, residual, predictedRate, poseConfidence, holdoverActive = state.corrector.predictionActive))
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

	private fun hardwareKey(tracker: Tracker): String? {
		val device = tracker.device ?: return null
		val hardware = device.hardwareIdentifier.takeIf { it.isNotBlank() && !it.equals("Unknown", true) } ?: return null
		return "${device.origin}:$hardware:${tracker.trackerNum}".takeIf { it.length <= 512 }
	}

	private fun temperatureModelStatus(tracker: Tracker, now: Long, predictedRate: Double?): String {
		if (!config.yawCorrectionEnabled) return "YAW_CORRECTION_DISABLED"
		if (!config.yawCorrectionStrength.isFinite() || config.yawCorrectionStrength <= 0f) return "CORRECTION_STRENGTH_ZERO"
		when (tracker.trackerPosition) {
			TrackerPosition.LEFT_FOOT, TrackerPosition.RIGHT_FOOT, TrackerPosition.LEFT_UPPER_ARM, TrackerPosition.RIGHT_UPPER_ARM -> {}
			else -> return "NO_INDEPENDENT_YAW_MODEL"
		}
		if (!config.temperatureLearningEnabled) return "DISABLED"
		val key = hardwareKey(tracker) ?: return "NO_STABLE_HARDWARE_ID"
		val age = tracker.lastTemperatureUpdateNanos?.let { now - it }
		if (age == null || age !in 0..30_000_000_000L) return "TEMPERATURE_UNAVAILABLE_OR_STALE"
		val temperature = tracker.temperature?.toDouble()
		if (temperature == null || !temperature.isFinite() || temperature !in CalibrationProfile.MIN_TEMPERATURE_C..CalibrationProfile.MAX_TEMPERATURE_C) return "TEMPERATURE_OUT_OF_RANGE"
		val status = calibration?.diagnostics(key) ?: return "PROFILE_NOT_LOADED"
		if (!status.loaded) return "PROFILE_LOADING"
		if (status.loadFailed) return "PROFILE_LOAD_FAILED"
		return if (predictedRate != null) "READY" else "PROFILE_IMMATURE_AT_THIS_TEMPERATURE"
	}

	private fun available(tracker: Tracker, now: Long): Boolean {
		if (!tracker.status.sendData || !tracker.hasRotation) return false
		val age = tracker.lastRotationUpdateNanos?.let { now - it }
		if ((age == null && tracker.usesTimeout) || (age != null && age !in 0..250_000_000L)) return false
		return heading(tracker.getRotationWithoutAdaptive()) != null
	}
}

private fun finite(v: Vector3) = v.x.isFinite() && v.y.isFinite() && v.z.isFinite()
private val FOOT_HOLDOVER_LIMIT = Math.toRadians(0.25).toFloat()
private fun heading(q: Quaternion): Float? {
	if (!q.w.isFinite() || !q.x.isFinite() || !q.y.isFinite() || !q.z.isFinite() || !q.lenSq().isFinite() || q.lenSq() <= 0f) return null
	val forward = q.unit().sandwich(Vector3(0f, 0f, 1f))
	if (forward.x * forward.x + forward.z * forward.z < 0.1f) return null
	return atan2(forward.x, forward.z)
}
