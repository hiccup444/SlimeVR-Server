package dev.slimevr.tracking.processor.adaptive

import dev.slimevr.config.AdaptiveTrackingConfig
import dev.slimevr.tracking.processor.skeleton.HumanSkeleton
import dev.slimevr.tracking.trackers.Tracker
import io.github.axisangles.ktmath.Quaternion
import io.github.axisangles.ktmath.Vector3
import kotlin.math.atan2

/** Corrects new yaw drift only while an externally anchored body chain remains still. */
class StationaryChainYawEstimator(private val config: AdaptiveTrackingConfig) {
	private data class Reference(
		val rawYaw: Float,
		val correctedYaw: Float,
		val anchorIds: List<Int>,
		val anchorPositions: List<Vector3>,
		val anchorRotations: List<Quaternion>,
		val witnesses: List<Pair<Int, Quaternion>>,
		val context: Long,
	)
	private class State(val tracker: Tracker) {
		val residual = ResidualTracker()
		val corrector = AdaptiveYawCorrector()
		var reference: Reference? = null
		var previousRotation: Quaternion? = null
		var previousTime: Long? = null
	}
	private data class Target(val tracker: Tracker, val witnesses: List<Tracker?>)
	private data class AnchorSample(val ids: List<Int>, val positions: List<Vector3>, val rotations: List<Quaternion>, val time: Long)

	private val states = mutableMapOf<Int, State>()
	private var previousAnchors: AnchorSample? = null
	private var nextContext = 0L
	var diagnostics: List<TrackerDriftDiagnostic> = emptyList()
		private set

	fun reset() {
		states.values.forEach { it.tracker.adaptiveYawBiasRadians = 0f }
		states.clear()
		previousAnchors = null
		diagnostics = emptyList()
	}

	fun update(s: HumanSkeleton, now: Long, observeCalibration: (Tracker, DriftResidual?, Long) -> Double?) {
		if (!config.yawCorrectionEnabled || !config.yawCorrectionStrength.isFinite() || config.yawCorrectionStrength <= 0f || s.getPauseTracking() || s.stayAlignedConfig.enabled || s.localizer.getEnabled()) {
			reset()
			return
		}
		val anchors = listOfNotNull(s.headTracker, s.leftHandTracker, s.rightHandTracker)
		val anchorsAvailable = anchors.size == 3 && anchors.distinctBy { it.id }.size == 3 && anchors.all { !it.isInternal && it.hasPosition && available(it, now) && finite(it.position) }
		val anchorIds = if (anchorsAvailable) anchors.map { it.id } else emptyList()
		val anchorPositions = if (anchorsAvailable) anchors.map { it.position } else emptyList()
		val anchorRotations = if (anchorsAvailable) anchors.map { it.getRotationWithoutAdaptive() } else emptyList()
		val previous = previousAnchors
		val elapsed = previous?.let { now - it.time }
		val anchorsQuiet = anchorsAvailable &&
			previous != null &&
			previous.ids == anchorIds &&
			elapsed != null &&
			elapsed in 1..250_000_000L &&
			anchorPositions.indices.all { index ->
				(anchorPositions[index] - previous.positions[index]).len() / (elapsed * 1e-9f) < 0.02f &&
					previous.rotations[index].angleToR(anchorRotations[index]) / (elapsed * 1e-9f) < Math.toRadians(0.5)
			}
		previousAnchors = if (anchorsAvailable) AnchorSample(anchorIds, anchorPositions, anchorRotations, now) else null
		val leftContact = s.legTweaks.adaptiveLeftFoot.snapshot
		val rightContact = s.legTweaks.adaptiveRightFoot.snapshot
		val contact = minOf(leftContact.weight, rightContact.weight)
		val feetPlanted = leftContact.state == FootContactState.PLANTED && rightContact.state == FootContactState.PLANTED && contact >= 0.85f
		val qualityFrame = s.humanPoseManager.adaptiveMeasurementQuality.observe(s, now)
		val qualityById = qualityFrame.samples.associateBy { it.id }
		val chest = s.upperChestTracker ?: s.chestTracker
		val waist = s.waistTracker
		val hip = s.hipTracker
		val leftThigh = s.leftUpperLegTracker
		val rightThigh = s.rightUpperLegTracker
		val leftShin = s.leftLowerLegTracker
		val rightShin = s.rightLowerLegTracker
		val targets = listOfNotNull(
			s.upperChestTracker?.let { Target(it, listOf(waist, hip, leftThigh, rightThigh)) },
			s.chestTracker?.let { Target(it, listOf(waist, hip, leftThigh, rightThigh)) },
			waist?.let { Target(it, listOf(chest, hip, leftThigh, rightThigh)) },
			hip?.let { Target(it, listOf(chest, waist, leftThigh, rightThigh)) },
			leftThigh?.let { Target(it, listOf(hip ?: waist, leftShin, rightThigh)) },
			rightThigh?.let { Target(it, listOf(hip ?: waist, rightShin, leftThigh)) },
			leftShin?.let { Target(it, listOf(leftThigh, s.leftFootTracker, rightShin)) },
			rightShin?.let { Target(it, listOf(rightThigh, s.rightFootTracker, leftShin)) },
		).distinctBy { it.tracker.id }
		val rawRotations = (targets.map { it.tracker } + targets.flatMap { it.witnesses.filterNotNull() })
			.distinctBy { it.id }
			.associate { it.id to it.getRotationWithoutAdaptive() }
		val results = mutableListOf<TrackerDriftDiagnostic>()
		for ((tracker, candidates) in targets) {
			if (!tracker.isImu()) continue
			if (states[tracker.id]?.tracker !== tracker) {
				states.remove(tracker.id)?.tracker?.adaptiveYawBiasRadians = 0f
				states[tracker.id] = State(tracker)
			}
			val state = states.getValue(tracker.id)
			val measured = rawRotations.getValue(tracker.id)
			val yaw = heading(measured)
			val delta = state.previousTime?.let { now - it }
			val speed = if (delta != null && delta in 1..250_000_000L) state.previousRotation?.angleToR(measured)?.div(delta * 1e-9f) else null
			state.previousRotation = measured
			state.previousTime = now
			val witnesses = candidates.filterNotNull().filter { it.id != tracker.id }.distinctBy { it.id }
			fun reject(reason: String, clearReference: Boolean = true) {
				if (clearReference) {
					state.reference = null
					state.residual.reset()
				}
				if (tracker.resetsHandler.isDriftCompensationActive || !available(tracker, now) || reason == "TARGET_ORIENTATION_DISCONTINUITY") {
					state.corrector.reset()
					tracker.adaptiveYawBiasRadians = 0f
				}
				val rate = observeCalibration(tracker, null, now)
				val mayPredict = reason == "CHAIN_WITNESSES_UNAVAILABLE" && anchorsQuiet && feetPlanted && available(tracker, now) && !tracker.resetsHandler.isDriftCompensationActive
				tracker.adaptiveYawBiasRadians = state.corrector.predict(if (mayPredict) rate else null, now, config.yawCorrectionStrength, HOLDOVER_LIMIT)
				results.add(TrackerDriftDiagnostic(tracker.id, tracker.adaptiveYawBiasRadians, DriftResidual(0f, 0f, 0.0, false, reason), rate, holdoverActive = state.corrector.predictionActive))
			}
			if (qualityById[tracker.id]?.health?.reasons?.contains("ORIENTATION_DISCONTINUITY") == true) {
				reject("TARGET_ORIENTATION_DISCONTINUITY")
				continue
			}
			if (!anchorsAvailable || !anchorsQuiet || !feetPlanted || !available(tracker, now) || yaw == null || !stationaryAcceleration(tracker, now) || speed == null || !speed.isFinite() || speed > Math.toRadians(0.5) || tracker.resetsHandler.isDriftCompensationActive) {
				reject(
					if (!feetPlanted) {
						"WAITING_FOR_BOTH_PLANTED_FEET"
					} else if (!anchorsQuiet) {
						"ABSOLUTE_ANCHORS_MOVING_OR_UNAVAILABLE"
					} else {
						"TARGET_MOTION_OR_UNAVAILABLE"
					},
				)
				continue
			}
			if (witnesses.size < 3 || witnesses.any { !it.isImu() || !available(it, now) || it.resetsHandler.isDriftCompensationActive }) {
				reject("CHAIN_WITNESSES_UNAVAILABLE")
				continue
			}
			val observedWitnesses = witnesses.map { it.id to rawRotations.getValue(it.id) }
			val reference = state.reference
			if (reference == null) {
				state.reference = Reference(yaw, wrapYaw(yaw - state.corrector.biasRadians), anchorIds, anchorPositions, anchorRotations, observedWitnesses, ++nextContext)
				state.residual.reset()
				reject("STATIONARY_REFERENCE_CAPTURING", clearReference = false)
				continue
			}
			val witnessChanges = reference.witnesses.zip(observedWitnesses).map { (prior, current) -> prior.second.angleToR(current.second).toDouble() }
			val stable = reference.anchorIds == anchorIds &&
				reference.witnesses.map { it.first } == observedWitnesses.map { it.first } &&
				anchorPositions.indices.all { index ->
					(anchorPositions[index] - reference.anchorPositions[index]).len() <= 0.02f &&
						reference.anchorRotations[index].angleToR(anchorRotations[index]) <= Math.toRadians(1.0)
				} &&
				witnessChanges.all { it <= Math.toRadians(1.0) }
			if (!stable) {
				reject("STATIONARY_REFERENCE_CHANGED")
				continue
			}
			val witnessYawChanges = reference.witnesses.zip(observedWitnesses).map { (prior, current) ->
				val before = heading(prior.second)
				val after = heading(current.second)
				if (before == null || after == null) null else wrapYaw(after - before)
			}
			if (witnessYawChanges.any { it == null }) {
				reject("CHAIN_HEADING_UNOBSERVABLE")
				continue
			}
			val targetChange = wrapYaw(yaw - reference.rawYaw)
			val coherentWitnesses = witnessYawChanges.filterNotNull().count { witnessChange ->
				targetChange * witnessChange > 0f &&
					kotlin.math.abs(witnessChange) >= maxOf(Math.toRadians(0.15).toFloat(), kotlin.math.abs(targetChange) * 0.5f)
			}
			if (kotlin.math.abs(targetChange) >= Math.toRadians(0.2) && coherentWitnesses >= 2) {
				reject("COHERENT_CHAIN_MOTION_OR_SHARED_DRIFT")
				continue
			}
			val participants = listOf(tracker) + anchors + witnesses
			val participantsGood = participants.all { (qualityById[it.id]?.confidence?.score ?: 0f) >= 0.85f }
			val measuredConfidence = CalibrationPoseConfidence.estimate(
				participants,
				anchors,
				witnessChanges,
				witnessChanges.take(2),
				0f,
				now,
				contact,
				qualityFrame,
			)
			val confidence = if (participantsGood) measuredConfidence else GlobalPoseConfidence(0f, measuredConfidence.reasons + "TRACKER_CONFIDENCE_LOW", false)
			val residual = state.residual.update(
				DriftEvidence(wrapYaw(yaw - reference.correctedYaw), DriftEvidenceSource.STATIONARY_CHAIN_REFERENCE, reference.context, confidence.score, confidence.score, 0.95f, witnesses.size, available = confidence.learningEligible),
				now,
			)
			val rate = observeCalibration(tracker, residual, now)
			val correctedResidual = if (rate != null && residual.eligibleForLearning) residual.copy(filteredErrorRadians = wrapYaw(residual.filteredErrorRadians + (rate * 2.0).toFloat())) else residual
			tracker.adaptiveYawBiasRadians = if (residual.eligibleForLearning) state.corrector.update(correctedResidual, now, config.yawCorrectionStrength) else state.corrector.predict(null, now, config.yawCorrectionStrength, HOLDOVER_LIMIT)
			results.add(TrackerDriftDiagnostic(tracker.id, tracker.adaptiveYawBiasRadians, residual, rate, confidence, holdoverActive = state.corrector.predictionActive))
		}
		val present = results.map { it.trackerId }.toSet()
		states.keys.filter { it !in present }.forEach { states.remove(it)?.tracker?.adaptiveYawBiasRadians = 0f }
		diagnostics = results
	}

	private fun stationaryAcceleration(tracker: Tracker, now: Long): Boolean = tracker.hasAcceleration && tracker.lastAccelerationUpdateNanos?.let { now - it in 0..250_000_000L } == true && finite(tracker.getAcceleration()) && tracker.getAcceleration().len() < 0.5f
	private fun available(tracker: Tracker, now: Long): Boolean {
		val age = tracker.lastRotationUpdateNanos?.let { now - it }
		val rotation = tracker.getRotationWithoutAdaptive()
		return tracker.status.sendData &&
			tracker.hasRotation &&
			((age == null && !tracker.usesTimeout) || (age != null && age in 0..250_000_000L)) &&
			rotation.w.isFinite() &&
			rotation.x.isFinite() &&
			rotation.y.isFinite() &&
			rotation.z.isFinite() &&
			rotation.lenSq().isFinite() &&
			rotation.lenSq() in 0.5f..1.5f
	}
	private fun finite(v: Vector3) = v.x.isFinite() && v.y.isFinite() && v.z.isFinite()
	private fun heading(q: Quaternion): Float? {
		val norm = q.lenSq()
		if (!q.w.isFinite() || !q.x.isFinite() || !q.y.isFinite() || !q.z.isFinite() || !norm.isFinite() || norm !in 0.5f..1.5f) return null
		val forward = q.unit().sandwich(Vector3(0f, 0f, 1f))
		if (forward.x * forward.x + forward.z * forward.z < 0.1f) return null
		return atan2(forward.x, forward.z)
	}
	private companion object {
		val HOLDOVER_LIMIT = Math.toRadians(0.25).toFloat()
	}
}
