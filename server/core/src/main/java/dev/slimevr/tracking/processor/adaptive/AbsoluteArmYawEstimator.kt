package dev.slimevr.tracking.processor.adaptive

import dev.slimevr.config.AdaptiveTrackingConfig
import dev.slimevr.tracking.processor.config.SkeletonConfigOffsets
import dev.slimevr.tracking.processor.skeleton.HumanSkeleton
import dev.slimevr.tracking.trackers.Tracker
import io.github.axisangles.ktmath.Quaternion
import io.github.axisangles.ktmath.Vector3
import kotlin.math.abs

/** Uses controller reach geometry, with a separate stable arm as a torso witness. */
class AbsoluteArmYawEstimator(private val config: AdaptiveTrackingConfig) {
	private data class Observation(val positions: List<Vector3>, val witnesses: List<Quaternion>, val localUp: Vector3)
	private class State(val tracker: Tracker) {
		val residual = ResidualTracker(preserveAbsoluteEvidenceAcrossContexts = true)
		val corrector = AdaptiveYawCorrector()
		var reference: Observation? = null
		var previous: Quaternion? = null
		var previousTime: Long? = null
		var context = 0L
	}
	private val states = mutableMapOf<Int, State>()
	var diagnostics: List<TrackerDriftDiagnostic> = emptyList()
		private set

	fun reset() {
		states.values.forEach { it.tracker.adaptiveYawBiasRadians = 0f }
		states.clear()
		diagnostics = emptyList()
	}

	fun update(s: HumanSkeleton, now: Long, observeCalibration: (Tracker, DriftResidual?, Long) -> Double?) {
		if (!config.yawCorrectionEnabled || !config.yawCorrectionStrength.isFinite() || config.yawCorrectionStrength <= 0f || config.armCalibrationMode != "disabled" || s.getPauseTracking() || s.stayAlignedConfig.enabled || s.localizer.getEnabled()) {
			reset()
			return
		}
		val qualityFrame = s.humanPoseManager.adaptiveMeasurementQuality.observe(s, now)
		val pose = s.humanPoseManager
		val upper = pose.getOffset(SkeletonConfigOffsets.UPPER_ARM)
		val lower = pose.getOffset(SkeletonConfigOffsets.LOWER_ARM)
		val handOffset = Vector3(0f, pose.getOffset(SkeletonConfigOffsets.HAND_Y), pose.getOffset(SkeletonConfigOffsets.HAND_Z))
		val head = s.headTracker
		val chest = s.upperChestTracker ?: s.chestTracker
		val armWitnessRotations = listOfNotNull(s.leftUpperArmTracker, s.rightUpperArmTracker).associate { it.id to it.getRotation() }
		val results = mutableListOf<TrackerDriftDiagnostic>()
		fun arm(tracker: Tracker?, hand: Tracker?, shoulder: Vector3, other: Tracker?, otherHand: Tracker?, otherShoulder: Vector3) {
			if (tracker == null || !tracker.isImu()) return
			if (states[tracker.id]?.tracker !== tracker) {
				states.remove(tracker.id)?.tracker?.adaptiveYawBiasRadians = 0f
				states[tracker.id] = State(tracker)
			}
			val state = states.getValue(tracker.id)
			val measured = tracker.getRotationWithoutAdaptive()
			val elapsed = state.previousTime?.let { now - it }
			val speed = if (elapsed != null && elapsed in 1..250_000_000L) state.previous?.angleToR(measured)?.div(elapsed * 1e-9f) else null
			state.previous = measured
			state.previousTime = now
			fun reject(reason: String) {
				if (tracker.resetsHandler.isDriftCompensationActive) {
					state.corrector.reset()
					tracker.adaptiveYawBiasRadians = 0f
				}
				state.reference = null
				val residual = if (reason == "ARM_MOTION" || reason == "ABSOLUTE_ARM_EVIDENCE_UNAVAILABLE") {
					state.residual.pause(now, reason)
				} else {
					state.residual.reset()
					DriftResidual(0f, 0f, 0.0, false, reason)
				}
				val predictedRate = observeCalibration(tracker, null, now)
				val canPredict = !tracker.resetsHandler.isDriftCompensationActive &&
					other?.resetsHandler?.isDriftCompensationActive != true &&
					reason != "OPPOSITE_ARM_DISAGREEMENT" &&
					available(tracker, now)
				tracker.adaptiveYawBiasRadians = state.corrector.predict(if (canPredict) predictedRate else null, now, config.yawCorrectionStrength)
				results.add(TrackerDriftDiagnostic(tracker.id, tracker.adaptiveYawBiasRadians, residual, predictedRate, holdoverActive = state.corrector.predictionActive))
			}
			if (head == null ||
				chest == null ||
				hand == null ||
				other == null ||
				otherHand == null ||
				!other.isImu() ||
				!head.hasPosition ||
				!hand.hasPosition ||
				!otherHand.hasPosition ||
				head.isInternal ||
				hand.isInternal ||
				otherHand.isInternal ||
				!listOf(tracker, head, chest, hand, other, otherHand).all { available(it, now) } ||
				!stationaryAcceleration(tracker, now) ||
				!stationaryAcceleration(other, now) ||
				tracker.resetsHandler.isDriftCompensationActive ||
				other.resetsHandler.isDriftCompensationActive ||
				!finite(shoulder) ||
				!finite(otherShoulder) ||
				!finite(head.position) ||
				!finite(hand.position) ||
				!finite(otherHand.position)
			) {
				reject("ABSOLUTE_ARM_EVIDENCE_UNAVAILABLE")
				return
			}
			if (speed == null || !speed.isFinite() || speed > Math.toRadians(0.5)) {
				reject("ARM_MOTION")
				return
			}
			val wrist = hand.position + hand.getRotationWithoutAdaptive().sandwich(handOffset)
			val otherWrist = otherHand.position + otherHand.getRotationWithoutAdaptive().sandwich(handOffset)
			val constraint = ArmYawConstraint.estimate(wrist - shoulder, measured.sandwich(Vector3.NEG_Y), upper.toDouble(), lower.toDouble())
			val otherRotation = armWitnessRotations[other.id] ?: other.getRotation()
			val otherConstraint = ArmYawConstraint.estimate(otherWrist - otherShoulder, otherRotation.sandwich(Vector3.NEG_Y), upper.toDouble(), lower.toDouble())
			if (constraint == null || otherConstraint == null) {
				reject("ARM_YAW_UNOBSERVABLE_OR_AMBIGUOUS")
				return
			}
			if (abs(otherConstraint.residualBiasRadians) > Math.toRadians(1.0)) {
				reject("OPPOSITE_ARM_DISAGREEMENT")
				return
			}
			val observation = Observation(
				listOf(head.position, wrist, otherWrist, shoulder, otherShoulder),
				listOf(head.getRotationWithoutAdaptive(), chest.getRotationWithoutAdaptive(), hand.getRotationWithoutAdaptive(), otherHand.getRotationWithoutAdaptive(), other.getRotationWithoutAdaptive()),
				measured.inv().sandwich(Vector3.POS_Y),
			)
			val reference = state.reference
			val stable = reference != null &&
				reference.positions.zip(observation.positions).all { (a, b) -> (a - b).len() <= 0.01f } &&
				reference.witnesses.zip(observation.witnesses).all { (a, b) -> a.angleToR(b) <= Math.toRadians(1.0) } &&
				(reference.localUp - observation.localUp).len() <= 0.0175f
			if (!stable) {
				state.reference = observation
				state.context++
			}
			val witnessChanges = reference?.witnesses?.zip(observation.witnesses)?.map { (a, b) -> a.angleToR(b).toDouble() } ?: emptyList()
			val poseConfidence = CalibrationPoseConfidence.estimate(
				listOf(tracker, head, chest, hand, other, otherHand),
				listOf(head, hand, otherHand),
				witnessChanges,
				listOf(abs(otherConstraint.residualBiasRadians)),
				if (stable) 0f else 1f,
				now,
				qualityFrame = qualityFrame,
			)
			val residual = if (!stable) state.residual.pause(now, "ARM_REFERENCE_CHANGED") else state.residual.update(DriftEvidence(constraint.residualBiasRadians.toFloat(), DriftEvidenceSource.ABSOLUTE_POSITION_CONSTRAINT, state.context, poseConfidence.score, poseConfidence.score, 0.95f, 3, available = poseConfidence.learningEligible), now)
			val predictedRate = observeCalibration(tracker, residual, now)
			val correctedResidual = if (predictedRate != null && residual.eligibleForLearning) residual.copy(filteredErrorRadians = wrapYaw(residual.filteredErrorRadians + (predictedRate * 2.0).toFloat())) else residual
			tracker.adaptiveYawBiasRadians = if (residual.eligibleForLearning) {
				state.corrector.update(correctedResidual, now, config.yawCorrectionStrength)
			} else {
				state.corrector.predict(predictedRate, now, config.yawCorrectionStrength)
			}
			results.add(TrackerDriftDiagnostic(tracker.id, tracker.adaptiveYawBiasRadians, residual, predictedRate, poseConfidence, holdoverActive = state.corrector.predictionActive))
		}
		arm(s.leftUpperArmTracker, s.leftHandTracker, s.leftUpperArmBone.getPosition(), s.rightUpperArmTracker, s.rightHandTracker, s.rightUpperArmBone.getPosition())
		arm(s.rightUpperArmTracker, s.rightHandTracker, s.rightUpperArmBone.getPosition(), s.leftUpperArmTracker, s.leftHandTracker, s.leftUpperArmBone.getPosition())
		val present = results.map { it.trackerId }.toSet()
		states.keys.filter { it !in present }.forEach { states.remove(it)?.tracker?.adaptiveYawBiasRadians = 0f }
		diagnostics = results
	}

	private fun stationaryAcceleration(tracker: Tracker, now: Long): Boolean = tracker.hasAcceleration && tracker.lastAccelerationUpdateNanos?.let { now - it in 0..250_000_000L } == true && finite(tracker.getAcceleration()) && tracker.getAcceleration().len() < 0.5f
	private fun available(tracker: Tracker, now: Long): Boolean {
		val age = tracker.lastRotationUpdateNanos?.let { now - it }
		val q = tracker.getRotationWithoutAdaptive()
		return tracker.status.sendData &&
			tracker.hasRotation &&
			((age == null && !tracker.usesTimeout) || (age != null && age in 0..250_000_000L)) &&
			q.w.isFinite() &&
			q.x.isFinite() &&
			q.y.isFinite() &&
			q.z.isFinite() &&
			q.lenSq().isFinite() &&
			q.lenSq() > 0f
	}
	private fun finite(v: Vector3) = v.x.isFinite() && v.y.isFinite() && v.z.isFinite()
}
