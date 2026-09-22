package dev.slimevr.tracking.processor.adaptive

import dev.slimevr.config.AdaptiveTrackingConfig
import dev.slimevr.tracking.processor.HumanPoseManager
import dev.slimevr.tracking.processor.config.SkeletonConfigOffsets
import dev.slimevr.tracking.processor.skeleton.HumanSkeleton
import dev.slimevr.tracking.trackers.Tracker
import io.github.axisangles.ktmath.Quaternion
import io.github.axisangles.ktmath.Vector3
import java.util.concurrent.Executors
import java.util.concurrent.TimeUnit
import java.util.concurrent.atomic.AtomicBoolean
import kotlin.math.abs

data class ArmCalibrationSideDiagnostic(val side: String, val status: String, val trustedSeconds: Double, val mountingDegrees: Float, val poseConfidence: GlobalPoseConfidence? = null)
data class ArmCalibrationDiagnostic(val mode: String, val upperArmMeters: Float, val lowerArmMeters: Float, val sides: List<ArmCalibrationSideDiagnostic>)

/** Fits calibration only from unoptimized, externally anchored stationary arm geometry. */
class OpportunisticArmCalibration(private val pose: HumanPoseManager, private val config: AdaptiveTrackingConfig) : AutoCloseable {
	private data class Observation(val head: Vector3, val wrist: Vector3, val shoulder: Vector3, val rotations: List<Quaternion>)
	private class Side(val tracker: Tracker, upper: Float, lower: Float) {
		val proportions = ArmProportionLearner(upper.toDouble(), lower.toDouble())
		val mounting = MountingDirectionLearner(upper.toDouble(), lower.toDouble())
		var reference: Observation? = null
		var stationarySince: Long? = null
		var lengths: ArmProportionLearner.ArmLengthEstimate? = null
		var direction: MountingDirectionEstimate? = null
		var status = "WAITING_FOR_STILL_POSE"
		var poseConfidence: GlobalPoseConfidence? = null
	}
	private val sides = mutableMapOf<String, Side>()
	private var mode = "disabled"
	private var previousTime: Long? = null
	private var upper = 0f
	private var lower = 0f
	private var lastSavedTime: Long? = null
	private var dirty = false
	private val saving = AtomicBoolean(false)
	private val writerDelegate = lazy { Executors.newSingleThreadExecutor { work -> Thread(work, "adaptive-arm-calibration-save").apply { isDaemon = true } } }
	private val writer by writerDelegate
	var diagnostic: ArmCalibrationDiagnostic? = null
		private set

	fun reset() {
		sides.values.forEach { it.tracker.adaptiveMountingRotation = Quaternion.IDENTITY }
		sides.clear()
		previousTime = null
		mode = "disabled"
		diagnostic = null
	}

	override fun close() {
		if (dirty) {
			pose.server?.configManager?.let { manager ->
				pose.saveConfig()
				writer.execute { manager.saveConfig() }
			}
			dirty = false
		}
		if (writerDelegate.isInitialized()) {
			writer.shutdown()
			writer.awaitTermination(2, TimeUnit.SECONDS)
		}
		reset()
	}

	/** Called after baseline forward kinematics and before the pose optimizer. Changes affect the next frame. */
	fun update(s: HumanSkeleton, now: Long) {
		val requested = config.armCalibrationMode
		if (requested !in setOf("proportions", "mounting") || s.getPauseTracking() || s.localizer.getEnabled() || s.stayAlignedConfig.enabled) {
			if (mode != "disabled") reset()
			return
		}
		val currentUpper = pose.getOffset(SkeletonConfigOffsets.UPPER_ARM)
		val currentLower = pose.getOffset(SkeletonConfigOffsets.LOWER_ARM)
		if (requested != mode || currentUpper != upper || currentLower != lower) {
			reset()
			if (currentUpper !in 0.15f..0.5f || currentLower !in 0.15f..0.5f) return
			mode = requested
			upper = currentUpper
			lower = currentLower
		}
		val dt = previousTime?.let { now - it }?.takeIf { it in 1..250_000_000L }?.times(1e-9f)
		previousTime = now
		val head = s.headTracker
		val chest = s.upperChestTracker ?: s.chestTracker
		fun observe(name: String, arm: Tracker?, hand: Tracker?, shoulder: Vector3) {
			if (arm == null) {
				sides.remove(name)?.tracker?.adaptiveMountingRotation = Quaternion.IDENTITY
				return
			}
			if (sides[name]?.tracker !== arm) {
				sides.remove(name)?.tracker?.adaptiveMountingRotation = Quaternion.IDENTITY
				sides[name] = Side(arm, upper, lower)
			}
			val state = sides.getValue(name)
			val q = arm.getRotationWithoutAdaptive()
			var displacement = Vector3.NULL
			var trusted = false
			state.poseConfidence = null
			val available = dt != null &&
				head?.hasPosition == true &&
				hand?.hasPosition == true &&
				!head.isInternal &&
				!hand.isInternal &&
				arm.isImu() &&
				chest != null &&
				listOf(head, hand, arm, chest).all { available(it, now) } &&
				!arm.resetsHandler.isDriftCompensationActive &&
				arm.hasAcceleration &&
				arm.lastAccelerationUpdateNanos?.let { now - it in 0..250_000_000L } == true &&
				finite(arm.getAcceleration()) &&
				arm.getAcceleration().len() < 0.5f &&
				finite(head.position) &&
				finite(hand.position) &&
				finite(shoulder)
			if (available) {
				val wrist = hand!!.position + hand.getRotationWithoutAdaptive().sandwich(Vector3(0f, pose.getOffset(SkeletonConfigOffsets.HAND_Y), pose.getOffset(SkeletonConfigOffsets.HAND_Z)))
				displacement = wrist - shoulder
				val observation = Observation(head!!.position, wrist, shoulder, listOf(q, hand.getRotationWithoutAdaptive(), head.getRotationWithoutAdaptive(), chest!!.getRotationWithoutAdaptive()))
				val reference = state.reference
				val stable = reference != null &&
					(reference.head - observation.head).len() < 0.01f &&
					(reference.wrist - observation.wrist).len() < 0.01f &&
					(reference.shoulder - observation.shoulder).len() < 0.01f &&
					reference.rotations.zip(observation.rotations).all { (a, b) -> a.angleToR(b) < Math.toRadians(1.0) }
				if (!stable) {
					state.reference = observation
					state.stationarySince = now
				}
				trusted = stable && state.stationarySince?.let { now - it >= 400_000_000L } == true
				val witnessChanges = reference?.rotations?.drop(1)?.zip(observation.rotations.drop(1))?.map { (a, b) -> a.angleToR(b).toDouble() } ?: emptyList()
				state.poseConfidence = CalibrationPoseConfidence.estimate(
					listOf(head, hand, arm, chest),
					listOf(head, hand),
					witnessChanges,
					witnessChanges,
					if (trusted) 0f else 1f,
					now,
				)
				trusted = trusted && state.poseConfidence?.learningEligible == true
			} else {
				state.reference = null
				state.stationarySince = null
			}
			val confidence = if (trusted) state.poseConfidence?.score ?: 0f else 0f
			state.status = if (trusted) "COLLECTING_VARIED_POSES" else "WAITING_FOR_FRESH_STILL_POSE"
			if (mode == "proportions") {
				state.lengths = state.proportions.observe(displacement, q.sandwich(Vector3.NEG_Y), confidence, now)
				if (state.lengths != null) state.status = "FIT_READY"
			} else {
				state.direction = state.mounting.observe(q, displacement, confidence, now)
				state.direction?.let { estimate ->
					if (dt != null && estimate.confidence >= 0.9f) {
						val target = Quaternion.fromTo(Vector3.NEG_Y, estimate.direction)
						val current = arm.adaptiveMountingRotation
						val angle = current.angleToR(target)
						arm.adaptiveMountingRotation = current.interpR(target, if (angle <= 1e-7f) 1f else (Math.toRadians(0.05).toFloat() * dt / angle).coerceIn(0f, 1f))
						state.status = "ADJUSTING_MOUNTING"
					}
				}
			}
		}
		observe("left", s.leftUpperArmTracker, s.leftHandTracker, s.leftUpperArmBone.getPosition())
		observe("right", s.rightUpperArmTracker, s.rightHandTracker, s.rightUpperArmBone.getPosition())
		if (mode == "proportions" && dt != null) {
			val left = sides["left"]?.lengths
			val right = sides["right"]?.lengths
			if (left != null && right != null && abs(left.upperArmMeters - right.upperArmMeters) <= 0.01 && abs(left.lowerArmMeters - right.lowerArmMeters) <= 0.01) {
				val step = 0.001f * dt
				upper += (((left.upperArmMeters + right.upperArmMeters) * 0.5).toFloat() - upper).coerceIn(-step, step)
				lower += (((left.lowerArmMeters + right.lowerArmMeters) * 0.5).toFloat() - lower).coerceIn(-step, step)
				pose.setOffset(SkeletonConfigOffsets.UPPER_ARM, upper)
				pose.setOffset(SkeletonConfigOffsets.LOWER_ARM, lower)
				dirty = true
				sides.values.forEach { it.status = "ADJUSTING_LENGTHS" }
				if (lastSavedTime?.let { now - it >= 60_000_000_000L } != false) {
					save()
					lastSavedTime = now
				}
			}
		}
		diagnostic = ArmCalibrationDiagnostic(mode, upper, lower, sides.map { (name, state) -> ArmCalibrationSideDiagnostic(name, state.status, state.lengths?.observedSeconds ?: state.direction?.trustedSeconds ?: 0.0, Math.toDegrees(state.tracker.adaptiveMountingRotation.angleR().toDouble()).toFloat(), state.poseConfidence) })
	}

	private fun save() {
		val manager = pose.server?.configManager ?: return
		pose.saveConfig()
		if (saving.compareAndSet(false, true)) {
			dirty = false
			writer.execute {
				try {
					manager.saveConfig()
				} finally {
					saving.set(false)
				}
			}
		}
	}
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
