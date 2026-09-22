package dev.slimevr.tracking.processor.adaptive

import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sqrt

enum class AdaptiveActivityState { UNKNOWN, STANDING, WALKING, RUNNING, CROUCHING, SITTING, LYING, HIGH_MOTION }

data class ActivityVector3(val x: Double, val y: Double, val z: Double) {
	fun length(): Double = sqrt(x * x + y * y + z * z)
	fun dot(other: ActivityVector3): Double = x * other.x + y * other.y + z * other.z
	fun isFinite(): Boolean = x.isFinite() && y.isFinite() && z.isFinite()
	fun normalized(): ActivityVector3 {
		val magnitude = length()
		return ActivityVector3(x / magnitude, y / magnitude, z / magnitude)
	}
}

/** Sensor evidence for one classifier update. Segment directions point from proximal to distal joint. */
data class AdaptiveActivityObservation(
	val nowNanos: Long,
	val normalizedHeadHeight: Double?,
	val headTimeNanos: Long?,
	val torsoUpY: Double?,
	val torsoTimeNanos: Long?,
	val leftThighDirection: ActivityVector3?,
	val leftShinDirection: ActivityVector3?,
	val rightThighDirection: ActivityVector3?,
	val rightShinDirection: ActivityVector3?,
	val legsTimeNanos: Long?,
	val headVelocityMetersPerSecond: ActivityVector3?,
	val leftFootVelocityMetersPerSecond: ActivityVector3?,
	val rightFootVelocityMetersPerSecond: ActivityVector3?,
	val motionTimeNanos: Long?,
	val leftFootContact: Double?,
	val rightFootContact: Double?,
	val contactTimeNanos: Long?,
)

data class AdaptiveActivityEstimate(
	val state: AdaptiveActivityState,
	val confidence: Double,
	val reason: String,
	val candidateSeconds: Double,
)

/** Conservative broad posture and movement classifier. It has no skeleton or tracker dependencies. */
class AdaptiveActivityClassifier {
	private var previousTimeNanos: Long? = null
	private var candidate = AdaptiveActivityState.UNKNOWN
	private var candidateSinceNanos: Long? = null
	private var published = AdaptiveActivityState.UNKNOWN
	var estimate = AdaptiveActivityEstimate(AdaptiveActivityState.UNKNOWN, 0.0, "NO_EVIDENCE", 0.0)
		private set

	fun reset() {
		previousTimeNanos = null
		candidate = AdaptiveActivityState.UNKNOWN
		candidateSinceNanos = null
		published = AdaptiveActivityState.UNKNOWN
		estimate = AdaptiveActivityEstimate(AdaptiveActivityState.UNKNOWN, 0.0, "RESET", 0.0)
	}

	fun update(observation: AdaptiveActivityObservation): AdaptiveActivityEstimate {
		val invalid = validate(observation)
		if (invalid != null) return unknown(invalid)
		val previousTime = previousTimeNanos
		if (previousTime != null && observation.nowNanos - previousTime !in 1..MAX_UPDATE_GAP_NANOS) return unknown("UPDATE_GAP")
		previousTimeNanos = observation.nowNanos

		val classification = classify(observation)
		if (classification.state == AdaptiveActivityState.UNKNOWN) return unknown(classification.reason)
		if (classification.state != candidate) {
			candidate = classification.state
			candidateSinceNanos = observation.nowNanos
		}
		val seconds = ((observation.nowNanos - (candidateSinceNanos ?: observation.nowNanos)) * 1e-9).coerceAtLeast(0.0)
		val dwell = dwellSeconds(classification.state)
		val required = if (published == AdaptiveActivityState.UNKNOWN || published == classification.state) dwell else max(dwell, SWITCH_DWELL_SECONDS)
		if (seconds < required) {
			if (published == AdaptiveActivityState.UNKNOWN) {
				estimate = AdaptiveActivityEstimate(AdaptiveActivityState.UNKNOWN, 0.0, "DWELLING_${classification.reason}", seconds)
			} else {
				estimate = AdaptiveActivityEstimate(published, min(0.8, classification.confidence), "HOLDING_${published.name}_DURING_${classification.reason}", seconds)
			}
			return estimate
		}
		published = classification.state
		estimate = AdaptiveActivityEstimate(published, classification.confidence, classification.reason, seconds)
		return estimate
	}

	private fun classify(input: AdaptiveActivityObservation): Classified {
		val headHeight = input.normalizedHeadHeight!!
		val torsoUp = input.torsoUpY!!
		val leftThigh = input.leftThighDirection!!.normalized()
		val leftShin = input.leftShinDirection!!.normalized()
		val rightThigh = input.rightThighDirection!!.normalized()
		val rightShin = input.rightShinDirection!!.normalized()
		val headSpeed = input.headVelocityMetersPerSecond!!.length()
		val leftSpeed = input.leftFootVelocityMetersPerSecond!!.length()
		val rightSpeed = input.rightFootVelocityMetersPerSecond!!.length()
		val fastestFoot = max(leftSpeed, rightSpeed)
		val slowestFoot = min(leftSpeed, rightSpeed)
		val leftBend = leftThigh.dot(leftShin)
		val rightBend = rightThigh.dot(rightShin)
		val haveContacts = input.leftFootContact != null && input.rightFootContact != null
		val bothPlanted = input.leftFootContact?.let { it >= PLANTED_CONTACT } == true && input.rightFootContact?.let { it >= PLANTED_CONTACT } == true
		val leftPlanted = input.leftFootContact?.let { it >= PLANTED_CONTACT } == true
		val rightPlanted = input.rightFootContact?.let { it >= PLANTED_CONTACT } == true
		val onePlanted = leftPlanted != rightPlanted

		if (fastestFoot >= HIGH_MOTION_SPEED || headSpeed >= HIGH_MOTION_SPEED) return Classified(AdaptiveActivityState.HIGH_MOTION, 0.88, "HIGH_VELOCITY")
		val upright = headHeight >= GAIT_MIN_HEAD_HEIGHT && torsoUp >= GAIT_MIN_TORSO_UP
		val alternatingSupport = haveContacts && onePlanted
		val translatedGait = headSpeed >= GAIT_TRANSLATION_SPEED && slowestFoot >= MIN_MOVING_FOOT_SPEED
		if (upright && fastestFoot >= RUNNING_SPEED && (alternatingSupport || translatedGait) && (headSpeed >= RUNNING_HEAD_SPEED || slowestFoot >= RUNNING_SUPPORT_SPEED)) return Classified(AdaptiveActivityState.RUNNING, 0.9, "RUNNING_VELOCITY_AND_GAIT_EVIDENCE")
		if (upright && haveContacts && fastestFoot >= WALKING_SPEED && (alternatingSupport || translatedGait)) return Classified(AdaptiveActivityState.WALKING, 0.82, "WALKING_VELOCITY_AND_GAIT_EVIDENCE")

		if (headHeight < LYING_MAX_HEAD_HEIGHT && torsoUp < LYING_MAX_TORSO_UP && headSpeed <= LYING_MAX_HEAD_SPEED && fastestFoot <= LYING_MAX_FOOT_SPEED) {
			return Classified(AdaptiveActivityState.LYING, 0.84, "LOW_HEAD_HORIZONTAL_TORSO_AND_LOW_MOTION")
		}
		if (headHeight >= STANDING_MIN_HEAD_HEIGHT && torsoUp >= UPRIGHT_TORSO_UP && leftBend >= STRAIGHT_LEG_DOT && rightBend >= STRAIGHT_LEG_DOT && bothPlanted && headSpeed <= STATIONARY_SPEED && fastestFoot <= STATIONARY_SPEED) {
			return Classified(AdaptiveActivityState.STANDING, 0.9, "UPRIGHT_STRAIGHT_LEGS_PAIRED_CONTACT")
		}
		if (headHeight in SITTING_HEAD_RANGE && torsoUp >= SITTING_MIN_TORSO_UP && abs(leftThigh.y) <= SITTING_THIGH_VERTICAL_MAX && abs(rightThigh.y) <= SITTING_THIGH_VERTICAL_MAX && leftShin.y <= DOWNWARD_SHIN_MAX && rightShin.y <= DOWNWARD_SHIN_MAX && fastestFoot <= STATIONARY_SPEED) {
			return Classified(AdaptiveActivityState.SITTING, 0.78, "UPRIGHT_TORSO_WITH_SEATED_LEG_GEOMETRY")
		}
		if (headHeight in CROUCH_HEAD_RANGE && torsoUp >= CROUCH_MIN_TORSO_UP && leftBend <= BENT_LEG_DOT && rightBend <= BENT_LEG_DOT && leftThigh.y < 0.0 && rightThigh.y < 0.0 && fastestFoot <= STATIONARY_SPEED) {
			return Classified(AdaptiveActivityState.CROUCHING, 0.8, "LOWER_HEAD_AND_BILATERAL_KNEE_BEND")
		}
		return Classified(AdaptiveActivityState.UNKNOWN, 0.0, "AMBIGUOUS_POSTURE_OR_ACTIVITY")
	}

	private fun validate(input: AdaptiveActivityObservation): String? {
		fun fresh(time: Long?) = time != null && input.nowNanos - time in 0..MAX_SENSOR_AGE_NANOS
		if (!fresh(input.headTimeNanos) || !fresh(input.torsoTimeNanos) || !fresh(input.legsTimeNanos) || !fresh(input.motionTimeNanos)) return "MISSING_OR_STALE_EVIDENCE"
		val hasLeftContact = input.leftFootContact != null
		val hasRightContact = input.rightFootContact != null
		if (hasLeftContact != hasRightContact) return "INCOMPLETE_CONTACT_EVIDENCE"
		if (hasLeftContact && !fresh(input.contactTimeNanos)) return "MISSING_OR_STALE_EVIDENCE"
		val headHeight = input.normalizedHeadHeight
		if (headHeight == null || !headHeight.isFinite() || headHeight !in 0.0..MAX_NORMALIZED_HEAD_HEIGHT) return "INVALID_HEAD_HEIGHT"
		val torsoUp = input.torsoUpY
		if (torsoUp == null || !torsoUp.isFinite() || torsoUp !in -1.0..1.0) return "INVALID_TORSO_DIRECTION"
		val directions = listOf(input.leftThighDirection, input.leftShinDirection, input.rightThighDirection, input.rightShinDirection)
		if (directions.any { it == null || !it.isFinite() || it.length() !in MIN_DIRECTION_LENGTH..MAX_DIRECTION_LENGTH }) return "INVALID_LEG_GEOMETRY"
		val velocities = listOf(input.headVelocityMetersPerSecond, input.leftFootVelocityMetersPerSecond, input.rightFootVelocityMetersPerSecond)
		if (velocities.any { it == null || !it.isFinite() || it.length() > MAX_SPEED }) return "INVALID_VELOCITY"
		val contacts = listOf(input.leftFootContact, input.rightFootContact).filterNotNull()
		if (contacts.any { !it.isFinite() || it !in 0.0..1.0 }) return "INVALID_CONTACT_CONFIDENCE"
		return null
	}

	private fun unknown(reason: String): AdaptiveActivityEstimate {
		previousTimeNanos = null
		candidate = AdaptiveActivityState.UNKNOWN
		candidateSinceNanos = null
		published = AdaptiveActivityState.UNKNOWN
		return AdaptiveActivityEstimate(AdaptiveActivityState.UNKNOWN, 0.0, reason, 0.0).also { estimate = it }
	}

	private fun dwellSeconds(state: AdaptiveActivityState): Double = when (state) {
		AdaptiveActivityState.STANDING, AdaptiveActivityState.WALKING, AdaptiveActivityState.RUNNING -> GAIT_DWELL_SECONDS
		AdaptiveActivityState.CROUCHING, AdaptiveActivityState.SITTING, AdaptiveActivityState.LYING -> POSTURE_DWELL_SECONDS
		AdaptiveActivityState.HIGH_MOTION -> HIGH_MOTION_DWELL_SECONDS
		AdaptiveActivityState.UNKNOWN -> Double.POSITIVE_INFINITY
	}

	private data class Classified(val state: AdaptiveActivityState, val confidence: Double, val reason: String)

	companion object {
		private const val MAX_SENSOR_AGE_NANOS = 250_000_000L
		private const val MAX_UPDATE_GAP_NANOS = 500_000_000L
		private const val MAX_NORMALIZED_HEAD_HEIGHT = 1.5
		private const val MIN_DIRECTION_LENGTH = 0.75
		private const val MAX_DIRECTION_LENGTH = 1.25
		private const val MAX_SPEED = 10.0
		private const val PLANTED_CONTACT = 0.8
		private const val STATIONARY_SPEED = 0.15
		private const val MIN_MOVING_FOOT_SPEED = 0.12
		private const val WALKING_SPEED = 0.35
		private const val RUNNING_SPEED = 2.0
		private const val RUNNING_HEAD_SPEED = 0.8
		private const val RUNNING_SUPPORT_SPEED = 0.5
		private const val HIGH_MOTION_SPEED = 4.0
		private const val GAIT_MIN_HEAD_HEIGHT = 0.82
		private const val GAIT_MIN_TORSO_UP = 0.72
		private const val GAIT_TRANSLATION_SPEED = 0.35
		private const val STANDING_MIN_HEAD_HEIGHT = 0.82
		private const val UPRIGHT_TORSO_UP = 0.82
		private const val STRAIGHT_LEG_DOT = 0.88
		private const val BENT_LEG_DOT = 0.72
		private const val LYING_MAX_HEAD_HEIGHT = 0.72
		private const val LYING_MAX_TORSO_UP = 0.35
		private const val LYING_MAX_HEAD_SPEED = 0.3
		private const val LYING_MAX_FOOT_SPEED = 0.5
		private const val SITTING_MIN_TORSO_UP = 0.78
		private const val SITTING_THIGH_VERTICAL_MAX = 0.5
		private const val DOWNWARD_SHIN_MAX = -0.65
		private val SITTING_HEAD_RANGE = 0.42..0.88
		private const val CROUCH_MIN_TORSO_UP = 0.72
		private val CROUCH_HEAD_RANGE = 0.45..0.82
		private const val GAIT_DWELL_SECONDS = 0.4
		private const val HIGH_MOTION_DWELL_SECONDS = 0.2
		private const val POSTURE_DWELL_SECONDS = 0.8
		private const val SWITCH_DWELL_SECONDS = 1.2
	}
}
