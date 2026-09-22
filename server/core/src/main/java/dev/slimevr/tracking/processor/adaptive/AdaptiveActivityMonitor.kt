package dev.slimevr.tracking.processor.adaptive

import dev.slimevr.tracking.processor.adaptive.FootContactState.AIRBORNE
import dev.slimevr.tracking.processor.skeleton.HumanSkeleton
import dev.slimevr.tracking.trackers.Tracker
import io.github.axisangles.ktmath.Vector3

/** Bridges fresh tracker-backed baseline skeleton geometry to the pure activity classifier. */
class AdaptiveActivityMonitor {
	private data class MotionHistory(var timeNanos: Long, var position: Vector3, var velocity: ActivityVector3?)

	private val classifier = AdaptiveActivityClassifier()
	private var headHistory: MotionHistory? = null
	private var leftFootHistory: MotionHistory? = null
	private var rightFootHistory: MotionHistory? = null

	fun reset() {
		classifier.reset()
		headHistory = null
		leftFootHistory = null
		rightFootHistory = null
	}

	fun update(skeleton: HumanSkeleton, nowNanos: Long): AdaptiveActivityEstimate {
		if (skeleton.getPauseTracking() || skeleton.localizer.getEnabled()) {
			reset()
			return classifier.estimate
		}
		val head = skeleton.headTracker
		val headTime = freshTime(head, nowNanos)
		if (head == null || !head.hasPosition || head.isInternal || headTime == null || !finite(head.position)) return invalid("HEAD_UNAVAILABLE")
		val height = skeleton.humanPoseManager.userHeightFromConfig.toDouble()
		val floor = skeleton.legTweaks.floorLevel.toDouble()
		if (!height.isFinite() || height <= 0.5 || !floor.isFinite()) return invalid("CALIBRATED_HEIGHT_UNAVAILABLE")

		val torsoTracker = skeleton.upperChestTracker ?: skeleton.chestTracker ?: skeleton.hipTracker
		val torsoTime = freshTime(torsoTracker, nowNanos) ?: return invalid("TORSO_TRACKER_UNAVAILABLE")
		val legTrackers = listOf(
			skeleton.leftUpperLegTracker,
			skeleton.leftLowerLegTracker,
			skeleton.rightUpperLegTracker,
			skeleton.rightLowerLegTracker,
		)
		val legTimes = legTrackers.map { freshTime(it, nowNanos) ?: return invalid("LEG_TRACKER_UNAVAILABLE") }
		val legsTime = legTimes.minOrNull() ?: return invalid("LEG_TRACKER_UNAVAILABLE")

		val headPosition = head.position
		val leftPosition = skeleton.leftFootBone.getTailPosition()
		val rightPosition = skeleton.rightFootBone.getTailPosition()
		if (!finite(leftPosition) || !finite(rightPosition)) return invalid("FOOT_POSITION_UNAVAILABLE")
		val leftMotionTime = freshTime(skeleton.leftFootTracker ?: skeleton.leftLowerLegTracker, nowNanos) ?: return invalid("LEFT_FOOT_TRACKER_UNAVAILABLE")
		val rightMotionTime = freshTime(skeleton.rightFootTracker ?: skeleton.rightLowerLegTracker, nowNanos) ?: return invalid("RIGHT_FOOT_TRACKER_UNAVAILABLE")
		val motionTime = minOf(headTime, leftMotionTime, rightMotionTime)
		val contactTime = minOf(leftMotionTime, rightMotionTime)
		val leftContact = knownContact(skeleton.legTweaks.adaptiveLeftFoot.snapshot)
		val rightContact = knownContact(skeleton.legTweaks.adaptiveRightFoot.snapshot)
		val pairedContact = if (leftContact != null && rightContact != null) leftContact to rightContact else null

		val headVelocity = velocity(headHistory, nowNanos, headPosition).also { headHistory = it.second }.first
		val leftVelocity = velocity(leftFootHistory, nowNanos, leftPosition).also { leftFootHistory = it.second }.first
		val rightVelocity = velocity(rightFootHistory, nowNanos, rightPosition).also { rightFootHistory = it.second }.first
		val torsoUp = skeleton.upperChestBone.getGlobalRotation().sandwich(Vector3.POS_Y).y.toDouble()
		return classifier.update(
			AdaptiveActivityObservation(
				nowNanos = nowNanos,
				normalizedHeadHeight = ((headPosition.y.toDouble() - floor) / height),
				headTimeNanos = headTime,
				torsoUpY = torsoUp,
				torsoTimeNanos = torsoTime,
				leftThighDirection = direction(skeleton.leftUpperLegBone.getTailPosition() - skeleton.leftUpperLegBone.getPosition()),
				leftShinDirection = direction(skeleton.leftLowerLegBone.getTailPosition() - skeleton.leftLowerLegBone.getPosition()),
				rightThighDirection = direction(skeleton.rightUpperLegBone.getTailPosition() - skeleton.rightUpperLegBone.getPosition()),
				rightShinDirection = direction(skeleton.rightLowerLegBone.getTailPosition() - skeleton.rightLowerLegBone.getPosition()),
				legsTimeNanos = legsTime,
				headVelocityMetersPerSecond = headVelocity,
				leftFootVelocityMetersPerSecond = leftVelocity,
				rightFootVelocityMetersPerSecond = rightVelocity,
				motionTimeNanos = motionTime,
				leftFootContact = pairedContact?.first,
				rightFootContact = pairedContact?.second,
				contactTimeNanos = pairedContact?.let { contactTime },
			),
		)
	}

	private fun invalid(reason: String): AdaptiveActivityEstimate {
		reset()
		return AdaptiveActivityEstimate(AdaptiveActivityState.UNKNOWN, 0.0, reason, 0.0)
	}

	private fun freshTime(tracker: Tracker?, nowNanos: Long): Long? {
		if (tracker == null || tracker.isInternal || !tracker.status.sendData || !tracker.hasRotation) return null
		val rotation = tracker.getRotationWithoutAdaptive()
		val norm = rotation.lenSq()
		if (!rotation.w.isFinite() || !rotation.x.isFinite() || !rotation.y.isFinite() || !rotation.z.isFinite() || !norm.isFinite() || norm !in 0.5f..1.5f) return null
		val time = tracker.lastRotationUpdateNanos ?: return null
		return time.takeIf { nowNanos - it in 0..MAX_SAMPLE_AGE_NANOS }
	}

	private fun knownContact(snapshot: FootContactSnapshot): Double? = if (snapshot.speedMetersPerSecond?.isFinite() != true || snapshot.angularSpeedRadiansPerSecond?.isFinite() != true) {
		null
	} else if (snapshot.state == AIRBORNE) {
		0.0
	} else {
		snapshot.weight.takeIf { it.isFinite() && it in 0f..1f }?.toDouble()
	}

	private fun direction(v: Vector3): ActivityVector3? {
		if (!finite(v)) return null
		val length = v.len().toDouble()
		if (!length.isFinite() || length < MIN_SEGMENT_LENGTH_METERS) return null
		return ActivityVector3(v.x.toDouble() / length, v.y.toDouble() / length, v.z.toDouble() / length)
	}

	private fun velocity(previous: MotionHistory?, time: Long, position: Vector3): Pair<ActivityVector3?, MotionHistory> {
		if (previous == null) return null to MotionHistory(time, position, null)
		if (time == previous.timeNanos) return previous.velocity to previous
		val elapsed = time - previous.timeNanos
		if (elapsed !in 1..MAX_VELOCITY_INTERVAL_NANOS) return null to MotionHistory(time, position, null)
		val seconds = elapsed * 1e-9
		val sample = ActivityVector3(
			(position.x.toDouble() - previous.position.x) / seconds,
			(position.y.toDouble() - previous.position.y) / seconds,
			(position.z.toDouble() - previous.position.z) / seconds,
		)
		return sample.takeIf { it.isFinite() } to MotionHistory(time, position, sample.takeIf { it.isFinite() })
	}

	private fun finite(v: Vector3) = v.x.isFinite() && v.y.isFinite() && v.z.isFinite()

	companion object {
		private const val MAX_SAMPLE_AGE_NANOS = 250_000_000L
		private const val MAX_VELOCITY_INTERVAL_NANOS = 500_000_000L
		private const val MIN_SEGMENT_LENGTH_METERS = 0.05f
	}
}
