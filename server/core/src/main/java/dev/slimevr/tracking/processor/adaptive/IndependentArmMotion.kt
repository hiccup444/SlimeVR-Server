package dev.slimevr.tracking.processor.adaptive

import dev.slimevr.tracking.processor.config.SkeletonConfigOffsets
import dev.slimevr.tracking.processor.skeleton.HumanSkeleton
import dev.slimevr.tracking.trackers.Tracker
import io.github.axisangles.ktmath.Vector3

/** Supplies target-specific motion evidence only for the supported, near-extended upper arms. */
internal class IndependentArmMotion {
	private val witnesses = mutableMapOf<Int, ReachMotionWitness>()

	fun reset() = witnesses.clear()

	fun observe(s: HumanSkeleton, frame: AdaptiveTelemetryFrame): Map<Int, TrackerMotionContext> {
		val now = frame.timestampNanos
		val samples = frame.samples.associateBy { it.id }
		val pose = s.humanPoseManager
		val upper = pose.getOffset(SkeletonConfigOffsets.UPPER_ARM)
		val lower = pose.getOffset(SkeletonConfigOffsets.LOWER_ARM)
		val offset = Vector3(0f, pose.getOffset(SkeletonConfigOffsets.HAND_Y), pose.getOffset(SkeletonConfigOffsets.HAND_Z))
		fun fresh(t: Tracker?): Boolean {
			val sample = samples[t?.id] ?: return false
			return sample.status.sendData &&
				(sample.confidence?.score ?: 0f) >= 0.85f &&
				sample.packetAgeNanos?.let { it in 0..100_000_000L } == true &&
				sample.adjustedRotation != null
		}
		val anchor = s.headTracker
		val chest = s.upperChestTracker ?: s.chestTracker
		val result = mutableMapOf<Int, TrackerMotionContext>()
		fun arm(target: Tracker?, hand: Tracker?, shoulder: Vector3) {
			if (target == null || !target.isImu()) return
			val raw = samples[target.id]?.rawRotation
			val trusted = raw != null &&
				fresh(anchor) &&
				fresh(chest) &&
				fresh(hand) &&
				fresh(target) &&
				anchor?.hasPosition == true &&
				!anchor.isInternal &&
				hand?.hasPosition == true &&
				!hand.isInternal
			val witness = witnesses.getOrPut(target.id) { ReachMotionWitness() }
			if (!trusted || hand == null || raw == null) {
				witness.reset()
				return
			}
			val wrist = hand.position + hand.getRotationWithoutAdaptive().sandwich(offset)
			val score = witness.observe(wrist - shoulder, raw, upper, lower, now, true)
			result[target.id] = TrackerMotionContext(score, now)
		}
		arm(s.leftUpperArmTracker, s.leftHandTracker, s.leftUpperArmBone.getPosition())
		arm(s.rightUpperArmTracker, s.rightHandTracker, s.rightUpperArmBone.getPosition())
		witnesses.keys.retainAll(listOfNotNull(s.leftUpperArmTracker?.id, s.rightUpperArmTracker?.id).toSet())
		return result
	}
}
