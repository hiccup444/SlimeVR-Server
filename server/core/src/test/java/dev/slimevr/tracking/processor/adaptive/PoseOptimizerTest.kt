package dev.slimevr.tracking.processor.adaptive

import dev.slimevr.tracking.processor.HumanPoseManager
import dev.slimevr.unit.TestTrackerSet
import io.github.axisangles.ktmath.Quaternion
import io.github.axisangles.ktmath.Vector3
import org.junit.jupiter.api.Test
import kotlin.test.assertEquals
import kotlin.test.assertFailsWith
import kotlin.test.assertTrue

class PoseOptimizerTest {
	@Test
	fun swingAndTwistRangesHaveNoPreferenceInsideTheirBounds() {
		val measured = Quaternion.rotationAroundXAxis(0.3f) * Quaternion.rotationAroundYAxis(0.4f)
		val result = PoseOptimizer().solve(
			listOf(PoseSegment(-1, 0.3f, Vector3.NULL, Quaternion.IDENTITY, 1f, true), PoseSegment(0, 0.3f, Vector3.NULL, measured, 0.8f)),
			emptyList(),
			listOf(SoftJoint(0, 1, 10f, maxSwingRadians = 0.6f, maxTwistRadians = 0.6f)),
		)
		assertEquals(0f, result.initialError, 1e-6f)
		assertTrue(result.rotations[1].angleToR(measured) < 0.001f)
	}

	@Test
	fun excessiveJointTwistHasFiniteSoftCostWithoutClampingThePose() {
		val measured = Quaternion.rotationAroundYAxis(2f)
		val result = PoseOptimizer().solve(
			listOf(PoseSegment(-1, 0.3f, Vector3.NULL, Quaternion.IDENTITY, 1f, true), PoseSegment(0, 0.3f, Vector3.NULL, measured, 0.8f)),
			emptyList(),
			listOf(SoftJoint(0, 1, 10f, maxSwingRadians = 1f, maxTwistRadians = 1f)),
		)
		assertTrue(result.initialError.isFinite() && result.initialError > 0f)
		assertTrue(result.finalError < result.initialError)
		assertTrue(result.rotations[1].angleToR(Quaternion.IDENTITY) > 1f)
		assertEquals(0.3f, (result.tails[1] - result.tails[0]).len(), 1e-6f)
	}

	@Test
	fun jointLimitsRoundTripThroughReplayAndRejectInvalidRanges() {
		val segments = listOf(PoseSegment(-1, 0.3f, Vector3.NULL, Quaternion.IDENTITY, 1f))
		val input = PoseSolverInput(segments, emptyList(), listOf(SoftJoint(0, 0, 0.1f, maxSwingRadians = 1.2f, maxTwistRadians = 0.9f)))
		val mapper = com.fasterxml.jackson.databind.ObjectMapper()
		assertEquals(input, AdaptivePoseReplay.decode(mapper.readTree(mapper.writeValueAsString(input.toRecord()))))
		assertFailsWith<IllegalArgumentException> {
			PoseOptimizer().solve(segments, emptyList(), listOf(SoftJoint(0, 0, 0.1f, maxSwingRadians = Float.NaN)))
		}
	}

	@Test
	fun softElbowPriorPreservesDeepFlexionAndHandlesReversedBoneOffsets() {
		val bend = Quaternion.rotationAroundXAxis(Math.toRadians(155.0).toFloat())
		val reverse = Quaternion.rotationAroundZAxis(Math.PI.toFloat())
		fun solve(reversed: Boolean): OptimizedPose {
			val lower = if (reversed) bend * reverse else bend
			return PoseOptimizer().solve(
				listOf(PoseSegment(-1, 0.3f, Vector3.NULL, Quaternion.IDENTITY, 1f), PoseSegment(0, 0.3f, Vector3.NULL, lower, 1f)),
				emptyList(),
				listOf(SoftJoint(0, 1, 0.02f, true, secondOffset = if (reversed) reverse.inv() else Quaternion.IDENTITY)),
			)
		}
		val forward = solve(false)
		val reversed = solve(true)
		assertTrue(forward.rotations[1].angleToR(bend) < 0.001f)
		assertTrue((reversed.rotations[1] * reverse.inv()).angleToR(bend) < 0.001f)
		assertEquals(forward.initialError, reversed.initialError, 1e-6f)
	}

	@Test
	fun improvesReachWhilePreservingLengthsAndRoot() {
		val root = Vector3(0f, 1f, 0f)
		val segments = listOf(
			PoseSegment(-1, 0.4f, root, Quaternion.IDENTITY, 0.8f),
			PoseSegment(0, 0.4f, Vector3.NULL, Quaternion.IDENTITY, 0.8f),
		)
		val target = Vector3(0.15f, 0.23f, 0f)
		val result = PoseOptimizer().solve(segments, listOf(PoseAnchor(1, target, 40f)))
		assertTrue(result.finalError < result.initialError * 0.7f)
		assertEquals(0.4f, (result.tails[0] - root).len(), 1e-6f)
		assertEquals(0.4f, (result.tails[1] - result.tails[0]).len(), 1e-6f)
	}

	@Test
	fun anchoredOrientationsRemainExactAndLowConfidenceYieldsMore() {
		fun solve(confidence: Float) = PoseOptimizer().solve(
			listOf(
				PoseSegment(-1, 0.4f, Vector3.NULL, Quaternion.IDENTITY, 1f, true),
				PoseSegment(0, 0.4f, Vector3.NULL, Quaternion.IDENTITY, confidence),
			),
			listOf(PoseAnchor(1, Vector3(0.06f, -0.8f, 0f), 10f)),
		)
		val low = solve(0f)
		val high = solve(1f)
		assertEquals(Quaternion.IDENTITY, low.rotations[0])
		assertTrue(low.tails[1].x > high.tails[1].x)
	}

	@Test
	fun invalidTopologyAndMeasurementsAreRejected() {
		assertFailsWith<IllegalArgumentException> {
			PoseOptimizer().solve(listOf(PoseSegment(0, 1f, Vector3.NULL, Quaternion.IDENTITY, 1f)), emptyList())
		}
		assertFailsWith<IllegalArgumentException> {
			PoseOptimizer().solve(listOf(PoseSegment(-1, Float.NaN, Vector3.NULL, Quaternion.IDENTITY, 1f)), emptyList())
		}
	}

	@Test
	fun realSkeletonPreservesHeadAndLengthsAndClearsHistoryOnReset() {
		val trackers = TestTrackerSet()
		trackers.head.position = Vector3(0f, 1.7f, 0f)
		val pose = HumanPoseManager(trackers.allL)
		pose.update()
		val head = pose.skeleton.headBone.getPosition()
		val lengths = pose.skeleton.allHumanBones.map { it.length }
		pose.adaptiveTrackingConfig.poseOptimizerEnabled = true
		pose.update()
		assertEquals(head, pose.skeleton.headBone.getPosition())
		assertEquals(lengths, pose.skeleton.allHumanBones.map { it.length })
		assertTrue(pose.adaptivePoseSolver.diagnostic!!.finalError <= pose.adaptivePoseSolver.diagnostic!!.initialError)
		pose.resetTrackersYaw("test")
		assertEquals(null, pose.adaptivePoseSolver.diagnostic)
	}
}
