package dev.slimevr.tracking.processor.adaptive

import com.fasterxml.jackson.databind.ObjectMapper
import dev.slimevr.tracking.processor.HumanPoseManager
import dev.slimevr.tracking.processor.config.SkeletonConfigToggles
import dev.slimevr.tracking.trackers.TrackerPosition
import dev.slimevr.unit.TestTrackerSet
import io.github.axisangles.ktmath.Quaternion
import io.github.axisangles.ktmath.Vector3
import org.junit.jupiter.api.Test
import kotlin.test.assertEquals
import kotlin.test.assertFailsWith
import kotlin.test.assertTrue

class AdaptivePoseReplayTest {
	@Test
	fun capturesBaselineForDiagnosticsWhenOptimizerIsDisabled() {
		val trackers = TestTrackerSet()
		trackers.head.position = Vector3(0f, 1.7f, 0f)
		val pose = HumanPoseManager(trackers.allL)
		pose.adaptiveTrackingConfig.telemetryEnabled = true
		pose.skeleton.updatePose(0)

		val baseline = pose.adaptivePoseSolver.rawPose
		assertTrue(baseline.isNotEmpty())
		assertEquals(pose.skeleton.headBone.getTailPosition(), baseline["HEAD"])
		assertTrue(pose.adaptivePoseSolver.predictedPose.isEmpty())
		assertEquals(null, pose.adaptivePoseSolver.diagnostic)
	}

	@Test
	fun avoidsBaselinePoseAllocationWhenOptimizerAndDiagnosticsAreDisabled() {
		val trackers = TestTrackerSet()
		trackers.head.position = Vector3(0f, 1.7f, 0f)
		val pose = HumanPoseManager(trackers.allL)
		pose.skeleton.updatePose(0)
		assertTrue(pose.adaptivePoseSolver.rawPose.isEmpty())
		assertTrue(pose.adaptivePoseSolver.predictedPose.isEmpty())
	}

	@Test
	fun invalidPositionalTrackerRotationSkipsOptimizerAndClearsReplayState() {
		val trackers = TestTrackerSet()
		trackers.head.position = Vector3(0f, 1.7f, 0f)
		val controller = TestTrackerSet(positional = true).mkTrack(50, TrackerPosition.LEFT_HAND)
		controller.position = Vector3(-0.4f, 1.15f, 0.25f)
		controller.setRotation(Quaternion(Float.NaN, 0f, 0f, 0f))
		val pose = HumanPoseManager(trackers.allL + controller)
		pose.skeleton.updateToggleState(SkeletonConfigToggles.FORCE_ARMS_FROM_HMD, false)
		pose.adaptiveTrackingConfig.poseOptimizerEnabled = true
		pose.adaptiveTrackingConfig.telemetryEnabled = true

		pose.skeleton.updatePose(0)

		assertEquals(null, pose.adaptivePoseSolver.diagnostic)
		assertEquals(null, pose.adaptivePoseSolver.replayInput)
		assertTrue(pose.adaptivePoseSolver.predictedPose.isEmpty())
		assertTrue(pose.adaptivePoseSolver.rawPose.isEmpty())
	}

	@Test
	fun reversedControllerArmRefreshesTrackerInputsWithoutOptimizerFeedback() {
		val trackers = TestTrackerSet()
		trackers.head.position = Vector3(0f, 1.7f, 0f)
		val controllerSet = TestTrackerSet(positional = true)
		val controller = controllerSet.mkTrack(50, TrackerPosition.LEFT_HAND)
		controller.position = Vector3(-0.4f, 1.15f, 0.25f)
		val upperArm = trackers.mkTrack(51, TrackerPosition.LEFT_UPPER_ARM)
		val pose = HumanPoseManager(trackers.allL + listOf(controller, upperArm))
		pose.skeleton.updateToggleState(SkeletonConfigToggles.FORCE_ARMS_FROM_HMD, false)
		pose.adaptiveTrackingConfig.poseOptimizerEnabled = true

		val changed = Quaternion.rotationAroundYAxis(0.4f) * Quaternion.rotationAroundXAxis(0.35f)
		trackers.chest.setRotation(Quaternion.IDENTITY)
		upperArm.setRotation(Quaternion.IDENTITY)
		pose.skeleton.updatePose(0)
		val firstKeys = pose.adaptivePoseSolver.rawPose.keys.toList()
		val upperArmIndex = firstKeys.indexOf("LEFT_UPPER_ARM")
		val chestIndex = firstKeys.indexOf("CHEST")
		assertTrue(upperArmIndex >= 0 && chestIndex >= 0)
		val firstInput = pose.adaptivePoseSolver.replayInput!!
		val firstArmRotation = firstInput.segments[upperArmIndex].measured
		val firstChestRotation = firstInput.segments[chestIndex].measured
		val firstArmPosition = pose.adaptivePoseSolver.rawPose.getValue("LEFT_UPPER_ARM")
		val firstChestPosition = pose.adaptivePoseSolver.rawPose.getValue("CHEST")

		trackers.chest.setRotation(changed)
		upperArm.setRotation(changed)
		pose.skeleton.updatePose(20_000_000L)
		val secondInput = pose.adaptivePoseSolver.replayInput!!
		val secondArmRotation = secondInput.segments[upperArmIndex].measured
		val secondChestRotation = secondInput.segments[chestIndex].measured
		val secondArmPosition = pose.adaptivePoseSolver.rawPose.getValue("LEFT_UPPER_ARM")
		val secondChestPosition = pose.adaptivePoseSolver.rawPose.getValue("CHEST")
		assertTrue(firstArmRotation.angleToR(secondArmRotation) > 0.2f)
		assertTrue(firstChestRotation.angleToR(secondChestRotation) > 0.2f)
		assertTrue((firstArmPosition - secondArmPosition).len() > 0.01f)
		assertTrue((firstChestPosition - secondChestPosition).len() > 0.01f)

		pose.skeleton.updatePose(40_000_000L)
		val thirdInput = pose.adaptivePoseSolver.replayInput!!
		assertTrue(secondArmRotation.angleToR(thirdInput.segments[upperArmIndex].measured) < 0.001f)
		assertTrue(secondChestRotation.angleToR(thirdInput.segments[chestIndex].measured) < 0.001f)
		assertTrue((secondArmPosition - pose.adaptivePoseSolver.rawPose.getValue("LEFT_UPPER_ARM")).len() < 0.001f)
		assertTrue((secondChestPosition - pose.adaptivePoseSolver.rawPose.getValue("CHEST")).len() < 0.001f)
	}

	@Test
	fun recordedInputReproducesTheRuntimeSolution() {
		val trackers = TestTrackerSet()
		trackers.head.position = Vector3(0f, 1.7f, 0f)
		val controller = TestTrackerSet(positional = true).mkTrack(50, TrackerPosition.LEFT_HAND)
		controller.position = Vector3(-0.4f, 1.15f, 0.25f)
		val pose = HumanPoseManager(trackers.allL + controller)
		pose.skeleton.updateToggleState(SkeletonConfigToggles.FORCE_ARMS_FROM_HMD, false)
		pose.adaptiveTrackingConfig.poseOptimizerEnabled = true
		pose.skeleton.updatePose(0)
		val input = pose.adaptivePoseSolver.replayInput!!
		val mapper = ObjectMapper()
		val decoded = AdaptivePoseReplay.decode(mapper.readTree(mapper.writeValueAsBytes(input.toRecord())))
		assertEquals(input, decoded)
		val solver = PoseOptimizer()
		val first = solver.solve(decoded.segments, decoded.anchors, decoded.joints)
		val second = solver.solve(decoded.segments, decoded.anchors, decoded.joints)
		assertEquals(first, second)
		assertEquals(first.finalError, pose.adaptivePoseSolver.diagnostic!!.finalError)
		pose.adaptivePoseSolver.predictedPose.values.forEachIndexed { index, position ->
			assertTrue((first.tails[index] - position).len() < 0.0001f, "Segment $index differs from runtime")
		}
		assertTrue(first.finalError < first.initialError)
		assertEquals(controller.position, pose.skeleton.leftHandTrackerBone.getPosition())
	}

	@Test
	fun rejectsIncompleteReplayInputs() {
		assertFailsWith<IllegalStateException> { AdaptivePoseReplay.decode(ObjectMapper().readTree("{}")) }
	}

	@Test
	fun reportsFullSkeletonProcessingDistribution() {
		val trackers = TestTrackerSet()
		trackers.head.position = Vector3(0f, 1.7f, 0f)
		val controller = TestTrackerSet(positional = true).mkTrack(50, TrackerPosition.LEFT_HAND)
		controller.position = Vector3(-0.4f, 1.15f, 0.25f)
		val pose = HumanPoseManager(trackers.allL + controller)
		pose.adaptiveTrackingConfig.poseOptimizerEnabled = true
		for (i in 0..100) pose.skeleton.updatePose(i * 10_000_000L)
		val elapsed = (101..500).map { i ->
			pose.skeleton.updatePose(i * 10_000_000L)
			pose.adaptivePoseSolver.diagnostic!!.processingNanos
		}.sorted()
		println("Adaptive pose processing: median=${elapsed[200] / 1e6}ms p95=${elapsed[380] / 1e6}ms p99=${elapsed[396] / 1e6}ms")
		assertTrue(pose.adaptivePoseSolver.diagnostic!!.finalError <= pose.adaptivePoseSolver.diagnostic!!.initialError)
	}
}
