package dev.slimevr.tracking.processor.adaptive

import dev.slimevr.tracking.processor.HumanPoseManager
import dev.slimevr.unit.TestTrackerSet
import io.github.axisangles.ktmath.Quaternion
import io.github.axisangles.ktmath.Vector3
import org.junit.jupiter.api.Test
import kotlin.test.assertEquals

class AdaptiveActivityMonitorTest {
	@Test
	fun diagnosticPipelineRecognizesSeatedGeometryWithoutContactOrOptimizer() {
		val trackers = TestTrackerSet()
		val pose = HumanPoseManager(trackers.allL)
		trackers.head.position = Vector3(0f, pose.userHeightFromConfig, 0f)
		pose.skeleton.updatePose(0L)
		val seatedHeight = pose.skeleton.legTweaks.floorLevel + pose.userHeightFromConfig * 0.7f
		pose.adaptiveTrackingConfig.liveDiagnosticsEnabled = true
		for (i in 1..20) {
			val now = i * 100_000_000L
			trackers.head.position = Vector3(0f, seatedHeight, 0f)
			trackers.leftThigh.setRotation(Quaternion.rotationAroundXAxis((-Math.PI / 2).toFloat()))
			trackers.rightThigh.setRotation(Quaternion.rotationAroundXAxis((-Math.PI / 2).toFloat()))
			trackers.allL.forEach { it.dataTick(now) }
			pose.skeleton.updatePose(now)
		}
		val s = pose.skeleton
		val evidence = "${pose.adaptivePoseSolver.activity} height=${(trackers.head.position.y - s.legTweaks.floorLevel) / pose.userHeightFromConfig} thigh=${s.leftUpperLegBone.getGlobalRotation().sandwich(Vector3.NEG_Y)} shin=${s.leftLowerLegBone.getGlobalRotation().sandwich(Vector3.NEG_Y)} torso=${s.upperChestBone.getGlobalRotation().sandwich(Vector3.POS_Y)}"
		assertEquals(AdaptiveActivityState.SITTING, pose.adaptivePoseSolver.activity?.state, evidence)
		assertEquals(null, pose.adaptivePoseSolver.diagnostic)
	}

	@Test
	fun missingRealTrackerTimestampsFailClosed() {
		val trackers = TestTrackerSet(computed = true, positional = true)
		val pose = HumanPoseManager(trackers.allL)
		val monitor = AdaptiveActivityMonitor()

		val result = monitor.update(pose.skeleton, 1_000_000_000L)

		assertEquals(AdaptiveActivityState.UNKNOWN, result.state)
		assertEquals("HEAD_UNAVAILABLE", result.reason)
	}

	@Test
	fun resetClearsPreviousClassifierAndVelocityState() {
		val trackers = TestTrackerSet()
		val pose = HumanPoseManager(trackers.allL)
		val monitor = AdaptiveActivityMonitor()
		monitor.update(pose.skeleton, 1_000_000_000L)

		monitor.reset()

		assertEquals(AdaptiveActivityState.UNKNOWN, monitor.update(pose.skeleton, 1_100_000_000L).state)
	}
}
