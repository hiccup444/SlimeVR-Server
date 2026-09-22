package dev.slimevr.tracking.processor.adaptive

import dev.slimevr.tracking.processor.HumanPoseManager
import dev.slimevr.tracking.processor.config.SkeletonConfigToggles
import dev.slimevr.tracking.trackers.Tracker
import dev.slimevr.tracking.trackers.TrackerPosition
import dev.slimevr.tracking.trackers.TrackerStatus
import dev.slimevr.tracking.trackers.udp.IMUType
import io.github.axisangles.ktmath.Quaternion
import io.github.axisangles.ktmath.Vector3
import org.junit.jupiter.api.Test
import kotlin.test.assertEquals
import kotlin.test.assertTrue

class FullBodyAdaptiveIntegrationTest {
	@Test
	fun tenImusAndThreeExternalAnchorsPreserveInputsAndGeometryInBothArmTopologies() {
		val roles = listOf(
			TrackerPosition.HEAD, TrackerPosition.LEFT_HAND, TrackerPosition.RIGHT_HAND,
			TrackerPosition.CHEST, TrackerPosition.HIP, TrackerPosition.LEFT_UPPER_ARM, TrackerPosition.RIGHT_UPPER_ARM,
			TrackerPosition.LEFT_UPPER_LEG, TrackerPosition.RIGHT_UPPER_LEG, TrackerPosition.LEFT_LOWER_LEG, TrackerPosition.RIGHT_LOWER_LEG,
			TrackerPosition.LEFT_FOOT, TrackerPosition.RIGHT_FOOT,
		)
		val trackers = roles.mapIndexed { index, role ->
			Tracker(null, index, role.name, trackerPosition = role, hasRotation = true, hasPosition = index < 3, hasAcceleration = index >= 3, imuType = if (index >= 3) IMUType.BNO085 else null, isComputed = index < 3, isHmd = index == 0, allowReset = false, allowMounting = false, trackRotDirection = false).apply { status = TrackerStatus.OK }
		}
		val pose = HumanPoseManager(trackers)
		pose.adaptiveTrackingConfig.poseOptimizerEnabled = true
		pose.adaptiveTrackingConfig.footAnchoringEnabled = true
		pose.adaptiveTrackingConfig.yawCorrectionEnabled = true
		trackers[0].position = Vector3(0f, 1.7f, 0f)
		trackers[1].position = Vector3(-0.45f, 1.25f, -0.2f)
		trackers[2].position = Vector3(0.45f, 1.25f, -0.2f)
		val lengths = pose.skeleton.allHumanBones.map { it.length }
		for (fromHead in listOf(false, true)) {
			pose.skeleton.updateToggleState(SkeletonConfigToggles.FORCE_ARMS_FROM_HMD, fromHead)
			for (frame in 0..60) {
				val now = (frame + if (fromHead) 100 else 0) * 20_000_000L
				trackers.drop(3).forEachIndexed { index, tracker ->
					tracker.setRotation(Quaternion.rotationAroundYAxis(frame * 0.004f) * Quaternion.rotationAroundXAxis(kotlin.math.sin(frame * 0.2f + index) * 0.15f))
					tracker.setAcceleration(Vector3.NULL, now)
				}
				val raw = trackers.map { it.getRawRotation() }
				trackers.forEach { it.dataTick(now) }
				pose.skeleton.updatePose(now)
				assertEquals(raw, trackers.map { it.getRawRotation() })
				assertEquals(trackers[0].position, pose.skeleton.headBone.getPosition())
				assertEquals(lengths, pose.skeleton.allHumanBones.map { it.length })
				val diagnostic = pose.adaptivePoseSolver.diagnostic!!
				assertTrue(diagnostic.finalError <= diagnostic.initialError + 1e-6f)
				assertTrue(pose.adaptivePoseSolver.predictedPose.values.all { it.x.isFinite() && it.y.isFinite() && it.z.isFinite() })
				assertEquals(trackers.drop(3).map { it.id }.toSet(), pose.adaptivePoseSolver.trackerPredictions.keys)
				assertTrue(pose.adaptivePoseSolver.trackerPredictions.values.all { it.residual.magnitudeRadians.isFinite() && !it.residual.independentlyConstrained })
				if (!fromHead) {
					assertEquals(trackers[1].position, pose.skeleton.leftHandTrackerBone.getPosition())
					assertEquals(trackers[2].position, pose.skeleton.rightHandTrackerBone.getPosition())
				}
			}
		}
	}
}
