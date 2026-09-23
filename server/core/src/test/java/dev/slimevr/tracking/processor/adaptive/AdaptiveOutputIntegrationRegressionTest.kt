package dev.slimevr.tracking.processor.adaptive

import com.fasterxml.jackson.databind.ObjectMapper
import dev.slimevr.tracking.processor.HumanPoseManager
import dev.slimevr.tracking.processor.config.SkeletonConfigToggles
import dev.slimevr.tracking.trackers.Tracker
import dev.slimevr.tracking.trackers.TrackerPosition
import dev.slimevr.tracking.trackers.TrackerStatus
import dev.slimevr.tracking.trackers.udp.IMUType
import dev.slimevr.unit.TestTrackerSet
import io.github.axisangles.ktmath.Quaternion
import io.github.axisangles.ktmath.Vector3
import org.junit.jupiter.api.Test
import kotlin.test.assertEquals
import kotlin.test.assertFalse
import kotlin.test.assertNotNull
import kotlin.test.assertSame
import kotlin.test.assertTrue

class AdaptiveOutputIntegrationRegressionTest {
	@Test
	fun exportedElbowsFollowSolvedUpperArmsInBothTopologies() {
		for (fromHead in listOf(true, false)) {
			val trackers = TestTrackerSet()
			trackers.head.position = Vector3(0f, 1.7f, 0f)
			val positional = TestTrackerSet(positional = true)
			val leftHand = positional.mkTrack(50, TrackerPosition.LEFT_HAND).apply { position = Vector3(-0.45f, 1.2f, 0.2f) }
			val rightHand = positional.mkTrack(51, TrackerPosition.RIGHT_HAND).apply { position = Vector3(0.45f, 1.2f, 0.2f) }
			val leftArm = trackers.mkTrack(52, TrackerPosition.LEFT_UPPER_ARM)
			val rightArm = trackers.mkTrack(53, TrackerPosition.RIGHT_UPPER_ARM)
			val inputs = trackers.allL + listOf(leftHand, rightHand, leftArm, rightArm)
			val pose = HumanPoseManager(inputs)
			pose.skeleton.updateToggleState(SkeletonConfigToggles.FORCE_ARMS_FROM_HMD, fromHead)
			pose.adaptiveTrackingConfig.poseOptimizerEnabled = true
			for (i in 0..5) {
				val now = i * 20_000_000L
				inputs.forEach { it.dataTick(now) }
				pose.skeleton.updatePose(now)
			}
			assertNotNull(pose.adaptivePoseSolver.diagnostic)
			val skeleton = pose.skeleton
			listOf(
				skeleton.leftUpperArmBone to skeleton.computedLeftElbowTracker!!,
				skeleton.rightUpperArmBone to skeleton.computedRightElbowTracker!!,
			).forEach { (bone, output) ->
				val expected = bone.getGlobalRotation() * bone.rotationOffset.inv()
				assertTrue(expected.angleToR(output.getRawRotation()) < 0.002f)
			}
			assertTrue(pose.adaptivePoseSolver.replayInput!!.segments.any { it.initialRotation != null && !it.fixed })
			pose.adaptivePoseSolver.reset()
			assertTrue(pose.adaptivePoseSolver.footAnchorsApplied.isEmpty())
			assertEquals(null, pose.adaptivePoseSolver.lastSolvedTimestampNanos)
		}
	}

	@Test
	fun currentLiveQualityCapsCalibrationAndUsesOneSnapshotPerTick() {
		val trackers = TestTrackerSet()
		val pose = HumanPoseManager(trackers.allL)
		trackers.head.position = Vector3(0f, 1.7f, 0f)
		var frame: AdaptiveTelemetryFrame? = null
		for (i in 0..100) {
			val now = i * 100_000_000L
			trackers.allL.forEach { it.dataTick(now) }
			frame = pose.adaptiveMeasurementQuality.observe(pose.skeleton, now)
			assertSame(frame, pose.adaptiveMeasurementQuality.observe(pose.skeleton, now))
		}
		val bad = frame!!.copy(
			samples = frame!!.samples.map { sample ->
				if (sample.id == trackers.head.id) sample.copy(confidence = TrackerConfidence(0.1f, listOf("TEST_HEALTH_CAP"), 10.0)) else sample
			},
		)
		val confidence = CalibrationPoseConfidence.estimate(
			trackers.allL,
			listOf(trackers.head),
			listOf(0.0, 0.0),
			listOf(0.0),
			0f,
			bad.timestampNanos,
			qualityFrame = bad,
		)
		assertFalse(confidence.learningEligible)
		pose.resetTrackersYaw("test")
		assertEquals(null, pose.adaptiveMeasurementQuality.latestFrame)
	}

	@Test
	fun replayPreservesSeedSeparateFromMeasurementAndExplicitNull() {
		val measured = Quaternion.IDENTITY
		val seed = Quaternion.rotationAroundYAxis(0.25f)
		for (initial in listOf(seed, null)) {
			val input = PoseSolverInput(
				listOf(
					PoseSegment(
						-1,
						0.4f,
						Vector3.NULL,
						measured,
						0.9f,
						previous = seed,
						initialRotation = initial,
					),
				),
				emptyList(),
				emptyList(),
			)
			val tree = ObjectMapper().valueToTree<com.fasterxml.jackson.databind.JsonNode>(input.toRecord())
			val decoded = AdaptivePoseReplay.decode(tree).segments.single()
			assertEquals(measured, decoded.measured)
			assertEquals(initial, decoded.initialRotation)
			assertEquals(seed, decoded.previous)
		}
	}

	@Test
	fun unhealthyArmRotationUsesShortRecoveryInsteadOfSeedingTheSolver() {
		val trackers = TestTrackerSet()
		trackers.head.position = Vector3(0f, 1.7f, 0f)
		val arm = Tracker(null, 60, "left-arm", trackerPosition = TrackerPosition.LEFT_UPPER_ARM, hasRotation = true, hasAcceleration = true, imuType = IMUType.BNO080, trackRotDirection = false).apply { status = TrackerStatus.OK }
		val inputs = trackers.allL + arm
		val pose = HumanPoseManager(inputs)
		pose.adaptiveTrackingConfig.poseOptimizerEnabled = true
		for (index in 0..100) {
			val now = index * 20_000_000L
			arm.setAcceleration(Vector3.NULL, now)
			inputs.forEach { it.dataTick(now) }
			pose.skeleton.updatePose(now)
		}
		val segment = pose.adaptivePoseSolver.rawPose.keys.indexOf("LEFT_UPPER_ARM")
		assertTrue(segment >= 0)
		val lastGood = pose.adaptivePoseSolver.replayInput!!.segments[segment].measured
		val now = 2_020_000_000L
		arm.setRotation(Quaternion.rotationAroundYAxis(1.5f))
		arm.setAcceleration(Vector3(100f, 0f, 0f), now)
		inputs.forEach { it.dataTick(now) }
		pose.skeleton.updatePose(now)
		val health = pose.adaptiveMeasurementQuality.latestFrame!!.samples.first { it.id == arm.id }.health!!
		assertTrue("IMPOSSIBLE_ACCELERATION" in health.reasons)
		val measured = pose.adaptivePoseSolver.replayInput!!.segments[segment].measured
		assertTrue(lastGood.angleToR(measured) < 0.02f)
		assertTrue(arm.getRotation().angleToR(measured) > 0.3f)
		assertTrue(arm.id in pose.adaptivePoseSolver.diagnostic!!.recoveryTrackerIds)
	}
}
