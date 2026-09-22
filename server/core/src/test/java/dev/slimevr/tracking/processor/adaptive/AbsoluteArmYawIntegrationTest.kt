package dev.slimevr.tracking.processor.adaptive

import dev.slimevr.tracking.processor.HumanPoseManager
import dev.slimevr.tracking.processor.config.SkeletonConfigOffsets
import dev.slimevr.tracking.trackers.Tracker
import dev.slimevr.tracking.trackers.TrackerPosition
import dev.slimevr.tracking.trackers.TrackerStatus
import dev.slimevr.tracking.trackers.udp.IMUType
import dev.slimevr.unit.TestTrackerSet
import io.github.axisangles.ktmath.Quaternion
import io.github.axisangles.ktmath.Vector3
import org.junit.jupiter.api.Test
import kotlin.test.assertEquals
import kotlin.test.assertNotEquals
import kotlin.test.assertTrue

class AbsoluteArmYawIntegrationTest {
	private class Fixture(
		private val biasLeftDegrees: Double = 8.0,
		private val biasRightDegrees: Double = 0.0,
		private val verticalArm: Boolean = false,
	) {
		val standard = TestTrackerSet(computed = true, positional = true)
		val leftArm = imu(30, TrackerPosition.LEFT_UPPER_ARM)
		val rightArm = imu(31, TrackerPosition.RIGHT_UPPER_ARM)
		val leftHand = external(32, TrackerPosition.LEFT_HAND)
		val rightHand = external(33, TrackerPosition.RIGHT_HAND)
		val pose = HumanPoseManager(standard.allL + listOf(leftArm, rightArm, leftHand, rightHand))
		private val displacement = if (verticalArm) Vector3(0.4f, 0f, 0f) else Vector3(0.3f, 0.1f, 0.2f)
		private val trueDirection = if (verticalArm) Vector3(0f, -1f, 0f) else Vector3(0.70710677f, 0.70710677f, 0f)
		private val upper = 0.3f
		private val lower = run {
			val d2 = displacement.lenSq().toDouble()
			val dot = displacement.x * trueDirection.x + displacement.y * trueDirection.y + displacement.z * trueDirection.z
			kotlin.math.sqrt(d2 + upper * upper - 2.0 * upper * dot).toFloat()
		}
		private var time = 0L
		private val leftShoulder: Vector3
		private val rightShoulder: Vector3
		private val handOffset: Vector3

		init {
			standard.head.position = Vector3(0f, 1.7f, 0f)
			standard.chest.setRotation(Quaternion.IDENTITY)
			pose.setOffset(SkeletonConfigOffsets.UPPER_ARM, upper)
			pose.setOffset(SkeletonConfigOffsets.LOWER_ARM, lower)
			pose.update()
			pose.adaptiveTrackingConfig.yawCorrectionEnabled = true
			leftShoulder = pose.skeleton.leftUpperArmBone.getPosition()
			rightShoulder = pose.skeleton.rightUpperArmBone.getPosition()
			handOffset = Vector3(0f, pose.getOffset(SkeletonConfigOffsets.HAND_Y), pose.getOffset(SkeletonConfigOffsets.HAND_Z))
		}

		fun step(controllerDelta: Vector3 = Vector3.NULL, refreshPacketTimes: Boolean = true) {
			time += 100_000_000L
			val leftRotation = Quaternion.rotationAroundYAxis(Math.toRadians(biasLeftDegrees).toFloat()) * Quaternion.fromTo(Vector3.NEG_Y, trueDirection)
			val rightRotation = Quaternion.rotationAroundYAxis(Math.toRadians(biasRightDegrees).toFloat()) * Quaternion.fromTo(Vector3.NEG_Y, trueDirection)
			leftArm.setRotation(leftRotation)
			rightArm.setRotation(rightRotation)
			leftArm.setAcceleration(Vector3.NULL, time)
			rightArm.setAcceleration(Vector3.NULL, time)
			setHand(leftHand, leftShoulder + displacement + controllerDelta)
			setHand(rightHand, rightShoulder + displacement)
			if (refreshPacketTimes) {
				(standard.allL + listOf(leftArm, rightArm, leftHand, rightHand)).forEach { it.dataTick(time) }
			}
			pose.skeleton.updatePose(time)
		}

		fun stationary(seconds: Int, refreshPacketTimes: Boolean = true) {
			repeat(seconds * 10) { step(refreshPacketTimes = refreshPacketTimes) }
		}

		private fun setHand(hand: Tracker, wrist: Vector3) {
			hand.setRotation(Quaternion.IDENTITY)
			hand.position = wrist - handOffset
		}

		companion object {
			private fun external(id: Int, position: TrackerPosition): Tracker = Tracker(
				device = null,
				id = id,
				name = "test-$position",
				trackerPosition = position,
				hasPosition = true,
				hasRotation = true,
				isComputed = true,
				isInternal = false,
				usesTimeout = false,
				trackRotDirection = false,
			).apply { status = TrackerStatus.OK }

			private fun imu(id: Int, position: TrackerPosition): Tracker = Tracker(
				device = null,
				id = id,
				name = "imu-$position",
				trackerPosition = position,
				hasRotation = true,
				hasAcceleration = true,
				isInternal = false,
				imuType = IMUType.BNO080,
				usesTimeout = false,
				trackRotDirection = false,
			).apply { status = TrackerStatus.OK }
		}
	}

	@Test
	fun stableControllerReachCorrectsOnlyAdaptiveOutputAfterTwentySeconds() {
		val f = Fixture()
		f.step()
		val raw = f.leftArm.getRawRotation()
		f.stationary(25)
		assertTrue(f.leftArm.adaptiveYawBiasRadians > Math.toRadians(0.2))
		assertTrue(f.leftArm.adaptiveYawBiasRadians < Math.toRadians(8.0))
		assertEquals(raw, f.leftArm.getRawRotation())
		assertNotEquals(f.leftArm.getRawRotation(), f.leftArm.getRotation())
	}

	@Test
	fun equallyBiasedOtherArmCannotWitnessYaw() {
		val f = Fixture(biasLeftDegrees = 8.0, biasRightDegrees = 8.0)
		f.stationary(25)
		assertEquals(0f, f.leftArm.adaptiveYawBiasRadians)
		assertTrue(f.pose.adaptiveEstimator.diagnostics.any { it.residual?.reason == "OPPOSITE_ARM_DISAGREEMENT" })
	}

	@Test
	fun controllerMotionInterruptsEvidenceAndClearsAccumulatedLearning() {
		val f = Fixture()
		f.stationary(23)
		val learned = f.leftArm.adaptiveYawBiasRadians
		assertTrue(learned > 0f)
		repeat(10) { f.step(Vector3(0.02f * (it + 1), 0f, 0f)) }
		assertEquals(learned, f.leftArm.adaptiveYawBiasRadians)
		assertTrue(
			f.pose.adaptiveEstimator.diagnostics
				.filter { it.trackerId == f.leftArm.id }
				.all { it.residual?.eligibleForLearning == false },
		)
	}

	@Test
	fun verticalAmbiguousArmGeometryDoesNotLearn() {
		val f = Fixture(verticalArm = true)
		f.stationary(25)
		assertEquals(0f, f.leftArm.adaptiveYawBiasRadians)
		assertTrue(f.pose.adaptiveEstimator.diagnostics.any { it.residual?.reason == "ARM_YAW_UNOBSERVABLE_OR_AMBIGUOUS" })
	}

	@Test
	fun disablingCorrectionAndResettingYawClearAdaptiveBias() {
		val f = Fixture()
		f.stationary(23)
		assertTrue(f.leftArm.adaptiveYawBiasRadians > 0f)
		f.pose.adaptiveTrackingConfig.yawCorrectionEnabled = false
		f.step()
		assertEquals(0f, f.leftArm.adaptiveYawBiasRadians)
		f.pose.adaptiveTrackingConfig.yawCorrectionEnabled = true
		f.stationary(23)
		assertTrue(f.leftArm.adaptiveYawBiasRadians > 0f)
		f.pose.resetTrackersYaw("test")
		assertEquals(0f, f.leftArm.adaptiveYawBiasRadians)
	}

	@Test
	fun anyArmCalibrationModeSuppressesYawLearning() {
		val f = Fixture()
		f.pose.adaptiveTrackingConfig.armCalibrationMode = "proportions"
		f.stationary(25)
		assertEquals(0f, f.leftArm.adaptiveYawBiasRadians)
	}

	@Test
	fun unknownRequiredAnchorPacketTimeStopsLearningWithConfidenceReason() {
		val f = Fixture()
		f.step(refreshPacketTimes = false)
		val raw = f.leftArm.getRawRotation()
		f.stationary(25, refreshPacketTimes = false)

		assertEquals(0f, f.leftArm.adaptiveYawBiasRadians)
		assertEquals(raw, f.leftArm.getRawRotation())
		val diagnostic = f.pose.adaptiveEstimator.diagnostics.single { it.trackerId == f.leftArm.id }
		assertEquals(false, diagnostic.poseConfidence?.learningEligible)
		assertTrue("ABSOLUTE_ANCHOR_QUALITY_LOW" in diagnostic.poseConfidence!!.reasons)
		assertTrue("TRACKER_CONFIDENCE_LOW" in diagnostic.poseConfidence!!.reasons)
	}

	@Test
	fun staleRequiredAnchorStopsAnAlreadyLearnedCorrection() {
		val f = Fixture()
		f.stationary(25)
		val learned = f.leftArm.adaptiveYawBiasRadians
		val raw = f.leftArm.getRawRotation()
		assertTrue(learned > 0f)

		repeat(3) { f.step(refreshPacketTimes = false) }

		assertEquals(learned, f.leftArm.adaptiveYawBiasRadians)
		assertEquals(raw, f.leftArm.getRawRotation())
		val diagnostic = f.pose.adaptiveEstimator.diagnostics.single { it.trackerId == f.leftArm.id }
		assertEquals("ABSOLUTE_ARM_EVIDENCE_UNAVAILABLE", diagnostic.residual?.reason)
	}
}
