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
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sin
import kotlin.test.assertEquals
import kotlin.test.assertNotEquals
import kotlin.test.assertTrue

class OpportunisticArmCalibrationTest {
	private class Fixture(includeRightArm: Boolean = true) {
		val standard = TestTrackerSet(computed = true, positional = true)
		val leftArm = imu(30, TrackerPosition.LEFT_UPPER_ARM)
		val rightArm = if (includeRightArm) imu(31, TrackerPosition.RIGHT_UPPER_ARM) else null
		val leftHand = external(32, TrackerPosition.LEFT_HAND, positional = true)
		val rightHand = if (includeRightArm) external(33, TrackerPosition.RIGHT_HAND, positional = true) else null
		val pose = HumanPoseManager(standard.allL + listOfNotNull(leftArm, rightArm, leftHand, rightHand))
		var time = 0L
		var expectedLeftRaw = Quaternion.IDENTITY
		private val trueUpper = 0.30f
		private val trueLower = 0.28f

		init {
			standard.head.position = Vector3(0f, 1.7f, 0f)
			standard.chest.setRotation(Quaternion.IDENTITY)
			pose.setOffset(SkeletonConfigOffsets.UPPER_ARM, 0.28f)
			pose.setOffset(SkeletonConfigOffsets.LOWER_ARM, 0.27f)
			pose.update()
			pose.adaptiveTrackingConfig.armCalibrationMode = "proportions"
		}

		fun frame(poseIndex: Int, freshAcceleration: Boolean = true) {
			time += 100_000_000L
			val shoulderLeft = pose.skeleton.leftUpperArmBone.getPosition()
			val upperLeft = upperDirection(poseIndex, 0)
			val deltaLeft = armDisplacement(poseIndex, upperLeft)
			val mountingError = Quaternion.rotationAroundZAxis((3.0 * PI / 180.0).toFloat())
			val leftRotation = Quaternion.fromTo(Vector3.NEG_Y, upperLeft)
			expectedLeftRaw = if (pose.adaptiveTrackingConfig.armCalibrationMode == "mounting") leftRotation * mountingError else leftRotation
			leftArm.setRotation(expectedLeftRaw)
			setHand(leftHand, shoulderLeft + deltaLeft)
			if (freshAcceleration) leftArm.setAcceleration(Vector3.NULL, time)

			if (rightArm != null && rightHand != null) {
				val shoulderRight = pose.skeleton.rightUpperArmBone.getPosition()
				val upperRight = upperDirection(poseIndex, 1)
				val deltaRight = armDisplacement(poseIndex + 2, upperRight)
				rightArm.setRotation(Quaternion.fromTo(Vector3.NEG_Y, upperRight))
				setHand(rightHand, shoulderRight + deltaRight)
				if (freshAcceleration) rightArm.setAcceleration(Vector3.NULL, time)
			}
			(standard.allL + listOfNotNull(leftArm, rightArm, leftHand, rightHand)).forEach { it.dataTick(time) }
			pose.skeleton.updatePose(time)
		}

		fun exercise(poses: Int = 8, holdSeconds: Int = 10) {
			for (poseIndex in 0 until poses) {
				repeat(holdSeconds * 10) { frame(poseIndex) }
			}
		}

		private fun setHand(hand: Tracker, wristTarget: Vector3) {
			val handRotation = Quaternion.IDENTITY
			hand.setRotation(handRotation)
			hand.position = wristTarget -
				handRotation.sandwich(
					Vector3(
						0f,
						pose.getOffset(SkeletonConfigOffsets.HAND_Y),
						pose.getOffset(SkeletonConfigOffsets.HAND_Z),
					),
				)
		}

		private fun armDisplacement(index: Int, upper: Vector3): Vector3 {
			val tangentA = normalized(Vector3(upper.y, -upper.x, 0f))
			val tangentB = normalized(cross(upper, tangentA))
			val theta = (0.28 + (index % 8) * 0.19).toFloat()
			val azimuth = ((index * 137) % 360 * PI / 180.0).toFloat()
			val tangent = tangentA * cos(azimuth) + tangentB * sin(azimuth)
			val forearm = upper * cos(theta) + tangent * sin(theta)
			return upper * trueUpper + forearm * trueLower
		}

		private fun normalized(v: Vector3): Vector3 {
			val length = v.len()
			return v * (1f / length)
		}

		private fun cross(a: Vector3, b: Vector3) = Vector3(
			a.y * b.z - a.z * b.y,
			a.z * b.x - a.x * b.z,
			a.x * b.y - a.y * b.x,
		)

		private fun upperDirection(index: Int, side: Int): Vector3 {
			val alpha = (0.05 + ((index + side * 3) % 5) * 0.045).toFloat()
			val beta = (side * 0.12f)
			return Vector3(sin(alpha) * cos(beta), -cos(alpha), sin(alpha) * sin(beta))
		}

		companion object {
			private fun external(id: Int, position: TrackerPosition, positional: Boolean): Tracker = Tracker(
				device = null,
				id = id,
				name = "test-$position",
				trackerPosition = position,
				hasPosition = positional,
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
	fun computedExternalSixDofTrackersAllowBothArmProportionFit() {
		val fixture = Fixture()
		assertTrue(fixture.standard.head.isComputed)
		assertTrue(!fixture.standard.head.isInternal)
		fixture.exercise()
		assertTrue(fixture.pose.getOffset(SkeletonConfigOffsets.UPPER_ARM) > 0.28f)
		assertTrue(fixture.pose.getOffset(SkeletonConfigOffsets.LOWER_ARM) > 0.27f)
		assertTrue(abs(fixture.pose.getOffset(SkeletonConfigOffsets.UPPER_ARM) - 0.30f) < 0.025f)
		assertTrue(abs(fixture.pose.getOffset(SkeletonConfigOffsets.LOWER_ARM) - 0.28f) < 0.025f)
	}

	@Test
	fun oneArmCannotChangeSharedLengthsAndStaleAccelerationProvidesNoEvidence() {
		val oneArm = Fixture(includeRightArm = false)
		oneArm.exercise()
		assertEquals(0.28f, oneArm.pose.getOffset(SkeletonConfigOffsets.UPPER_ARM), 0.0001f)
		assertEquals(0.27f, oneArm.pose.getOffset(SkeletonConfigOffsets.LOWER_ARM), 0.0001f)

		val stale = Fixture()
		repeat(800) { stale.frame(it / 100, freshAcceleration = false) }
		assertEquals(0.28f, stale.pose.getOffset(SkeletonConfigOffsets.UPPER_ARM), 0.0001f)
		assertEquals(0.27f, stale.pose.getOffset(SkeletonConfigOffsets.LOWER_ARM), 0.0001f)
	}

	@Test
	fun mountingModePreservesRawRotationLimitsCorrectionAndResetClearsIt() {
		val fixture = Fixture()
		fixture.pose.setOffset(SkeletonConfigOffsets.UPPER_ARM, 0.30f)
		fixture.pose.setOffset(SkeletonConfigOffsets.LOWER_ARM, 0.28f)
		fixture.pose.adaptiveTrackingConfig.armCalibrationMode = "mounting"
		val mountError = Quaternion.rotationAroundZAxis((3.0 * PI / 180.0).toFloat())
		val beforeRaw = fixture.leftArm.getRawRotation()
		fixture.leftArm.setRotation(beforeRaw * mountError)
		var previousAdaptive = fixture.leftArm.adaptiveMountingRotation
		repeat(8) { poseIndex ->
			repeat(100) {
				fixture.frame(poseIndex)
				val current = fixture.leftArm.adaptiveMountingRotation
				val stepDegrees = Math.toDegrees(previousAdaptive.angleToR(current).toDouble())
				assertTrue(stepDegrees <= 0.0051, "mounting correction exceeded 0.05 degrees per second")
				previousAdaptive = current
			}
		}
		assertNotEquals(beforeRaw, fixture.leftArm.getRawRotation())
		assertEquals(fixture.expectedLeftRaw, fixture.leftArm.getRawRotation())
		val corrected = fixture.leftArm.adaptiveMountingRotation
		fixture.pose.adaptiveTrackingConfig.armCalibrationMode = "disabled"
		fixture.frame(0)
		assertEquals(Quaternion.IDENTITY, fixture.leftArm.adaptiveMountingRotation)
		assertNotEquals(Quaternion.IDENTITY, corrected)
	}
}
