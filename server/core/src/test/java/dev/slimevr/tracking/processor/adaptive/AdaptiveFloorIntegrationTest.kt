package dev.slimevr.tracking.processor.adaptive

import dev.slimevr.tracking.processor.HumanPoseManager
import dev.slimevr.tracking.processor.skeleton.LegTweaks
import dev.slimevr.tracking.trackers.Tracker
import dev.slimevr.tracking.trackers.TrackerPosition
import dev.slimevr.tracking.trackers.TrackerStatus
import dev.slimevr.unit.TestTrackerSet
import io.github.axisangles.ktmath.Quaternion
import io.github.axisangles.ktmath.Vector3
import org.junit.jupiter.api.Test
import kotlin.test.assertEquals
import kotlin.test.assertNotNull
import kotlin.test.assertTrue

class AdaptiveFloorIntegrationTest {
	private class Fixture {
		val trackers = TestTrackerSet(computed = false, positional = true)
		val leftFoot = footTracker(20, TrackerPosition.LEFT_FOOT)
		val rightFoot = footTracker(21, TrackerPosition.RIGHT_FOOT)
		val pose = HumanPoseManager(trackers.allL + listOf(leftFoot, rightFoot))
		val legs: LegTweaks = pose.skeleton.legTweaks
		var now = 0L

		init {
			trackers.head.position = Vector3(0f, pose.userHeightFromConfig, 0f)
			trackers.chest.position = Vector3(0f, 1.25f, 0f)
			trackers.hip.position = Vector3(0f, 0.9f, 0f)
			trackers.leftThigh.position = Vector3(-0.1f, 0.65f, 0f)
			trackers.rightThigh.position = Vector3(0.1f, 0.65f, 0f)
			trackers.leftCalf.position = Vector3(-0.1f, 0.3f, 0f)
			trackers.rightCalf.position = Vector3(0.1f, 0.3f, 0f)
			leftFoot.position = Vector3(-0.1f, 0f, 0f)
			rightFoot.position = Vector3(0.1f, 0f, 0f)
			trackers.allL.forEach { it.setRotation(Quaternion.IDENTITY) }
			leftFoot.setRotation(Quaternion.IDENTITY)
			rightFoot.setRotation(Quaternion.IDENTITY)
			pose.adaptiveTrackingConfig.floorEstimationEnabled = true
			pose.update()
			legs.setFloorClipEnabled(true)
			legs.resetFloorLevel()
			frame(leftY = 0f, rightY = 0f)
		}

		fun frame(leftY: Float = 0.02f, rightY: Float = 0.02f, hmdHeight: Float = pose.userHeightFromConfig) {
			now += 100_000_000L
			// Foot tracker positions are not skeleton anchors; move the HMD and bend a leg.
			trackers.head.position = Vector3(0f, hmdHeight + leftY, 0f)
			trackers.rightThigh.setRotation(Quaternion.rotationAroundZAxis(if (rightY - leftY > 0.04f) 0.8f else 0f))
			leftFoot.position = Vector3(-0.1f, leftY, 0f)
			rightFoot.position = Vector3(0.1f, rightY, 0f)
			leftFoot.setAcceleration(Vector3.NULL, now)
			rightFoot.setAcceleration(Vector3.NULL, now)
			pose.skeleton.updatePose(now)
		}

		fun hold(seconds: Int, leftY: Float = 0.02f, rightY: Float = 0.02f, hmdHeight: Float = pose.userHeightFromConfig) {
			repeat(seconds * 10) { frame(leftY, rightY, hmdHeight) }
		}

		private fun footTracker(id: Int, position: TrackerPosition) = Tracker(
			device = null,
			id = id,
			name = "test-$position",
			trackerPosition = position,
			hasPosition = true,
			hasRotation = true,
			hasAcceleration = true,
			usesTimeout = false,
			trackRotDirection = false,
		).apply { status = TrackerStatus.OK }
	}

	@Test
	fun pairedStationaryFeetLearnAgainstCalibratedFloorWithoutChangingItsBaseline() {
		val fixture = Fixture()
		val calibratedFloor = fixture.legs.floorLevel
		fixture.hold(35)
		val diagnostic = assertNotNull(fixture.legs.adaptiveFloorDiagnostic)
		assertTrue(diagnostic.trustedSeconds >= 5.0, diagnostic.reason)
		assertEquals(calibratedFloor.toDouble(), diagnostic.calibratedHeightMeters, 1e-6)
		assertTrue(diagnostic.estimatedHeightMeters > diagnostic.calibratedHeightMeters)
		assertTrue(diagnostic.estimatedHeightMeters - diagnostic.calibratedHeightMeters <= 0.03)
		assertEquals(calibratedFloor, fixture.legs.floorLevel, 1e-6f)
	}

	@Test
	fun raisedFootCrouchAndDisabledModeStopLearningAndPreserveCalibration() {
		val fixture = Fixture()
		val calibratedFloor = fixture.legs.floorLevel
		fixture.hold(8)
		fixture.hold(2, leftY = 0.02f, rightY = 0.10f)
		val afterRaised = assertNotNull(fixture.legs.adaptiveFloorDiagnostic)
		assertTrue(afterRaised.reason != "LEARNING")
		assertEquals(0.0, afterRaised.trustedSeconds)

		fixture.hold(2, hmdHeight = fixture.pose.userHeightFromConfig * 0.75f)
		assertEquals("NOT_UPRIGHT", assertNotNull(fixture.legs.adaptiveFloorDiagnostic).reason)
		fixture.pose.adaptiveTrackingConfig.floorEstimationEnabled = false
		fixture.frame()
		assertEquals(calibratedFloor, fixture.legs.floorLevel, 1e-6f)
		assertEquals(calibratedFloor.toDouble(), assertNotNull(fixture.legs.adaptiveFloorDiagnostic).estimatedHeightMeters, 1e-6)
	}

	@Test
	fun calibrationResetClearsLearnedOffsetAndLocalizerBypassesIt() {
		val fixture = Fixture()
		fixture.hold(35)
		assertTrue(assertNotNull(fixture.legs.adaptiveFloorDiagnostic).estimatedHeightMeters > fixture.legs.floorLevel)
		fixture.legs.resetFloorLevel()
		assertEquals(fixture.legs.floorLevel.toDouble(), assertNotNull(fixture.legs.adaptiveFloorDiagnostic).estimatedHeightMeters, 1e-6)
		fixture.hold(35)
		fixture.legs.setLocalizerMode(true)
		assertEquals(0f, fixture.legs.floorLevel, 1e-6f)
		assertEquals(0.0, assertNotNull(fixture.legs.adaptiveFloorDiagnostic).estimatedHeightMeters, 1e-6)
	}
}
