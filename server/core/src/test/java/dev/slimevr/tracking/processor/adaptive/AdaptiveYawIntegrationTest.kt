package dev.slimevr.tracking.processor.adaptive

import dev.slimevr.tracking.processor.HumanPoseManager
import dev.slimevr.tracking.trackers.Tracker
import dev.slimevr.tracking.trackers.TrackerPosition
import dev.slimevr.tracking.trackers.TrackerStatus
import dev.slimevr.tracking.trackers.udp.IMUType
import dev.slimevr.unit.TestTrackerSet
import io.github.axisangles.ktmath.Quaternion
import io.github.axisangles.ktmath.Vector3
import org.junit.jupiter.api.Test
import kotlin.test.assertEquals
import kotlin.test.assertTrue

class AdaptiveYawIntegrationTest {
	private class Fixture {
		val trackers = TestTrackerSet()
		val foot = Tracker(
			null,
			40,
			"foot",
			trackerPosition = TrackerPosition.LEFT_FOOT,
			hasRotation = true,
			hasAcceleration = true,
			imuType = IMUType.BNO085,
			trackRotDirection = false,
		).apply { status = TrackerStatus.OK }
		val pose = HumanPoseManager(trackers.allL + foot)
		init {
			foot.status = TrackerStatus.OK
			trackers.head.position = Vector3(0f, 1.7f, 0f)
			pose.adaptiveTrackingConfig.yawCorrectionEnabled = true
		}
		fun step(index: Int, yawDegrees: Double = index * 0.001) {
			val time = index * 100_000_000L
			foot.setAcceleration(Vector3.NULL, time)
			foot.setRotation(Quaternion.rotationAroundYAxis(Math.toRadians(yawDegrees).toFloat()))
			(trackers.allL + foot).forEach { it.dataTick(time) }
			pose.skeleton.legTweaks.adaptiveLeftFoot.update(Vector3.NULL, foot.getRotationWithoutAdaptive(), 0f, Vector3.NULL, true, time)
			pose.adaptiveEstimator.update(pose.skeleton, time)
		}
	}

	@Test
	fun persistentStationaryDriftCorrectsOutputWithoutChangingInput() {
		val f = Fixture()
		for (i in 0..190) f.step(i)
		assertEquals(0f, f.foot.adaptiveYawBiasRadians)
		for (i in 191..600) f.step(i)
		assertTrue(f.foot.adaptiveYawBiasRadians > Math.toRadians(0.4))
		assertTrue(f.foot.adaptiveYawBiasRadians < Math.toRadians(0.6))
		assertEquals(Quaternion.rotationAroundYAxis(Math.toRadians(0.6).toFloat()), f.foot.getRawRotation())
		assertTrue(f.foot.getRotation().angleToR(Quaternion.IDENTITY) < f.foot.getRotationWithoutAdaptive().angleToR(Quaternion.IDENTITY))
	}

	@Test
	fun intentionalBodyTurnAndDisconnectStopLearning() {
		val f = Fixture()
		for (i in 0..400) f.step(i)
		val bias = f.foot.adaptiveYawBiasRadians
		for (i in 401..700) {
			f.trackers.head.setRotation(Quaternion.rotationAroundYAxis(i * 0.001f))
			f.step(i)
		}
		assertEquals(bias, f.foot.adaptiveYawBiasRadians)
		f.foot.status = TrackerStatus.DISCONNECTED
		for (i in 701..1000) f.step(i)
		assertEquals(bias, f.foot.adaptiveYawBiasRadians)
	}

	@Test
	fun disablingAndExistingResetClearTransientBias() {
		val f = Fixture()
		for (i in 0..400) f.step(i)
		assertTrue(f.foot.adaptiveYawBiasRadians > 0f)
		f.pose.adaptiveTrackingConfig.yawCorrectionEnabled = false
		f.step(401)
		assertEquals(0f, f.foot.adaptiveYawBiasRadians)
		f.pose.adaptiveTrackingConfig.yawCorrectionEnabled = true
		for (i in 402..800) f.step(i)
		assertTrue(f.foot.adaptiveYawBiasRadians > 0f)
		f.pose.resetTrackersYaw("test")
		assertEquals(0f, f.foot.adaptiveYawBiasRadians)
	}
}
