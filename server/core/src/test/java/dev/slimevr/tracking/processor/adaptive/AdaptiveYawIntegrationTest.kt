package dev.slimevr.tracking.processor.adaptive

import dev.slimevr.tracking.processor.HumanPoseManager
import dev.slimevr.tracking.trackers.Device
import dev.slimevr.tracking.trackers.DeviceOrigin
import dev.slimevr.tracking.trackers.Tracker
import dev.slimevr.tracking.trackers.TrackerPosition
import dev.slimevr.tracking.trackers.TrackerStatus
import dev.slimevr.tracking.trackers.udp.IMUType
import dev.slimevr.unit.TestTrackerSet
import io.github.axisangles.ktmath.Quaternion
import io.github.axisangles.ktmath.Vector3
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.io.TempDir
import java.nio.file.Path
import kotlin.test.assertEquals
import kotlin.test.assertTrue

class AdaptiveYawIntegrationTest {
	private class Fixture(directory: Path? = null) {
		val trackers = TestTrackerSet()
		private val device = directory?.let {
			object : Device(DeviceOrigin.UDP) {
				override val hardwareIdentifier = "adaptive-foot-test"
			}
		}
		val foot = Tracker(
			device,
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
			if (directory != null) {
				pose.adaptiveTrackingConfig.temperatureLearningEnabled = true
				pose.adaptiveTrackingConfig.calibrationDirectory = directory.toString()
			}
		}
		fun step(index: Int, yawDegrees: Double = index * 0.001, contactTrusted: Boolean = true) {
			val time = index * 100_000_000L
			foot.setAcceleration(Vector3.NULL, time)
			if (device != null) foot.setTemperature(20.5f, time)
			foot.setRotation(Quaternion.rotationAroundYAxis(Math.toRadians(yawDegrees).toFloat()))
			(trackers.allL + foot).forEach { it.dataTick(time) }
			pose.skeleton.legTweaks.adaptiveLeftFoot.update(Vector3.NULL, foot.getRotationWithoutAdaptive(), 0f, Vector3.NULL, contactTrusted, time)
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
			val turn = Quaternion.rotationAroundYAxis(i * 0.001f)
			listOf(f.trackers.head, f.trackers.hip, f.trackers.leftThigh, f.trackers.leftCalf).forEach { it.setRotation(turn) }
			f.step(i)
		}
		assertEquals(bias, f.foot.adaptiveYawBiasRadians)
		f.foot.status = TrackerStatus.DISCONNECTED
		for (i in 701..1000) f.step(i)
		assertEquals(bias, f.foot.adaptiveYawBiasRadians)
	}

	@Test
	fun headYawAloneDoesNotInvalidateAStationaryFootReference() {
		val f = Fixture()
		for (i in 0..400) f.step(i)
		val before = f.foot.adaptiveYawBiasRadians
		for (i in 401..700) {
			f.trackers.head.setRotation(Quaternion.rotationAroundYAxis((i - 400) * 0.003f))
			f.step(i)
		}
		assertTrue(f.foot.adaptiveYawBiasRadians > before + Math.toRadians(0.1))
	}

	@Test
	fun preExistingFootBiasIsNotFalselyClaimedAsRecovered() {
		val f = Fixture()
		for (i in 0..600) f.step(i, 9.0)
		assertEquals(0f, f.foot.adaptiveYawBiasRadians)
		f.pose.adaptiveEstimator.updateAbsoluteConstraints(f.pose.skeleton, 60_000_000_000L)
		val diagnostic = f.pose.adaptiveEstimator.diagnostics.first { it.trackerId == f.foot.id }
		assertEquals("PLANTED_REFERENCE_INCREMENTAL_ONLY", diagnostic.correctionMode)
		assertEquals(false, diagnostic.canRecoverPreExistingBias)
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

	@Test
	fun matureTemperatureRateCarriesFootCorrectionAcrossContactLoss(@TempDir directory: Path) {
		val f = Fixture(directory)
		try {
			for (index in 0..900) f.step(index)
			val before = f.foot.adaptiveYawBiasRadians
			for (index in 901..930) f.step(index, contactTrusted = false)
			assertTrue(f.foot.adaptiveYawBiasRadians > before)
			f.pose.adaptiveEstimator.updateAbsoluteConstraints(f.pose.skeleton, 93_000_000_000L)
			val diagnostic = f.pose.adaptiveEstimator.diagnostics.first { it.trackerId == f.foot.id }
			assertTrue(diagnostic.holdoverActive)
		} finally {
			f.pose.adaptiveEstimator.close()
		}
	}
}
