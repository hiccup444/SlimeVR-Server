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

class StationaryChainYawIntegrationTest {
	private class Fixture(directory: Path? = null, private val hardwareRole: TrackerPosition? = null) {
		private val head = TestTrackerSet().head.apply { position = Vector3(0f, 1.7f, 0f) }
		private val device = directory?.let {
			object : Device(DeviceOrigin.UDP) {
				override val hardwareIdentifier = "stationary-chain-test"
			}
		}
		val trackers = listOf(
			TrackerPosition.UPPER_CHEST,
			TrackerPosition.CHEST,
			TrackerPosition.WAIST,
			TrackerPosition.HIP,
			TrackerPosition.LEFT_UPPER_LEG,
			TrackerPosition.RIGHT_UPPER_LEG,
			TrackerPosition.LEFT_LOWER_LEG,
			TrackerPosition.RIGHT_LOWER_LEG,
			TrackerPosition.LEFT_FOOT,
			TrackerPosition.RIGHT_FOOT,
			TrackerPosition.LEFT_HAND,
			TrackerPosition.RIGHT_HAND,
		).mapIndexed { index, role ->
			Tracker(if (role == hardwareRole) device else null, index + 1, role.name, trackerPosition = role, hasPosition = role == TrackerPosition.LEFT_HAND || role == TrackerPosition.RIGHT_HAND, hasRotation = true, hasAcceleration = true, imuType = IMUType.BNO085, trackRotDirection = false).apply {
				status = TrackerStatus.OK
				if (role == TrackerPosition.LEFT_HAND) position = Vector3(-0.4f, 1.3f, 0.3f)
				if (role == TrackerPosition.RIGHT_HAND) position = Vector3(0.4f, 1.3f, 0.3f)
			}
		}
		val pose = HumanPoseManager(listOf(head) + trackers)
		init {
			pose.adaptiveTrackingConfig.yawCorrectionEnabled = true
			if (directory != null) {
				pose.adaptiveTrackingConfig.temperatureLearningEnabled = true
				pose.adaptiveTrackingConfig.calibrationDirectory = directory.toString()
			}
		}

		fun tracker(role: TrackerPosition) = trackers.first { it.trackerPosition == role }

		fun step(index: Int, driftRole: TrackerPosition? = null, driftDegrees: Double = 0.0, bodyTurnDegrees: Double = 0.0, contactTrusted: Boolean = true, handOffset: Float = 0f, coherentDriftDegrees: Double = 0.0, fullPose: Boolean = false) {
			val time = index * 100_000_000L
			val bodyTurn = Quaternion.rotationAroundYAxis(Math.toRadians(bodyTurnDegrees).toFloat())
			head.setRotation(bodyTurn)
			head.dataTick(time)
			for (tracker in trackers) {
				if (tracker.trackerPosition == TrackerPosition.LEFT_HAND) tracker.position = Vector3(-0.4f + handOffset, 1.3f, 0.3f)
				val shared = if (tracker.trackerPosition != TrackerPosition.LEFT_FOOT && tracker.trackerPosition != TrackerPosition.RIGHT_FOOT && tracker.trackerPosition != TrackerPosition.LEFT_HAND && tracker.trackerPosition != TrackerPosition.RIGHT_HAND) coherentDriftDegrees else 0.0
				val yaw = bodyTurnDegrees + shared + if (tracker.trackerPosition == driftRole) driftDegrees else 0.0
				tracker.setRotation(Quaternion.rotationAroundYAxis(Math.toRadians(yaw).toFloat()))
				tracker.setAcceleration(Vector3.NULL, time)
				if (tracker.trackerPosition == hardwareRole) tracker.setTemperature(20.5f, time)
				tracker.dataTick(time)
			}
			if (!fullPose) {
				pose.skeleton.legTweaks.adaptiveLeftFoot.update(Vector3.NULL, tracker(TrackerPosition.LEFT_FOOT).getRotationWithoutAdaptive(), 0f, Vector3.NULL, contactTrusted, time)
				pose.skeleton.legTweaks.adaptiveRightFoot.update(Vector3.NULL, tracker(TrackerPosition.RIGHT_FOOT).getRotationWithoutAdaptive(), 0f, Vector3.NULL, contactTrusted, time)
			}
			if (fullPose) pose.skeleton.updatePose(time) else pose.adaptiveEstimator.updateAbsoluteConstraints(pose.skeleton, time)
		}
	}

	@Test
	fun fullPoseTickAppliesStationaryChainBiasWithoutChangingRawImuData() {
		val fixture = Fixture()
		fixture.pose.adaptiveTrackingConfig.poseOptimizerEnabled = true
		fixture.pose.skeleton.legTweaks.resetFloorLevel()
		val hip = fixture.tracker(TrackerPosition.HIP)
		for (index in 0..600) {
			fixture.step(index, TrackerPosition.HIP, index * 0.001, fullPose = true)
			assertEquals(Quaternion.rotationAroundYAxis(Math.toRadians(index * 0.001).toFloat()), hip.getRawRotation())
		}
		assertTrue(hip.adaptiveYawBiasRadians > Math.toRadians(0.15), "${fixture.pose.adaptiveEstimator.diagnostics.first { it.trackerId == hip.id }}")
		assertTrue(hip.getRotation().angleToR(hip.getRotationWithoutAdaptive()) > Math.toRadians(0.15))
		assertTrue(fixture.pose.skeleton.hipBone.getGlobalRotation().lenSq().isFinite())
		assertEquals("STATIONARY_CHAIN_INCREMENTAL_ONLY", fixture.pose.adaptiveEstimator.diagnostics.first { it.trackerId == hip.id }.correctionMode)
	}

	@Test
	fun newlyAccumulatedDriftIsCorrectedAcrossTorsoAndLegRoles() {
		val roles = listOf(
			TrackerPosition.UPPER_CHEST,
			TrackerPosition.CHEST,
			TrackerPosition.WAIST,
			TrackerPosition.HIP,
			TrackerPosition.LEFT_UPPER_LEG,
			TrackerPosition.RIGHT_UPPER_LEG,
			TrackerPosition.LEFT_LOWER_LEG,
			TrackerPosition.RIGHT_LOWER_LEG,
		)
		for (role in roles) {
			val fixture = Fixture()
			for (index in 0..600) fixture.step(index, role, index * 0.001)
			val tracker = fixture.tracker(role)
			assertTrue(tracker.adaptiveYawBiasRadians > Math.toRadians(0.15), "$role did not learn new drift")
			val diagnostic = fixture.pose.adaptiveEstimator.diagnostics.first { it.trackerId == tracker.id }
			assertEquals("STATIONARY_CHAIN_INCREMENTAL_ONLY", diagnostic.correctionMode)
			assertEquals(false, diagnostic.canRecoverPreExistingBias)
			assertTrue(diagnostic.residual?.eligibleForLearning == true, "$role was not eligible")
		}
	}

	@Test
	fun initialBiasAndCoherentBodyMotionDoNotBecomeCorrection() {
		val fixture = Fixture()
		for (index in 0..600) fixture.step(index, TrackerPosition.HIP, 9.0)
		assertEquals(0f, fixture.tracker(TrackerPosition.HIP).adaptiveYawBiasRadians)
		for (index in 601..900) fixture.step(index, bodyTurnDegrees = (index - 600) * 0.01)
		assertEquals(0f, fixture.tracker(TrackerPosition.HIP).adaptiveYawBiasRadians)
	}

	@Test
	fun contactLossControllerMotionAndMissingWitnessPauseLearning() {
		val fixture = Fixture()
		for (index in 0..600) fixture.step(index, TrackerPosition.HIP, index * 0.001)
		val tracker = fixture.tracker(TrackerPosition.HIP)
		val learned = tracker.adaptiveYawBiasRadians
		assertTrue(learned > 0f)
		for (index in 601..700) fixture.step(index, TrackerPosition.HIP, index * 0.001, contactTrusted = false)
		assertEquals(learned, tracker.adaptiveYawBiasRadians)
		for (index in 701..800) fixture.step(index, TrackerPosition.HIP, index * 0.001, handOffset = (index - 700) * 0.01f)
		assertEquals(learned, tracker.adaptiveYawBiasRadians)
		fixture.tracker(TrackerPosition.RIGHT_UPPER_LEG).status = TrackerStatus.DISCONNECTED
		for (index in 801..900) fixture.step(index, TrackerPosition.HIP, index * 0.001)
		assertEquals(learned, tracker.adaptiveYawBiasRadians)
	}

	@Test
	fun disconnectedFootBlocksLearningBeforeThePreviousContactSnapshotReleases() {
		val fixture = Fixture()
		for (index in 0..600) fixture.step(index, TrackerPosition.HIP, index * 0.001)
		val hip = fixture.tracker(TrackerPosition.HIP)
		val bias = hip.adaptiveYawBiasRadians
		assertTrue(bias > 0f)
		fixture.tracker(TrackerPosition.LEFT_FOOT).status = TrackerStatus.DISCONNECTED
		fixture.step(601, TrackerPosition.HIP, 0.601)
		assertEquals(bias, hip.adaptiveYawBiasRadians)
		assertEquals("WAITING_FOR_BOTH_PLANTED_FEET", fixture.pose.adaptiveEstimator.diagnostics.first { it.trackerId == hip.id }.residual?.reason)
	}

	@Test
	fun disablingCorrectionAndYawResetClearTheTransientBias() {
		val fixture = Fixture()
		for (index in 0..600) fixture.step(index, TrackerPosition.LEFT_LOWER_LEG, index * 0.001)
		val tracker = fixture.tracker(TrackerPosition.LEFT_LOWER_LEG)
		assertTrue(tracker.adaptiveYawBiasRadians > 0f)
		fixture.pose.adaptiveTrackingConfig.yawCorrectionEnabled = false
		fixture.step(601)
		assertEquals(0f, tracker.adaptiveYawBiasRadians)
		fixture.pose.adaptiveTrackingConfig.yawCorrectionEnabled = true
		for (index in 602..1000) fixture.step(index, TrackerPosition.LEFT_LOWER_LEG, index * 0.001)
		assertTrue(tracker.adaptiveYawBiasRadians > 0f)
		fixture.pose.resetTrackersYaw("stationary-chain-test")
		assertEquals(0f, tracker.adaptiveYawBiasRadians)
	}

	@Test
	fun targetDisconnectClearsAStaleCorrection() {
		val fixture = Fixture()
		for (index in 0..600) fixture.step(index, TrackerPosition.CHEST, index * 0.001)
		val tracker = fixture.tracker(TrackerPosition.CHEST)
		assertTrue(tracker.adaptiveYawBiasRadians > 0f)
		tracker.status = TrackerStatus.DISCONNECTED
		fixture.step(601, TrackerPosition.CHEST, 0.601)
		assertEquals(0f, tracker.adaptiveYawBiasRadians)
	}

	@Test
	fun controllerDropoutPausesLearningWithoutDiscardingAcceptedBias() {
		val fixture = Fixture()
		for (index in 0..600) fixture.step(index, TrackerPosition.RIGHT_UPPER_LEG, index * 0.001)
		val tracker = fixture.tracker(TrackerPosition.RIGHT_UPPER_LEG)
		val accepted = tracker.adaptiveYawBiasRadians
		assertTrue(accepted > 0f)
		fixture.tracker(TrackerPosition.LEFT_HAND).status = TrackerStatus.DISCONNECTED
		for (index in 601..650) fixture.step(index, TrackerPosition.RIGHT_UPPER_LEG, index * 0.001)
		assertEquals(accepted, tracker.adaptiveYawBiasRadians)
		val diagnostic = fixture.pose.adaptiveEstimator.diagnostics.first { it.trackerId == tracker.id }
		assertEquals("ABSOLUTE_ANCHORS_MOVING_OR_UNAVAILABLE", diagnostic.residual?.reason)
	}

	@Test
	fun coherentChainYawChangeIsNotAssignedToIndividualTrackers() {
		val fixture = Fixture()
		for (index in 0..600) fixture.step(index, coherentDriftDegrees = index * 0.001)
		for (role in listOf(TrackerPosition.CHEST, TrackerPosition.HIP, TrackerPosition.LEFT_UPPER_LEG, TrackerPosition.LEFT_LOWER_LEG)) {
			assertEquals(0f, fixture.tracker(role).adaptiveYawBiasRadians, "$role learned a shared change")
		}
	}

	@Test
	fun matureTemperatureRateCarriesOnlyABriefWitnessDropout(@TempDir directory: Path) {
		val fixture = Fixture(directory, TrackerPosition.HIP)
		try {
			for (index in 0..1000) fixture.step(index, TrackerPosition.HIP, index * 0.001)
			val tracker = fixture.tracker(TrackerPosition.HIP)
			val before = tracker.adaptiveYawBiasRadians
			fixture.tracker(TrackerPosition.RIGHT_UPPER_LEG).status = TrackerStatus.DISCONNECTED
			for (index in 1001..1020) fixture.step(index, TrackerPosition.HIP, index * 0.001)
			val diagnostic = fixture.pose.adaptiveEstimator.diagnostics.first { it.trackerId == tracker.id }
			assertTrue(tracker.adaptiveYawBiasRadians > before)
			assertTrue(diagnostic.holdoverActive)
			assertEquals("READY", diagnostic.temperatureModelStatus)
		} finally {
			fixture.pose.adaptiveEstimator.close()
		}
	}

	@Test
	fun suddenTargetOrientationJumpDiscardsTheOldBias() {
		val fixture = Fixture()
		for (index in 0..600) fixture.step(index, TrackerPosition.HIP, index * 0.001)
		val tracker = fixture.tracker(TrackerPosition.HIP)
		assertTrue(tracker.adaptiveYawBiasRadians > 0f)
		fixture.step(601, TrackerPosition.HIP, 90.601)
		assertEquals(0f, tracker.adaptiveYawBiasRadians)
		val diagnostic = fixture.pose.adaptiveEstimator.diagnostics.first { it.trackerId == tracker.id }
		assertEquals("TARGET_ORIENTATION_DISCONTINUITY", diagnostic.residual?.reason)
	}
}
