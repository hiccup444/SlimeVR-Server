package dev.slimevr.tracking.processor.adaptive

import dev.slimevr.tracking.processor.HumanPoseManager
import dev.slimevr.tracking.trackers.TrackerPosition
import dev.slimevr.tracking.trackers.TrackerStatus
import dev.slimevr.unit.TestTrackerSet
import io.github.axisangles.ktmath.Quaternion
import io.github.axisangles.ktmath.Vector3
import org.junit.jupiter.api.Test
import kotlin.test.assertEquals
import kotlin.test.assertTrue

class AdaptiveFootIntegrationTest {
	private class Fixture {
		val trackers = TestTrackerSet()
		val left = trackers.mkTrack(20, TrackerPosition.LEFT_FOOT)
		val right = trackers.mkTrack(21, TrackerPosition.RIGHT_FOOT)
		val pose = HumanPoseManager(trackers.allL + listOf(left, right))
		val legs = pose.skeleton.legTweaks
		init {
			trackers.head.position = Vector3(0f, pose.userHeightFromConfig, 0f)
			pose.update()
			pose.adaptiveTrackingConfig.footAnchoringEnabled = true
			legs.setSkatingCorrectionEnabled(true)
			legs.setFloorClipEnabled(false)
			legs.footPlantEnabled = false
			legs.toeSnapEnabled = false
			legs.resetFloorLevel()
		}
		fun step(millis: Long, offset: Float = 0f): Vector3 {
			val skeleton = pose.skeleton
			skeleton.computedHipTracker!!.position = Vector3(0f, 1f, 0f)
			skeleton.computedLeftKneeTracker!!.position = Vector3(-0.1f, 0.5f, 0f)
			skeleton.computedRightKneeTracker!!.position = Vector3(0.1f, 0.5f, 0f)
			skeleton.computedLeftFootTracker!!.position = Vector3(-0.1f + offset, 0f, 0f)
			skeleton.computedRightFootTracker!!.position = Vector3(0.1f, 0f, 0f)
			skeleton.computedLeftFootTracker!!.setRotation(Quaternion.IDENTITY)
			skeleton.computedRightFootTracker!!.setRotation(Quaternion.IDENTITY)
			legs.tweakLegs(millis * 1_000_000)
			return skeleton.computedLeftFootTracker!!.position
		}
		fun plant() {
			for (time in 0L..400L step 20) step(time)
			assertEquals(FootContactState.PLANTED, legs.adaptiveLeftFoot.snapshot.state)
		}
	}

	@Test
	fun enabledAnchoringReducesSlowHorizontalFootSlide() {
		val fixture = Fixture()
		fixture.plant()
		var result = Vector3.NULL
		for (index in 1..20) result = fixture.step(400L + index * 20, index * 0.001f)
		val uncorrected = -0.1f + 0.02f
		assertTrue(result.x < uncorrected - 0.005f)
		assertTrue(result.x > -0.1f)
		assertEquals(0f, result.y)
		assertEquals(Quaternion.IDENTITY, fixture.left.getRawRotation())
	}

	@Test
	fun trackerLossDiscardsAnchorAndRecoversWithoutOldPlant() {
		val fixture = Fixture()
		fixture.plant()
		fixture.left.status = TrackerStatus.DISCONNECTED
		val fallback = fixture.step(420, 0.02f)
		// No adaptive anchor survives loss; the enabled legacy correction may still act.
		assertTrue(fallback.x.isFinite() && fallback.y.isFinite() && fallback.z.isFinite())
		assertEquals(FootContactState.AIRBORNE, fixture.legs.adaptiveLeftFoot.snapshot.state)
		fixture.left.status = TrackerStatus.OK
		fixture.step(440, 0.02f)
		assertEquals(0f, fixture.legs.adaptiveLeftFoot.snapshot.weight)
	}

	@Test
	fun resetPauseAndLocalizerTransitionsClearAnchors() {
		val fixture = Fixture()
		fixture.plant()
		fixture.pose.setPauseTracking(true, "test")
		assertEquals(0f, fixture.legs.adaptiveLeftFoot.snapshot.weight)
		fixture.pose.setPauseTracking(false, "test")
		fixture.plant()
		fixture.legs.setLocalizerMode(true)
		fixture.step(420)
		assertEquals(FootContactState.AIRBORNE, fixture.legs.adaptiveLeftFoot.snapshot.state)
		fixture.legs.setLocalizerMode(false)
		fixture.plant()
		fixture.pose.resetTrackersYaw("test")
		assertEquals(0f, fixture.legs.adaptiveLeftFoot.snapshot.weight)
	}

	@Test
	fun existingSkatingSwitchStillControlsWhetherAnchoringApplies() {
		val fixture = Fixture()
		fixture.legs.setSkatingCorrectionEnabled(false)
		for (time in 0L..400L step 20) fixture.step(time)
		assertEquals(FootContactState.AIRBORNE, fixture.legs.adaptiveLeftFoot.snapshot.state)
		assertEquals(-0.08f, fixture.step(420, 0.02f).x, 0.00001f)
	}

	@Test
	fun contactDiagnosticsDoNotApplyTheirDetectedAnchor() {
		val fixture = Fixture()
		fixture.pose.adaptiveTrackingConfig.footAnchoringEnabled = false
		fixture.pose.adaptiveTrackingConfig.telemetryEnabled = true
		fixture.legs.setSkatingCorrectionEnabled(false)
		fixture.plant()
		for (index in 1..20) {
			val displacement = index * 0.001f
			assertEquals(-0.1f + displacement, fixture.step(400L + index * 20, displacement).x, 0.00001f)
		}
		assertEquals(FootContactState.PLANTED, fixture.legs.adaptiveLeftFoot.snapshot.state)
	}

	@Test
	fun optimizerDoesNotAnchorContactsWhenLegacySkatingSwitchIsOff() {
		val fixture = Fixture()
		fixture.pose.adaptiveTrackingConfig.poseOptimizerEnabled = true
		fixture.pose.adaptiveTrackingConfig.footAnchoringEnabled = true
		fixture.pose.adaptiveTrackingConfig.telemetryEnabled = true
		fixture.pose.adaptiveTrackingConfig.footContactDiagnosticsEnabled = true
		fixture.legs.setSkatingCorrectionEnabled(false)
		fixture.plant()
		assertEquals(FootContactState.PLANTED, fixture.legs.adaptiveLeftFoot.snapshot.state)

		fixture.pose.adaptivePoseSolver.update(fixture.pose.skeleton, 420_000_000L)
		assertTrue(fixture.pose.adaptivePoseSolver.replayInput!!.anchors.isEmpty())
	}

	@Test
	fun optimizerFootAnchorStrengthScalesTheSoftConstraintAndZeroDisablesIt() {
		fun leftAnchorWeight(strength: Float): Float? {
			val fixture = Fixture()
			fixture.pose.adaptiveTrackingConfig.poseOptimizerEnabled = true
			fixture.pose.adaptiveTrackingConfig.footAnchorStrength = strength
			fixture.plant()
			assertEquals(FootContactState.PLANTED, fixture.legs.adaptiveLeftFoot.snapshot.state)
			fixture.pose.adaptivePoseSolver.update(fixture.pose.skeleton, 420_000_000L)
			val keys = fixture.pose.adaptivePoseSolver.rawPose.keys.toList()
			val leftFootSegment = keys.indexOf("LEFT_FOOT_TRACKER")
			return fixture.pose.adaptivePoseSolver.replayInput!!.anchors
				.firstOrNull { it.segment == leftFootSegment }
				?.weight
		}

		assertEquals(null, leftAnchorWeight(0f))
		val halfStrength = leftAnchorWeight(0.5f) ?: error("Expected a half-strength foot anchor")
		val fullStrength = leftAnchorWeight(1f) ?: error("Expected a full-strength foot anchor")
		assertEquals(fullStrength * 0.5f, halfStrength, 0.00001f)
	}

	@Test
	fun zeroStrengthPreservesLegacySkatingExactly() {
		val adaptive = Fixture()
		val legacy = Fixture()
		adaptive.pose.adaptiveTrackingConfig.footAnchorStrength = 0f
		legacy.pose.adaptiveTrackingConfig.footAnchoringEnabled = false
		for (i in 0..80) {
			val offset = if (i <= 20) 0f else (i - 20) * 0.001f
			val expected = legacy.step(i * 20L, offset)
			val actual = adaptive.step(i * 20L, offset)
			assertEquals(expected.x, actual.x, 0.00001f)
			assertEquals(expected.y, actual.y, 0.00001f)
			assertEquals(expected.z, actual.z, 0.00001f)
		}
	}
}
