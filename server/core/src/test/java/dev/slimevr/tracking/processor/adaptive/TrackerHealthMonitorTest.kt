package dev.slimevr.tracking.processor.adaptive

import dev.slimevr.tracking.trackers.TrackerStatus
import io.github.axisangles.ktmath.Quaternion
import io.github.axisangles.ktmath.Vector3
import org.junit.jupiter.api.Test
import kotlin.math.PI
import kotlin.test.assertEquals
import kotlin.test.assertFalse
import kotlin.test.assertTrue

class TrackerHealthMonitorTest {
	private fun sample(
		time: Long,
		id: Int = 14,
		rotation: Quaternion = Quaternion.IDENTITY,
		age: Long? = 10_000_000L,
		acceleration: Vector3? = Vector3.NULL,
		speed: Float? = 0f,
		continuous: Boolean = true,
		status: TrackerStatus = TrackerStatus.OK,
	) = TelemetrySample(
		id = id,
		name = "test-imu",
		role = null,
		status = status,
		rawRotation = rotation,
		adjustedRotation = rotation,
		position = null,
		acceleration = acceleration,
		derivedLinearVelocity = null,
		packetAgeNanos = age,
		temperatureCelsius = null,
		angularSpeedRadiansPerSecond = speed,
		rotationExpected = true,
		positionExpected = false,
		continuousObservation = continuous,
		accelerationAgeNanos = age,
	)

	@Test
	fun warmupAndSmoothRapidTurnDoNotLookLikeJitter() {
		val monitor = TrackerHealthMonitor()
		var result = monitor.observe(sample(0L), 0L)
		assertTrue("WARMING_UP" in result.reasons)
		for (index in 1..30) {
			val time = index * 100_000_000L
			val rotation = Quaternion.rotationAroundYAxis((index * 0.4).toFloat())
			result = monitor.observe(sample(time, rotation = rotation, speed = 4f), time)
		}
		assertFalse("ALTERNATING_HIGH_FREQUENCY_NOISE" in result.reasons)
		assertFalse(result.suspectedFrozen)
		assertTrue(result.qualityMultiplier > 0.75f)
	}

	@Test
	fun alternatingRapidSmallRotationsAreReportedAsNoise() {
		val monitor = TrackerHealthMonitor()
		var result = monitor.observe(sample(0L), 0L)
		for (index in 1..20) {
			val time = index * 20_000_000L
			val sign = if (index % 2 == 0) -1.0 else 1.0
			val rotation = Quaternion.rotationAroundYAxis((sign * 0.06).toFloat())
			result = monitor.observe(sample(time, rotation = rotation, speed = 6f), time)
		}
		assertTrue("ALTERNATING_HIGH_FREQUENCY_NOISE" in result.reasons)
		assertTrue(result.qualityMultiplier <= 0.3f)
		assertTrue(result.stationaryConfidence < 0.5f)
	}

	@Test
	fun frozenOrientationRequiresFreshArrivalsAndIndependentMotionEvidence() {
		val monitor = TrackerHealthMonitor()
		var result = monitor.observe(sample(0L), 0L, TrackerMotionContext(1.0, 0L))
		for (index in 1..25) {
			val time = index * 100_000_000L
			result = monitor.observe(sample(time), time, TrackerMotionContext(1.0, time))
		}
		assertTrue(result.suspectedFrozen)
		assertTrue("FROZEN_WITH_INDEPENDENT_TARGET_MOTION" in result.reasons)

		val stillMonitor = TrackerHealthMonitor()
		for (index in 0..30) {
			val time = index * 100_000_000L
			result = stillMonitor.observe(sample(time), time)
		}
		assertFalse(result.suspectedFrozen)
		assertFalse("FROZEN_WITH_INDEPENDENT_TARGET_MOTION" in result.reasons)
	}

	@Test
	fun briefMotionEvidenceAfterStillnessDoesNotTriggerFrozenDetection() {
		val monitor = TrackerHealthMonitor()
		for (index in 0..30) {
			val time = index * 100_000_000L
			monitor.observe(sample(time), time)
		}
		val now = 3_100_000_000L
		val result = monitor.observe(sample(now), now, TrackerMotionContext(1.0, now))
		assertFalse(result.suspectedFrozen)
	}

	@Test
	fun staleAccelerationCannotReduceCurrentOrientationQuality() {
		val monitor = TrackerHealthMonitor()
		val result = monitor.observe(
			sample(0L, acceleration = Vector3(45f, 0f, 0f)).copy(accelerationAgeNanos = 1_000_000_000L),
			0L,
		)
		assertFalse("IMPOSSIBLE_ACCELERATION" in result.reasons)
		assertTrue(result.qualityMultiplier > 0.08f)
	}

	@Test
	fun repeatedFreshButUnchangedPacketArrivalDoesNotTriggerFrozenDetection() {
		val monitor = TrackerHealthMonitor()
		var result = monitor.observe(sample(0L), 0L, TrackerMotionContext(1.0, 0L))
		for (index in 1..30) {
			val now = index * 100_000_000L
			val age = now - 10_000_000L
			result = monitor.observe(sample(now, age = age), now, TrackerMotionContext(1.0, now))
		}
		assertFalse(result.suspectedFrozen)
		assertFalse("FROZEN_WITH_INDEPENDENT_TARGET_MOTION" in result.reasons)
	}

	@Test
	fun impossibleAccelerationAndUnexplainedRotationJumpReduceQuality() {
		val accelerationMonitor = TrackerHealthMonitor()
		var result = accelerationMonitor.observe(sample(0L), 0L)
		result = accelerationMonitor.observe(sample(100_000_000L, acceleration = Vector3(45f, 0f, 0f)), 100_000_000L)
		assertTrue("IMPOSSIBLE_ACCELERATION" in result.reasons)
		assertTrue(result.qualityMultiplier <= 0.08f)

		val discontinuityMonitor = TrackerHealthMonitor()
		discontinuityMonitor.observe(sample(0L), 0L)
		result = discontinuityMonitor.observe(
			sample(20_000_000L, rotation = Quaternion.rotationAroundYAxis((PI * 0.75).toFloat()), speed = 0f),
			20_000_000L,
		)
		assertTrue("ORIENTATION_DISCONTINUITY" in result.reasons)
		assertTrue(result.qualityMultiplier <= 0.25f)
	}

	@Test
	fun stalePacketAndInvalidQuaternionFailClosed() {
		val monitor = TrackerHealthMonitor()
		var result = monitor.observe(sample(0L, age = 700_000_000L), 0L)
		assertEquals(0f, result.qualityMultiplier)
		assertEquals(listOf("STALE_PACKET"), result.reasons)

		val invalid = Quaternion(Float.NaN, 0f, 0f, 0f)
		result = monitor.observe(sample(100_000_000L, rotation = invalid), 100_000_000L)
		assertEquals(0f, result.qualityMultiplier)
		assertTrue("INVALID_ORIENTATION" in result.reasons)
	}

	@Test
	fun unknownPacketAgeReducesTrustWithoutErasingHistory() {
		val monitor = TrackerHealthMonitor()
		monitor.observe(sample(0L), 0L)
		var result = monitor.observe(sample(100_000_000L, age = null), 100_000_000L)
		assertTrue("FRESHNESS_UNKNOWN" in result.reasons)
		assertTrue(result.qualityMultiplier > 0f)
		result = monitor.observe(sample(200_000_000L), 200_000_000L)
		assertTrue(result.observedSeconds > 0.0)
	}

	@Test
	fun qualityRecoversGraduallyAndResetDropsHistory() {
		val monitor = TrackerHealthMonitor()
		monitor.observe(sample(0L), 0L)
		var result = monitor.observe(sample(100_000_000L, rotation = Quaternion.rotationAroundYAxis(2f)), 100_000_000L)
		assertTrue(result.qualityMultiplier <= 0.25f)
		val low = result.qualityMultiplier
		for (index in 2..40) {
			val time = index * 100_000_000L
			result = monitor.observe(sample(time, rotation = Quaternion.rotationAroundYAxis(2f)), time)
		}
		assertTrue(result.qualityMultiplier > low)
		assertTrue(result.qualityMultiplier < 1f)
		monitor.resetTracker(14)
		result = monitor.observe(sample(4_200_000_000L, rotation = Quaternion.rotationAroundYAxis(2f)), 4_200_000_000L)
		assertTrue("WARMING_UP" in result.reasons)
		assertEquals(0.0, result.observedSeconds)
	}

	@Test
	fun perTrackerHistoryIsBounded() {
		val monitor = TrackerHealthMonitor(maxTrackers = 2)
		monitor.observe(sample(0L, id = 1), 0L)
		monitor.observe(sample(0L, id = 2), 0L)
		monitor.observe(sample(0L, id = 3), 0L)
		val result = monitor.observe(sample(100_000_000L, id = 1), 100_000_000L)
		assertEquals(0.0, result.observedSeconds)
	}
}
