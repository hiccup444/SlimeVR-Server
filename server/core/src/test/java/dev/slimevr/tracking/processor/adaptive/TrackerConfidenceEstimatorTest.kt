package dev.slimevr.tracking.processor.adaptive

import dev.slimevr.tracking.trackers.Tracker
import dev.slimevr.tracking.trackers.TrackerStatus
import org.junit.jupiter.api.Test
import kotlin.test.assertEquals
import kotlin.test.assertNotNull
import kotlin.test.assertNull
import kotlin.test.assertTrue

class TrackerConfidenceEstimatorTest {
	private val base = SensorStateManager().sample(
		listOf(Tracker(null, 1, "test", trackerPosition = null, hasRotation = true, trackRotDirection = false).apply { status = TrackerStatus.OK }),
		emptyList(),
		0L,
	).samples.single().copy(packetAgeNanos = 0, continuousObservation = true)

	private fun observe(estimator: TrackerConfidenceEstimator, time: Long, sample: TelemetrySample = base, epoch: Long = 0) = assertNotNull(estimator.observe(AdaptiveTelemetryFrame(time, listOf(sample), emptyList(), epoch)).samples.single().confidence)

	@Test
	fun steadyFreshMeasurementsWarmUpGradually() {
		val estimator = TrackerConfidenceEstimator()
		var previous = observe(estimator, 0).score
		assertEquals(0.6f, previous)
		for (step in 1..150) {
			val next = observe(estimator, step * 20_000_000L)
			assertTrue(next.score >= previous)
			assertTrue(next.score - previous < 0.02f)
			assertTrue("INDEPENDENT_CONSTRAINTS_UNAVAILABLE" in next.reasons)
			previous = next.score
		}
		assertTrue(previous > 0.88f && previous < 0.91f)
	}

	@Test
	fun lossIsImmediateAndRecoveryWarmsUp() {
		val estimator = TrackerConfidenceEstimator()
		for (step in 0..100) observe(estimator, step * 20_000_000L)
		val lost = observe(estimator, 2_020_000_000L, base.copy(status = TrackerStatus.TIMED_OUT))
		assertEquals(0f, lost.score)
		assertTrue("TRACKING_UNAVAILABLE" in lost.reasons)
		assertEquals(0.6f, observe(estimator, 2_040_000_000L).score)
		assertEquals(0f, observe(estimator, 2_060_000_000L, base.copy(packetAgeNanos = 600_000_000L)).score)
	}

	@Test
	fun missingAndUnknownDataCannotBecomeHighlyTrusted() {
		val estimator = TrackerConfidenceEstimator()
		var result = observe(estimator, 0, base.copy(packetAgeNanos = null))
		for (step in 1..100) result = observe(estimator, step * 20_000_000L, base.copy(packetAgeNanos = null))
		assertTrue(result.score <= 0.6f)
		assertTrue("FRESHNESS_UNKNOWN" in result.reasons)
		val invalid = observe(estimator, 2_020_000_000L, base.copy(adjustedRotation = null))
		assertEquals(0f, invalid.score)
	}

	@Test
	fun rapidMotionChangesConfidenceGraduallyWithoutChangingMeasurements() {
		val estimator = TrackerConfidenceEstimator()
		for (step in 0..100) observe(estimator, step * 20_000_000L)
		val sample = base.copy(angularSpeedRadiansPerSecond = 20f)
		val frame = AdaptiveTelemetryFrame(2_020_000_000L, listOf(sample), listOf(sample), 0)
		val result = estimator.observe(frame)
		val score = assertNotNull(result.samples.single().confidence)
		assertTrue(score.score > 0.8f)
		assertTrue("HIGH_MOTION" in score.reasons)
		assertEquals(sample, result.samples.single().copy(confidence = null))
		assertNull(result.computedSamples.single().confidence)
	}

	@Test
	fun resetGapsAndRemovalDoNotCarryOldConfidence() {
		val estimator = TrackerConfidenceEstimator()
		for (step in 0..100) observe(estimator, step * 20_000_000L)
		assertEquals(0.6f, observe(estimator, 2_020_000_000L, epoch = 1).score)
		assertEquals(0.0, observe(estimator, 2_020_000_000L, epoch = 1).observationSeconds)
		assertEquals(0.0, observe(estimator, 3_020_000_000L).observationSeconds)
		estimator.observe(AdaptiveTelemetryFrame(3_040_000_000L, emptyList(), emptyList(), 0))
		assertEquals(0.0, observe(estimator, 3_060_000_000L).observationSeconds)
		estimator.reset()
		assertEquals(0.0, observe(estimator, 3_080_000_000L).observationSeconds)
	}

	@Test
	fun elapsedTimeControlsSmoothingRatherThanSampleCount() {
		fun score(step: Long): Float {
			val estimator = TrackerConfidenceEstimator()
			var time = 0L
			while (time <= 3_000_000_000L) {
				observe(estimator, time)
				time += step
			}
			return observe(estimator, 3_100_000_000L).score
		}
		assertEquals(score(10_000_000L), score(50_000_000L), 0.01f)
	}
}
