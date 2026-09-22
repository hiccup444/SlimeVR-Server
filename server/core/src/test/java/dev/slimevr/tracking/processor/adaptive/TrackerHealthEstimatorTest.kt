package dev.slimevr.tracking.processor.adaptive

import dev.slimevr.tracking.trackers.Tracker
import dev.slimevr.tracking.trackers.TrackerStatus
import io.github.axisangles.ktmath.Quaternion
import org.junit.jupiter.api.Test
import kotlin.test.assertEquals
import kotlin.test.assertNotNull
import kotlin.test.assertTrue

class TrackerHealthEstimatorTest {
	private val base = SensorStateManager().sample(
		listOf(Tracker(null, 1, "test", trackerPosition = null, hasRotation = true, trackRotDirection = false).apply { status = TrackerStatus.OK }),
		emptyList(),
		0L,
	).samples.single().copy(
		packetAgeNanos = 0,
		continuousObservation = true,
		confidence = TrackerConfidence(0.9f, emptyList(), 3.0),
	)

	@Test
	fun jitterCapsConfidenceWithoutChangingMeasurementsOrOutputs() {
		val estimator = TrackerHealthEstimator()
		for (index in 0..20) {
			val rotation = Quaternion.rotationAroundYAxis(if (index % 2 == 0) -0.06f else 0.06f)
			val sample = base.copy(rawRotation = rotation, adjustedRotation = rotation)
			val frame = AdaptiveTelemetryFrame(index * 20_000_000L, listOf(sample), listOf(sample), 0)
			val result = estimator.observe(frame)
			val observed = result.samples.single()
			assertEquals(sample, observed.copy(confidence = sample.confidence, health = null))
			assertEquals(frame.computedSamples, result.computedSamples)
			if (index == 20) {
				assertTrue(assertNotNull(observed.confidence).score <= 0.3f)
				assertTrue("ALTERNATING_HIGH_FREQUENCY_NOISE" in assertNotNull(observed.health).reasons)
			}
		}
	}

	@Test
	fun epochChangeAndRemovalRestartHealthHistory() {
		val estimator = TrackerHealthEstimator()
		fun frame(time: Long, epoch: Long = 0) = AdaptiveTelemetryFrame(time, listOf(base), emptyList(), epoch)
		estimator.observe(frame(0))
		assertTrue(assertNotNull(estimator.observe(frame(20_000_000)).samples.single().health).observedSeconds > 0)
		assertEquals(0.0, assertNotNull(estimator.observe(frame(40_000_000, 1)).samples.single().health).observedSeconds)
		estimator.observe(frame(60_000_000, 1).copy(samples = emptyList()))
		assertEquals(0.0, assertNotNull(estimator.observe(frame(80_000_000, 1)).samples.single().health).observedSeconds)
	}

	@Test
	fun healthNeverRaisesAnExistingLowConfidenceScore() {
		val estimator = TrackerHealthEstimator()
		val sample = base.copy(confidence = TrackerConfidence(0.1f, listOf("LOW_CONFIDENCE"), 0.0))
		val result = estimator.observe(AdaptiveTelemetryFrame(0, listOf(sample), emptyList(), 0))
		assertEquals(0.1f, result.samples.single().confidence?.score)
		assertTrue("LOW_CONFIDENCE" in assertNotNull(result.samples.single().confidence).reasons)
	}
}
