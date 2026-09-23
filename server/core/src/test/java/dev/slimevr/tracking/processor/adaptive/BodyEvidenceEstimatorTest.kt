package dev.slimevr.tracking.processor.adaptive

import dev.slimevr.tracking.trackers.Tracker
import dev.slimevr.tracking.trackers.TrackerStatus
import io.github.axisangles.ktmath.Quaternion
import org.junit.jupiter.api.Test
import kotlin.test.assertEquals
import kotlin.test.assertTrue

class BodyEvidenceEstimatorTest {
	private val trackers = (1..3).map { id -> Tracker(null, id, "sensor-$id", trackerPosition = null, hasRotation = true, trackRotDirection = false).apply { status = TrackerStatus.OK } }
	private fun run(vararg angles: Float, continuousSeconds: Double = 3.0): AdaptiveTelemetryFrame {
		val sensors = SensorStateManager()
		val model = BodyEvidenceEstimator()
		fun sample(time: Long) = sensors.sample(trackers, emptyList(), time).let { frame -> frame.copy(samples = frame.samples.map { it.copy(confidence = TrackerConfidence(0.9f, emptyList(), continuousSeconds)) }) }
		model.observe(sample(0), listOf(listOf(1, 2, 3)))
		trackers.forEachIndexed { index, tracker -> tracker.setRotation(Quaternion.rotationAroundYAxis(angles[index])) }
		return model.observe(sample(100_000_000), listOf(listOf(1, 2, 3)))
	}

	@Test
	fun coherentTurnPreservesConfidence() {
		assertTrue(run(0.5f, 0.5f, 0.5f).samples.all { it.confidence!!.score == 0.9f })
	}

	@Test
	fun isolatedJumpLowersTrustWithoutChangingTheMeasurement() {
		val frame = run(0.5f, 0f, 0f)
		assertTrue(frame.samples.first().confidence!!.score < 0.8f)
		assertEquals(trackers.first().getRotation(), frame.samples.first().adjustedRotation)
		assertTrue(frame.samples.first().confidence!!.reasons.single().contains("ARTICULATION"))
	}

	@Test
	fun oppositeTurnDoesNotOverridePossibleArticulation() {
		val frame = run(-0.5f, 0.5f, 0.5f)
		assertEquals(0.9f, frame.samples.first().confidence!!.score)
	}

	@Test
	fun conflictingNeighborsDoNotFormConsensus() {
		val frame = run(0.5f, 0.5f, -0.5f)
		assertEquals(0.9f, frame.samples.first().confidence!!.score)
	}

	@Test
	fun startupMovementDoesNotReduceTrust() {
		val frame = run(0.5f, 0f, 0f, continuousSeconds = 0.1)
		assertEquals(0.9f, frame.samples.first().confidence!!.score)
	}
}
