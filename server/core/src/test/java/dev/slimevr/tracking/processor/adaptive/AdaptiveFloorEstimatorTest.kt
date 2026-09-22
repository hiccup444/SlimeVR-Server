package dev.slimevr.tracking.processor.adaptive

import io.github.axisangles.ktmath.Vector3
import org.junit.jupiter.api.Test
import kotlin.test.assertEquals
import kotlin.test.assertNotNull
import kotlin.test.assertTrue

class AdaptiveFloorEstimatorTest {
	private fun planted(weight: Float = 0.95f) = FootContactSnapshot(FootContactState.PLANTED, weight = weight)
	private fun foot(y: Double) = Vector3(0f, y.toFloat(), 0f)

	private fun observe(estimator: AdaptiveFloorEstimator, time: Long, y: Double = 0.02, upright: Double = 1.0) = estimator.observe(0.0, foot(y), planted(), foot(y), planted(), upright, time)

	@Test
	fun learnsSmallCorrectionOnlyAfterPairedPlantedDwellAndSlewsSlowly() {
		val estimator = AdaptiveFloorEstimator()
		var estimate = observe(estimator, 0L)
		for (index in 1..49) estimate = observe(estimator, index * 100_000_000L)
		assertEquals("COLLECTING_PAIRED_CONTACT", estimate.reason)
		assertEquals(0.0, estimate.estimatedHeightMeters, 1e-9)

		var previous = estimate.estimatedHeightMeters
		for (index in 50..300) {
			estimate = observe(estimator, index * 100_000_000L)
			assertTrue(kotlin.math.abs(estimate.estimatedHeightMeters - previous) <= 0.000100001)
			previous = estimate.estimatedHeightMeters
		}
		assertEquals("LEARNING", estimate.reason)
		assertEquals(0.02, estimate.estimatedHeightMeters, 0.0002)
		assertTrue(estimate.trustedSeconds >= 5.0)
		assertTrue(estimate.estimatedHeightMeters <= 0.03)
	}

	@Test
	fun oneRaisedFootCrouchingAndLowConfidenceCannotContributeEvidence() {
		val estimator = AdaptiveFloorEstimator()
		var result = observe(estimator, 0L)
		for (index in 1..50) result = observe(estimator, index * 100_000_000L)
		assertTrue(result.trustedSeconds >= 4.9)

		result = estimator.observe(0.0, foot(0.0), planted(), foot(0.08), planted(), 1.0, 5_100_000_000L)
		assertEquals("FEET_DISAGREE", result.reason)
		assertEquals(0.0, result.trustedSeconds)
		result = observe(estimator, 5_200_000_000L, upright = 0.5)
		assertEquals("NOT_UPRIGHT", result.reason)
		assertEquals(0.0, result.trustedSeconds)
		result = estimator.observe(0.0, foot(0.02), planted(0.5f), foot(0.02), planted(), 1.0, 5_300_000_000L)
		assertEquals("BOTH_FEET_NOT_PLANTED", result.reason)
	}

	@Test
	fun outliersGapsAndResetDoNotReuseOldDwell() {
		val estimator = AdaptiveFloorEstimator()
		for (index in 0..30) observe(estimator, index * 100_000_000L)
		var result = observe(estimator, 3_100_000_000L, y = 0.07)
		assertEquals("OUTSIDE_CALIBRATED_BAND", result.reason)
		assertEquals(0.0, result.trustedSeconds)

		for (index in 0..20) result = observe(estimator, 4_000_000_000L + index * 100_000_000L)
		result = observe(estimator, 7_000_000_000L)
		assertEquals("TIME_GAP", result.reason)
		assertEquals(0.0, result.trustedSeconds)

		for (index in 0..20) result = observe(estimator, 7_100_000_000L + index * 100_000_000L)
		assertTrue(result.trustedSeconds < 5.0)
		estimator.reset()
		assertEquals("RESET", assertNotNull(estimator.estimate).reason)
		assertEquals(0.0, estimator.estimate!!.estimatedHeightMeters)
	}

	@Test
	fun correctionIsBoundedAndCalibratedFloorChangesResetEvidence() {
		val estimator = AdaptiveFloorEstimator()
		var result = observe(estimator, 0L, y = 0.06)
		for (index in 1..400) result = observe(estimator, index * 100_000_000L, y = 0.06)
		assertEquals(0.03, result.estimatedHeightMeters, 0.0001)
		result = estimator.observe(0.01, foot(0.03), planted(), foot(0.03), planted(), 1.0, 40_100_000_000L)
		assertEquals(0.01, result.calibratedHeightMeters, 1e-9)
		assertEquals(0.0, result.trustedSeconds)
		assertEquals(0.01, result.estimatedHeightMeters, 1e-9)
	}
}
