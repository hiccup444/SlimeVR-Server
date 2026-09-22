package dev.slimevr.tracking.processor.adaptive

import org.junit.jupiter.api.Test
import kotlin.test.assertEquals
import kotlin.test.assertFalse
import kotlin.test.assertTrue

class DriftLearningTest {
	private fun evidence(degrees: Double = 8.0) = DriftEvidence(Math.toRadians(degrees).toFloat(), DriftEvidenceSource.PLANTED_CONTACT_WITH_STABLE_CHAIN, 1, 0.95f, 0.95f, 0.95f, 3)

	@Test
	fun persistentSupportedResidualBecomesEligibleAfterObservation() {
		val tracker = ResidualTracker()
		for (step in 0 until 1000) assertFalse(tracker.update(evidence(), step * 20_000_000L).eligibleForLearning)
		assertTrue(tracker.update(evidence(), 20_000_000_000L).eligibleForLearning)
	}

	@Test
	fun independentEvidenceAndContinuousContextAreRequired() {
		val tracker = ResidualTracker(minimumSeconds = 1.0)
		for (step in 0..100) assertFalse(tracker.update(evidence().copy(source = DriftEvidenceSource.JOINT_PRIOR), step * 20_000_000L).eligibleForLearning)
		for (step in 101..160) tracker.update(evidence(), step * 20_000_000L)
		assertFalse(tracker.update(evidence().copy(contextId = 2), 3_220_000_000L).eligibleForLearning)
		assertEquals("MOTION", tracker.update(evidence().copy(highMotion = true), 3_240_000_000L).reason)
		assertEquals("POSSIBLE_MOUNTING_SHIFT", tracker.update(evidence(40.0), 3_260_000_000L).reason)
	}

	@Test
	fun disagreementAloneAndInvalidNumbersCannotLearn() {
		val tracker = ResidualTracker(0.0)
		assertFalse(tracker.update(evidence().copy(independentConstraints = 1), 0).eligibleForLearning)
		assertFalse(tracker.update(evidence().copy(errorRadians = Float.NaN), 1).eligibleForLearning)
		assertFalse(tracker.update(evidence().copy(poseConfidence = 0.2f), 2).eligibleForLearning)
		assertEquals(0f, tracker.update(evidence().copy(errorRadians = Float.NaN), 3).errorRadians)
	}

	@Test
	fun correctionIsRateLimitedAndStopsAtTargetWithoutDoubleCounting() {
		val corrector = AdaptiveYawCorrector()
		val target = Math.toRadians(2.0).toFloat()
		val residual = DriftResidual(target, target, 30.0, true, "test")
		for (step in 0..100) corrector.update(residual, step * 10_000_000L)
		assertEquals(Math.toRadians(0.2).toFloat(), corrector.biasRadians, 0.00001f)
		for (step in 101..2000) corrector.update(residual, step * 10_000_000L)
		assertEquals(target, corrector.biasRadians, 0.00001f)
		val frozen = corrector.biasRadians
		corrector.update(residual.copy(eligibleForLearning = false, filteredErrorRadians = -target), 20_010_000_000L)
		assertEquals(frozen, corrector.biasRadians)
		corrector.reset()
		assertEquals(0f, corrector.biasRadians)
	}

	@Test
	fun gapsAndNonForwardTimeCannotCauseLargeCorrection() {
		val corrector = AdaptiveYawCorrector()
		val residual = DriftResidual(1f, 1f, 30.0, true, "test")
		corrector.update(residual, 0)
		corrector.update(residual, 1_000_000_000)
		assertEquals(0f, corrector.biasRadians)
		corrector.update(residual, -1)
		assertEquals(0f, corrector.biasRadians)
	}
}
