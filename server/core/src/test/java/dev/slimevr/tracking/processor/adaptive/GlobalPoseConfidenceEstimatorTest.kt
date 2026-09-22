package dev.slimevr.tracking.processor.adaptive

import dev.slimevr.tracking.trackers.TrackerRole
import org.junit.jupiter.api.Test
import kotlin.test.assertEquals
import kotlin.test.assertFalse
import kotlin.test.assertTrue

class GlobalPoseConfidenceEstimatorTest {
	private val estimator = GlobalPoseConfidenceEstimator()
	private val roles = setOf(TrackerRole.HEAD, TrackerRole.CHEST, TrackerRole.LEFT_HAND)

	private fun trustedInput() = GlobalPoseConfidenceInput(
		trackerConfidences = roles.associateWith { 0.96f },
		requiredRoles = roles,
		absoluteAnchorAvailable = true,
		absoluteAnchorQuality = 0.98f,
		independentChainDisagreementsRadians = listOf(Math.toRadians(0.5), Math.toRadians(1.0)),
		independentResidualsRadians = listOf(Math.toRadians(2.0), Math.toRadians(4.0)),
		motionUncertainty = 0.03f,
	)

	@Test
	fun independentFreshEvidenceProducesHighConfidenceWithoutContactRequirement() {
		val confidence = estimator.estimate(trustedInput())

		assertTrue(confidence.score > 0.8f, "score was ${confidence.score}")
		assertTrue(confidence.learningEligible)
	}

	@Test
	fun missingAbsoluteAnchorCannotBeAveragedAway() {
		val confidence = estimator.estimate(trustedInput().copy(absoluteAnchorAvailable = false))

		assertFalse(confidence.learningEligible)
		assertTrue(confidence.score < 0.5f)
		assertTrue("ABSOLUTE_ANCHOR_MISSING" in confidence.reasons)
	}

	@Test
	fun absentIndependentResidualCoverageIsNotTreatedAsPerfect() {
		val confidence = estimator.estimate(trustedInput().copy(independentResidualsRadians = emptyList()))

		assertFalse(confidence.learningEligible)
		assertEquals(0f, confidence.score)
		assertTrue("INDEPENDENT_RESIDUAL_COVERAGE_LOW" in confidence.reasons)
	}

	@Test
	fun missingRequiredRoleAndContradictoryChainsBlockLearning() {
		val missingRole = estimator.estimate(trustedInput().copy(trackerConfidences = mapOf(TrackerRole.HEAD to 0.98f)))
		assertFalse(missingRole.learningEligible)
		assertTrue("REQUIRED_TRACKER_ROLE_MISSING" in missingRole.reasons)
		val staleEssentialRole = estimator.estimate(trustedInput().copy(trackerConfidences = trustedInput().trackerConfidences + (TrackerRole.CHEST to 0.2f)))
		assertFalse(staleEssentialRole.learningEligible)
		assertTrue("TRACKER_CONFIDENCE_LOW" in staleEssentialRole.reasons)

		val contradictory = estimator.estimate(
			trustedInput().copy(independentChainDisagreementsRadians = listOf(Math.toRadians(20.0), Math.toRadians(22.0))),
		)
		assertFalse(contradictory.learningEligible)
		assertTrue("INDEPENDENT_CHAINS_DISAGREE" in contradictory.reasons)
	}

	@Test
	fun invalidEvidenceCannotBeIgnoredWhenEnoughOtherSamplesRemain() {
		val badChain = estimator.estimate(
			trustedInput().copy(independentChainDisagreementsRadians = listOf(Math.toRadians(0.5), Math.toRadians(1.0), Double.NaN)),
		)
		assertFalse(badChain.learningEligible)
		assertTrue("INVALID_CHAIN_DISAGREEMENT" in badChain.reasons)

		val badResidual = estimator.estimate(
			trustedInput().copy(independentResidualsRadians = listOf(Math.toRadians(2.0), Math.toRadians(4.0), Double.NaN)),
		)
		assertFalse(badResidual.learningEligible)
		assertTrue("INVALID_INDEPENDENT_RESIDUAL" in badResidual.reasons)

		val badAnchor = estimator.estimate(trustedInput().copy(absoluteAnchorQuality = 1.2f))
		assertFalse(badAnchor.learningEligible)
		assertTrue("ABSOLUTE_ANCHOR_QUALITY_INVALID" in badAnchor.reasons)
	}

	@Test
	fun motionUncertaintyBlocksPersistentLearning() {
		val confidence = estimator.estimate(trustedInput().copy(motionUncertainty = 0.8f))

		assertFalse(confidence.learningEligible)
		assertTrue("MOTION_UNCERTAINTY_HIGH" in confidence.reasons)
	}

	@Test
	fun contactReliabilityIsRequiredOnlyForContactBasedContexts() {
		val armContext = estimator.estimate(trustedInput())
		assertTrue(armContext.learningEligible)

		val missingContact = estimator.estimate(trustedInput().copy(contactRequiredForLearning = true))
		assertFalse(missingContact.learningEligible)
		assertTrue("CONTACT_EVIDENCE_MISSING" in missingContact.reasons)

		val supportedContact = estimator.estimate(trustedInput().copy(contactRequiredForLearning = true, contactReliability = 0.97f))
		assertTrue(supportedContact.learningEligible)
	}
}
