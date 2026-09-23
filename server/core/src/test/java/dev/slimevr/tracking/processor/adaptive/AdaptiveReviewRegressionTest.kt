package dev.slimevr.tracking.processor.adaptive

import io.github.axisangles.ktmath.Quaternion
import io.github.axisangles.ktmath.Vector3
import org.junit.jupiter.api.Test
import kotlin.test.assertEquals
import kotlin.test.assertFailsWith
import kotlin.test.assertFalse
import kotlin.test.assertTrue

class AdaptiveReviewRegressionTest {
	@Test
	fun warmStartConvergesAcrossFramesWithoutChangingMeasurement() {
		val measured = Quaternion.IDENTITY
		val targetRotation = Quaternion.rotationAroundZAxis(Math.toRadians(30.0).toFloat())
		val target = targetRotation.sandwich(Vector3(0f, -0.4f, 0f))
		var previous: Quaternion? = null
		var firstError = 0f
		var finalError = 0f
		repeat(180) { frame ->
			val segment = PoseSegment(-1, 0.4f, Vector3.NULL, measured, 0f, previous = previous)
			val result = PoseOptimizer().solve(listOf(segment), listOf(PoseAnchor(0, target, 100f)))
			assertEquals(measured, segment.measured)
			assertTrue(result.finalError <= result.initialError + 1e-6f)
			assertEquals(0.4f, result.tails[0].len(), 1e-6f)
			finalError = (result.tails[0] - target).len()
			if (frame == 0) firstError = finalError
			previous = result.rotations[0]
		}
		assertTrue(firstError > 0.1f)
		assertTrue(finalError < 0.02f, "Final endpoint error: $finalError")
	}

	@Test
	fun fixedAndFollowerRotationsIgnoreWarmStart() {
		val seed = Quaternion.rotationAroundXAxis(1f)
		val offset = Quaternion.rotationAroundZAxis(0.3f)
		val result = PoseOptimizer().solve(
			listOf(
				PoseSegment(-1, 0.2f, Vector3.NULL, Quaternion.IDENTITY, 1f, fixed = true, initialRotation = seed),
				PoseSegment(0, 0.2f, Vector3.NULL, offset, 1f, rotationSource = 0, sourceOffset = offset, initialRotation = seed),
			),
			emptyList(),
		)
		assertEquals(Quaternion.IDENTITY, result.rotations[0])
		assertTrue(result.rotations[1].angleToR(offset) < 0.001f)
	}

	@Test
	fun worseSeedFallsBackToMeasurementsAndInvalidSeedIsRejected() {
		val bad = Quaternion.rotationAroundXAxis(2f)
		val segment = PoseSegment(-1, 0.4f, Vector3.NULL, Quaternion.IDENTITY, 1f, initialRotation = bad)
		val result = PoseOptimizer().solve(listOf(segment), emptyList())
		assertEquals(Quaternion.IDENTITY, result.rotations.single())
		assertFailsWith<IllegalArgumentException> {
			PoseOptimizer().solve(listOf(segment.copy(initialRotation = Quaternion(0f, 0f, 0f, 0f))), emptyList())
		}
	}

	private fun evidence(context: Long = 1L, error: Float = 0.1f, source: DriftEvidenceSource = DriftEvidenceSource.ABSOLUTE_POSITION_CONSTRAINT) = DriftEvidence(error, source, context, 1f, 1f, 1f, 3)

	@Test
	fun shortAbsolutePauseRetainsEvidenceButNeverCountsTheGap() {
		val tracker = ResidualTracker(preserveAbsoluteEvidenceAcrossContexts = true)
		var last: DriftResidual? = null
		for (i in 0..210) last = tracker.update(evidence(), i * 100_000_000L)
		assertTrue(last!!.eligibleForLearning)
		val duration = last!!.consistentSeconds
		assertFalse(tracker.pause(21_500_000_000L, "ARM_MOTION").eligibleForLearning)
		last = tracker.update(evidence(context = 2), 22_000_000_000L)
		assertTrue(last.consistentSeconds > 19.0)
		assertTrue(last.consistentSeconds < duration)
	}

	@Test
	fun longPauseAndIncompatibleAbsoluteEvidenceResetLearning() {
		val tracker = ResidualTracker(preserveAbsoluteEvidenceAcrossContexts = true)
		for (i in 0..210) tracker.update(evidence(), i * 100_000_000L)
		tracker.pause(24_000_000_000L, "UNAVAILABLE")
		assertEquals(0.0, tracker.update(evidence(2), 24_100_000_000L).consistentSeconds)
		for (i in 242..452) tracker.update(evidence(2), i * 100_000_000L)
		assertEquals(0.0, tracker.update(evidence(3, 0.3f), 45_300_000_000L).consistentSeconds)
	}

	@Test
	fun footReferencesNeverCarryEvidenceAcrossContactContexts() {
		val tracker = ResidualTracker(preserveAbsoluteEvidenceAcrossContexts = true)
		for (i in 0..210) tracker.update(evidence(source = DriftEvidenceSource.PLANTED_CONTACT_WITH_STABLE_CHAIN), i * 100_000_000L)
		assertEquals(0.0, tracker.update(evidence(context = 2, source = DriftEvidenceSource.PLANTED_CONTACT_WITH_STABLE_CHAIN), 21_100_000_000L).consistentSeconds)
	}

	@Test
	fun learningStatisticsExcludeGapsAndUnsupportedIntervals() {
		val stats = YawLearningStatistics()
		val valid = DriftResidual(0.1f, 0.1f, 25.0, true, "SUPPORTED_PERSISTENT_RESIDUAL", DriftEvidenceSource.ABSOLUTE_POSITION_CONSTRAINT, 1L)
		stats.observe(valid, 0)
		val one = stats.observe(valid, 100_000_000L)
		assertEquals(0.1, one.learningSeconds, 1e-9)
		val gap = stats.observe(null, 10_000_000_000L)
		assertEquals(0.1, gap.observedSeconds, 1e-9)
		assertEquals(9.9, gap.secondsSinceSupportedEvidence!!, 1e-9)
		val changed = stats.observe(valid.copy(contextId = 2L), 10_100_000_000L)
		assertEquals(1L, changed.contextRestarts)
		assertEquals(0.1, changed.learningSeconds, 1e-9)
	}

	@Test
	fun footPolicyFallsBackForMissingZeroStrengthAndInvalidContacts() {
		val planted = FootContactSnapshot(FootContactState.PLANTED, Vector3.NULL, 0.95f)
		assertFalse(FootCorrectionPolicy.ownsCorrection(true, FootContactSnapshot(), 1f))
		assertFalse(FootCorrectionPolicy.ownsCorrection(true, planted, 0f))
		assertFalse(FootCorrectionPolicy.ownsCorrection(true, planted, Float.NaN))
		assertFalse(FootCorrectionPolicy.ownsCorrection(false, planted, 1f))
		assertFalse(FootCorrectionPolicy.ownsCorrection(true, planted.copy(weight = 0f), 1f))
		assertFalse(FootCorrectionPolicy.ownsCorrection(true, planted.copy(plantPosition = Vector3(Float.NaN, 0f, 0f)), 1f))
		assertTrue(FootCorrectionPolicy.ownsCorrection(true, planted, 1f))
		assertTrue(FootCorrectionPolicy.ownsCorrection(true, planted.copy(state = FootContactState.RELEASING), 1f))
	}

	@Test
	fun footFallbackAndAdaptiveCorrectionAreChosenIndependently() {
		val missing = FootContactDetector()
		val planted = FootContactDetector()
		for (i in 0..30) planted.update(Vector3.NULL, Quaternion.IDENTITY, 0f, Vector3.NULL, true, i * 20_000_000L)
		val raw = Vector3(0.02f, 0f, 0f)
		val legacy = Vector3(0.01f, 0f, 0f)
		assertEquals(legacy, FootCorrectionPolicy.select(raw, legacy, missing, true, 1f))
		assertEquals(legacy, FootCorrectionPolicy.select(raw, legacy, planted, true, 0f))
		assertEquals(planted.correct(raw, 1f), FootCorrectionPolicy.select(raw, legacy, planted, true, 1f))
	}
}
