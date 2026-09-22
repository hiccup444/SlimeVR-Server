package dev.slimevr.tracking.processor.adaptive

import org.junit.jupiter.api.Test
import kotlin.test.assertEquals
import kotlin.test.assertFailsWith
import kotlin.test.assertFalse
import kotlin.test.assertNotEquals
import kotlin.test.assertNotNull
import kotlin.test.assertNull
import kotlin.test.assertTrue

class TrackerReliabilityProfileTest {
	@Test
	fun onlyIndependentConfidentStationaryObservationsAccumulateTrustedTime() {
		val profile = TrackerReliabilityProfile("device:serial:tracker-1")
		assertTrue(profile.observe(0.2, 0.9, 0, true, true))
		assertTrue(profile.observe(0.2, 0.9, 20_000_000, true, true))
		assertEquals(0.02, profile.currentTrustedSeconds, 1e-9)
		assertEquals(0.02, profile.currentContinuousSeconds, 1e-9)

		assertFalse(profile.observe(0.2, 0.9, 40_000_000, false, true))
		assertEquals(0.0, profile.currentContinuousSeconds)
		assertEquals(0.02, profile.currentTrustedSeconds, 1e-9)
		assertFalse(profile.observe(0.2, 0.9, 60_000_000, true, false))
		assertFalse(profile.observe(0.2, 0.9, 80_000_000, true, true, uncertain = true))
		assertFalse(profile.observe(0.2, 0.5, 100_000_000, true, true))
		assertEquals(0.02, profile.currentTrustedSeconds, 1e-9)

		assertTrue(profile.observe(0.1, 0.95, 200_000_000, true, true))
		assertEquals(0.02, profile.currentContinuousSeconds, 1e-9)
		assertEquals(0.04, profile.currentTrustedSeconds, 1e-9)
	}

	@Test
	fun rejectsInvalidValuesAndBackwardsTimesWithoutDamagingAcceptedStatistics() {
		val profile = TrackerReliabilityProfile("device:serial:tracker-2")
		assertTrue(profile.observe(0.15, 0.9, 10_000_000, true, true))
		assertFalse(profile.observe(Double.NaN, 0.9, 20_000_000, true, true))
		assertFalse(profile.observe(Math.PI + 0.01, 0.9, 30_000_000, true, true))
		assertFalse(profile.observe(0.1, Double.POSITIVE_INFINITY, 40_000_000, true, true))
		assertFalse(profile.observe(0.1, 0.9, -1, true, true))
		assertNotNull(profile.currentResidualMeanRadians)
		assertEquals(0.15, profile.currentResidualMeanRadians!!, 1e-9)

		val backwards = TrackerReliabilityProfile("device:serial:backwards")
		assertTrue(backwards.observe(0.1, 0.9, 50_000_000, true, true))
		assertTrue(backwards.observe(0.1, 0.9, 70_000_000, true, true))
		assertFalse(backwards.observe(0.1, 0.9, 60_000_000, true, true))
		assertEquals(0.0, backwards.currentContinuousSeconds)
		assertEquals(0.02, backwards.currentTrustedSeconds, 1e-9)
	}

	@Test
	fun timeGapsBreakTheRunAndResetPreservesLearnedEvidence() {
		val profile = TrackerReliabilityProfile("device:serial:tracker-3")
		assertTrue(profile.observe(0.2, 0.9, 0, true, true))
		assertTrue(profile.observe(0.2, 0.9, 20_000_000, true, true))
		val trusted = profile.currentTrustedSeconds
		assertTrue(profile.observe(0.2, 0.9, 1_000_000_000, true, true))
		assertEquals(0.0, profile.currentContinuousSeconds)
		assertEquals(trusted, profile.currentTrustedSeconds, 1e-9)
		assertTrue(profile.observe(0.2, 0.9, 1_020_000_000, true, true))
		val snapshot = profile.snapshot()
		profile.resetTransient()
		assertEquals(0.0, profile.currentContinuousSeconds)
		assertEquals(snapshot.trustedSeconds, profile.currentTrustedSeconds)
		assertEquals(snapshot.residualMeanRadians, profile.currentResidualMeanRadians)
	}

	@Test
	fun matureProfileUsesBoundedExponentialStatisticsAndCanBeRestored() {
		val profile = TrackerReliabilityProfile("device:serial:tracker-4")
		train(profile, 0.2, 0.9)
		assertTrue(profile.isMature)
		assertEquals(60.0, profile.currentTrustedSeconds, 1e-6)
		assertEquals(60.0, profile.currentContinuousSeconds, 1e-6)
		assertEquals(0.2, profile.currentResidualMeanRadians!!, 1e-9)
		assertEquals(0.9, profile.currentConfidenceBaseline!!, 1e-9)
		assertEquals(0.0, profile.currentResidualVarianceRadiansSquared!!, 1e-12)

		assertTrue(profile.observe(0.4, 0.8, 60_020_000_000, true, true))
		assertTrue(profile.currentResidualMeanRadians!! in 0.2..0.4)
		assertTrue(profile.currentResidualVarianceRadiansSquared!! in 0.0..(Math.PI * Math.PI))
		assertTrue(profile.currentConfidenceBaseline!! in 0.8..0.9)
		assertNotEquals("device:serial:tracker-4", profile.hardwareKeyHash)

		val restored = TrackerReliabilityProfile.restore(profile.snapshot())
		assertEquals(profile.snapshot(), restored.snapshot())
		assertTrue(restored.isMature)
	}

	@Test
	fun invalidSnapshotsAndKeysAreRejected() {
		val profile = TrackerReliabilityProfile("device:serial:tracker-5")
		assertFailsWith<IllegalArgumentException> {
			TrackerReliabilityProfile.restore(
				TrackerReliabilityProfile.Snapshot("bad-hash", 60.0, 60.0, 0.1, 0.01, 0.9, 100, true),
			)
		}
		assertFailsWith<IllegalArgumentException> { TrackerReliabilityProfile("") }
		assertNull(profile.currentConfidenceBaseline)
		assertFalse(profile.isMature)
	}

	private fun train(profile: TrackerReliabilityProfile, residual: Double, confidence: Double) {
		for (index in 0..3000) {
			check(profile.observe(residual, confidence, index * 20_000_000L, independentlySupported = true, lowMotion = true))
		}
	}
}
