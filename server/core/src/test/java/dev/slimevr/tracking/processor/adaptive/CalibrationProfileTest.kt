package dev.slimevr.tracking.processor.adaptive

import org.junit.jupiter.api.Test
import kotlin.test.assertEquals
import kotlin.test.assertFailsWith
import kotlin.test.assertFalse
import kotlin.test.assertNull
import kotlin.test.assertTrue

class CalibrationProfileTest {
	private fun train(profile: CalibrationProfile, temperature: Double, rate: Double, seconds: Double = 60.0) {
		assertTrue(profile.update(temperature, rate, seconds, 0.95))
	}

	@Test
	fun interpolatesBetweenMatureTemperatureBuckets() {
		val profile = CalibrationProfile("sensor-a")
		train(profile, 20.0, 0.0002)
		train(profile, 30.0, 0.0012)
		assertEquals(0.0007, profile.predictRate(25.0)!!, 1e-9)
		assertNull(profile.predictRate(15.0))
	}

	@Test
	fun wideUntrainedTemperatureGapDoesNotProduceAProjectedRate() {
		val profile = CalibrationProfile("sensor-wide-gap")
		train(profile, 20.0, 0.0002)
		train(profile, 40.0, 0.0012)
		assertEquals(0.0002, profile.predictRate(22.0)!!, 1e-9)
		assertEquals(0.0012, profile.predictRate(38.0)!!, 1e-9)
		assertNull(profile.predictRate(30.0))
	}

	@Test
	fun requiresTrustedAccumulatedEvidence() {
		val profile = CalibrationProfile("sensor-a")
		assertTrue(profile.update(20.0, 0.0004, 59.9, 0.95))
		assertNull(profile.predictRate(20.0))
		assertTrue(profile.update(20.0, 0.0004, 0.1, 0.95))
		assertEquals(0.0004, profile.predictRate(20.0)!!, 1e-9)
		assertEquals(0.0004, profile.predictRate(22.4)!!, 1e-9)
		assertEquals(0.0004, profile.predictRate(17.6)!!, 1e-9)
		assertNull(profile.predictRate(22.6))
		assertNull(profile.predictRate(17.4))
	}

	@Test
	fun rejectsInvalidInputsAndRobustlyIgnoresOutlier() {
		val profile = CalibrationProfile("sensor-a")
		train(profile, 20.0, 0.0003)
		assertFalse(profile.update(Double.NaN, 0.0003, 60.0, 0.95))
		assertFalse(profile.update(20.0, Double.POSITIVE_INFINITY, 60.0, 0.95))
		assertFalse(profile.update(20.0, 0.0003, 60.0, 0.4))
		assertFalse(profile.update(20.0, 0.02, 60.0, 0.95))
		assertFalse(profile.update(20.0, 0.003, 60.0, 0.95))
		assertEquals(0.0003, profile.predictRate(20.0)!!, 1e-9)
	}

	@Test
	fun bucketCountIsBoundedAndSnapshotIsDetached() {
		val profile = CalibrationProfile("sensor-a", maxBuckets = 2)
		train(profile, 10.0, 0.0001)
		train(profile, 20.0, 0.0002)
		assertFalse(profile.update(30.0, 0.0003, 60.0, 0.95))
		val snapshot = profile.snapshot()
		assertEquals("sensor-a", snapshot.hardwareId)
		assertEquals(2, snapshot.buckets.size)
		assertEquals(0.0001, snapshot.buckets[0].rateRadiansPerSecond, 1e-9)
		assertEquals(20.0, snapshot.buckets[1].temperatureCelsius)
	}

	@Test
	fun profilesKeepHardwareEvidenceSeparateAndResetStartsEmpty() {
		val first = CalibrationProfile("sensor-a")
		val second = CalibrationProfile("sensor-b")
		train(first, 20.0, 0.0004)
		assertNull(second.predictRate(20.0))
		first.reset()
		assertNull(first.predictRate(20.0))
		assertTrue(first.snapshot().buckets.isEmpty())
	}

	@Test
	fun restoresOnlyValidatedMatureSnapshots() {
		val profile = CalibrationProfile("sensor-a")
		train(profile, 20.0, 0.0004)
		val restored = CalibrationProfile.restore(profile.snapshot())
		assertEquals(profile.predictRate(20.0), restored.predictRate(20.0))
		assertFailsWith<IllegalArgumentException> {
			CalibrationProfile.restore(profile.snapshot().copy(minimumObservationSeconds = 1.0))
		}
		assertFailsWith<IllegalArgumentException> {
			CalibrationProfile.restore(
				profile.snapshot().copy(
					buckets = listOf(
						profile.snapshot().buckets.single().copy(rateRadiansPerSecond = 0.02),
					),
				),
			)
		}
	}
}
