package dev.slimevr.tracking.processor.adaptive

import dev.slimevr.tracking.trackers.TrackerStatus
import io.github.axisangles.ktmath.Quaternion
import io.github.axisangles.ktmath.Vector3
import org.junit.jupiter.api.Test
import kotlin.test.assertEquals
import kotlin.test.assertFalse
import kotlin.test.assertTrue

class IndependentMotionAndReliabilityTest {
	@Test
	fun controllerMotionInsideTheSameReachConeIsNotTargetMotionEvidence() {
		val witness = ReachMotionWitness()
		for (i in 0..30) {
			val reach = Quaternion.rotationAroundYAxis(i * 0.003f).sandwich(Vector3(0f, 0f, 0.6f))
			assertEquals(0.0, witness.observe(reach, Quaternion.IDENTITY, 0.3f, 0.3f, i * 100_000_000L, true))
		}
	}

	@Test
	fun disjointReachConesProvideIndependentMotionEvidence() {
		val witness = ReachMotionWitness()
		assertEquals(0.0, witness.observe(Vector3(0f, 0f, 0.6f), Quaternion.IDENTITY, 0.3f, 0.3f, 0L, true))
		assertTrue(witness.observe(Vector3(0.6f, 0f, 0f), Quaternion.IDENTITY, 0.3f, 0.3f, 100_000_000L, true) >= 0.8)
	}

	@Test
	fun motionGapsAndUntrustedOrBentArmsDiscardTheWitness() {
		val witness = ReachMotionWitness()
		witness.observe(Vector3(0f, 0f, 0.6f), Quaternion.IDENTITY, 0.3f, 0.3f, 0L, true)
		assertEquals(0.0, witness.observe(Vector3(0.6f, 0f, 0f), Quaternion.IDENTITY, 0.3f, 0.3f, 1_000_000_000L, true))
		assertEquals(0.0, witness.observe(Vector3(0f, 0f, 0.6f), Quaternion.rotationAroundYAxis(0.1f), 0.3f, 0.3f, 1_100_000_000L, true))
		assertEquals(0.0, witness.observe(Vector3(0.6f, 0f, 0f), Quaternion.IDENTITY, 0.3f, 0.3f, 1_200_000_000L, false))
		assertEquals(0.0, witness.observe(Vector3(0.3f, 0f, 0f), Quaternion.IDENTITY, 0.3f, 0.3f, 1_300_000_000L, true))
		assertEquals(0.0, witness.observe(Vector3(Float.NaN, 0f, 0f), Quaternion.IDENTITY, 0.3f, 0.3f, 1_400_000_000L, true))
	}

	@Test
	fun witnessCanDriveFrozenDetectionWithoutFlaggingAStationaryReach() {
		fun run(changedReach: Boolean): TrackerHealthDiagnostic {
			val witness = ReachMotionWitness()
			val monitor = TrackerHealthMonitor()
			var result: TrackerHealthDiagnostic? = null
			for (i in 0..40) {
				val now = i * 100_000_000L
				val reach = if (changedReach && i > 0) Vector3(0.6f, 0f, 0f) else Vector3(0f, 0f, 0.6f)
				val movement = witness.observe(reach, Quaternion.IDENTITY, 0.3f, 0.3f, now, true)
				val sample = TelemetrySample(
					1, "arm", null, TrackerStatus.OK, Quaternion.IDENTITY, Quaternion.IDENTITY,
					null, Vector3.NULL, null, 0L, null, 0f, continuousObservation = true, accelerationAgeNanos = 0L,
				)
				result = monitor.observe(sample, now, TrackerMotionContext(movement, now))
			}
			return result!!
		}
		assertTrue(run(true).suspectedFrozen)
		assertFalse(run(false).suspectedFrozen)
	}

	@Test
	fun immatureAndUnsupportedReliabilityNeverReduceConfidence() {
		val profile = TrackerReliabilityProfile("test-device")
		for (i in 0..800) profile.observe(Math.toRadians(20.0), 0.95, i * 100_000_000L, false, true)
		assertEquals(1f, ReliabilityConfidence.multiplier(profile.snapshot()))
		for (i in 801..900) profile.observe(Math.toRadians(20.0), 0.95, i * 100_000_000L, true, true)
		assertFalse(profile.isMature)
		assertEquals(1f, ReliabilityConfidence.multiplier(profile.snapshot()))
	}

	@Test
	fun matureReliabilityIsOnlyABoundedDownwardPrior() {
		fun multiplier(degrees: Double): Float {
			val profile = TrackerReliabilityProfile("test-$degrees")
			for (i in 0..800) profile.observe(Math.toRadians(degrees), 0.95, i * 100_000_000L, true, true)
			assertTrue(profile.isMature)
			return ReliabilityConfidence.multiplier(profile.snapshot())
		}
		assertEquals(1f, multiplier(0.0))
		assertEquals(0.9f, multiplier(7.5), 0.0001f)
		assertEquals(0.8f, multiplier(25.0), 0.0001f)
	}
}
