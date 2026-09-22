package dev.slimevr.tracking.processor.adaptive

import org.junit.jupiter.api.Test
import org.junit.jupiter.api.io.TempDir
import java.nio.file.Path
import java.util.concurrent.TimeUnit
import kotlin.test.assertEquals
import kotlin.test.assertFalse
import kotlin.test.assertNotNull
import kotlin.test.assertNull
import kotlin.test.assertTrue

class CalibrationLearnerTest {
	private fun withLearner(
		directory: Path,
		queueCapacity: Int = CalibrationStore.DEFAULT_QUEUE_CAPACITY,
		maxProfiles: Int = CalibrationLearner.MAX_CACHED_PROFILES,
		block: (CalibrationStore, CalibrationLearner) -> Unit,
	) {
		val store = CalibrationStore(directory, queueCapacity = queueCapacity)
		val learner = CalibrationLearner(store, maxProfiles)
		try {
			block(store, learner)
		} finally {
			learner.close()
			check(store.awaitClosed(5_000)) { "Calibration store did not finish queued work" }
		}
	}

	private fun residual(error: Float, seconds: Double, eligible: Boolean = true) = DriftResidual(error, error, seconds, eligible, if (eligible) "SUPPORTED_PERSISTENT_RESIDUAL" else "MOTION")

	@Test
	fun learnsOnlyFromContinuousTrustedResidualSlopeAfterAsyncLoad(@TempDir directory: Path) {
		withLearner(directory) { store, learner ->
			val hardwareId = "tracker-a"
			assertNull(learner.observe(hardwareId, 20.0, residual(0f, 20.0), 0L))
			assertTrue(learner.readiness(hardwareId).get(2, TimeUnit.SECONDS))
			assertNull(learner.observe(hardwareId, 20.0, residual(0f, 20.0), 100_000_000L))
			val expectedRate = 0.0005
			var prediction: Double? = null
			for (step in 1..601) {
				val elapsed = step * 0.1
				val error = (expectedRate * elapsed).toFloat()
				prediction = learner.observe(
					hardwareId,
					20.0,
					residual(error, 20.0 + elapsed),
					100_000_000L + step * 100_000_000L,
				)
			}
			assertEquals(expectedRate, prediction!!, 0.00002)
			assertTrue(learner.diagnostics(hardwareId)!!.trustedObservationSeconds >= 60.0)
			val persisted = store.load(hardwareId).get(2, TimeUnit.SECONDS)
			assertEquals(expectedRate, assertNotNull(persisted).predictRate(20.0)!!, 0.00002)
		}
	}

	@Test
	fun ineligibleAndMissingTemperatureBreakTheTemporalSlope(@TempDir directory: Path) {
		withLearner(directory) { _, learner ->
			val id = "tracker-b"
			learner.readiness(id).get(2, TimeUnit.SECONDS)
			learner.observe(id, 20.0, residual(0f, 20.0), 0L)
			learner.observe(id, 20.0, residual(0f, 20.1, eligible = false), 100_000_000L)
			learner.observe(id, 20.0, residual(0f, 20.2), 200_000_000L)
			learner.observe(id, 20.0, residual(0.00005f, 20.3), 300_000_000L)
			assertEquals(0.1, learner.diagnostics(id)!!.trustedObservationSeconds, 1e-6)
			learner.observe(id, null, residual(0.0001f, 20.4), 400_000_000L)
			learner.observe(id, 20.0, residual(0.00015f, 20.5), 500_000_000L)
			assertEquals(0.1, learner.diagnostics(id)!!.trustedObservationSeconds, 1e-6)
		}
	}

	@Test
	fun transientResetKeepsLearnedModelAndClearRemovesIt(@TempDir directory: Path) {
		withLearner(directory) { store, learner ->
			val id = "tracker-c"
			learner.readiness(id).get(2, TimeUnit.SECONDS)
			learner.observe(id, 20.0, residual(0f, 20.0), 0L)
			for (step in 1..601) {
				val elapsed = step * 0.1
				learner.observe(id, 20.0, residual((0.0004 * elapsed).toFloat(), 20.0 + elapsed), step * 100_000_000L)
			}
			assertTrue(learner.observe(id, 20.0, null, 60_200_000_000L) != null)
			learner.resetTransient()
			assertTrue(learner.observe(id, 20.0, null, 60_300_000_000L) != null)
			val otherProfile = CalibrationProfile("profile-from-previous-session").apply {
				check(update(25.0, 0.0002, 60.0, 0.95))
			}
			assertTrue(store.save(otherProfile).get(2, TimeUnit.SECONDS))
			assertTrue(learner.clearLearned().get(2, TimeUnit.SECONDS))
			assertNull(learner.observe(id, 20.0, null, 60_400_000_000L))
			assertNull(store.load(id).get(2, TimeUnit.SECONDS))
			assertNull(store.load(otherProfile.hardwareId).get(2, TimeUnit.SECONDS))
		}
	}

	@Test
	fun missingModelLoadCompletesWithoutBlockingPoseUpdates(@TempDir directory: Path) {
		withLearner(directory, queueCapacity = 1) { _, learner ->
			val id = "tracker-d"
			assertNull(learner.observe(id, 20.0, residual(0f, 20.0), 0L))
			assertTrue(learner.readiness(id).get(2, TimeUnit.SECONDS))
			val state = learner.diagnostics(id)
			assertTrue(state!!.loaded)
			assertFalse(state.loadFailed)
		}
	}

	@Test
	fun cacheEvictionIsBoundedAndCompletesPendingReadiness(@TempDir directory: Path) {
		withLearner(directory, maxProfiles = 1) { _, learner ->
			val first = learner.readiness("tracker-first")
			learner.readiness("tracker-second").get(2, TimeUnit.SECONDS)
			assertFalse(first.get(2, TimeUnit.SECONDS))
			assertNull(learner.diagnostics("tracker-first"))
			assertTrue(learner.diagnostics("tracker-second")!!.loaded)
		}
	}
}
