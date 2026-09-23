package dev.slimevr.tracking.processor.adaptive

import org.junit.jupiter.api.Test
import org.junit.jupiter.api.io.TempDir
import java.nio.file.Path
import java.util.concurrent.TimeUnit
import kotlin.test.assertEquals
import kotlin.test.assertNull
import kotlin.test.assertTrue

class TrackerReliabilityLearnerTest {
	@Test
	fun clearingLearnedHistoryReturnsToNeutralWithoutReloadingOldProfile(@TempDir directory: Path) {
		val key = "test-device:serial:sensor"
		val profile = TrackerReliabilityProfile(key)
		for (index in 0..3000) assertTrue(profile.observe(0.2, 0.95, index * 20_000_000L, true, true))
		val store = TrackerReliabilityStore(directory)
		assertTrue(store.save(profile).get(2, TimeUnit.SECONDS))
		val learner = TrackerReliabilityLearner(store)
		try {
			var multiplier = learner.multiplier(key)
			val deadline = System.nanoTime() + 2_000_000_000L
			while (multiplier == 1f && System.nanoTime() < deadline) {
				Thread.sleep(5)
				multiplier = learner.multiplier(key)
			}
			assertTrue(multiplier < 1f)
			assertTrue(learner.clearLearned().get(2, TimeUnit.SECONDS))
			assertEquals(1f, learner.multiplier(key))
			assertNull(store.load(key).get(2, TimeUnit.SECONDS))
		} finally {
			learner.close()
			assertTrue(store.awaitClosed(5_000))
		}
	}
}
