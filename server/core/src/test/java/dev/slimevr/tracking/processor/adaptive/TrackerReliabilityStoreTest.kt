package dev.slimevr.tracking.processor.adaptive

import org.junit.jupiter.api.Test
import org.junit.jupiter.api.io.TempDir
import java.nio.file.Files
import java.nio.file.Path
import kotlin.test.assertEquals
import kotlin.test.assertFalse
import kotlin.test.assertNull
import kotlin.test.assertTrue

class TrackerReliabilityStoreTest {
	@Test
	fun storesOnlyMatureProfilesWithHashedNamesAndNoRawHardwareKey(@TempDir directory: Path) {
		val key = "private-device-origin:serial-identifier:tracker-2"
		val profile = trained(key)
		TrackerReliabilityStore(directory).use { store ->
			assertTrue(store.save(profile).get())
			val file = Files.list(directory).use { it.findFirst().orElseThrow() }
			assertTrue(file.fileName.toString().matches(Regex("reliability-[0-9a-f]{64}\\.json")))
			assertFalse(Files.readString(file).contains(key))
			val loaded = store.load(key).get()
			assertEquals(profile.snapshot().copy(continuousTrustedSeconds = 0.0), loaded?.snapshot())
			assertTrue(loaded?.isMature == true)
		}
	}

	@Test
	fun immatureCorruptAndOversizedProfilesAreRejected(@TempDir directory: Path) {
		TrackerReliabilityStore(directory, maxFileBytes = 512).use { store ->
			val immature = TrackerReliabilityProfile("immature-key")
			assertTrue(immature.observe(0.1, 0.9, 0, true, true))
			assertFalse(store.save(immature).get())

			val trained = trained("corrupt-key")
			assertTrue(store.save(trained).get())
			val file = Files.list(directory).use { entries -> entries.filter { it.fileName.toString().contains(TrackerReliabilityProfile.hashHardwareKey("corrupt-key")) }.findFirst().orElseThrow() }
			Files.writeString(file, "{broken json")
			assertNull(store.load("corrupt-key").get())
			Files.write(file, ByteArray(513))
			assertNull(store.load("corrupt-key").get())
		}
	}

	@Test
	fun fileCapClearAndClearAllAreBoundedToReliabilityFiles(@TempDir directory: Path) {
		TrackerReliabilityStore(directory, maxFiles = 1).use { store ->
			val first = trained("first-private-key")
			val second = trained("second-private-key")
			assertTrue(store.save(first).get())
			assertFalse(store.save(second).get())
			val unrelated = directory.resolve("notes.json")
			Files.writeString(unrelated, "leave this file")
			assertTrue(store.clear(hardwareKey = "first-private-key").get())
			assertNull(store.load("first-private-key").get())
			assertTrue(store.save(second).get())
			assertTrue(store.clearAllKnownProfiles().get())
			assertNull(store.load("second-private-key").get())
			assertTrue(Files.exists(unrelated))
		}
	}

	@Test
	fun invalidKeysAndUnsupportedProfileVersionsAreRejected(@TempDir directory: Path) {
		TrackerReliabilityStore(directory).use { store ->
			assertNull(store.load("").get())
			assertFalse(store.clear("").get())
			val profile = trained("version-key")
			assertTrue(store.save(profile).get())
			val file = Files.list(directory).use { it.findFirst().orElseThrow() }
			Files.writeString(file, """{"schemaVersion":99}""")
			assertNull(store.load("version-key").get())
		}
	}

	private fun trained(hardwareKey: String): TrackerReliabilityProfile {
		val profile = TrackerReliabilityProfile(hardwareKey)
		for (index in 0..3000) {
			check(profile.observe(0.15, 0.92, index * 20_000_000L, independentlySupported = true, lowMotion = true))
		}
		check(profile.isMature)
		return profile
	}
}
