package dev.slimevr.tracking.processor.adaptive

import org.junit.jupiter.api.Test
import org.junit.jupiter.api.io.TempDir
import java.nio.file.Files
import java.nio.file.Path
import kotlin.test.assertEquals
import kotlin.test.assertFalse
import kotlin.test.assertNull
import kotlin.test.assertTrue

class CalibrationStoreTest {
	private fun trained(id: String, rate: Double = 0.0004): CalibrationProfile = CalibrationProfile(id).apply { check(update(20.0, rate, 60.0, 0.95)) }

	@Test
	fun savesAndLoadsValidatedSnapshotUsingHashedFilename(@TempDir directory: Path) {
		CalibrationStore(directory).use { store ->
			val original = trained("tracker serial/with private value")
			assertTrue(store.save(original).get())
			val savedFile = Files.list(directory).use { it.findFirst().orElseThrow() }
			assertFalse(savedFile.fileName.toString().contains("tracker"))
			val loaded = store.load(original.hardwareId).get()
			assertEquals(original.hardwareId, loaded?.hardwareId)
			assertEquals(original.predictRate(20.0), loaded?.predictRate(20.0))
		}
	}

	@Test
	fun corruptOrOversizedFilesAreRejectedGracefully(@TempDir directory: Path) {
		CalibrationStore(directory).use { store ->
			val profile = trained("broken-device")
			assertTrue(store.save(profile).get())
			val file = Files.list(directory).use { it.findFirst().orElseThrow() }
			Files.writeString(file, "{broken json")
			assertNull(store.load(profile.hardwareId).get())
			Files.write(file, ByteArray(CalibrationStore.DEFAULT_MAX_FILE_BYTES + 1))
			assertNull(store.load(profile.hardwareId).get())
		}
	}

	@Test
	fun fileLimitAndClearApplyOnlyToPersistence(@TempDir directory: Path) {
		CalibrationStore(directory, maxFiles = 1).use { store ->
			val first = trained("first-device")
			val second = trained("second-device")
			assertTrue(store.save(first).get())
			assertFalse(store.save(second).get())
			assertTrue(store.clear(first.hardwareId).get())
			assertNull(store.load(first.hardwareId).get())
			assertEquals(0.0004, first.predictRate(20.0)!!, 1e-9)
			assertTrue(store.save(second).get())
		}
	}

	@Test
	fun unrelatedJsonDoesNotConsumeCalibrationFileLimit(@TempDir directory: Path) {
		Files.writeString(directory.resolve("notes.json"), "keep")
		CalibrationStore(directory, maxFiles = 1).use { store ->
			assertTrue(store.save(trained("first-device")).get())
			assertFalse(store.save(trained("second-device")).get())
			assertTrue(store.clearAllKnownProfiles().get())
			assertTrue(Files.exists(directory.resolve("notes.json")))
		}
	}

	@Test
	fun jacksonTreeCodecDoesNotDependOnKotlinDataClassModule(@TempDir directory: Path) {
		CalibrationStore(directory).use { store ->
			val profile = trained("dto-device")
			assertTrue(store.save(profile).get())
			val text = Files.readString(Files.list(directory).use { it.findFirst().orElseThrow() })
			assertTrue(text.contains("\"schemaVersion\":1"))
			assertEquals(0.0004, store.load(profile.hardwareId).get()?.predictRate(20.0)!!, 1e-9)
		}
	}

	@Test
	fun clearAllRemovesOnlyKnownProfileFiles(@TempDir directory: Path) {
		CalibrationStore(directory).use { store ->
			val profile = trained("clear-all-device")
			assertTrue(store.save(profile).get())
			val unrelated = directory.resolve("notes.json")
			Files.writeString(unrelated, "keep this file")
			assertTrue(store.clearAllKnownProfiles().get())
			assertNull(store.load(profile.hardwareId).get())
			assertTrue(Files.exists(unrelated))
		}
	}
}
