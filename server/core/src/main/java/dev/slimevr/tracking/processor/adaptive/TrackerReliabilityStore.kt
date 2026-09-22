package dev.slimevr.tracking.processor.adaptive

import com.fasterxml.jackson.databind.JsonNode
import com.fasterxml.jackson.databind.ObjectMapper
import java.nio.file.AtomicMoveNotSupportedException
import java.nio.file.Files
import java.nio.file.LinkOption
import java.nio.file.Path
import java.nio.file.StandardCopyOption
import java.util.concurrent.ArrayBlockingQueue
import java.util.concurrent.CompletableFuture
import java.util.concurrent.RejectedExecutionException
import java.util.concurrent.ThreadPoolExecutor
import java.util.concurrent.TimeUnit

/** Bounded asynchronous storage for mature hardware-keyed reliability snapshots. */
class TrackerReliabilityStore(
	private val directory: Path,
	private val mapper: ObjectMapper = ObjectMapper(),
	private val maxFiles: Int = DEFAULT_MAX_FILES,
	private val maxFileBytes: Int = DEFAULT_MAX_FILE_BYTES,
	queueCapacity: Int = DEFAULT_QUEUE_CAPACITY,
) : AutoCloseable {
	private val executor = ThreadPoolExecutor(
		1,
		1,
		0L,
		TimeUnit.MILLISECONDS,
		ArrayBlockingQueue(queueCapacity),
		{ runnable -> Thread(runnable, "tracker-reliability-store").apply { isDaemon = true } },
		ThreadPoolExecutor.AbortPolicy(),
	)

	init {
		require(maxFiles in 1..MAX_ALLOWED_FILES)
		require(maxFileBytes in 128..MAX_ALLOWED_FILE_BYTES)
		require(queueCapacity in 1..MAX_ALLOWED_QUEUE_CAPACITY)
	}

	/** Only a validated mature profile is saved. The write never runs on the caller thread. */
	fun save(profile: TrackerReliabilityProfile): CompletableFuture<Boolean> {
		val snapshot = profile.snapshot()
		if (!snapshot.mature) return CompletableFuture.completedFuture(false)
		val validated = try {
			TrackerReliabilityProfile.restore(snapshot)
		} catch (_: IllegalArgumentException) {
			return CompletableFuture.completedFuture(false)
		}
		val trustedSnapshot = validated.snapshot()
		return submit {
			Files.createDirectories(directory)
			val destination = profilePath(trustedSnapshot.hardwareKeyHash)
			if (!Files.exists(destination, LinkOption.NOFOLLOW_LINKS) && countProfiles() >= maxFiles) return@submit false
			val bytes = encode(trustedSnapshot)
			if (bytes.size > maxFileBytes) return@submit false
			val temp = Files.createTempFile(directory, ".reliability-", ".tmp")
			try {
				Files.write(temp, bytes)
				try {
					Files.move(temp, destination, StandardCopyOption.ATOMIC_MOVE, StandardCopyOption.REPLACE_EXISTING)
				} catch (_: AtomicMoveNotSupportedException) {
					Files.move(temp, destination, StandardCopyOption.REPLACE_EXISTING)
				}
				true
			} finally {
				Files.deleteIfExists(temp)
			}
		}
	}

	/** Loads by hardware key and rejects malformed, oversized, or mismatched snapshots. */
	fun load(hardwareKey: String): CompletableFuture<TrackerReliabilityProfile?> {
		val hash = try {
			TrackerReliabilityProfile.hashHardwareKey(hardwareKey)
		} catch (_: IllegalArgumentException) {
			return CompletableFuture.completedFuture(null)
		}
		return submit {
			val file = profilePath(hash)
			if (!Files.isRegularFile(file, LinkOption.NOFOLLOW_LINKS)) return@submit null
			try {
				val bytes = Files.newInputStream(file).use { it.readNBytes(maxFileBytes + 1) }
				if (bytes.size > maxFileBytes) return@submit null
				val snapshot = decode(bytes) ?: return@submit null
				if (snapshot.hardwareKeyHash != hash) return@submit null
				TrackerReliabilityProfile.restore(snapshot)
			} catch (_: Exception) {
				null
			}
		}
	}

	fun clear(hardwareKey: String): CompletableFuture<Boolean> {
		val hash = try {
			TrackerReliabilityProfile.hashHardwareKey(hardwareKey)
		} catch (_: IllegalArgumentException) {
			return CompletableFuture.completedFuture(false)
		}
		return submit {
			Files.deleteIfExists(profilePath(hash))
			true
		}
	}

	/** Removes only this store's bounded set of hashed reliability snapshots. */
	fun clearAllKnownProfiles(): CompletableFuture<Boolean> = submit {
		if (!Files.exists(directory, LinkOption.NOFOLLOW_LINKS)) return@submit true
		if (!Files.isDirectory(directory, LinkOption.NOFOLLOW_LINKS)) return@submit false
		val profiles = mutableListOf<Path>()
		Files.newDirectoryStream(directory).use { entries ->
			var scanned = 0
			for (entry in entries) {
				if (++scanned > maxFiles * 2) return@submit false
				if (!PROFILE_FILENAME_REGEX.matches(entry.fileName.toString())) continue
				if (Files.isRegularFile(entry, LinkOption.NOFOLLOW_LINKS)) profiles.add(entry)
				if (profiles.size > maxFiles) return@submit false
			}
		}
		for (profile in profiles) Files.deleteIfExists(profile)
		true
	}

	private fun countProfiles(): Int {
		if (!Files.isDirectory(directory, LinkOption.NOFOLLOW_LINKS)) return 0
		Files.newDirectoryStream(directory).use { entries ->
			var count = 0
			var scanned = 0
			for (entry in entries) {
				if (++scanned > maxFiles * 2) return maxFiles
				if (PROFILE_FILENAME_REGEX.matches(entry.fileName.toString()) && Files.isRegularFile(entry, LinkOption.NOFOLLOW_LINKS) && ++count >= maxFiles) return count
			}
			return count
		}
	}

	private fun profilePath(hash: String): Path = directory.resolve("reliability-$hash.json")

	private fun encode(snapshot: TrackerReliabilityProfile.Snapshot): ByteArray {
		val root = mapper.createObjectNode()
		root.put("schemaVersion", TrackerReliabilityProfile.SCHEMA_VERSION)
		root.put("hardwareKeyHash", snapshot.hardwareKeyHash)
		root.put("trustedSeconds", snapshot.trustedSeconds)
		root.put("continuousTrustedSeconds", snapshot.continuousTrustedSeconds)
		root.put("residualMeanRadians", snapshot.residualMeanRadians)
		root.put("residualVarianceRadiansSquared", snapshot.residualVarianceRadiansSquared)
		root.put("confidenceBaseline", snapshot.confidenceBaseline)
		root.put("sampleCount", snapshot.sampleCount)
		root.put("mature", snapshot.mature)
		return mapper.writeValueAsBytes(root)
	}

	private fun decode(bytes: ByteArray): TrackerReliabilityProfile.Snapshot? {
		val root = mapper.readTree(bytes) ?: return null
		if (!root.isObject || root.intField("schemaVersion") != TrackerReliabilityProfile.SCHEMA_VERSION) return null
		return TrackerReliabilityProfile.Snapshot(
			hardwareKeyHash = root.textField("hardwareKeyHash") ?: return null,
			trustedSeconds = root.doubleField("trustedSeconds") ?: return null,
			continuousTrustedSeconds = root.doubleField("continuousTrustedSeconds") ?: return null,
			residualMeanRadians = root.doubleField("residualMeanRadians") ?: return null,
			residualVarianceRadiansSquared = root.doubleField("residualVarianceRadiansSquared") ?: return null,
			confidenceBaseline = root.doubleField("confidenceBaseline") ?: return null,
			sampleCount = root.intField("sampleCount") ?: return null,
			mature = root.booleanField("mature") ?: return null,
		)
	}

	private inline fun <T> submit(crossinline task: () -> T): CompletableFuture<T> {
		val future = CompletableFuture<T>()
		try {
			executor.execute {
				try {
					future.complete(task())
				} catch (error: Exception) {
					future.completeExceptionally(error)
				}
			}
		} catch (error: RejectedExecutionException) {
			future.completeExceptionally(error)
		}
		return future
	}

	private fun JsonNode.textField(name: String): String? = get(name)?.takeIf { it.isTextual }?.textValue()
	private fun JsonNode.doubleField(name: String): Double? = get(name)?.takeIf { it.isNumber }?.doubleValue()
	private fun JsonNode.intField(name: String): Int? = get(name)?.takeIf { it.isIntegralNumber && it.canConvertToInt() }?.intValue()
	private fun JsonNode.booleanField(name: String): Boolean? = get(name)?.takeIf { it.isBoolean }?.booleanValue()

	override fun close() {
		executor.shutdown()
	}

	fun awaitClosed(timeoutMillis: Long): Boolean {
		require(timeoutMillis >= 0)
		return executor.awaitTermination(timeoutMillis, TimeUnit.MILLISECONDS)
	}

	companion object {
		const val DEFAULT_MAX_FILES = 512
		const val DEFAULT_MAX_FILE_BYTES = 16 * 1024
		const val MAX_ALLOWED_FILE_BYTES = 1024 * 1024
		const val DEFAULT_QUEUE_CAPACITY = 16
		const val MAX_ALLOWED_QUEUE_CAPACITY = 256
		const val MAX_ALLOWED_FILES = 4096
		private val PROFILE_FILENAME_REGEX = Regex("reliability-[0-9a-f]{64}\\.json")
	}
}
