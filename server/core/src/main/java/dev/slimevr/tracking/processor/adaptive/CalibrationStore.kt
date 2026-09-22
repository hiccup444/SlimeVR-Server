package dev.slimevr.tracking.processor.adaptive

import com.fasterxml.jackson.databind.JsonNode
import com.fasterxml.jackson.databind.ObjectMapper
import java.nio.file.AtomicMoveNotSupportedException
import java.nio.file.Files
import java.nio.file.LinkOption
import java.nio.file.Path
import java.nio.file.StandardCopyOption
import java.security.MessageDigest
import java.util.concurrent.ArrayBlockingQueue
import java.util.concurrent.CompletableFuture
import java.util.concurrent.RejectedExecutionException
import java.util.concurrent.ThreadPoolExecutor
import java.util.concurrent.TimeUnit

/** Bounded asynchronous persistence for validated calibration snapshots. */
class CalibrationStore(
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
		{ runnable -> Thread(runnable, "calibration-store").apply { isDaemon = true } },
		ThreadPoolExecutor.AbortPolicy(),
	)

	init {
		require(maxFiles in 1..MAX_ALLOWED_FILES)
		require(maxFileBytes in 128..MAX_ALLOWED_FILE_BYTES)
		require(queueCapacity in 1..MAX_ALLOWED_QUEUE_CAPACITY)
	}

	/** Queues persistence away from the caller thread. The result is false when the file cap is full. */
	fun save(profile: CalibrationProfile): CompletableFuture<Boolean> {
		val snapshot = profile.snapshot()
		if (snapshot.hardwareId.length > MAX_HARDWARE_ID_CHARS) return CompletableFuture.completedFuture(false)
		return submit {
			CalibrationProfile.restore(snapshot)
			Files.createDirectories(directory)
			val destination = profilePath(snapshot.hardwareId)
			if (!Files.exists(destination, LinkOption.NOFOLLOW_LINKS) && countProfiles() >= maxFiles) return@submit false
			val bytes = encode(snapshot)
			if (bytes.size > maxFileBytes) return@submit false
			val temp = Files.createTempFile(directory, ".calibration-", ".tmp")
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

	/** Loads one hardware ID, returning null for missing or invalid persisted data. */
	fun load(hardwareId: String): CompletableFuture<CalibrationProfile?> = submit {
		if (hardwareId.isBlank() || hardwareId.length > MAX_HARDWARE_ID_CHARS) return@submit null
		val file = profilePath(hardwareId)
		if (!Files.isRegularFile(file, LinkOption.NOFOLLOW_LINKS)) return@submit null
		try {
			val bytes = Files.newInputStream(file).use { it.readNBytes(maxFileBytes + 1) }
			if (bytes.size > maxFileBytes) return@submit null
			val snapshot = decode(bytes) ?: return@submit null
			if (snapshot.hardwareId != hardwareId) return@submit null
			CalibrationProfile.restore(snapshot)
		} catch (_: Exception) {
			null
		}
	}

	/** Removes only the persisted snapshot. It does not change any in-memory profile. */
	fun clear(hardwareId: String): CompletableFuture<Boolean> = submit {
		if (hardwareId.isBlank() || hardwareId.length > MAX_HARDWARE_ID_CHARS) return@submit false
		Files.deleteIfExists(profilePath(hardwareId))
		true
	}

	/** Removes persisted profile files created by this store without following links or paths. */
	fun clearAllKnownProfiles(): CompletableFuture<Boolean> = submit {
		if (!Files.exists(directory, LinkOption.NOFOLLOW_LINKS)) return@submit true
		if (!Files.isDirectory(directory, LinkOption.NOFOLLOW_LINKS)) return@submit false
		val profiles = mutableListOf<Path>()
		Files.newDirectoryStream(directory).use { entries ->
			var scanned = 0
			for (entry in entries) {
				if (++scanned > maxFiles) return@submit false
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
		Files.newDirectoryStream(directory, "*.json").use { entries ->
			var count = 0
			for (entry in entries) {
				if (Files.isRegularFile(entry, LinkOption.NOFOLLOW_LINKS) && ++count >= maxFiles) return count
			}
			return count
		}
	}

	private fun profilePath(hardwareId: String): Path = directory.resolve("${sha256(hardwareId)}.json")

	private fun encode(snapshot: CalibrationProfile.Snapshot): ByteArray {
		// Use Jackson's tree API because the server mapper does not install the Kotlin module.
		val root = mapper.createObjectNode()
		root.put("schemaVersion", SCHEMA_VERSION)
		root.put("hardwareId", snapshot.hardwareId)
		root.put("bucketWidthCelsius", snapshot.bucketWidthCelsius)
		root.put("minimumObservationSeconds", snapshot.minimumObservationSeconds)
		root.put("minimumConfidence", snapshot.minimumConfidence)
		val savedBuckets = root.putArray("buckets")
		for (bucket in snapshot.buckets) {
			val node = savedBuckets.addObject()
			node.put("temperatureCelsius", bucket.temperatureCelsius)
			node.put("rateRadiansPerSecond", bucket.rateRadiansPerSecond)
			node.put("observationSeconds", bucket.observationSeconds)
			node.put("confidence", bucket.confidence)
		}
		return mapper.writeValueAsBytes(root)
	}

	private fun decode(bytes: ByteArray): CalibrationProfile.Snapshot? {
		val root = mapper.readTree(bytes) ?: return null
		if (!root.isObject || root.intField("schemaVersion") != SCHEMA_VERSION) return null
		val id = root.textField("hardwareId") ?: return null
		val width = root.doubleField("bucketWidthCelsius") ?: return null
		val minimumSeconds = root.doubleField("minimumObservationSeconds") ?: return null
		val confidence = root.doubleField("minimumConfidence") ?: return null
		val bucketNodes = root.get("buckets")
		if (bucketNodes == null || !bucketNodes.isArray || bucketNodes.size() > CalibrationProfile.DEFAULT_MAX_BUCKETS) return null
		val buckets = ArrayList<CalibrationProfile.BucketSnapshot>(bucketNodes.size())
		for (node in bucketNodes) {
			if (!node.isObject) return null
			buckets.add(
				CalibrationProfile.BucketSnapshot(
					node.doubleField("temperatureCelsius") ?: return null,
					node.doubleField("rateRadiansPerSecond") ?: return null,
					node.doubleField("observationSeconds") ?: return null,
					node.doubleField("confidence") ?: return null,
				),
			)
		}
		return CalibrationProfile.Snapshot(id, width, minimumSeconds, confidence, buckets)
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

	private fun sha256(value: String): String = MessageDigest.getInstance("SHA-256")
		.digest(value.toByteArray(Charsets.UTF_8))
		.joinToString("") { "%02x".format(it) }

	private fun JsonNode.textField(name: String): String? = get(name)?.takeIf { it.isTextual }?.textValue()
	private fun JsonNode.doubleField(name: String): Double? = get(name)?.takeIf { it.isNumber }?.doubleValue()
	private fun JsonNode.intField(name: String): Int? = get(name)?.takeIf { it.isIntegralNumber && it.canConvertToInt() }?.intValue()

	override fun close() {
		executor.shutdown()
	}

	/** Waits for queued work after close. This is intended for tests and shutdown code, never pose threads. */
	fun awaitClosed(timeoutMillis: Long): Boolean {
		require(timeoutMillis >= 0)
		return executor.awaitTermination(timeoutMillis, TimeUnit.MILLISECONDS)
	}

	companion object {
		const val DEFAULT_MAX_FILES = 512
		const val DEFAULT_MAX_FILE_BYTES = 64 * 1024
		const val MAX_ALLOWED_FILE_BYTES = 1024 * 1024
		const val DEFAULT_QUEUE_CAPACITY = 32
		const val MAX_ALLOWED_QUEUE_CAPACITY = 256
		const val MAX_ALLOWED_FILES = 4096
		const val MAX_HARDWARE_ID_CHARS = 256
		private const val SCHEMA_VERSION = 1
		private val PROFILE_FILENAME_REGEX = Regex("[0-9a-f]{64}\\.json")
	}
}
