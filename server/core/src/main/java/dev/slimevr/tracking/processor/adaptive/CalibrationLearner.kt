package dev.slimevr.tracking.processor.adaptive

import java.util.concurrent.CompletableFuture

/** Converts trusted temporal residual changes into temperature-bucketed profile observations. */
class CalibrationLearner(
	private val store: CalibrationStore,
	private val maxProfiles: Int = MAX_CACHED_PROFILES,
) : AutoCloseable {
	private data class Entry(
		val hardwareId: String,
		var profile: CalibrationProfile? = null,
		var loaded: Boolean = false,
		var loadFailed: Boolean = false,
		var loadPending: Boolean = true,
		var lastErrorRadians: Double? = null,
		var lastObservationSeconds: Double? = null,
		var lastTimeNanos: Long? = null,
		var lastRateRadiansPerSecond: Double? = null,
		var trustedObservationSeconds: Double = 0.0,
		var lastSaveTimeNanos: Long? = null,
		val ready: CompletableFuture<Boolean> = CompletableFuture(),
	)

	data class Diagnostics(
		val loaded: Boolean,
		val loadFailed: Boolean,
		val trustedObservationSeconds: Double,
		val lastRateRadiansPerSecond: Double?,
	)

	private val lock = Any()
	private val entries = LinkedHashMap<String, Entry>(16, 0.75f, true)
	private var closed = false
	private var clearInProgress: CompletableFuture<Boolean>? = null

	init {
		require(maxProfiles in 1..MAX_CACHED_PROFILES)
	}

	/**
	 * Observes an externally gated residual. The returned rate is a mature profile prediction;
	 * this class never applies that rate as a correction. Temperatures are in Celsius and yaw
	 * residuals and rates are in radians and radians per second.
	 */
	fun observe(
		hardwareId: String,
		temperature: Double?,
		residual: DriftResidual?,
		now: Long,
	): Double? = synchronized(lock) {
		if (closed || clearInProgress != null || hardwareId.isBlank() || hardwareId.length > CalibrationStore.MAX_HARDWARE_ID_CHARS) return null
		val entry = getOrLoad(hardwareId)
		if (!entry.loaded) {
			resetTemporal(entry)
			return null
		}
		val profile = entry.profile ?: return null
		if (temperature == null ||
			!temperature.isFinite() ||
			temperature !in CalibrationProfile.MIN_TEMPERATURE_C..CalibrationProfile.MAX_TEMPERATURE_C ||
			residual == null ||
			!residual.eligibleForLearning ||
			!residual.filteredErrorRadians.isFinite() ||
			!residual.consistentSeconds.isFinite()
		) {
			resetTemporal(entry)
			return if (temperature != null) profile.predictRate(temperature) else null
		}

		val previousTime = entry.lastTimeNanos
		val previousObservationSeconds = entry.lastObservationSeconds
		val previousError = entry.lastErrorRadians
		if (previousTime != null && previousObservationSeconds != null && previousError != null) {
			val deltaNanos = now - previousTime
			if (deltaNanos in MIN_OBSERVATION_GAP_NANOS..MAX_OBSERVATION_GAP_NANOS) {
				val elapsedSeconds = deltaNanos * 1e-9
				val observedSeconds = residual.consistentSeconds - previousObservationSeconds
				if (observedSeconds in (elapsedSeconds * MIN_PERIOD_PROGRESS)..(elapsedSeconds * MAX_PERIOD_PROGRESS)) {
					val rate = wrapYaw(residual.filteredErrorRadians - previousError.toFloat()).toDouble() / elapsedSeconds
					entry.lastRateRadiansPerSecond = rate
					if (profile.update(temperature, rate, elapsedSeconds, TRUSTED_RESIDUAL_CONFIDENCE)) {
						entry.trustedObservationSeconds = minOf(
							MAX_DIAGNOSTIC_SECONDS,
							entry.trustedObservationSeconds + elapsedSeconds,
						)
						persistIfDue(entry, profile, now)
					}
				}
			}
		}
		entry.lastErrorRadians = residual.filteredErrorRadians.toDouble()
		entry.lastObservationSeconds = residual.consistentSeconds
		entry.lastTimeNanos = now
		profile.predictRate(temperature)
	}

	/** Returns a future that completes after the one-time asynchronous load for this ID. */
	fun readiness(hardwareId: String): CompletableFuture<Boolean> = synchronized(lock) {
		if (closed || clearInProgress != null || hardwareId.isBlank() || hardwareId.length > CalibrationStore.MAX_HARDWARE_ID_CHARS) {
			return CompletableFuture.completedFuture(false)
		}
		getOrLoad(hardwareId).ready
	}

	/** Clears temporal derivatives while retaining all loaded calibration profiles. */
	fun resetTransient() = synchronized(lock) {
		entries.values.forEach(::resetTemporal)
	}

	/** Clears cached profiles and all validated profile files in the configured store directory. */
	fun clearLearned(): CompletableFuture<Boolean> {
		return synchronized(lock) {
			if (closed) return CompletableFuture.completedFuture(false)
			clearInProgress?.let { return it }
			entries.values.forEach { entry ->
				entry.loadPending = false
				entry.loaded = true
				entry.loadFailed = false
				entry.profile = CalibrationProfile(entry.hardwareId)
				entry.trustedObservationSeconds = 0.0
				entry.lastSaveTimeNanos = null
				resetTemporal(entry)
				entry.ready.complete(true)
			}
			val clearing = store.clearAllKnownProfiles()
			clearInProgress = clearing
			clearing.whenComplete { _, _ ->
				synchronized(lock) {
					if (clearInProgress === clearing) {
						entries.clear()
						clearInProgress = null
					}
				}
			}
			clearing
		}
	}

	/** Read-only state useful for support diagnostics. */
	fun diagnostics(hardwareId: String): Diagnostics? = synchronized(lock) {
		entries[hardwareId]?.let {
			Diagnostics(it.loaded, it.loadFailed, it.trustedObservationSeconds, it.lastRateRadiansPerSecond)
		}
	}

	private fun getOrLoad(hardwareId: String): Entry {
		entries[hardwareId]?.let { return it }
		if (entries.size >= maxProfiles) {
			val eldest = entries.entries.iterator().next()
			val removed = eldest.value
			removed.ready.complete(false)
			if (removed.loaded) removed.profile?.let(::saveIfMature)
			entries.remove(eldest.key)
		}
		val entry = Entry(hardwareId)
		entries[hardwareId] = entry
		store.load(hardwareId).whenComplete { profile, error ->
			synchronized(lock) {
				if (entries[hardwareId] === entry && entry.loadPending) {
					entry.profile = profile ?: CalibrationProfile(hardwareId)
					entry.loadFailed = error != null
					entry.loaded = true
					entry.loadPending = false
					entry.ready.complete(error == null)
				}
			}
		}
		return entry
	}

	private fun persistIfDue(entry: Entry, profile: CalibrationProfile, now: Long) {
		val previous = entry.lastSaveTimeNanos
		if (previous == null || (now >= previous && now - previous >= SAVE_INTERVAL_NANOS)) {
			saveIfMature(profile)
			entry.lastSaveTimeNanos = now
		}
	}

	private fun saveIfMature(profile: CalibrationProfile) {
		if (profile.snapshot().buckets.isNotEmpty()) store.save(profile)
	}

	private fun resetTemporal(entry: Entry) {
		entry.lastErrorRadians = null
		entry.lastObservationSeconds = null
		entry.lastTimeNanos = null
		entry.lastRateRadiansPerSecond = null
	}

	override fun close() {
		synchronized(lock) {
			if (closed) return
			closed = true
			entries.values.forEach { entry ->
				entry.ready.complete(false)
				if (entry.loaded) entry.profile?.let(::saveIfMature)
			}
		}
		store.close()
	}

	companion object {
		const val MAX_CACHED_PROFILES = 512
		private const val TRUSTED_RESIDUAL_CONFIDENCE = 0.95
		private const val MIN_OBSERVATION_GAP_NANOS = 1_000_000L
		private const val MAX_OBSERVATION_GAP_NANOS = 500_000_000L
		private const val MIN_PERIOD_PROGRESS = 0.5
		private const val MAX_PERIOD_PROGRESS = 1.5
		private const val SAVE_INTERVAL_NANOS = 60_000_000_000L
		private const val MAX_DIAGNOSTIC_SECONDS = 3600.0 * CalibrationProfile.DEFAULT_MAX_BUCKETS
	}
}
