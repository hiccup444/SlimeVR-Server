package dev.slimevr.tracking.processor.adaptive

import java.util.concurrent.CompletableFuture

/** Loads/saves on the store's bounded worker; profile state is protected from its load callbacks. */
internal class TrackerReliabilityLearner(private val store: TrackerReliabilityStore) : AutoCloseable {
	private data class Entry(var profile: TrackerReliabilityProfile? = null, var lastSave: Long? = null)
	private val lock = Any()
	private val entries = LinkedHashMap<String, Entry>(16, 0.75f, true)
	private var closed = false
	private var clearInProgress: CompletableFuture<Boolean>? = null

	fun observe(key: String, residual: Double, confidence: Double, now: Long, supported: Boolean) = synchronized(lock) {
		if (closed || clearInProgress != null) return@synchronized
		val entry = getOrLoad(key)
		val profile = entry.profile ?: return@synchronized
		profile.observe(residual, confidence, now, independentlySupported = supported, lowMotion = supported)
		if (profile.isMature && entry.lastSave?.let { now - it in 0 until 60_000_000_000L } != true) {
			store.save(profile)
			entry.lastSave = now
		}
	}

	fun multiplier(key: String): Float = synchronized(lock) {
		if (closed || clearInProgress != null) return@synchronized 1f
		getOrLoad(key).profile?.let { ReliabilityConfidence.multiplier(it.snapshot()) } ?: 1f
	}

	fun resetTransient() = synchronized(lock) {
		entries.values.forEach { it.profile?.resetTransient() }
	}

	fun clearLearned(): CompletableFuture<Boolean> = synchronized(lock) {
		if (closed) return@synchronized CompletableFuture.completedFuture(false)
		clearInProgress?.let { return@synchronized it }
		// Old load callbacks cannot resurrect an entry removed by this operation.
		entries.clear()
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

	private fun getOrLoad(key: String): Entry {
		entries[key]?.let { return it }
		if (entries.size >= 64) {
			val oldest = entries.entries.first()
			oldest.value.profile?.takeIf { it.isMature }?.let { store.save(it) }
			entries.remove(oldest.key)
		}
		val entry = Entry()
		entries[key] = entry
		store.load(key).whenComplete { loaded, _ ->
			synchronized(lock) {
				if (!closed && entries[key] === entry) {
					entry.profile = loaded ?: TrackerReliabilityProfile(key)
				}
			}
		}
		return entry
	}

	override fun close() {
		synchronized(lock) {
			if (closed) return
			closed = true
			entries.values.forEach { entry -> entry.profile?.takeIf { it.isMature }?.let { store.save(it) } }
			entries.clear()
		}
		store.close()
	}
}
