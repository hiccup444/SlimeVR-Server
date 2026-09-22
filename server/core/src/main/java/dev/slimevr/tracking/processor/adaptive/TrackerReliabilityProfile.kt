package dev.slimevr.tracking.processor.adaptive

import java.security.MessageDigest
import kotlin.math.exp

/** Bounded residual-quality history for one hardware key. This profile never raises live confidence. */
class TrackerReliabilityProfile private constructor(val hardwareKeyHash: String, @Suppress("UNUSED_PARAMETER") restoredHash: Boolean) {
	constructor(hardwareKey: String) : this(hashHardwareKey(hardwareKey), true)
	private var trustedSeconds = 0.0
	private var continuousTrustedSeconds = 0.0
	private var residualMeanRadians = 0.0
	private var residualVarianceRadiansSquared = 0.0
	private var confidenceBaseline = 0.0
	private var sampleCount = 0
	private var lastObservationNanos: Long? = null
	private var hasStatistics = false

	val isMature: Boolean get() = trustedSeconds >= MINIMUM_TRUSTED_SECONDS
	val currentTrustedSeconds: Double get() = trustedSeconds
	val currentContinuousSeconds: Double get() = continuousTrustedSeconds
	val currentResidualMeanRadians: Double? get() = residualMeanRadians.takeIf { hasStatistics }
	val currentResidualVarianceRadiansSquared: Double? get() = residualVarianceRadiansSquared.takeIf { hasStatistics }
	val currentConfidenceBaseline: Double? get() = confidenceBaseline.takeIf { hasStatistics }
	val currentSampleCount: Int get() = sampleCount

	/**
	 * Adds a residual only when it is independently supported and confidence is high. Motion,
	 * uncertainty, invalid values, and observation gaps break continuous trusted duration. All times
	 * are monotonic nanoseconds; residuals are shortest-arc radians and confidence is in [0, 1].
	 */
	fun observe(
		independentResidualMagnitudeRadians: Double,
		confidence: Double,
		nowNanos: Long,
		independentlySupported: Boolean,
		lowMotion: Boolean,
		uncertain: Boolean = false,
	): Boolean {
		if (!independentResidualMagnitudeRadians.isFinite() ||
			independentResidualMagnitudeRadians !in 0.0..MAX_RESIDUAL_RADIANS ||
			!confidence.isFinite() ||
			confidence !in MINIMUM_CONFIDENCE..1.0 ||
			nowNanos < 0L ||
			!independentlySupported ||
			!lowMotion ||
			uncertain
		) {
			breakContinuousRun()
			return false
		}

		val previous = lastObservationNanos
		val deltaNanos = previous?.let { nowNanos - it }
		if (deltaNanos != null && deltaNanos <= 0L) {
			breakContinuousRun()
			return false
		}
		if (deltaNanos != null && deltaNanos !in MIN_OBSERVATION_GAP_NANOS..MAX_OBSERVATION_GAP_NANOS) {
			breakContinuousRun()
		}
		val elapsed = deltaNanos?.takeIf { it in MIN_OBSERVATION_GAP_NANOS..MAX_OBSERVATION_GAP_NANOS }?.times(1e-9)
		if (elapsed != null) {
			trustedSeconds = (trustedSeconds + elapsed).coerceAtMost(MAX_TRUSTED_SECONDS)
			continuousTrustedSeconds = (continuousTrustedSeconds + elapsed).coerceAtMost(MAX_TRUSTED_SECONDS)
		}

		if (!hasStatistics) {
			residualMeanRadians = independentResidualMagnitudeRadians
			residualVarianceRadiansSquared = 0.0
			confidenceBaseline = confidence
			hasStatistics = true
		} else if (elapsed != null) {
			val alpha = 1.0 - exp(-elapsed / EW_TIME_CONSTANT_SECONDS)
			val residualDifference = independentResidualMagnitudeRadians - residualMeanRadians
			residualMeanRadians += alpha * residualDifference
			residualVarianceRadiansSquared =
				(1.0 - alpha) * (residualVarianceRadiansSquared + alpha * residualDifference * residualDifference)
			confidenceBaseline += alpha * (confidence - confidenceBaseline)
		}
		if (sampleCount < Int.MAX_VALUE) sampleCount++
		lastObservationNanos = nowNanos
		return true
	}

	/** Breaks timing continuity while preserving mature evidence and statistics across resets. */
	fun resetTransient() = breakContinuousRun()

	fun snapshot(): Snapshot = Snapshot(
		hardwareKeyHash = hardwareKeyHash,
		trustedSeconds = trustedSeconds,
		continuousTrustedSeconds = continuousTrustedSeconds,
		residualMeanRadians = residualMeanRadians,
		residualVarianceRadiansSquared = residualVarianceRadiansSquared,
		confidenceBaseline = confidenceBaseline,
		sampleCount = sampleCount,
		mature = isMature,
	)

	private fun breakContinuousRun() {
		lastObservationNanos = null
		continuousTrustedSeconds = 0.0
	}

	data class Snapshot(
		val hardwareKeyHash: String,
		val trustedSeconds: Double,
		val continuousTrustedSeconds: Double,
		val residualMeanRadians: Double,
		val residualVarianceRadiansSquared: Double,
		val confidenceBaseline: Double,
		val sampleCount: Int,
		val mature: Boolean,
	)

	companion object {
		const val SCHEMA_VERSION = 1
		const val MINIMUM_TRUSTED_SECONDS = 60.0
		const val MINIMUM_CONFIDENCE = 0.75
		const val MAX_RESIDUAL_RADIANS = 3.141592653589793
		const val MAX_TRUSTED_SECONDS = 7.0 * 24.0 * 60.0 * 60.0
		private const val MIN_OBSERVATION_GAP_NANOS = 1_000_000L
		private const val MAX_OBSERVATION_GAP_NANOS = 500_000_000L
		private const val EW_TIME_CONSTANT_SECONDS = 30.0
		private val HASH_REGEX = Regex("[0-9a-f]{64}")

		fun restore(snapshot: Snapshot): TrackerReliabilityProfile {
			require(HASH_REGEX.matches(snapshot.hardwareKeyHash))
			require(snapshot.trustedSeconds.isFinite() && snapshot.trustedSeconds in 0.0..MAX_TRUSTED_SECONDS)
			require(snapshot.continuousTrustedSeconds.isFinite() && snapshot.continuousTrustedSeconds in 0.0..snapshot.trustedSeconds)
			require(snapshot.residualMeanRadians.isFinite() && snapshot.residualMeanRadians in 0.0..MAX_RESIDUAL_RADIANS)
			require(snapshot.residualVarianceRadiansSquared.isFinite() && snapshot.residualVarianceRadiansSquared in 0.0..(MAX_RESIDUAL_RADIANS * MAX_RESIDUAL_RADIANS))
			require(snapshot.confidenceBaseline.isFinite() && snapshot.confidenceBaseline in MINIMUM_CONFIDENCE..1.0)
			require(snapshot.sampleCount > 0)
			require(snapshot.mature == (snapshot.trustedSeconds >= MINIMUM_TRUSTED_SECONDS))
			val profile = TrackerReliabilityProfile(snapshot.hardwareKeyHash, true)
			profile.trustedSeconds = snapshot.trustedSeconds
			profile.continuousTrustedSeconds = snapshot.continuousTrustedSeconds
			profile.residualMeanRadians = snapshot.residualMeanRadians
			profile.residualVarianceRadiansSquared = snapshot.residualVarianceRadiansSquared
			profile.confidenceBaseline = snapshot.confidenceBaseline
			profile.sampleCount = snapshot.sampleCount
			profile.hasStatistics = true
			return profile
		}

		internal fun hashHardwareKey(hardwareKey: String): String {
			require(hardwareKey.isNotBlank() && hardwareKey.length <= MAX_HARDWARE_KEY_CHARS)
			return MessageDigest.getInstance("SHA-256")
				.digest(hardwareKey.toByteArray(Charsets.UTF_8))
				.joinToString("") { "%02x".format(it) }
		}

		private const val MAX_HARDWARE_KEY_CHARS = 512
	}
}
