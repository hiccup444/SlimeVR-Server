package dev.slimevr.tracking.processor.adaptive

import kotlin.math.sqrt

/** A weak pose-fitting prior, never independent evidence for recalibration. */
internal object ReliabilityConfidence {
	fun multiplier(snapshot: TrackerReliabilityProfile.Snapshot): Float {
		if (!snapshot.mature || snapshot.trustedSeconds < TrackerReliabilityProfile.MINIMUM_TRUSTED_SECONDS) return 1f
		val error = snapshot.residualMeanRadians + sqrt(snapshot.residualVarianceRadiansSquared)
		if (!error.isFinite() || error < 0.0) return 1f
		return (1.0 - 0.2 * (error / Math.toRadians(15.0)).coerceIn(0.0, 1.0)).toFloat()
	}
}
