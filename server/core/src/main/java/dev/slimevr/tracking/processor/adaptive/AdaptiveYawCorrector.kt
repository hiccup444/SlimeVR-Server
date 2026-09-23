package dev.slimevr.tracking.processor.adaptive

/** Bias is subtracted in world yaw after mounting/reset transforms. Never persists transient offsets. */
class AdaptiveYawCorrector {
	var biasRadians = 0f
		private set
	var predictionActive = false
		private set
	private var lastTime: Long? = null
	private var lastTrustedTime: Long? = null
	private var lastTrustedBiasRadians = 0f

	fun reset() {
		biasRadians = 0f
		predictionActive = false
		lastTime = null
		lastTrustedTime = null
		lastTrustedBiasRadians = 0f
	}

	fun update(residual: DriftResidual, now: Long, strength: Float = 1f): Float {
		predictionActive = false
		val prior = lastTime
		lastTime = now
		val dt = prior?.let { now - it }
		if (!residual.eligibleForLearning ||
			!residual.filteredErrorRadians.isFinite() ||
			!strength.isFinite() ||
			dt == null ||
			dt !in 1..500_000_000L
		) {
			return biasRadians
		}
		lastTrustedTime = now
		val limit = Math.toRadians(0.2).toFloat() * (dt * 1e-9f) * strength.coerceIn(0f, 1f)
		val error = wrapYaw(residual.filteredErrorRadians - biasRadians)
		val maximum = Math.toRadians(30.0).toFloat()
		biasRadians = (biasRadians + error.coerceIn(-limit, limit)).coerceIn(-maximum, maximum)
		lastTrustedBiasRadians = biasRadians
		return biasRadians
	}

	/** Carries a mature rate through a short loss of absolute arm evidence. */
	fun predict(rateRadiansPerSecond: Double?, now: Long, strength: Float = 1f, maximumHoldoverRadians: Float = Math.toRadians(0.5).toFloat()): Float {
		predictionActive = false
		val previous = lastTime
		lastTime = now
		val elapsed = previous?.let { now - it }
		val trustedAge = lastTrustedTime?.let { now - it }
		if (rateRadiansPerSecond == null ||
			!rateRadiansPerSecond.isFinite() ||
			!strength.isFinite() ||
			!maximumHoldoverRadians.isFinite() ||
			maximumHoldoverRadians <= 0f ||
			elapsed == null ||
			elapsed !in 1..500_000_000L ||
			trustedAge == null ||
			trustedAge !in 0..30_000_000_000L
		) {
			return biasRadians
		}
		val rate = rateRadiansPerSecond.coerceIn(-0.001, 0.001)
		val delta = rate * (elapsed * 1e-9) * strength.coerceIn(0f, 1f)
		val maximum = Math.toRadians(30.0)
		val maximumHoldover = maximumHoldoverRadians.coerceAtMost(Math.toRadians(0.5).toFloat())
		val previousBias = biasRadians
		biasRadians = (biasRadians + delta.toFloat())
			.coerceIn(lastTrustedBiasRadians - maximumHoldover, lastTrustedBiasRadians + maximumHoldover)
			.coerceIn(-maximum.toFloat(), maximum.toFloat())
		predictionActive = biasRadians != previousBias
		return biasRadians
	}
}
