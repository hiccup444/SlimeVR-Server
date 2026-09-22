package dev.slimevr.tracking.processor.adaptive

/** Bias is subtracted in world yaw after mounting/reset transforms. Never persists transient offsets. */
class AdaptiveYawCorrector {
	var biasRadians = 0f
		private set
	private var lastTime: Long? = null

	fun reset() {
		biasRadians = 0f
		lastTime = null
	}

	fun update(residual: DriftResidual, now: Long, strength: Float = 1f): Float {
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
		val limit = Math.toRadians(0.2).toFloat() * (dt * 1e-9f) * strength.coerceIn(0f, 1f)
		val error = wrapYaw(residual.filteredErrorRadians - biasRadians)
		val maximum = Math.toRadians(30.0).toFloat()
		biasRadians = (biasRadians + error.coerceIn(-limit, limit)).coerceIn(-maximum, maximum)
		return biasRadians
	}
}
