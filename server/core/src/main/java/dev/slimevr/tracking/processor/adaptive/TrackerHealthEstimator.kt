package dev.slimevr.tracking.processor.adaptive

/** Caps live measurement confidence with health evidence; it never changes sensor values. */
class TrackerHealthEstimator {
	private val monitor = TrackerHealthMonitor()
	private var epoch: Long? = null
	private var ids = emptySet<Int>()

	fun reset() {
		monitor.reset()
		epoch = null
		ids = emptySet()
	}

	fun observe(frame: AdaptiveTelemetryFrame, movement: Map<Int, TrackerMotionContext> = emptyMap()): AdaptiveTelemetryFrame {
		if (epoch != frame.resetEpoch) reset()
		epoch = frame.resetEpoch
		val currentIds = frame.samples.map { it.id }.toSet()
		(ids - currentIds).forEach(monitor::resetTracker)
		ids = currentIds
		return frame.copy(
			samples = frame.samples.map { sample ->
				if (!sample.rotationExpected) return@map sample
				val health = monitor.observe(sample, frame.timestampNanos, movement[sample.id] ?: TrackerMotionContext())
				val confidence = sample.confidence?.let {
					it.copy(score = minOf(it.score, health.qualityMultiplier), reasons = (it.reasons + health.reasons.filter { reason -> reason != "HEALTHY" }).distinct())
				}
				sample.copy(confidence = confidence, health = health)
			},
		)
	}
}
