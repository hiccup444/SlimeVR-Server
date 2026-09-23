package dev.slimevr.tracking.processor.adaptive

/** Durations count sampled intervals, not wall-clock gaps or unsupported motion. */
data class YawLearningDiagnostic(
	val observedSeconds: Double,
	val supportedSeconds: Double,
	val learningSeconds: Double,
	val learningDutyCycle: Double,
	val contextRestarts: Long,
	val secondsSinceSupportedEvidence: Double?,
)

internal class YawLearningStatistics {
	private var lastTime: Long? = null
	private var lastSupported: Long? = null
	private var lastContext: Pair<DriftEvidenceSource?, Long>? = null
	private var wasSupported = false
	private var wasLearning = false
	private var observed = 0.0
	private var supported = 0.0
	private var learning = 0.0
	private var restarts = 0L

	fun observe(residual: DriftResidual?, now: Long): YawLearningDiagnostic {
		val dt = lastTime?.let { now - it }?.takeIf { it in 1..500_000_000L }?.times(1e-9) ?: 0.0
		val informative = residual?.reason == "OBSERVING" || residual?.eligibleForLearning == true
		val eligible = residual?.eligibleForLearning == true
		observed += dt
		if (informative && wasSupported) supported += dt
		if (eligible && wasLearning) learning += dt
		if (informative) {
			lastSupported = now
			residual?.contextId?.let { id ->
				val context = residual.source to id
				if (lastContext != null && context != lastContext) restarts++
				lastContext = context
			}
		}
		lastTime = now
		wasSupported = informative
		wasLearning = eligible
		return YawLearningDiagnostic(observed, supported, learning, if (observed > 0.0) learning / observed else 0.0, restarts, lastSupported?.let { ((now - it) * 1e-9).coerceAtLeast(0.0) })
	}
}
