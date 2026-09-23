package dev.slimevr.tracking.processor.adaptive

/** Rejects pose constraints and recovery inputs when a live measurement is unreliable. */
internal object MeasurementQualityGate {
	fun trusted(sample: TelemetrySample?): Boolean = sample != null &&
		sample.status.sendData &&
		(sample.confidence?.score ?: 0f) >= 0.5f &&
		(sample.health?.qualityMultiplier ?: 1f) >= 0.5f
}
