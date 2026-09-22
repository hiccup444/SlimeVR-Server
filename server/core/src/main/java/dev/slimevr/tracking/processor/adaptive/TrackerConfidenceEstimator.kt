package dev.slimevr.tracking.processor.adaptive

import kotlin.math.exp
import kotlin.math.min

/** Observes input quality only. Independent constraints are required before any calibration. */
class TrackerConfidenceEstimator(private val maxHistory: Int = 256) {
	private data class History(val timestamp: Long, val epoch: Long, val score: Float, val seconds: Double)
	private val history = LinkedHashMap<Int, History>()

	init {
		require(maxHistory > 0)
	}

	fun reset() = history.clear()

	fun observe(frame: AdaptiveTelemetryFrame): AdaptiveTelemetryFrame {
		val ids = frame.samples.map { it.id }.toSet()
		history.keys.retainAll(ids)
		return frame.copy(
			samples = frame.samples.map { sample ->
				sample.copy(confidence = estimate(sample, frame.timestampNanos, frame.resetEpoch))
			},
		)
	}

	private fun estimate(sample: TelemetrySample, now: Long, epoch: Long): TrackerConfidence {
		val reasons = mutableListOf("INDEPENDENT_CONSTRAINTS_UNAVAILABLE")
		var target = 0.9f
		var unavailable = false
		if (!sample.status.sendData) {
			reasons.add("TRACKING_UNAVAILABLE")
			unavailable = true
		}
		if ((sample.rotationExpected && (sample.rawRotation == null || sample.adjustedRotation == null)) ||
			(sample.positionExpected && sample.position == null) ||
			(!sample.rotationExpected && !sample.positionExpected)
		) {
			reasons.add("MISSING_OR_INVALID_MEASUREMENT")
			unavailable = true
		}
		val age = sample.packetAgeNanos
		when {
			age == null || age < 0 -> {
				target = min(target, 0.6f)
				reasons.add("FRESHNESS_UNKNOWN")
			}

			age >= 500_000_000 -> {
				reasons.add("STALE_ROTATION")
				unavailable = true
			}

			age > 100_000_000 -> {
				target *= (1.0 - (age - 100_000_000) / 400_000_000.0).toFloat()
				reasons.add("AGING_ROTATION")
			}
		}
		val speed = sample.angularSpeedRadiansPerSecond
		if (speed != null && speed.isFinite() && speed > 8f) {
			target *= (1f - 0.35f * ((speed - 8f) / 12f).coerceIn(0f, 1f))
			reasons.add("HIGH_MOTION")
		}
		if (unavailable) {
			history.remove(sample.id)
			return TrackerConfidence(0f, reasons, 0.0)
		}
		val prior = history[sample.id]?.takeIf {
			sample.continuousObservation && it.epoch == epoch && now - it.timestamp in 1..500_000_000L
		}
		val dt = prior?.let { (now - it.timestamp) * 1e-9 } ?: 0.0
		val seconds = (prior?.seconds ?: 0.0) + dt
		if (seconds < 1.0) {
			target = min(target, (0.6 + 0.3 * seconds).toFloat())
			reasons.add("WARMING_UP")
		}
		val score = if (prior == null) {
			min(target, 0.6f)
		} else {
			val timeConstant = if (target < prior.score) 0.25 else 0.8
			(prior.score + (target - prior.score) * (1.0 - exp(-dt / timeConstant))).toFloat()
		}
		history[sample.id] = History(now, epoch, score.coerceIn(0f, 1f), seconds)
		while (history.size > maxHistory) history.remove(history.keys.first())
		return TrackerConfidence(score.coerceIn(0f, 1f), reasons, seconds)
	}
}
