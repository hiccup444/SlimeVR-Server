package dev.slimevr.tracking.processor.adaptive

import io.github.axisangles.ktmath.Vector3
import kotlin.math.abs
import kotlin.math.min

data class AdaptiveFloorEstimate(
	val calibratedHeightMeters: Double,
	val estimatedHeightMeters: Double,
	val trustedSeconds: Double,
	val reason: String,
)

/** Learns a small floor-height correction from paired planted feet without changing the skeleton. */
class AdaptiveFloorEstimator {
	private data class Sample(val timeNanos: Long, val heightMeters: Double)

	private val samples = ArrayDeque<Sample>()
	private var calibratedHeight: Double? = null
	private var estimatedHeight: Double? = null
	private var lastTimeNanos: Long? = null
	private var trustedSeconds = 0.0
	var estimate: AdaptiveFloorEstimate? = null
		private set

	/**
	 * Accepts only two planted, near-floor observations while the user's upright confidence is high.
	 * Positions and heights are meters; times are monotonic nanoseconds.
	 */
	fun observe(
		calibratedFloorHeightMeters: Double,
		leftPosition: Vector3?,
		leftContact: FootContactSnapshot,
		rightPosition: Vector3?,
		rightContact: FootContactSnapshot,
		uprightConfidence: Double,
		nowNanos: Long,
	): AdaptiveFloorEstimate {
		if (!calibratedFloorHeightMeters.isFinite()) {
			clearEvidence()
			return (estimate ?: AdaptiveFloorEstimate(0.0, 0.0, 0.0, "INVALID_CALIBRATED_FLOOR"))
				.copy(reason = "INVALID_CALIBRATED_FLOOR")
				.also { estimate = it }
		}
		val oldCalibrated = calibratedHeight
		if (oldCalibrated == null || abs(oldCalibrated - calibratedFloorHeightMeters) > 0.001) {
			clearEvidence()
			calibratedHeight = calibratedFloorHeightMeters
			estimatedHeight = calibratedFloorHeightMeters
		}
		val baseline = calibratedHeight ?: calibratedFloorHeightMeters
		val previous = lastTimeNanos
		val dtNanos = previous?.let { nowNanos - it }
		lastTimeNanos = nowNanos
		if (previous != null && (dtNanos == null || dtNanos !in 1..250_000_000L)) {
			clearEvidence()
			lastTimeNanos = nowNanos
			return publish(baseline, trustedSeconds, "TIME_GAP")
		}
		if (!uprightConfidence.isFinite() || uprightConfidence !in MIN_UPRIGHT_CONFIDENCE..1.0) return reject(baseline, "NOT_UPRIGHT")
		if (leftPosition == null || rightPosition == null || !finite(leftPosition) || !finite(rightPosition)) return reject(baseline, "MISSING_OR_INVALID_FOOT")
		if (!planted(leftContact) || !planted(rightContact)) return reject(baseline, "BOTH_FEET_NOT_PLANTED")
		val left = leftPosition.y.toDouble()
		val right = rightPosition.y.toDouble()
		if (abs(left - right) > MAX_FOOT_DISAGREEMENT_METERS) return reject(baseline, "FEET_DISAGREE")
		if (abs(left - baseline) > MAX_CALIBRATED_DEVIATION_METERS || abs(right - baseline) > MAX_CALIBRATED_DEVIATION_METERS) return reject(baseline, "OUTSIDE_CALIBRATED_BAND")
		val dtSeconds = dtNanos?.times(1e-9) ?: 0.0
		if (dtSeconds > 0.0) trustedSeconds = min(MAX_DWELL_SECONDS, trustedSeconds + dtSeconds)
		samples.addLast(Sample(nowNanos, (left + right) * 0.5))
		while (samples.isNotEmpty() && nowNanos - samples.first().timeNanos > WINDOW_NANOS) samples.removeFirst()
		while (samples.size > MAX_SAMPLES) samples.removeFirst()
		if (trustedSeconds < MIN_DWELL_SECONDS || samples.size < MIN_SAMPLES) return publish(baseline, trustedSeconds, "COLLECTING_PAIRED_CONTACT")
		val median = median(samples.map { it.heightMeters })
		val mad = median(samples.map { abs(it.heightMeters - median) })
		if (mad > MAX_MEDIAN_ABSOLUTE_DEVIATION_METERS) return publish(baseline, trustedSeconds, "UNSTABLE_FOOT_HEIGHT")
		val target = median.coerceIn(baseline - MAX_CORRECTION_METERS, baseline + MAX_CORRECTION_METERS)
		val current = estimatedHeight ?: baseline
		val maximumStep = MAX_SLEW_METERS_PER_SECOND * dtSeconds
		val next = current + (target - current).coerceIn(-maximumStep, maximumStep)
		estimatedHeight = next
		return publish(baseline, trustedSeconds, "LEARNING")
	}

	/** Clears accumulated evidence and returns the estimate to the latest calibrated floor. */
	fun reset() {
		clearEvidence()
		val baseline = calibratedHeight
		estimatedHeight = baseline
		estimate = baseline?.let { AdaptiveFloorEstimate(it, it, 0.0, "RESET") }
	}

	/** Resets evidence and adopts a newly calibrated floor baseline in meters. */
	fun reset(calibratedFloorHeightMeters: Double) {
		clearEvidence()
		if (!calibratedFloorHeightMeters.isFinite()) {
			estimate = null
			estimatedHeight = null
			calibratedHeight = null
			return
		}
		calibratedHeight = calibratedFloorHeightMeters
		estimatedHeight = calibratedFloorHeightMeters
		estimate = AdaptiveFloorEstimate(calibratedFloorHeightMeters, calibratedFloorHeightMeters, 0.0, "RESET")
	}

	private fun reject(baseline: Double, reason: String): AdaptiveFloorEstimate {
		clearEvidence()
		return publish(baseline, 0.0, reason)
	}

	private fun clearEvidence() {
		samples.clear()
		lastTimeNanos = null
		trustedSeconds = 0.0
	}

	private fun publish(baseline: Double, seconds: Double, reason: String): AdaptiveFloorEstimate {
		val safeEstimate = (estimatedHeight ?: baseline).coerceIn(baseline - MAX_CORRECTION_METERS, baseline + MAX_CORRECTION_METERS)
		return AdaptiveFloorEstimate(baseline, safeEstimate, seconds, reason).also { estimate = it }
	}

	private fun planted(contact: FootContactSnapshot): Boolean = contact.state == FootContactState.PLANTED && contact.weight.isFinite() && contact.weight in MIN_CONTACT_WEIGHT..1f

	private fun median(values: List<Double>): Double {
		val sorted = values.sorted()
		val middle = sorted.size / 2
		return if (sorted.size % 2 == 0) (sorted[middle - 1] + sorted[middle]) * 0.5 else sorted[middle]
	}

	private fun finite(v: Vector3) = v.x.isFinite() && v.y.isFinite() && v.z.isFinite()

	companion object {
		private const val MIN_UPRIGHT_CONFIDENCE = 0.9
		private const val MIN_CONTACT_WEIGHT = 0.9f
		private const val MAX_FOOT_DISAGREEMENT_METERS = 0.02
		private const val MAX_CALIBRATED_DEVIATION_METERS = 0.06
		private const val MAX_CORRECTION_METERS = 0.03
		private const val MAX_SLEW_METERS_PER_SECOND = 0.001
		private const val MIN_DWELL_SECONDS = 5.0
		private const val MAX_DWELL_SECONDS = 3600.0
		private const val MAX_MEDIAN_ABSOLUTE_DEVIATION_METERS = 0.008
		private const val MIN_SAMPLES = 21
		private const val MAX_SAMPLES = 301
		private const val WINDOW_NANOS = 120_000_000_000L
	}
}
