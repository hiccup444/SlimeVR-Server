package dev.slimevr.tracking.processor.adaptive

import io.github.axisangles.ktmath.Vector3
import kotlin.math.sqrt
import kotlin.random.Random

/** Estimates arm segment lengths from known upper-arm directions and stationary HMD geometry. */
class ArmProportionLearner(
	private val baselineUpperArmMeters: Double,
	private val baselineLowerArmMeters: Double,
) {
	private data class Sample(val x: Double, val y: Double, val timeNanos: Long, var trustedSeconds: Double)
	private data class Line(val slope: Double, val intercept: Double)
	private data class Fit(val line: Line, val inliers: List<Sample>, val distanceRms: Double, val equationRms: Double)

	data class ArmLengthEstimate(
		val upperArmMeters: Double,
		val lowerArmMeters: Double,
		val upperArmUncertaintyMeters: Double,
		val lowerArmUncertaintyMeters: Double,
		val lowerArmDistanceRmsMeters: Double,
		val observedSeconds: Double,
		val inlierCount: Int,
	)

	private val samples = ArrayList<Sample>(MAX_SAMPLES)
	private var lastSampleTimeNanos: Long? = null
	private var observedSeconds = 0.0
	private var estimate: ArmLengthEstimate? = null
	private var samplesSinceFit = 0

	init {
		require(baselineUpperArmMeters.isFinite() && baselineUpperArmMeters in MIN_ARM_LENGTH_METERS..MAX_ARM_LENGTH_METERS)
		require(baselineLowerArmMeters.isFinite() && baselineLowerArmMeters in MIN_ARM_LENGTH_METERS..MAX_ARM_LENGTH_METERS)
	}

	/**
	 * Adds a stationary shoulder-to-wrist displacement in meters and an externally known unit
	 * upper-arm direction. Confidence must already include the caller's motion and tracking gates.
	 * Samples are accepted at most 5 Hz. Returns only an estimate supported by diverse geometry,
	 * at least 60 seconds of evidence, and a robust fit consistent with the supplied baselines.
	 */
	fun observe(displacement: Vector3, upperDirection: Vector3, confidence: Float, now: Long): ArmLengthEstimate? {
		pruneOldSamples(now)
		val previous = lastSampleTimeNanos
		if (previous != null && now <= previous) {
			reset()
		}
		if (!validGeometry(displacement, upperDirection) || !confidence.isFinite() || confidence !in MIN_CONFIDENCE..1.0f) {
			resetTemporalClock()
			return null
		}

		var sampleTrustSeconds = 0.0
		val currentPrevious = lastSampleTimeNanos
		if (currentPrevious != null) {
			val delta = now - currentPrevious
			if (delta < MIN_SAMPLE_INTERVAL_NANOS) {
				return estimate
			} else if (delta <= MAX_TRUSTED_GAP_NANOS) {
				sampleTrustSeconds = delta * 1e-9
				observedSeconds += sampleTrustSeconds
			} else {
				resetTemporalClock()
			}
		}

		val upperComponents = vectorComponents(upperDirection)
		val upperLength = sqrt(upperComponents.sumOf { it * it })
		val normalized = upperComponents.map { it / upperLength }
		val d = vectorComponents(displacement)
		val x = d[0] * normalized[0] + d[1] * normalized[1] + d[2] * normalized[2]
		val y = d[0] * d[0] + d[1] * d[1] + d[2] * d[2]
		if (!x.isFinite() || !y.isFinite()) {
			reset()
			return null
		}
		if (samples.size == MAX_SAMPLES) removeOldestSample()
		samples.add(Sample(x, y, now, sampleTrustSeconds))
		lastSampleTimeNanos = now
		samplesSinceFit++

		if (observedSeconds < MIN_OBSERVATION_SECONDS || samples.size < MIN_FIT_SAMPLES) return null
		if (samplesSinceFit < FIT_SAMPLE_STRIDE) {
			estimate = estimate?.copy(observedSeconds = observedSeconds)
			return estimate
		}
		samplesSinceFit = 0
		estimate = fitEstimate()
		return estimate
	}

	/** Clears all pose evidence and any estimate derived from it. */
	fun reset() {
		samples.clear()
		lastSampleTimeNanos = null
		observedSeconds = 0.0
		estimate = null
		samplesSinceFit = 0
	}

	private fun validGeometry(displacement: Vector3, upperDirection: Vector3): Boolean {
		val d = vectorComponents(displacement)
		val u = vectorComponents(upperDirection)
		val dLength = sqrt(d.sumOf { it * it })
		val uLength = sqrt(u.sumOf { it * it })
		return d.all(Double::isFinite) &&
			u.all(Double::isFinite) &&
			dLength.isFinite() &&
			dLength in MIN_DISPLACEMENT_METERS..MAX_DISPLACEMENT_METERS &&
			uLength.isFinite() &&
			kotlin.math.abs(uLength - 1.0) <= MAX_DIRECTION_LENGTH_ERROR
	}

	private fun resetTemporalClock() {
		lastSampleTimeNanos = null
		estimate = null
		samplesSinceFit = 0
	}

	private fun pruneOldSamples(now: Long) {
		while (samples.isNotEmpty() && now - samples.first().timeNanos > MAX_OBSERVATION_AGE_NANOS) removeOldestSample(invalidateEstimate = true)
		if (samples.isEmpty()) {
			observedSeconds = 0.0
			estimate = null
			samplesSinceFit = 0
		}
	}

	private fun removeOldestSample(invalidateEstimate: Boolean = false) {
		val removed = samples.removeAt(0)
		observedSeconds = (observedSeconds - removed.trustedSeconds).coerceAtLeast(0.0)
		if (samples.isNotEmpty()) {
			val crossingInterval = samples.first().trustedSeconds
			observedSeconds = (observedSeconds - crossingInterval).coerceAtLeast(0.0)
			samples.first().trustedSeconds = 0.0
		}
		if (invalidateEstimate) estimate = null
	}

	private fun fitEstimate(): ArmLengthEstimate? {
		val xValues = samples.map { it.x }
		if (!hasDiverseGeometry(xValues)) return null
		val fit = robustFit(samples) ?: return null
		if (fit.inliers.size.toDouble() / samples.size < MIN_INLIER_FRACTION || fit.distanceRms > MAX_LOWER_ARM_DISTANCE_RMS_METERS) return null
		if (!hasDiverseGeometry(fit.inliers.map { it.x })) return null

		val upper = fit.line.slope * 0.5
		val lowerSquared = fit.line.intercept + upper * upper
		if (!upper.isFinite() || upper <= 0.0 || !lowerSquared.isFinite() || lowerSquared <= 0.0) return null
		val lower = sqrt(lowerSquared)
		if (!withinBaseline(upper, baselineUpperArmMeters) || !withinBaseline(lower, baselineLowerArmMeters)) return null

		val meanX = fit.inliers.map { it.x }.average()
		val sxx = fit.inliers.sumOf { (it.x - meanX) * (it.x - meanX) }
		if (!sxx.isFinite() || sxx <= 0.0) return null
		val n = fit.inliers.size.toDouble()
		val slopeUncertainty = fit.equationRms / sqrt(sxx)
		val interceptUncertainty = fit.equationRms * sqrt(1.0 / n + meanX * meanX / sxx)
		val upperUncertainty = slopeUncertainty * 0.5
		val lowerUncertainty = sqrt(interceptUncertainty * interceptUncertainty + (upper * slopeUncertainty) * (upper * slopeUncertainty)) / (2.0 * lower)
		if (!upperUncertainty.isFinite() || !lowerUncertainty.isFinite()) return null

		return ArmLengthEstimate(upper, lower, upperUncertainty, lowerUncertainty, fit.distanceRms, observedSeconds, fit.inliers.size)
	}

	private fun robustFit(data: List<Sample>): Fit? {
		var bestInliers: List<Sample> = emptyList()
		var bestRms = Double.POSITIVE_INFINITY
		val random = Random(data.size * 31 + data.first().timeNanos.toInt())
		for (trial in 0 until RANSAC_TRIALS) {
			val first = data[random.nextInt(data.size)]
			val second = data[random.nextInt(data.size)]
			val deltaX = second.x - first.x
			if (kotlin.math.abs(deltaX) < MIN_PAIR_SEPARATION_METERS) continue
			val line = Line((second.y - first.y) / deltaX, 0.0)
			val candidate = Line(line.slope, first.y - line.slope * first.x)
			if (!plausibleLine(candidate)) continue
			val inliers = data.filter { isInlier(it, candidate) }
			if (inliers.size < bestInliers.size) continue
			val refined = leastSquares(inliers) ?: continue
			val refinedInliers = data.filter { isInlier(it, refined) }
			val finalLine = leastSquares(refinedInliers) ?: continue
			val rms = lowerArmDistanceRms(finalLine, refinedInliers)
			if (refinedInliers.size > bestInliers.size || (refinedInliers.size == bestInliers.size && rms < bestRms)) {
				bestInliers = refinedInliers
				bestRms = rms
				if (bestInliers.size.toDouble() / data.size >= EARLY_EXIT_INLIER_FRACTION && bestRms <= EARLY_EXIT_DISTANCE_RMS_METERS) break
			}
		}
		if (bestInliers.isEmpty()) return null
		val finalLine = leastSquares(bestInliers) ?: return null
		val finalInliers = data.filter { isInlier(it, finalLine) }
		val line = leastSquares(finalInliers) ?: return null
		return Fit(line, finalInliers, lowerArmDistanceRms(line, finalInliers), equationRms(line, finalInliers))
	}

	private fun leastSquares(data: List<Sample>): Line? {
		if (data.size < MIN_FIT_SAMPLES) return null
		val meanX = data.map { it.x }.average()
		val meanY = data.map { it.y }.average()
		val sxx = data.sumOf { (it.x - meanX) * (it.x - meanX) }
		if (!sxx.isFinite() || sxx <= MIN_VARIANCE) return null
		val slope = data.sumOf { (it.x - meanX) * (it.y - meanY) } / sxx
		val intercept = meanY - slope * meanX
		return if (slope.isFinite() && intercept.isFinite()) Line(slope, intercept) else null
	}

	private fun plausibleLine(line: Line): Boolean {
		if (!plausibleSlope(line.slope)) return false
		val upper = line.slope * 0.5
		val lowerSquared = line.intercept + upper * upper
		return lowerSquared > 0.0 && withinBaseline(sqrt(lowerSquared), baselineLowerArmMeters)
	}

	private fun isInlier(sample: Sample, line: Line): Boolean {
		if (!plausibleLine(line)) return false
		val upper = line.slope * 0.5
		val lower = sqrt(line.intercept + upper * upper)
		val measuredLowerSquared = sample.y - 2.0 * upper * sample.x + upper * upper
		if (!measuredLowerSquared.isFinite() || measuredLowerSquared <= 0.0) return false
		return kotlin.math.abs(sqrt(measuredLowerSquared) - lower) <= MAX_INLIER_DISTANCE_METERS
	}

	private fun lowerArmDistanceRms(line: Line, data: List<Sample>): Double {
		val upper = line.slope * 0.5
		val lower = sqrt(line.intercept + upper * upper)
		return sqrt(
			data.sumOf { sample ->
				val measuredSquared = sample.y - 2.0 * upper * sample.x + upper * upper
				val error = if (measuredSquared > 0.0) sqrt(measuredSquared) - lower else -lower
				error * error
			} /
				data.size,
		)
	}

	private fun equationRms(line: Line, data: List<Sample>): Double = sqrt(
		data.sumOf {
			val error = it.y - (line.slope * it.x + line.intercept)
			error * error
		} /
			data.size,
	)

	private fun hasDiverseGeometry(values: List<Double>): Boolean {
		if (values.size < MIN_FIT_SAMPLES) return false
		val mean = values.average()
		val variance = values.sumOf { (it - mean) * (it - mean) } / values.size
		return values.maxOrNull()!! - values.minOrNull()!! >= MIN_POSITION_RANGE_METERS && variance >= MIN_POSITION_VARIANCE
	}

	private fun plausibleSlope(slope: Double): Boolean = slope.isFinite() && withinBaseline(slope * 0.5, baselineUpperArmMeters)

	private fun withinBaseline(value: Double, baseline: Double): Boolean = value.isFinite() &&
		value in MIN_ARM_LENGTH_METERS..MAX_ARM_LENGTH_METERS &&
		kotlin.math.abs(value - baseline) <= baseline * MAX_BASELINE_DEVIATION

	private fun vectorComponents(vector: Vector3) = listOf(vector.x.toDouble(), vector.y.toDouble(), vector.z.toDouble())
	companion object {
		const val MIN_ARM_LENGTH_METERS = 0.15
		const val MAX_ARM_LENGTH_METERS = 0.50
		private const val MIN_DISPLACEMENT_METERS = 0.05
		private const val MAX_DISPLACEMENT_METERS = 1.25
		private const val MAX_DIRECTION_LENGTH_ERROR = 0.02
		private const val MIN_CONFIDENCE = 0.90f
		private const val MIN_SAMPLE_INTERVAL_NANOS = 200_000_000L
		private const val MAX_TRUSTED_GAP_NANOS = 500_000_000L
		private const val MAX_OBSERVATION_AGE_NANOS = 300_000_000_000L
		private const val MIN_OBSERVATION_SECONDS = 60.0
		private const val MAX_SAMPLES = 512
		private const val MIN_FIT_SAMPLES = 60
		private const val FIT_SAMPLE_STRIDE = 5
		private const val RANSAC_TRIALS = 64
		private const val MIN_PAIR_SEPARATION_METERS = 0.05
		private const val MIN_POSITION_RANGE_METERS = 0.15
		private const val MIN_POSITION_VARIANCE = 0.0025
		private const val MIN_VARIANCE = 1e-10
		private const val MAX_INLIER_DISTANCE_METERS = 0.01
		private const val MAX_LOWER_ARM_DISTANCE_RMS_METERS = 0.01
		private const val MIN_INLIER_FRACTION = 0.90
		private const val EARLY_EXIT_INLIER_FRACTION = 0.95
		private const val EARLY_EXIT_DISTANCE_RMS_METERS = 0.001
		private const val MAX_BASELINE_DEVIATION = 0.10
	}
}
