package dev.slimevr.tracking.processor.adaptive

import java.util.Collections
import kotlin.math.floor

/** Learns temperature-dependent yaw drift from trusted external observations for one hardware ID. */
class CalibrationProfile(
	val hardwareId: String,
	private val bucketWidthCelsius: Double = DEFAULT_BUCKET_WIDTH_CELSIUS,
	private val minimumObservationSeconds: Double = DEFAULT_MINIMUM_OBSERVATION_SECONDS,
	private val minimumConfidence: Double = DEFAULT_MINIMUM_CONFIDENCE,
	private val maxBuckets: Int = DEFAULT_MAX_BUCKETS,
) {
	private data class MutableBucket(
		val temperatureCelsius: Double,
		var meanRate: Double = 0.0,
		var observationSeconds: Double = 0.0,
		var meanConfidence: Double = 0.0,
	)

	private val buckets = sortedMapOf<Double, MutableBucket>()

	init {
		require(hardwareId.isNotBlank())
		require(bucketWidthCelsius.isFinite() && bucketWidthCelsius == DEFAULT_BUCKET_WIDTH_CELSIUS)
		require(minimumObservationSeconds.isFinite() && minimumObservationSeconds in DEFAULT_MINIMUM_OBSERVATION_SECONDS..MAX_ACCUMULATED_SECONDS)
		require(minimumConfidence.isFinite() && minimumConfidence in DEFAULT_MINIMUM_CONFIDENCE..1.0)
		require(maxBuckets in 1..DEFAULT_MAX_BUCKETS)
	}

	/** Clears all learned temperature evidence for this hardware ID. */
	fun reset() = buckets.clear()

	/**
	 * Adds an externally validated observation. Temperature is in Celsius, yaw drift rate in
	 * radians per second, elapsedSeconds is the trusted duration represented by the observation,
	 * and confidence is in [0, 1]. No orientation-derived velocity or gyro data is inferred here.
	 * Returns false when the input is invalid, insufficiently trusted, out of range, or an outlier.
	 */
	fun update(temperatureCelsius: Double, rateRadiansPerSecond: Double, elapsedSeconds: Double, confidence: Double): Boolean {
		if (!temperatureCelsius.isFinite() ||
			temperatureCelsius !in MIN_TEMPERATURE_C..MAX_TEMPERATURE_C ||
			!rateRadiansPerSecond.isFinite() ||
			kotlin.math.abs(rateRadiansPerSecond) > MAX_ABSOLUTE_RATE ||
			!elapsedSeconds.isFinite() ||
			elapsedSeconds <= 0.0 ||
			elapsedSeconds > MAX_OBSERVATION_SECONDS ||
			!confidence.isFinite() ||
			confidence !in minimumConfidence..1.0
		) {
			return false
		}

		val center = floor(temperatureCelsius / bucketWidthCelsius + 0.5) * bucketWidthCelsius
		val bucket = buckets[center]
		if (bucket == null && buckets.size >= maxBuckets) return false
		if (bucket != null &&
			bucket.observationSeconds >= minimumObservationSeconds &&
			kotlin.math.abs(rateRadiansPerSecond - bucket.meanRate) > MAX_OUTLIER_DEVIATION
		) {
			return false
		}

		val target = bucket ?: MutableBucket(center).also { buckets[center] = it }
		val acceptedSeconds = elapsedSeconds
		if (acceptedSeconds > 0.0) {
			val total = target.observationSeconds + acceptedSeconds
			target.meanRate += (rateRadiansPerSecond - target.meanRate) * acceptedSeconds / total
			target.meanConfidence += (confidence - target.meanConfidence) * acceptedSeconds / total
			target.observationSeconds = minOf(total, MAX_ACCUMULATED_SECONDS)
		}
		return true
	}

	/** Returns an interpolated yaw drift rate in radians per second, or null without enough evidence. */
	fun predictRate(temperatureCelsius: Double): Double? {
		if (!temperatureCelsius.isFinite() || temperatureCelsius !in MIN_TEMPERATURE_C..MAX_TEMPERATURE_C) return null
		val ready = buckets.values.filter {
			it.observationSeconds >= minimumObservationSeconds && it.meanConfidence >= minimumConfidence
		}
		if (ready.isEmpty()) return null
		val exact = ready.firstOrNull { it.temperatureCelsius == temperatureCelsius }
		if (exact != null) return exact.meanRate
		val lower = ready.lastOrNull { it.temperatureCelsius < temperatureCelsius }
		val upper = ready.firstOrNull { it.temperatureCelsius > temperatureCelsius }
		if (lower == null) return upper?.takeIf { it.temperatureCelsius - temperatureCelsius <= bucketWidthCelsius / 2.0 }?.meanRate
		if (upper == null) return lower.takeIf { temperatureCelsius - it.temperatureCelsius <= bucketWidthCelsius / 2.0 }?.meanRate
		if (upper.temperatureCelsius - lower.temperatureCelsius > bucketWidthCelsius * 2.0) {
			if (temperatureCelsius - lower.temperatureCelsius <= bucketWidthCelsius / 2.0) return lower.meanRate
			if (upper.temperatureCelsius - temperatureCelsius <= bucketWidthCelsius / 2.0) return upper.meanRate
			return null
		}
		val fraction = (temperatureCelsius - lower.temperatureCelsius) / (upper.temperatureCelsius - lower.temperatureCelsius)
		return (lower.meanRate + (upper.meanRate - lower.meanRate) * fraction).coerceIn(-MAX_ABSOLUTE_RATE, MAX_ABSOLUTE_RATE)
	}

	/** Exports an immutable snapshot containing only mature, trusted buckets. */
	fun snapshot(): Snapshot = Snapshot(
		hardwareId,
		bucketWidthCelsius,
		minimumObservationSeconds,
		minimumConfidence,
		Collections.unmodifiableList(
			buckets.values.filter {
				it.observationSeconds >= minimumObservationSeconds && it.meanConfidence >= minimumConfidence
			}.map {
				BucketSnapshot(it.temperatureCelsius, it.meanRate, it.observationSeconds, it.meanConfidence)
			},
		),
	)

	data class BucketSnapshot(
		val temperatureCelsius: Double,
		val rateRadiansPerSecond: Double,
		val observationSeconds: Double,
		val confidence: Double,
	)

	data class Snapshot(
		val hardwareId: String,
		val bucketWidthCelsius: Double,
		val minimumObservationSeconds: Double,
		val minimumConfidence: Double,
		val buckets: List<BucketSnapshot>,
	)

	companion object {
		/** Restores a validated snapshot without allowing persisted values to loosen model limits. */
		fun restore(snapshot: Snapshot): CalibrationProfile {
			require(snapshot.hardwareId.isNotBlank())
			require(snapshot.bucketWidthCelsius.isFinite() && snapshot.bucketWidthCelsius == DEFAULT_BUCKET_WIDTH_CELSIUS)
			require(snapshot.minimumObservationSeconds.isFinite() && snapshot.minimumObservationSeconds >= DEFAULT_MINIMUM_OBSERVATION_SECONDS)
			require(snapshot.minimumConfidence.isFinite() && snapshot.minimumConfidence in DEFAULT_MINIMUM_CONFIDENCE..1.0)
			require(snapshot.buckets.size <= DEFAULT_MAX_BUCKETS)
			val profile = CalibrationProfile(
				snapshot.hardwareId,
				snapshot.bucketWidthCelsius,
				snapshot.minimumObservationSeconds,
				snapshot.minimumConfidence,
				DEFAULT_MAX_BUCKETS,
			)
			for (saved in snapshot.buckets) {
				require(saved.temperatureCelsius.isFinite() && saved.temperatureCelsius in MIN_TEMPERATURE_C..MAX_TEMPERATURE_C)
				require(saved.rateRadiansPerSecond.isFinite() && kotlin.math.abs(saved.rateRadiansPerSecond) <= MAX_ABSOLUTE_RATE)
				require(saved.observationSeconds.isFinite() && saved.observationSeconds in snapshot.minimumObservationSeconds..MAX_ACCUMULATED_SECONDS)
				require(saved.confidence.isFinite() && saved.confidence in snapshot.minimumConfidence..1.0)
				val center = floor(saved.temperatureCelsius / snapshot.bucketWidthCelsius + 0.5) * snapshot.bucketWidthCelsius
				require(saved.temperatureCelsius == center && !profile.buckets.containsKey(center))
				profile.buckets[center] = MutableBucket(
					center,
					saved.rateRadiansPerSecond,
					saved.observationSeconds,
					saved.confidence,
				)
			}
			return profile
		}

		const val DEFAULT_BUCKET_WIDTH_CELSIUS = 5.0
		const val DEFAULT_MINIMUM_OBSERVATION_SECONDS = 60.0
		const val DEFAULT_MINIMUM_CONFIDENCE = 0.75
		const val DEFAULT_MAX_BUCKETS = 24
		const val MIN_TEMPERATURE_C = -40.0
		const val MAX_TEMPERATURE_C = 125.0
		const val MAX_ABSOLUTE_RATE = 0.01
		const val MAX_OBSERVATION_SECONDS = 60.0
		const val MAX_ACCUMULATED_SECONDS = 3600.0
		const val MAX_OUTLIER_DEVIATION = 0.002
	}
}
