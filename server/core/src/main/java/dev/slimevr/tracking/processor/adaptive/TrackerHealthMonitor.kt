package dev.slimevr.tracking.processor.adaptive

import io.github.axisangles.ktmath.Quaternion
import io.github.axisangles.ktmath.Vector3
import java.util.ArrayDeque
import kotlin.math.abs
import kotlin.math.acos
import kotlin.math.exp
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sqrt

data class TrackerMotionContext(
	/** Target-specific movement corroborated independently of this tracker's orientation. */
	val independentTargetMovementScore: Double = 0.0,
	val timestampNanos: Long? = null,
)

data class TrackerHealthDiagnostic(
	val trackerId: Int,
	val qualityMultiplier: Float,
	val stationaryConfidence: Float,
	val reasons: List<String>,
	val observedSeconds: Double,
	val suspectedFrozen: Boolean,
)

/** Reports sensor-health evidence without changing tracker measurements or state. */
class TrackerHealthMonitor(private val maxTrackers: Int = 64) {
	private data class Rotation(val w: Double, val x: Double, val y: Double, val z: Double) {
		fun dot(other: Rotation) = w * other.w + x * other.x + y * other.y + z * other.z
		fun negated() = Rotation(-w, -x, -y, -z)
		fun conjugate() = Rotation(w, -x, -y, -z)
		fun multiply(b: Rotation) = Rotation(
			w * b.w - x * b.x - y * b.y - z * b.z,
			w * b.x + x * b.w + y * b.z - z * b.y,
			w * b.y - x * b.z + y * b.w + z * b.x,
			w * b.z + x * b.y - y * b.x + z * b.w,
		)
		fun angleTo(other: Rotation): Double = 2.0 * acos(abs(dot(other)).coerceIn(0.0, 1.0))
	}
	private data class Step(val time: Long, val orientation: Rotation, val angularVelocity: DoubleArray, val angle: Double)
	private data class History(
		var lastTime: Long,
		var lastOrientation: Rotation?,
		var quality: Float,
		var observedSeconds: Double,
		var unchangedSince: Long?,
		var lastArrivalNanos: Long?,
		var lastAngularVelocity: DoubleArray?,
		val steps: ArrayDeque<Step> = ArrayDeque(),
	)
	private data class RotationChange(val orientation: Rotation, val angle: Double, val angularVelocity: DoubleArray)

	private val histories = LinkedHashMap<Int, History>()

	init {
		require(maxTrackers > 0)
	}

	fun reset() = histories.clear()

	fun resetTracker(trackerId: Int) {
		histories.remove(trackerId)
	}

	fun observe(sample: TelemetrySample, nowNanos: Long, context: TrackerMotionContext = TrackerMotionContext()): TrackerHealthDiagnostic {
		val reasons = mutableListOf<String>()
		val prior = histories[sample.id]
		val age = sample.packetAgeNanos
		val rotation = sample.rawRotation?.let(::normalize)
		if (!sample.rotationExpected) {
			histories.remove(sample.id)
			return TrackerHealthDiagnostic(sample.id, 1f, 0f, listOf("ORIENTATION_NOT_EXPECTED"), 0.0, false)
		}
		val hardFailure = when {
			!sample.status.sendData -> "TRACKING_UNAVAILABLE"
			sample.rotationExpected && rotation == null -> "INVALID_ORIENTATION"
			age != null && age < 0 -> "INVALID_PACKET_AGE"
			age != null && age >= STALE_PACKET_NANOS -> "STALE_PACKET"
			else -> null
		}
		if (hardFailure != null) {
			histories.remove(sample.id)
			return TrackerHealthDiagnostic(sample.id, 0f, 0f, listOf(hardFailure), 0.0, false)
		}
		if (rotation == null) {
			histories.remove(sample.id)
			return TrackerHealthDiagnostic(sample.id, 0f, 0f, listOf("ROTATION_UNAVAILABLE"), 0.0, false)
		}

		val gap = prior?.let { nowNanos - it.lastTime }
		val continuous = prior != null && gap != null && gap in 1..MAX_SAMPLE_GAP_NANOS && sample.continuousObservation
		val arrivalNanos = age?.let { nowNanos - it }
		val previousArrival = prior?.lastArrivalNanos
		val history = if (continuous) {
			prior!!
		} else {
			History(nowNanos, rotation, INITIAL_QUALITY, 0.0, null, arrivalNanos, null).also {
				if (prior != null) reasons.add("CONTINUITY_RESET")
			}
		}
		val dt = if (continuous) gap!! * 1e-9 else 0.0
		if (continuous) history.observedSeconds = min(MAX_OBSERVATION_SECONDS, history.observedSeconds + dt)

		var target = 1f
		if (age == null) {
			target = min(target, UNKNOWN_FRESHNESS_QUALITY)
			reasons.add("FRESHNESS_UNKNOWN")
		} else if (age > AGING_PACKET_NANOS) {
			target *= (1.0 - ((age - AGING_PACKET_NANOS).toDouble() / (STALE_PACKET_NANOS - AGING_PACKET_NANOS)).coerceIn(0.0, 1.0)).toFloat()
			reasons.add("AGING_PACKET")
		}
		val acceleration = sample.acceleration
		if (acceleration != null && sample.accelerationAgeNanos?.let { it in 0..MAX_CONTEXT_AGE_NANOS } == true) {
			val magnitudeSq = acceleration.x.toDouble() * acceleration.x + acceleration.y.toDouble() * acceleration.y + acceleration.z.toDouble() * acceleration.z
			if (!magnitudeSq.isFinite() || sqrt(magnitudeSq) > MAX_ACCELERATION_METERS_PER_SECOND_SQUARED) {
				target = min(target, INVALID_SENSOR_QUALITY)
				reasons.add("IMPOSSIBLE_ACCELERATION")
			}
		}

		var rotationChange: RotationChange? = null
		if (continuous) {
			val previousOrientation = prior!!.lastOrientation
			if (previousOrientation != null) {
				rotationChange = rotationChange(previousOrientation, rotation, dt)
				val previousSpeed = prior.lastAngularVelocity?.let { velocity -> sqrt(velocity.sumOf { it * it }) }
				val expectedChange = previousSpeed?.times(dt) ?: 0.0
				if (rotationChange.angle >= DISCONTINUITY_MIN_ANGLE && rotationChange.angle > max(DISCONTINUITY_BASE_ANGLE, expectedChange + DISCONTINUITY_ALLOWANCE)) {
					target = min(target, DISCONTINUITY_QUALITY)
					reasons.add("ORIENTATION_DISCONTINUITY")
				}
				history.steps.addLast(Step(nowNanos, rotation, rotationChange.angularVelocity, rotationChange.angle))
				while (history.steps.size > MAX_NOISE_STEPS) history.steps.removeFirst()
				if (isAlternatingNoise(history.steps)) {
					target = min(target, NOISE_QUALITY)
					reasons.add("ALTERNATING_HIGH_FREQUENCY_NOISE")
				}
				history.lastAngularVelocity = rotationChange.angularVelocity
			}
		}
		val packetAgeNanos = age ?: STALE_PACKET_NANOS

		val freshArrivals = sample.continuousObservation && age != null && packetAgeNanos <= FRESH_PACKET_NANOS && previousArrival != null && arrivalNanos != null && arrivalNanos > previousArrival && arrivalNanos - previousArrival <= MAX_ARRIVAL_GAP_NANOS
		val contextScoreValid = context.independentTargetMovementScore.isFinite() && context.independentTargetMovementScore in 0.0..1.0
		val independentMotion = contextScoreValid &&
			context.independentTargetMovementScore >= MIN_INDEPENDENT_MOTION_SCORE &&
			context.timestampNanos?.let { nowNanos - it in 0..MAX_CONTEXT_AGE_NANOS } == true
		val change = rotationChange
		val unchanged = continuous && change != null && change.angle <= UNCHANGED_ANGLE_EPSILON
		if (unchanged && independentMotion && packetAgeNanos <= FRESH_PACKET_NANOS) {
			if (history.unchangedSince == null) history.unchangedSince = nowNanos
		} else {
			history.unchangedSince = null
		}
		val frozen = freshArrivals && independentMotion && unchanged && history.unchangedSince?.let { nowNanos - it >= FROZEN_DWELL_NANOS } == true
		if (frozen) {
			target = min(target, FROZEN_QUALITY)
			reasons.add("FROZEN_WITH_INDEPENDENT_TARGET_MOTION")
		}

		if (history.observedSeconds < WARMUP_SECONDS) {
			target = min(target, (INITIAL_QUALITY + (1f - INITIAL_QUALITY) * (history.observedSeconds / WARMUP_SECONDS).toFloat()))
			reasons.add("WARMING_UP")
		}
		val currentQuality = if (target < history.quality || !continuous) {
			target
		} else {
			val blend = (1.0 - exp(-dt / RECOVERY_TIME_CONSTANT_SECONDS)).toFloat()
			history.quality + (target - history.quality) * blend
		}
		history.quality = currentQuality.coerceIn(0f, 1f)
		history.lastTime = nowNanos
		history.lastOrientation = rotation
		history.lastArrivalNanos = arrivalNanos
		histories[sample.id] = history
		while (histories.size > maxTrackers) histories.remove(histories.keys.first())

		val derivedSpeed = rotationChange?.let { sqrt(it.angularVelocity.sumOf { component -> component * component }) }
		val speed = sample.angularSpeedRadiansPerSecond?.takeIf { it.isFinite() }?.toDouble() ?: derivedSpeed
		val stationary = speed?.let { (1.0 - it / MAX_STATIONARY_SPEED).coerceIn(0.0, 1.0) }?.toFloat()?.times(history.quality) ?: 0f
		if (reasons.isEmpty()) reasons.add("HEALTHY")
		if (!contextScoreValid) reasons.add("INVALID_MOTION_CONTEXT")
		return TrackerHealthDiagnostic(sample.id, history.quality, stationary.coerceIn(0f, 1f), reasons, history.observedSeconds, frozen)
	}

	private fun normalize(q: Quaternion): Rotation? {
		val values = listOf(q.w.toDouble(), q.x.toDouble(), q.y.toDouble(), q.z.toDouble())
		if (values.any { !it.isFinite() }) return null
		val norm = sqrt(values.sumOf { it * it })
		if (!norm.isFinite() || norm !in MIN_QUATERNION_NORM..MAX_QUATERNION_NORM) return null
		return Rotation(q.w.toDouble() / norm, q.x.toDouble() / norm, q.y.toDouble() / norm, q.z.toDouble() / norm)
	}

	private fun rotationChange(previous: Rotation, current: Rotation, seconds: Double): RotationChange {
		var end = if (previous.dot(current) < 0.0) current.negated() else current
		var relative = previous.conjugate().multiply(end)
		if (relative.w < 0.0) {
			end = end.negated()
			relative = relative.negated()
		}
		val w = relative.w.coerceIn(-1.0, 1.0)
		val angle = 2.0 * acos(w)
		val vectorLength = sqrt(relative.x * relative.x + relative.y * relative.y + relative.z * relative.z)
		val scale = if (vectorLength < 1e-9 || seconds <= 0.0) 0.0 else angle / vectorLength / seconds
		return RotationChange(
			end,
			angle,
			doubleArrayOf(relative.x * scale, relative.y * scale, relative.z * scale),
		)
	}

	private fun isAlternatingNoise(steps: ArrayDeque<Step>): Boolean {
		if (steps.size < MIN_NOISE_STEPS) return false
		val recent = steps.toList().takeLast(MIN_NOISE_STEPS)
		if (recent.zipWithNext().any { (a, b) -> b.time - a.time !in 1..MAX_NOISE_INTERVAL_NANOS }) return false
		if (recent.any { sqrt(it.angularVelocity.sumOf { component -> component * component }) < MIN_NOISE_ANGULAR_SPEED }) return false
		var reversals = 0
		for ((a, b) in recent.zipWithNext()) {
			val dot = a.angularVelocity.indices.sumOf { index -> a.angularVelocity[index] * b.angularVelocity[index] }
			val aNorm = sqrt(a.angularVelocity.sumOf { it * it })
			val bNorm = sqrt(b.angularVelocity.sumOf { it * it })
			if (aNorm > 0.0 && bNorm > 0.0 && dot / (aNorm * bNorm) < NOISE_REVERSAL_COSINE) reversals++
		}
		val netAngle = recent.first().orientation.angleTo(recent.last().orientation)
		val pathLength = recent.sumOf { it.angle }
		return reversals >= MIN_NOISE_REVERSALS && netAngle <= MAX_NOISE_NET_ANGLE && pathLength >= MIN_NOISE_PATH_ANGLE
	}

	private companion object {
		const val INITIAL_QUALITY = 0.55f
		const val INVALID_SENSOR_QUALITY = 0.08f
		const val UNKNOWN_FRESHNESS_QUALITY = 0.7f
		const val DISCONTINUITY_QUALITY = 0.25f
		const val NOISE_QUALITY = 0.3f
		const val FROZEN_QUALITY = 0.2f
		const val STALE_PACKET_NANOS = 500_000_000L
		const val AGING_PACKET_NANOS = 100_000_000L
		const val FRESH_PACKET_NANOS = 100_000_000L
		const val MAX_SAMPLE_GAP_NANOS = 500_000_000L
		const val MAX_CONTEXT_AGE_NANOS = 250_000_000L
		const val MAX_ARRIVAL_GAP_NANOS = 250_000_000L
		const val MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 40.0
		const val MIN_QUATERNION_NORM = 0.5
		const val MAX_QUATERNION_NORM = 1.5
		const val DISCONTINUITY_MIN_ANGLE = 1.0
		const val DISCONTINUITY_BASE_ANGLE = 1.2
		const val DISCONTINUITY_ALLOWANCE = 0.35
		const val UNCHANGED_ANGLE_EPSILON = 0.0005
		const val FROZEN_DWELL_NANOS = 2_000_000_000L
		const val MIN_INDEPENDENT_MOTION_SCORE = 0.8
		const val WARMUP_SECONDS = 1.5
		const val MAX_OBSERVATION_SECONDS = 3600.0
		const val RECOVERY_TIME_CONSTANT_SECONDS = 2.0
		const val MAX_STATIONARY_SPEED = 0.3
		const val MIN_NOISE_STEPS = 8
		const val MAX_NOISE_STEPS = 16
		const val MAX_NOISE_INTERVAL_NANOS = 120_000_000L
		const val MIN_NOISE_ANGULAR_SPEED = 2.5
		const val NOISE_REVERSAL_COSINE = -0.6
		const val MIN_NOISE_REVERSALS = 5
		const val MAX_NOISE_NET_ANGLE = 0.25
		const val MIN_NOISE_PATH_ANGLE = 0.45
	}
}
