package dev.slimevr.tracking.processor.adaptive

import dev.slimevr.tracking.trackers.Tracker
import dev.slimevr.tracking.trackers.TrackerPosition
import io.github.axisangles.ktmath.Quaternion
import io.github.axisangles.ktmath.Vector3
import kotlin.math.abs
import kotlin.math.acos
import kotlin.math.sqrt

/** Captures tracker state without changing tracker or pose state. */
class SensorStateManager(
	private val maxHistory: Int = 256,
	private val maxGapNanos: Long = 500_000_000L,
) {
	private data class Key(val computed: Boolean, val id: Int)
	private data class Previous(
		val tracker: Tracker,
		val designation: TrackerPosition?,
		val timestampNanos: Long,
		val rotation: Quaternion?,
		val position: Vector3?,
	)
	private val previous = LinkedHashMap<Key, Previous>()
	private var resetEpoch = 0L

	init {
		require(maxHistory > 0)
		require(maxGapNanos > 0)
	}

	fun sample(trackers: List<Tracker>, computedTrackers: List<Tracker>, timestampNanos: Long): AdaptiveTelemetryFrame {
		val seen = HashSet<Key>()
		fun capture(items: List<Tracker>, computed: Boolean) = items.map { tracker ->
			val key = Key(computed, tracker.id)
			seen.add(key)
			captureOne(key, tracker, timestampNanos)
		}
		val samples = capture(trackers, false)
		val computedSamples = capture(computedTrackers, true)
		previous.keys.retainAll(seen)
		return AdaptiveTelemetryFrame(timestampNanos, samples, computedSamples, resetEpoch)
	}

	fun reset() {
		resetEpoch++
		previous.clear()
	}

	private fun captureOne(key: Key, tracker: Tracker, now: Long): TelemetrySample {
		val raw = if (tracker.hasRotation) tracker.getRawRotation().takeIf(::finite) else null
		val adjusted = if (raw != null) tracker.getRotation().takeIf(::finite) else null
		val position = if (tracker.hasPosition) tracker.position.takeIf(::finite) else null
		val acceleration = if (tracker.hasAcceleration) tracker.getAcceleration().takeIf(::finite) else null
		val temperature = tracker.temperature?.takeIf { it.isFinite() }
		val packetAge = tracker.lastRotationUpdateNanos?.let { now - it }
		val fresh = if (tracker.lastRotationUpdateNanos != null) packetAge != null && packetAge in 0..maxGapNanos else !tracker.usesTimeout
		val usable = tracker.status.sendData && fresh && (!tracker.hasRotation || adjusted != null) && (!tracker.hasPosition || position != null)
		val prior = previous[key]?.takeIf {
			it.tracker === tracker &&
				it.designation == tracker.trackerPosition &&
				now - it.timestampNanos in 1..maxGapNanos &&
				usable
		}
		val seconds = prior?.let { (now - it.timestampNanos) * 1e-9 }
		val speed = if (seconds != null && adjusted != null && prior.rotation != null) {
			(angle(adjusted, prior.rotation) / seconds).toFloat().takeIf { it.isFinite() }
		} else {
			null
		}
		val velocity = if (seconds != null && position != null && prior.position != null) {
			((position - prior.position) / seconds.toFloat()).takeIf(::finite)
		} else {
			null
		}
		if (usable) {
			previous[key] = Previous(tracker, tracker.trackerPosition, now, adjusted, position)
			while (previous.size > maxHistory) previous.remove(previous.keys.first())
		} else {
			previous.remove(key)
		}
		return TelemetrySample(
			tracker.id, tracker.name, tracker.trackerPosition?.trackerRole, tracker.status,
			raw, adjusted, position, acceleration, velocity, packetAge, temperature, speed,
			rotationExpected = tracker.hasRotation,
			positionExpected = tracker.hasPosition,
			continuousObservation = prior != null,
			accelerationAgeNanos = tracker.lastAccelerationUpdateNanos?.let { (now - it).takeIf { age -> age >= 0L } },
			temperatureAgeNanos = tracker.lastTemperatureUpdateNanos?.let { (now - it).takeIf { age -> age >= 0L } },
		)
	}
}

private fun finite(value: Vector3) = value.x.isFinite() && value.y.isFinite() && value.z.isFinite()
private fun finite(value: Quaternion) = value.w.isFinite() && value.x.isFinite() && value.y.isFinite() && value.z.isFinite() && value.lenSq().isFinite() && value.lenSq() > 0f

private fun angle(a: Quaternion, b: Quaternion): Double {
	val dot = a.w.toDouble() * b.w + a.x.toDouble() * b.x + a.y.toDouble() * b.y + a.z.toDouble() * b.z
	fun norm(q: Quaternion) = q.w.toDouble() * q.w + q.x.toDouble() * q.x + q.y.toDouble() * q.y + q.z.toDouble() * q.z
	return 2.0 * acos((abs(dot) / sqrt(norm(a) * norm(b))).coerceIn(0.0, 1.0))
}
