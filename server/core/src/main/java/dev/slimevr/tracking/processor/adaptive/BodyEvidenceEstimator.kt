package dev.slimevr.tracking.processor.adaptive

import io.github.axisangles.ktmath.Quaternion
import io.github.axisangles.ktmath.Vector3
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.sqrt

enum class AdaptiveMotionState { STATIONARY, MOVING, HIGH_MOTION, UNKNOWN }

/** Temporal consensus reduces measurement trust without treating anatomy as calibration truth. */
class BodyEvidenceEstimator {
	private data class Prior(val rotation: Quaternion, val timestamp: Long)
	private val prior = mutableMapOf<Int, Prior>()
	var motion = AdaptiveMotionState.UNKNOWN
		private set

	fun reset() {
		prior.clear()
		motion = AdaptiveMotionState.UNKNOWN
	}

	fun observe(frame: AdaptiveTelemetryFrame, chains: List<List<Int>>): AdaptiveTelemetryFrame {
		val changes = mutableMapOf<Int, Vector3>()
		for (sample in frame.samples) {
			val rotation = sample.adjustedRotation ?: continue
			val previous = prior[sample.id]
			if (sample.continuousObservation && previous != null && frame.timestampNanos - previous.timestamp in 1..250_000_000L) {
				rotationChange(previous.rotation, rotation)?.let { changes[sample.id] = it }
			}
			prior[sample.id] = Prior(rotation, frame.timestampNanos)
		}
		prior.keys.retainAll(frame.samples.map { it.id }.toSet())
		val speeds = frame.samples.mapNotNull { it.angularSpeedRadiansPerSecond }
		motion = when {
			speeds.isEmpty() -> AdaptiveMotionState.UNKNOWN
			speeds.any { it > 8f } -> AdaptiveMotionState.HIGH_MOTION
			speeds.all { it < 0.02f } -> AdaptiveMotionState.STATIONARY
			else -> AdaptiveMotionState.MOVING
		}
		return frame.copy(
			samples = frame.samples.map { sample ->
				val confidence = sample.confidence ?: return@map sample
				val delta = changes[sample.id] ?: return@map sample
				val neighbors = chains.filter { sample.id in it }.flatten().distinct().filter { it != sample.id }.mapNotNull { changes[it] }
				if (neighbors.size < 2) return@map sample
				val median = Vector3(
					neighbors.map { it.x }.sorted()[neighbors.size / 2],
					neighbors.map { it.y }.sorted()[neighbors.size / 2],
					neighbors.map { it.z }.sorted()[neighbors.size / 2],
				)
				if (neighbors.count { (it - median).len() <= 0.08f } < 2) return@map sample
				if (median.len() >= 0.05f) return@map sample
				val disagreement = (delta - median).len()
				if (disagreement < 0.15f) return@map sample
				val reason = if (disagreement > 0.44f && median.len() < 0.05f) "POSSIBLE_MOUNTING_SHIFT_OR_ARTICULATION" else "TEMPORAL_NEIGHBOR_DISAGREEMENT"
				val factor = (1f - disagreement * 0.5f).coerceIn(0.4f, 1f)
				sample.copy(confidence = confidence.copy(score = confidence.score * factor, reasons = confidence.reasons + reason))
			},
		)
	}

	private fun rotationChange(previous: Quaternion, current: Quaternion): Vector3? {
		if (!valid(previous) || !valid(current)) return null
		val relative = (current.unit() * previous.unit().inv()).unit()
		val sign = if (relative.w < 0f) -1f else 1f
		val x = relative.x * sign
		val y = relative.y * sign
		val z = relative.z * sign
		val vectorLength = sqrt(x * x + y * y + z * z)
		if (!vectorLength.isFinite()) return null
		if (vectorLength < 1e-8f) return Vector3.NULL
		val angle = 2f * atan2(vectorLength, abs(relative.w))
		return Vector3(x * angle / vectorLength, y * angle / vectorLength, z * angle / vectorLength)
	}

	private fun valid(q: Quaternion) = q.w.isFinite() && q.x.isFinite() && q.y.isFinite() && q.z.isFinite() && q.lenSq().isFinite() && q.lenSq() > 0f
}
