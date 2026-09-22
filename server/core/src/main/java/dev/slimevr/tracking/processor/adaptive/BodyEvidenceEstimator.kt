package dev.slimevr.tracking.processor.adaptive

import io.github.axisangles.ktmath.Quaternion
import kotlin.math.abs

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
		val changes = mutableMapOf<Int, Float>()
		for (sample in frame.samples) {
			val rotation = sample.adjustedRotation ?: continue
			val previous = prior[sample.id]
			if (sample.continuousObservation && previous != null && frame.timestampNanos - previous.timestamp in 1..250_000_000L) {
				changes[sample.id] = rotation.angleToR(previous.rotation)
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
				val median = neighbors.sorted()[neighbors.size / 2]
				val disagreement = abs(delta - median)
				if (disagreement < 0.15f) return@map sample
				val reason = if (disagreement > 0.44f && median < 0.05f) "POSSIBLE_MOUNTING_SHIFT_OR_ARTICULATION" else "TEMPORAL_NEIGHBOR_DISAGREEMENT"
				val factor = (1f - disagreement * 0.5f).coerceIn(0.4f, 1f)
				sample.copy(confidence = confidence.copy(score = confidence.score * factor, reasons = confidence.reasons + reason))
			},
		)
	}
}
