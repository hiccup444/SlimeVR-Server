package dev.slimevr.tracking.processor.adaptive

import io.github.axisangles.ktmath.Quaternion
import io.github.axisangles.ktmath.Vector3
import kotlin.math.atan2
import kotlin.math.exp
import kotlin.math.sqrt

/** A bounded diagnostic of expected-versus-measured orientation error. Angles are radians. */
data class PoseResidualDiagnostic(
	val trackerId: Int,
	/** Shortest-arc rotation vector for expected^-1 * measured, in expected-local axes. */
	val residualVectorRadians: Vector3,
	val magnitudeRadians: Double,
	val directionalConsistency: Double?,
	val continuousSeconds: Double,
	val meanMagnitudeRadians: Double,
	val magnitudeVarianceRadiansSquared: Double,
	val sampleCount: Int,
	val independentlyConstrained: Boolean,
	val reason: String,
)

/**
 * Tracks bounded residual history for diagnostics only. A solver-conditioned expected rotation is
 * not independent truth and this class never decides whether a residual should change tracking.
 */
class PoseResidualMonitor(
	private val maximumTrackers: Int = 64,
	private val historyLimit: Int = 120,
	private val maximumGapNanos: Long = 500_000_000L,
	private val discontinuityRadians: Double = Math.toRadians(30.0),
	private val smoothingSeconds: Double = 2.0,
) {
	private data class Sample(val vector: Vector3, val magnitude: Double)
	private class State {
		val history = ArrayDeque<Sample>()
		var lastTimeNanos: Long? = null
		var lastVector: Vector3? = null
		var continuousSeconds = 0.0
		var meanMagnitude = 0.0
		var magnitudeVariance = 0.0
		var sampleCount = 0
	}

	private val states = LinkedHashMap<Int, State>()

	init {
		require(maximumTrackers > 0)
		require(historyLimit > 0)
		require(maximumGapNanos > 0)
		require(discontinuityRadians.isFinite() && discontinuityRadians > 0.0)
		require(smoothingSeconds.isFinite() && smoothingSeconds > 0.0)
	}

	/** Returns null for invalid quaternions or non-forward timestamps and clears that tracker's run. */
	fun observe(
		id: Int,
		measured: Quaternion,
		expected: Quaternion,
		now: Long,
		independentlyConstrained: Boolean = false,
	): PoseResidualDiagnostic? {
		val residual = residualVector(measured, expected)
		if (residual == null) {
			reset(id)
			return null
		}
		var state = states[id]
		if (state == null) {
			if (states.size >= maximumTrackers) {
				val oldestId = states.minByOrNull { it.value.lastTimeNanos ?: Long.MIN_VALUE }?.key
				if (oldestId != null) states.remove(oldestId)
			}
			state = State()
			states[id] = state
		}

		val previousTime = state.lastTimeNanos
		val deltaNanos = previousTime?.let { now - it }
		if (deltaNanos != null && deltaNanos <= 0L) {
			reset(id)
			return null
		}
		var reason = "OBSERVING"
		var dtSeconds = 0.0
		if (deltaNanos != null && deltaNanos <= maximumGapNanos) {
			val previousVector = state.lastVector
			if (previousVector != null && vectorDistance(previousVector, residual) > discontinuityRadians) {
				clearRun(state)
				reason = "RESIDUAL_DISCONTINUITY"
			} else {
				dtSeconds = deltaNanos * 1e-9
				state.continuousSeconds += dtSeconds
			}
		} else if (deltaNanos != null) {
			clearRun(state)
			reason = "TIME_GAP"
		}

		val magnitude = vectorLength(residual)
		val alpha = if (state.sampleCount == 0) 1.0 else 1.0 - exp(-dtSeconds / smoothingSeconds)
		val difference = magnitude - state.meanMagnitude
		state.meanMagnitude += alpha * difference
		state.magnitudeVariance = (1.0 - alpha) * (state.magnitudeVariance + alpha * difference * difference)
		if (state.sampleCount < Int.MAX_VALUE) state.sampleCount++
		state.history.addLast(Sample(residual, magnitude))
		while (state.history.size > historyLimit) state.history.removeFirst()
		state.lastTimeNanos = now
		state.lastVector = residual
		states[id] = state

		return PoseResidualDiagnostic(
			trackerId = id,
			residualVectorRadians = residual,
			magnitudeRadians = magnitude,
			directionalConsistency = directionalConsistency(state.history),
			continuousSeconds = state.continuousSeconds,
			meanMagnitudeRadians = state.meanMagnitude,
			magnitudeVarianceRadiansSquared = state.magnitudeVariance,
			sampleCount = state.sampleCount,
			independentlyConstrained = independentlyConstrained,
			reason = reason,
		)
	}

	fun reset(id: Int) {
		states.remove(id)
	}

	fun reset() {
		states.clear()
	}

	/** Drops histories for trackers that are no longer part of the active set. */
	fun retain(trackerIds: Set<Int>) {
		states.keys.retainAll(trackerIds)
	}

	private fun clearRun(state: State) {
		state.history.clear()
		state.lastVector = null
		state.continuousSeconds = 0.0
		state.meanMagnitude = 0.0
		state.magnitudeVariance = 0.0
		state.sampleCount = 0
	}

	private fun residualVector(measured: Quaternion, expected: Quaternion): Vector3? {
		if (!valid(expected) || !valid(measured)) return null
		val e = expected.unit()
		val m = measured.unit()
		val q = e.inv() * m
		val normSq = q.lenSq()
		if (!normSq.isFinite() || normSq <= MIN_QUATERNION_NORM_SQUARED) return null
		val invNorm = 1.0 / sqrt(normSq.toDouble())
		val w = q.w.toDouble() * invNorm
		var x = q.x.toDouble() * invNorm
		var y = q.y.toDouble() * invNorm
		var z = q.z.toDouble() * invNorm
		var canonicalW = w
		val tieBreakFlip = canonicalW == 0.0 &&
			(x < 0.0 || (x == 0.0 && (y < 0.0 || (y == 0.0 && z < 0.0))))
		if (canonicalW < 0.0 || tieBreakFlip) {
			canonicalW = -canonicalW
			x = -x
			y = -y
			z = -z
		}
		val vectorNorm = sqrt(x * x + y * y + z * z)
		if (!vectorNorm.isFinite()) return null
		if (vectorNorm < VECTOR_EPSILON) return Vector3(0f, 0f, 0f)
		val angle = 2.0 * atan2(vectorNorm, canonicalW)
		val scale = angle / vectorNorm
		val result = Vector3((x * scale).toFloat(), (y * scale).toFloat(), (z * scale).toFloat())
		return result.takeIf { it.x.isFinite() && it.y.isFinite() && it.z.isFinite() }
	}

	private fun valid(q: Quaternion): Boolean {
		val normSq = q.lenSq()
		return q.w.isFinite() && q.x.isFinite() && q.y.isFinite() && q.z.isFinite() && normSq.isFinite() && normSq > MIN_QUATERNION_NORM_SQUARED
	}

	private fun directionalConsistency(history: Collection<Sample>): Double? {
		var sumX = 0.0
		var sumY = 0.0
		var sumZ = 0.0
		var count = 0
		for (sample in history) {
			if (sample.magnitude <= VECTOR_EPSILON) continue
			val inverseMagnitude = 1.0 / sample.magnitude
			sumX += sample.vector.x.toDouble() * inverseMagnitude
			sumY += sample.vector.y.toDouble() * inverseMagnitude
			sumZ += sample.vector.z.toDouble() * inverseMagnitude
			count++
		}
		if (count == 0) return null
		return (sqrt(sumX * sumX + sumY * sumY + sumZ * sumZ) / count).coerceIn(0.0, 1.0)
	}

	private fun vectorLength(v: Vector3) = sqrt(v.x.toDouble() * v.x.toDouble() + v.y.toDouble() * v.y.toDouble() + v.z.toDouble() * v.z.toDouble())
	private fun vectorDistance(a: Vector3, b: Vector3) = vectorLength(Vector3(a.x - b.x, a.y - b.y, a.z - b.z))

	private companion object {
		const val MIN_QUATERNION_NORM_SQUARED = 1e-12f
		const val VECTOR_EPSILON = 1e-8
	}
}
