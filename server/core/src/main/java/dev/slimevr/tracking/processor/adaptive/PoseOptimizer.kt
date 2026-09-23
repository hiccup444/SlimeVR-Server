package dev.slimevr.tracking.processor.adaptive

import io.github.axisangles.ktmath.Quaternion
import io.github.axisangles.ktmath.Vector3
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

/** Global segment orientations with fixed lengths and parent-before-child ordering. */
data class PoseSegment(
	val parent: Int,
	val length: Float,
	val rootPosition: Vector3,
	val measured: Quaternion,
	val confidence: Float,
	val fixed: Boolean = false,
	val previous: Quaternion? = null,
	val temporalWeight: Float = 0f,
	val rotationSource: Int = -1,
	val sourceOffset: Quaternion = Quaternion.IDENTITY,
	/** Motion-predicted previous solution; distinct from the measurement being fitted. */
	val initialRotation: Quaternion? = previous,
)

data class PoseAnchor(val segment: Int, val target: Vector3, val weight: Float, val otherSegment: Int = -1)
data class SoftJoint(
	val first: Int,
	val second: Int,
	val weight: Float,
	val hinge: Boolean = false,
	val firstOffset: Quaternion = Quaternion.IDENTITY,
	val secondOffset: Quaternion = Quaternion.IDENTITY,
	val maxSwingRadians: Float? = null,
	val maxTwistRadians: Float? = null,
)
data class OptimizedPose(val rotations: List<Quaternion>, val tails: List<Vector3>, val initialError: Float, val finalError: Float)
data class PoseSolverInput(val segments: List<PoseSegment>, val anchors: List<PoseAnchor>, val joints: List<SoftJoint>)

/** Bounded coordinate descent. Only lower-energy steps are accepted; segment lengths never change. */
class PoseOptimizer {
	fun solve(segments: List<PoseSegment>, anchors: List<PoseAnchor>, joints: List<SoftJoint> = emptyList()): OptimizedPose {
		require(segments.size <= 128)
		segments.forEachIndexed { index, s ->
			require(s.parent in -1 until index && s.length.isFinite() && s.length >= 0f)
			require(valid(s.measured) && valid(s.rootPosition) && s.confidence in 0f..1f)
			require(s.temporalWeight.isFinite() && s.temporalWeight >= 0f && (s.previous == null || valid(s.previous)))
			require(s.rotationSource in -1 until index && valid(s.sourceOffset))
			require(s.initialRotation == null || valid(s.initialRotation))
		}
		anchors.forEach { require(it.segment in segments.indices && it.otherSegment in -1 until segments.size && valid(it.target) && it.weight.isFinite() && it.weight >= 0f) }
		joints.forEach {
			require(it.first in segments.indices && it.second in segments.indices && it.weight.isFinite() && it.weight >= 0f && valid(it.firstOffset) && valid(it.secondOffset))
			require(it.maxSwingRadians == null || (it.maxSwingRadians.isFinite() && it.maxSwingRadians in 0f..Math.PI.toFloat()))
			require(it.maxTwistRadians == null || (it.maxTwistRadians.isFinite() && it.maxTwistRadians in 0f..Math.PI.toFloat()))
		}
		val swingLimits = joints.map { it.maxSwingRadians?.let { angle -> sin(angle * 0.5f) } }
		val twistLimits = joints.map { it.maxTwistRadians?.let { angle -> sin(angle * 0.5f) } }
		val rotations = segments.map { it.measured.unit() }.toMutableList()
		val tails = MutableList(segments.size) { Vector3.NULL }
		fun error(): Float {
			var total = 0f
			for (i in segments.indices) {
				val s = segments[i]
				if (s.rotationSource >= 0) rotations[i] = (rotations[s.rotationSource] * s.sourceOffset).unit()
				val head = if (s.parent < 0) s.rootPosition else tails[s.parent]
				tails[i] = head + rotations[i].sandwich(Vector3(0f, -s.length, 0f))
				total += (0.05f + s.confidence * 2f) * distance(rotations[i], s.measured)
				s.previous?.let { total += s.temporalWeight * distance(rotations[i], it) }
			}
			for (a in anchors) {
				val target = if (a.otherSegment < 0) a.target else tails[a.otherSegment]
				val d = tails[a.segment] - target
				total += a.weight * d.lenSq()
			}
			for ((index, j) in joints.withIndex()) {
				val relative = ((rotations[j.first] * j.firstOffset).inv() * rotations[j.second] * j.secondOffset).unit()
				// Soft preferences retain a finite cost for unusual poses.
				var penalty = if (j.hinge) {
					relative.y * relative.y + relative.z * relative.z
				} else if (j.maxSwingRadians == null && j.maxTwistRadians == null) {
					(1f - abs(relative.w)).let { it * it }
				} else {
					0f
				}
				swingLimits[index]?.let { limit ->
					val excess = (sqrt(relative.x * relative.x + relative.z * relative.z) - limit).coerceAtLeast(0f)
					penalty += excess * excess
				}
				twistLimits[index]?.let { limit ->
					val projection = relative.w * relative.w + relative.y * relative.y
					// At a half turn of swing, twist is unobservable and must not add a preference.
					if (projection > 1e-8f) {
						val excess = (abs(relative.y) / sqrt(projection) - limit).coerceAtLeast(0f)
						penalty += excess * excess
					}
				}
				total += j.weight * penalty
			}
			return total
		}
		// Report the measured-pose objective, and never accept a worse warm start.
		val initial = error()
		require(initial.isFinite())
		val measuredRotations = rotations.toList()
		for (i in segments.indices) {
			val s = segments[i]
			if (!s.fixed && s.rotationSource < 0) {
				rotations[i] = (s.initialRotation ?: s.measured).unit()
			}
		}
		val warmError = error()
		var best = if (warmError.isFinite() && warmError <= initial) {
			warmError
		} else {
			measuredRotations.forEachIndexed { i, rotation -> rotations[i] = rotation }
			initial
		}
		for (degrees in listOf(4.0, 2.0, 1.0)) {
			val half = Math.toRadians(degrees).toFloat() / 2f
			for (i in segments.indices) {
				if (segments[i].fixed || segments[i].rotationSource >= 0) continue
				for (axis in 0..2) {
					val start = rotations[i]
					var chosen = start
					for (sign in listOf(-1f, 1f)) {
						val a = sin(half) * sign
						val delta = Quaternion(cos(half), if (axis == 0) a else 0f, if (axis == 1) a else 0f, if (axis == 2) a else 0f)
						rotations[i] = (delta * start).unit()
						val candidate = error()
						if (candidate.isFinite() && candidate < best) {
							best = candidate
							chosen = rotations[i]
						}
					}
					rotations[i] = chosen
				}
			}
		}
		val finalError = error()
		require(finalError.isFinite() && rotations.all(::valid) && tails.all(::valid))
		return OptimizedPose(rotations.toList(), tails.toList(), initial, finalError)
	}
}

private fun distance(a: Quaternion, b: Quaternion): Float {
	val dot = abs(a.w.toDouble() * b.w + a.x.toDouble() * b.x + a.y.toDouble() * b.y + a.z.toDouble() * b.z)
	val norm = kotlin.math.sqrt(a.lenSq().toDouble() * b.lenSq())
	return (8.0 * (1.0 - (dot / norm).coerceIn(0.0, 1.0))).toFloat()
}
private fun valid(q: Quaternion) = q.w.isFinite() && q.x.isFinite() && q.y.isFinite() && q.z.isFinite() && q.lenSq().isFinite() && q.lenSq() > 1e-12f
private fun valid(v: Vector3) = v.x.isFinite() && v.y.isFinite() && v.z.isFinite()
