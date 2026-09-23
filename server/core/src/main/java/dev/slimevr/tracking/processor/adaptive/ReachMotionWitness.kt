package dev.slimevr.tracking.processor.adaptive

import io.github.axisangles.ktmath.Quaternion
import io.github.axisangles.ktmath.Vector3
import kotlin.math.abs
import kotlin.math.acos
import kotlin.math.asin

/**
 * A wrist and two limb lengths constrain the upper arm to a cone of possible directions.
 * Non-overlapping cones require upper-arm motion, unlike controller motion by itself.
 * The target IMU only resets the observation interval; it never supplies the predicted direction.
 */
internal class ReachMotionWitness {
	private data class Cone(val direction: Vector3, val radius: Double)
	private data class Reference(val cone: Cone, val rotation: Quaternion, val upper: Float, val lower: Float)
	private var reference: Reference? = null
	private var lastTime: Long? = null

	fun reset() {
		reference = null
		lastTime = null
	}

	fun observe(reach: Vector3, measuredRotation: Quaternion, upper: Float, lower: Float, now: Long, trusted: Boolean): Double {
		val cone = if (trusted) cone(reach, upper, lower) else null
		if (cone == null || !valid(measuredRotation)) {
			reset()
			return 0.0
		}
		val prior = reference
		val dt = lastTime?.let { now - it }
		lastTime = now
		if (prior == null ||
			dt == null ||
			dt !in 1..250_000_000L ||
			prior.upper != upper ||
			prior.lower != lower ||
			prior.rotation.angleToR(measuredRotation) > 0.001f
		) {
			reference = Reference(cone, measuredRotation.unit(), upper, lower)
			return 0.0
		}
		val dot = (prior.cone.direction.x.toDouble() * cone.direction.x + prior.cone.direction.y.toDouble() * cone.direction.y + prior.cone.direction.z.toDouble() * cone.direction.z).coerceIn(-1.0, 1.0)
		val requiredAngle = (acos(dot) - prior.cone.radius - cone.radius).coerceAtLeast(0.0)
		return if (requiredAngle >= Math.toRadians(5.0)) 0.95 else 0.0
	}

	private fun cone(reach: Vector3, upper: Float, lower: Float): Cone? {
		if (!reach.x.isFinite() || !reach.y.isFinite() || !reach.z.isFinite() || upper !in 0.15f..0.5f || lower !in 0.15f..0.5f) return null
		val distance = reach.len().toDouble()
		// Only high-extension configurations provide a useful independent bound.
		if (!distance.isFinite() || distance < 0.90 * (upper + lower) || distance > upper + lower + 0.02) return null
		// Widen the cone for two centimeters of length/shoulder uncertainty.
		val maximumLower = lower + 0.02
		val cosine = ((distance * distance + upper * upper - maximumLower * maximumLower) / (2.0 * distance * upper)).coerceIn(-1.0, 1.0)
		val radius = acos(cosine) + asin((0.02 / distance).coerceIn(0.0, 1.0))
		return Cone(reach / distance.toFloat(), radius)
	}

	private fun valid(q: Quaternion) = q.w.isFinite() && q.x.isFinite() && q.y.isFinite() && q.z.isFinite() && q.lenSq().isFinite() && abs(q.lenSq() - 1f) < 0.2f
}
