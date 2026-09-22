package dev.slimevr.tracking.processor.adaptive

import io.github.axisangles.ktmath.Vector3
import kotlin.math.abs
import kotlin.math.acos
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sin
import kotlin.math.sqrt

data class ArmYawConstraintEstimate(
	val residualBiasRadians: Double,
	val sensitivityMetersPerRadian: Double,
	val alternativeSeparationRadians: Double,
)

/** Infers a bounded yaw bias from arm geometry without deciding whether the evidence is trusted. */
object ArmYawConstraint {
	fun estimate(
		shoulderToWrist: Vector3,
		measuredUpperDirection: Vector3,
		upperArmLengthMeters: Double,
		lowerArmLengthMeters: Double,
	): ArmYawConstraintEstimate? {
		val dx = shoulderToWrist.x.toDouble()
		val dy = shoulderToWrist.y.toDouble()
		val dz = shoulderToWrist.z.toDouble()
		val ux = measuredUpperDirection.x.toDouble()
		val uy = measuredUpperDirection.y.toDouble()
		val uz = measuredUpperDirection.z.toDouble()
		if (listOf(dx, dy, dz, ux, uy, uz).any { !it.isFinite() }) return null
		if (!upperArmLengthMeters.isFinite() || upperArmLengthMeters !in MIN_ARM_LENGTH_METERS..MAX_ARM_LENGTH_METERS) return null
		if (!lowerArmLengthMeters.isFinite() || lowerArmLengthMeters !in MIN_ARM_LENGTH_METERS..MAX_ARM_LENGTH_METERS) return null

		val directionLength = sqrt(ux * ux + uy * uy + uz * uz)
		if (!directionLength.isFinite() || abs(directionLength - 1.0) > UNIT_DIRECTION_TOLERANCE) return null

		val displacementSquared = dx * dx + dy * dy + dz * dz
		val target = (displacementSquared + upperArmLengthMeters * upperArmLengthMeters - lowerArmLengthMeters * lowerArmLengthMeters) /
			(2.0 * upperArmLengthMeters)
		val a = dx * ux + dz * uz
		val b = dx * uz - dz * ux
		val horizontalMagnitude = hypot(a, b)
		val c = target - dy * uy
		if (!target.isFinite() || !horizontalMagnitude.isFinite() || !c.isFinite() || horizontalMagnitude <= MIN_HORIZONTAL_MAGNITUDE) return null
		if (abs(c) > horizontalMagnitude + GEOMETRY_TOLERANCE) return null

		val phase = atan2(b, a)
		val offset = acos((c / horizontalMagnitude).coerceIn(-1.0, 1.0))
		val roots = mutableListOf(normalizeAngle(phase + offset))
		val second = normalizeAngle(phase - offset)
		if (angularDistance(roots.first(), second) > ROOT_DEDUPLICATION_RADIANS) roots.add(second)

		val candidates = roots.map { theta ->
			val sensitivity = abs(-a * sin(theta) + b * cos(theta))
			Candidate(theta, sensitivity)
		}.sortedBy { abs(it.theta) }
		val best = candidates.firstOrNull() ?: return null
		if (abs(best.theta) > MAX_CORRECTION_RADIANS || best.sensitivity < MIN_SENSITIVITY_METERS_PER_RADIAN) return null

		val alternative = candidates.drop(1).firstOrNull()
		if (alternative != null && abs(abs(alternative.theta) - abs(best.theta)) <= AMBIGUOUS_MAGNITUDE_DELTA_RADIANS) return null

		return ArmYawConstraintEstimate(
			normalizeAngle(-best.theta),
			best.sensitivity,
			alternative?.let { angularDistance(best.theta, it.theta) } ?: 0.0,
		)
	}

	private data class Candidate(val theta: Double, val sensitivity: Double)

	private fun normalizeAngle(angle: Double): Double {
		val wrapped = (angle + Math.PI) % TWO_PI
		return (if (wrapped < 0.0) wrapped + TWO_PI else wrapped) - Math.PI
	}

	private fun angularDistance(a: Double, b: Double) = abs(normalizeAngle(a - b))

	private const val MIN_ARM_LENGTH_METERS = 0.15
	private const val MAX_ARM_LENGTH_METERS = 0.50
	private const val UNIT_DIRECTION_TOLERANCE = 0.02
	private const val MIN_HORIZONTAL_MAGNITUDE = 1e-9
	private const val GEOMETRY_TOLERANCE = 1e-9
	private const val ROOT_DEDUPLICATION_RADIANS = 1e-8
	private const val MIN_SENSITIVITY_METERS_PER_RADIAN = 0.05
	private val MAX_CORRECTION_RADIANS = Math.toRadians(20.0)
	private val AMBIGUOUS_MAGNITUDE_DELTA_RADIANS = Math.toRadians(10.0)
	private const val TWO_PI = 2.0 * Math.PI
}
