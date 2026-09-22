package dev.slimevr.tracking.processor.adaptive

import io.github.axisangles.ktmath.Quaternion
import io.github.axisangles.ktmath.Vector3
import kotlin.math.acos
import kotlin.math.ceil
import kotlin.math.sqrt

data class MountingDirectionEstimate(
	val direction: Vector3,
	val confidence: Float,
	val sampleCount: Int,
	val inlierCount: Int,
	val trustedSeconds: Double,
	val conditionNumber: Double,
	val distanceConstraintRmsMeters: Double,
)

/** Learns a fixed upper-arm direction from caller-gated external arm geometry. */
class MountingDirectionLearner(upperLength: Double, lowerLength: Double) {
	private data class Equation(val timeNanos: Long, val continuity: Long, val vector: DoubleArray, val value: Double)

	private val upper = upperLength
	private val lower = lowerLength
	private val equations = ArrayList<Equation>(MAX_SAMPLES)
	private var lastAcceptedNanos: Long? = null
	private var lastSampleNanos: Long? = null
	private var trustedNanos = 0L
	private var continuity = 0L
	private var lastEstimate: MountingDirectionEstimate? = null

	init {
		require(upperLength.isFinite() && upperLength > 0.0)
		require(lowerLength.isFinite() && lowerLength > 0.0)
	}

	fun reset() {
		equations.clear()
		lastAcceptedNanos = null
		lastSampleNanos = null
		trustedNanos = 0L
		continuity = 0L
		lastEstimate = null
	}

	/**
	 * Adds one stationary, externally constrained shoulder-to-wrist observation. Confidence
	 * thresholds trust only; stationarity and tracker freshness remain the caller's responsibility.
	 */
	fun observe(rotation: Quaternion, displacement: Vector3, confidence: Float, now: Long): MountingDirectionEstimate? {
		if (!valid(rotation) || !valid(displacement) || !confidence.isFinite() || confidence !in MIN_CONFIDENCE..1f) {
			lastAcceptedNanos = null
			lastSampleNanos = null
			continuity++
			lastEstimate = null
			return null
		}
		val lastSample = lastSampleNanos
		if (lastSample != null && now <= lastSample) {
			reset()
			return null
		}
		if (lastSample != null && now - lastSample > MAX_GAP_NANOS) {
			lastAcceptedNanos = null
			continuity++
		}
		lastSampleNanos = now
		val lastAccepted = lastAcceptedNanos
		if (lastAccepted != null && now - lastAccepted < SAMPLE_INTERVAL_NANOS) return lastEstimate
		lastAcceptedNanos = now
		val q = rotation.unit()
		val localDisplacement = q.inv().sandwich(displacement)
		val squaredDistance = dot(localDisplacement, localDisplacement)
		val projected = (squaredDistance + upper * upper - lower * lower) / (2.0 * upper)
		if (!projected.isFinite()) {
			lastAcceptedNanos = null
			continuity++
			lastEstimate = null
			return null
		}
		equations.add(Equation(now, continuity, doubleArrayOf(localDisplacement.x.toDouble(), localDisplacement.y.toDouble(), localDisplacement.z.toDouble()), projected))
		while (equations.isNotEmpty() && now - equations.first().timeNanos > MAX_EVIDENCE_AGE_NANOS) equations.removeAt(0)
		while (equations.size > MAX_SAMPLES) equations.removeAt(0)
		trustedNanos = trustedDuration(equations)
		lastEstimate = estimate()
		return lastEstimate
	}

	private fun trustedDuration(rows: List<Equation>): Long {
		var total = 0L
		for (index in 1 until rows.size) {
			val prior = rows[index - 1]
			val current = rows[index]
			val interval = current.timeNanos - prior.timeNanos
			if (current.continuity == prior.continuity && interval in 1..MAX_GAP_NANOS) total += interval
		}
		return total
	}

	private fun estimate(): MountingDirectionEstimate? {
		if (trustedNanos < MIN_TRUSTED_NANOS || equations.size < MIN_SAMPLES) return null
		val initial = fit(equations) ?: return null
		if (initial.conditionNumber > MAX_CONDITION_NUMBER) return null
		val initialNorm = sqrt(dot(initial.solution, initial.solution))
		if (!initialNorm.isFinite() || initialNorm <= 0.0) return null
		val initialDirection = DoubleArray(3) { initial.solution[it] / initialNorm }
		val ordered = equations.sortedBy { distanceResidual(it, initialDirection) }
		val keep = ceil(equations.size * (1.0 - MAX_TRIM_FRACTION)).toInt().coerceAtLeast(MIN_SAMPLES)
		val trimmed = ordered.take(keep)
		val robust = fit(trimmed) ?: return null
		if (robust.conditionNumber > MAX_CONDITION_NUMBER) return null
		val norm = sqrt(dot(robust.solution, robust.solution))
		if (!norm.isFinite() || kotlin.math.abs(norm - 1.0) > MAX_NORM_ERROR) return null
		val unitSolution = DoubleArray(3) { robust.solution[it] / norm }
		val residuals = equations.map { distanceResidual(it, unitSolution) }
		val inliers = residuals.count { it <= MAX_RESIDUAL_METERS }
		val inlierRatio = inliers.toDouble() / equations.size
		if (inlierRatio < MIN_INLIER_RATIO) return null
		val rms = sqrt(residuals.filter { it <= MAX_RESIDUAL_METERS }.map { it * it }.average())
		if (!rms.isFinite() || rms > MAX_RMS_METERS) return null
		val direction = Vector3(
			(robust.solution[0] / norm).toFloat(),
			(robust.solution[1] / norm).toFloat(),
			(robust.solution[2] / norm).toFloat(),
		)
		val angle = acos((-direction.y.toDouble()).coerceIn(-1.0, 1.0))
		if (!angle.isFinite() || angle > MAX_DEVIATION_RADIANS) return null
		val fitQuality = (1.0 - rms / MAX_RMS_METERS).coerceIn(0.0, 1.0)
		return MountingDirectionEstimate(
			direction,
			(inlierRatio * fitQuality).toFloat(),
			equations.size,
			inliers,
			trustedNanos * 1e-9,
			robust.conditionNumber,
			rms,
		)
	}

	private data class Fit(val solution: DoubleArray, val conditionNumber: Double)

	private fun fit(rows: List<Equation>): Fit? {
		val gram = Array(3) { DoubleArray(3) }
		val rhs = DoubleArray(3)
		for (row in rows) {
			for (i in 0..2) {
				rhs[i] += row.vector[i] * row.value
				for (j in 0..2) gram[i][j] += row.vector[i] * row.vector[j]
			}
		}
		val eigenvalues = symmetricEigenvalues(gram) ?: return null
		val smallest = eigenvalues.minOrNull() ?: return null
		val largest = eigenvalues.maxOrNull() ?: return null
		if (smallest <= 1e-12 || !largest.isFinite()) return null
		val condition = largest / smallest
		if (!condition.isFinite()) return null
		val solution = solve3(gram, rhs) ?: return null
		return Fit(solution, condition)
	}

	private fun symmetricEigenvalues(matrix: Array<DoubleArray>): DoubleArray? {
		val a = Array(3) { matrix[it].copyOf() }
		repeat(24) {
			var p = 0
			var q = 1
			var largest = kotlin.math.abs(a[p][q])
			for (i in 0..2) {
				for (j in i + 1..2) {
					if (kotlin.math.abs(a[i][j]) > largest) {
						p = i
						q = j
						largest = kotlin.math.abs(a[i][j])
					}
				}
			}
			if (largest < 1e-14) return@repeat
			val angle = 0.5 * kotlin.math.atan2(2.0 * a[p][q], a[q][q] - a[p][p])
			val c = kotlin.math.cos(angle)
			val s = kotlin.math.sin(angle)
			val app = c * c * a[p][p] - 2 * s * c * a[p][q] + s * s * a[q][q]
			val aqq = s * s * a[p][p] + 2 * s * c * a[p][q] + c * c * a[q][q]
			for (k in 0..2) {
				if (k != p && k != q) {
					val akp = c * a[k][p] - s * a[k][q]
					val akq = s * a[k][p] + c * a[k][q]
					a[k][p] = akp
					a[p][k] = akp
					a[k][q] = akq
					a[q][k] = akq
				}
			}
			a[p][p] = app
			a[q][q] = aqq
			a[p][q] = 0.0
			a[q][p] = 0.0
		}
		val values = doubleArrayOf(a[0][0], a[1][1], a[2][2])
		return values.takeIf { values -> values.all { it.isFinite() } }
	}

	private fun solve3(matrix: Array<DoubleArray>, vector: DoubleArray): DoubleArray? {
		val a = Array(3) { i -> DoubleArray(4) { j -> if (j == 3) vector[i] else matrix[i][j] } }
		for (column in 0..2) {
			val pivot = (column..2).maxByOrNull { kotlin.math.abs(a[it][column]) } ?: return null
			if (kotlin.math.abs(a[pivot][column]) < 1e-12) return null
			val swap = a[column]
			a[column] = a[pivot]
			a[pivot] = swap
			val scale = a[column][column]
			for (j in column..3) a[column][j] /= scale
			for (i in 0..2) {
				if (i != column) {
					val factor = a[i][column]
					for (j in column..3) a[i][j] -= factor * a[column][j]
				}
			}
		}
		return DoubleArray(3) { a[it][3] }.takeIf { it.all(Double::isFinite) }
	}

	private fun distanceResidual(equation: Equation, direction: DoubleArray): Double {
		val radicand = dot(equation.vector, equation.vector) + upper * upper - 2.0 * upper * dot(equation.vector, direction)
		return kotlin.math.abs(sqrt(radicand.coerceAtLeast(0.0)) - lower)
	}
	private fun dot(a: DoubleArray, b: DoubleArray) = a.indices.sumOf { a[it] * b[it] }
	private fun dot(a: Vector3, b: Vector3) = a.x.toDouble() * b.x + a.y.toDouble() * b.y + a.z.toDouble() * b.z
	private fun valid(q: Quaternion) = q.w.isFinite() && q.x.isFinite() && q.y.isFinite() && q.z.isFinite() && q.lenSq().isFinite() && q.lenSq() > 0f
	private fun valid(v: Vector3) = v.x.isFinite() && v.y.isFinite() && v.z.isFinite()

	companion object {
		const val MIN_CONFIDENCE = 0.9f
		const val MAX_SAMPLES = 512
		const val MIN_SAMPLES = 24
		const val SAMPLE_INTERVAL_NANOS = 200_000_000L
		const val MAX_GAP_NANOS = 500_000_000L
		const val MAX_EVIDENCE_AGE_NANOS = 300_000_000_000L
		const val MIN_TRUSTED_NANOS = 60_000_000_000L
		const val MAX_CONDITION_NUMBER = 100.0
		const val MAX_TRIM_FRACTION = 0.1
		const val MIN_INLIER_RATIO = 0.9
		const val MAX_NORM_ERROR = 0.03
		const val MAX_RESIDUAL_METERS = 0.008
		const val MAX_RMS_METERS = 0.008
		const val MAX_DEVIATION_RADIANS = Math.PI * 5.0 / 180.0
	}
}
