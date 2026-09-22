package dev.slimevr.tracking.processor.adaptive

import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.exp
import kotlin.math.sin

enum class DriftEvidenceSource { ABSOLUTE_POSITION_CONSTRAINT, PLANTED_CONTACT_WITH_STABLE_CHAIN, JOINT_PRIOR }

/** Residuals are measured before this subsystem's yaw correction; angles are radians. */
data class DriftEvidence(
	val errorRadians: Float,
	val source: DriftEvidenceSource,
	val contextId: Long,
	val poseConfidence: Float,
	val neighborAgreement: Float,
	val stationaryConfidence: Float,
	val independentConstraints: Int,
	val available: Boolean = true,
	val highMotion: Boolean = false,
)

data class DriftResidual(
	val errorRadians: Float,
	val filteredErrorRadians: Float,
	val consistentSeconds: Double,
	val eligibleForLearning: Boolean,
	val reason: String,
)

/** Accumulates circular residuals only while several independent checks agree. */
class ResidualTracker(private val minimumSeconds: Double = 20.0) {
	private var lastTime: Long? = null
	private var context: Pair<DriftEvidenceSource, Long>? = null
	private var filtered = 0f
	private var duration = 0.0

	init {
		require(minimumSeconds.isFinite() && minimumSeconds >= 0.0)
	}

	fun reset() {
		lastTime = null
		context = null
		duration = 0.0
		filtered = 0f
	}

	fun update(evidence: DriftEvidence, now: Long): DriftResidual {
		val angle = evidence.errorRadians
		val reason = when {
			!angle.isFinite() || evidence.poseConfidence !in 0f..1f || evidence.neighborAgreement !in 0f..1f || evidence.stationaryConfidence !in 0f..1f -> "INVALID_EVIDENCE"
			!evidence.available -> "UNAVAILABLE"
			evidence.source == DriftEvidenceSource.JOINT_PRIOR || evidence.independentConstraints < 2 -> "INSUFFICIENT_INDEPENDENT_EVIDENCE"
			evidence.highMotion || evidence.stationaryConfidence < 0.9f -> "MOTION"
			evidence.poseConfidence < 0.85f || evidence.neighborAgreement < 0.85f -> "LOW_CONFIDENCE"
			abs(wrapYaw(angle)) > Math.toRadians(25.0) -> "POSSIBLE_MOUNTING_SHIFT"
			else -> null
		}
		if (reason != null) {
			reset()
			val finiteAngle = if (angle.isFinite()) wrapYaw(angle) else 0f
			return DriftResidual(finiteAngle, finiteAngle, 0.0, false, reason)
		}
		val value = wrapYaw(angle)
		val nextContext = evidence.source to evidence.contextId
		val previous = lastTime
		val delta = previous?.let { now - it }
		if (context != nextContext || delta == null || delta !in 1..500_000_000L) {
			filtered = value
			duration = 0.0
		} else {
			val dt = delta * 1e-9
			val difference = wrapYaw(value - filtered)
			if (abs(difference) > Math.toRadians(3.0)) {
				duration = 0.0
				filtered = value
			} else {
				filtered = wrapYaw(filtered + difference * (1.0 - exp(-dt / 2.0)).toFloat())
				duration += dt
			}
		}
		context = nextContext
		lastTime = now
		val mature = duration + 1e-9 >= minimumSeconds
		return DriftResidual(value, filtered, duration, mature, if (mature) "SUPPORTED_PERSISTENT_RESIDUAL" else "OBSERVING")
	}
}

internal fun wrapYaw(angle: Float): Float = atan2(sin(angle), cos(angle))
