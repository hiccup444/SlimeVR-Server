package dev.slimevr.tracking.processor.adaptive

import io.github.axisangles.ktmath.Quaternion
import kotlin.math.exp

/** Short extrapolation from measured rotations, followed by a bounded recovery blend. */
class OrientationRecovery {
	private var lastGood: Quaternion? = null
	private var priorGood: Quaternion? = null
	private var lastGoodTime: Long? = null
	private var priorGoodTime: Long? = null
	private var lastUpdate: Long? = null
	private var output: Quaternion? = null
	private var recovering = false

	fun reset() {
		lastGood = null
		priorGood = null
		lastGoodTime = null
		priorGoodTime = null
		lastUpdate = null
		output = null
		recovering = false
	}

	fun update(measured: Quaternion, available: Boolean, now: Long): Quaternion {
		val delta = lastUpdate?.let { now - it }
		if (delta != null && delta !in 1..500_000_000L) reset()
		lastUpdate = now
		val valid = measured.w.isFinite() && measured.x.isFinite() && measured.y.isFinite() && measured.z.isFinite() && measured.lenSq().isFinite() && measured.lenSq() > 0f
		if (available && valid) {
			val normalized = measured.unit()
			priorGood = lastGood
			priorGoodTime = lastGoodTime
			lastGood = normalized
			lastGoodTime = now
			val previous = output
			output = if (recovering && previous != null && delta != null) previous.interpR(normalized, (1.0 - exp(-delta * 1e-9 / 0.05)).toFloat()) else normalized
			if (output!!.angleToR(normalized) < 0.002f) recovering = false
			return output!!
		}
		recovering = true
		val good = lastGood ?: return if (valid) measured.unit() else Quaternion.IDENTITY
		val age = (now - (lastGoodTime ?: now)).coerceAtLeast(0)
		val interval = priorGoodTime?.let { (lastGoodTime ?: now) - it }
		val prior = priorGood
		val prediction = if (prior != null && interval != null && interval in 1..250_000_000L) {
			val angle = prior.angleToR(good)
			val factor = minOf(age.coerceAtMost(200_000_000L).toFloat() / interval, if (angle > 0.00001f) 0.26f / angle else 0f)
			prior.interpR(good, 1f + factor)
		} else {
			good
		}
		output = prediction
		return prediction
	}
}
