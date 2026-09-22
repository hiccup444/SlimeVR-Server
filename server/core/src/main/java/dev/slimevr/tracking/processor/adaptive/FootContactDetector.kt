package dev.slimevr.tracking.processor.adaptive

import io.github.axisangles.ktmath.Quaternion
import io.github.axisangles.ktmath.Vector3
import kotlin.math.abs
import kotlin.math.acos
import kotlin.math.min
import kotlin.math.sqrt

enum class FootContactState { AIRBORNE, CONTACT_CANDIDATE, PLANTED, RELEASING }

data class FootContactSnapshot(
	val state: FootContactState = FootContactState.AIRBORNE,
	val plantPosition: Vector3? = null,
	val weight: Float = 0f,
	val speedMetersPerSecond: Float? = null,
	val angularSpeedRadiansPerSecond: Float? = null,
)

/** Contact inference from uncorrected solver positions and measured foot motion. */
class FootContactDetector {
	private data class Observation(val time: Long, val position: Vector3, val rotation: Quaternion)
	private var previous: Observation? = null
	private var stateSince = 0L
	private var releaseWeight = 0f
	var snapshot = FootContactSnapshot()
		private set

	fun reset() {
		previous = null
		snapshot = FootContactSnapshot()
		releaseWeight = 0f
	}

	fun update(
		position: Vector3,
		rotation: Quaternion,
		floorHeight: Float,
		acceleration: Vector3?,
		trusted: Boolean,
		nowNanos: Long,
	): FootContactSnapshot {
		if (!trusted || !finite(position) || !finite(rotation) || !floorHeight.isFinite() || (acceleration != null && !finite(acceleration))) {
			reset()
			return snapshot
		}
		val prior = previous
		val dt = prior?.let { nowNanos - it.time }
		if (prior == null || dt == null || dt !in 1..250_000_000L) {
			reset()
			previous = Observation(nowNanos, position, rotation)
			return snapshot
		}
		previous = Observation(nowNanos, position, rotation)
		val seconds = dt * 1e-9f
		val speed = (position - prior.position).len() / seconds
		val angularSpeed = angle(prior.rotation, rotation) / seconds
		if (!speed.isFinite() || !angularSpeed.isFinite()) {
			reset()
			return snapshot
		}
		val height = abs(position.y - floorHeight)
		val stable = height <= 0.06f && speed <= 0.08f && angularSpeed <= 0.35f && (acceleration == null || acceleration.len() <= 1f)
		val plant = snapshot.plantPosition
		val distance = if (plant == null) 0f else (position - plant).len()
		val release = height > 0.10f || speed > 0.25f || angularSpeed > 1f || distance > 0.12f || (acceleration != null && acceleration.len() > 2f)
		var state = snapshot.state
		var anchor = plant
		var weight = snapshot.weight
		when (state) {
			FootContactState.AIRBORNE -> if (stable) {
				state = FootContactState.CONTACT_CANDIDATE
				stateSince = nowNanos
			}

			FootContactState.CONTACT_CANDIDATE -> if (!stable) {
				state = FootContactState.AIRBORNE
			} else if (nowNanos - stateSince >= 150_000_000L) {
				state = FootContactState.PLANTED
				stateSince = nowNanos
				anchor = position
			}

			FootContactState.PLANTED -> if (release) {
				state = FootContactState.RELEASING
				stateSince = nowNanos
				releaseWeight = weight
			} else {
				// Missing acceleration evidence reduces the maximum constraint weight.
				val maximum = if (acceleration == null) 0.65f else 0.95f
				weight = maximum * ((nowNanos - stateSince) / 100_000_000f).coerceIn(0f, 1f)
			}

			FootContactState.RELEASING -> {
				weight = releaseWeight * (1f - (nowNanos - stateSince) / 50_000_000f).coerceIn(0f, 1f)
				if (weight <= 0f) {
					state = FootContactState.AIRBORNE
					anchor = null
				}
			}
		}
		snapshot = FootContactSnapshot(state, anchor, weight, speed, angularSpeed)
		return snapshot
	}

	/** Keeps vertical motion intact and limits horizontal displacement to six centimeters. */
	fun correct(position: Vector3, strength: Float): Vector3 {
		val plant = snapshot.plantPosition ?: return position
		if (!finite(position) || !strength.isFinite()) return position
		val offset = Vector3(plant.x - position.x, 0f, plant.z - position.z)
		val length = offset.len()
		if (!length.isFinite() || length <= 0f) return position
		val weight = snapshot.weight * strength.coerceIn(0f, 1f)
		return position + offset * min(weight, 0.06f / length)
	}
}

private fun finite(v: Vector3) = v.x.isFinite() && v.y.isFinite() && v.z.isFinite()
private fun finite(q: Quaternion) = q.w.isFinite() && q.x.isFinite() && q.y.isFinite() && q.z.isFinite() && q.lenSq().isFinite() && q.lenSq() > 0f
private fun angle(a: Quaternion, b: Quaternion): Float {
	val dot = a.w.toDouble() * b.w + a.x.toDouble() * b.x + a.y.toDouble() * b.y + a.z.toDouble() * b.z
	fun norm(q: Quaternion) = q.w.toDouble() * q.w + q.x.toDouble() * q.x + q.y.toDouble() * q.y + q.z.toDouble() * q.z
	return (2.0 * acos((abs(dot) / sqrt(norm(a) * norm(b))).coerceIn(0.0, 1.0))).toFloat()
}
