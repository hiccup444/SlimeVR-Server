package dev.slimevr.tracking.processor.adaptive

import io.github.axisangles.ktmath.Vector3

/** Chooses an effective contact, not merely an enabled setting. */
internal object FootCorrectionPolicy {
	fun ownsCorrection(enabled: Boolean, contact: FootContactSnapshot, strength: Float): Boolean {
		val plant = contact.plantPosition ?: return false
		return enabled &&
			strength.isFinite() &&
			strength > 0f &&
			contact.weight.isFinite() &&
			contact.weight > 0f &&
			(contact.state == FootContactState.PLANTED || contact.state == FootContactState.RELEASING) &&
			plant.x.isFinite() &&
			plant.y.isFinite() &&
			plant.z.isFinite()
	}

	fun select(raw: Vector3, legacy: Vector3, detector: FootContactDetector, enabled: Boolean, strength: Float): Vector3 = if (ownsCorrection(enabled, detector.snapshot, strength)) detector.correct(raw, strength) else legacy
}
