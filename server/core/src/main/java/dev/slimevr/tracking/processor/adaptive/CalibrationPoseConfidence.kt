package dev.slimevr.tracking.processor.adaptive

import dev.slimevr.tracking.trackers.Tracker
import kotlin.math.min

/** Builds context confidence from fresh measurements and independently observed witnesses. */
internal object CalibrationPoseConfidence {
	private val estimator = GlobalPoseConfidenceEstimator(minimumLearningScore = 0.85f)

	fun estimate(
		trackers: List<Tracker>,
		anchors: List<Tracker>,
		witnessChanges: List<Double>,
		independentResiduals: List<Double>,
		motionUncertainty: Float,
		now: Long,
		contact: Float? = null,
		qualityFrame: AdaptiveTelemetryFrame,
	): GlobalPoseConfidence {
		val live = if (qualityFrame.timestampNanos == now) qualityFrame.samples.associateBy { it.id } else emptyMap()
		fun liveQuality(t: Tracker) = min(quality(t, now), live[t.id]?.confidence?.score ?: 0f)
		val scores = trackers.mapNotNull { t -> t.trackerPosition?.trackerRole?.let { it to liveQuality(t) } }
			.groupBy({ it.first }, { it.second }).mapValues { it.value.min() }
		val anchorQuality = anchors.minOfOrNull { liveQuality(it) } ?: 0f
		return estimator.estimate(
			GlobalPoseConfidenceInput(
				trackerConfidences = scores,
				requiredRoles = scores.keys,
				absoluteAnchorAvailable = anchors.isNotEmpty() && anchors.all { !it.isInternal && it.hasPosition },
				absoluteAnchorQuality = anchorQuality,
				independentChainDisagreementsRadians = witnessChanges,
				independentResidualsRadians = independentResiduals,
				contactReliability = contact,
				contactRequiredForLearning = contact != null,
				motionUncertainty = motionUncertainty,
				minimumIndependentChains = 2,
				minimumIndependentResiduals = 1,
			),
		)
	}

	private fun quality(t: Tracker, now: Long): Float {
		if (!t.status.sendData || !t.hasRotation) return 0f
		val q = t.getRotationWithoutAdaptive()
		if (!q.w.isFinite() || !q.x.isFinite() || !q.y.isFinite() || !q.z.isFinite() || !q.lenSq().isFinite() || q.lenSq() <= 0f) return 0f
		if (t.hasPosition && (!t.position.x.isFinite() || !t.position.y.isFinite() || !t.position.z.isFinite())) return 0f
		val age = t.lastRotationUpdateNanos?.let { now - it } ?: return 0f
		if (age !in 0..250_000_000L) return 0f
		return min(0.95f, (1f - age / 250_000_000f).coerceIn(0f, 1f))
	}
}
