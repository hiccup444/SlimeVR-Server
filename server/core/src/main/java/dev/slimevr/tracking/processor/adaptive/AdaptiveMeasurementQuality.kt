package dev.slimevr.tracking.processor.adaptive

import dev.slimevr.tracking.processor.skeleton.HumanSkeleton
import dev.slimevr.tracking.trackers.Tracker
import dev.slimevr.tracking.trackers.TrackerPosition

/** One live input-quality result per pose tick, independent of telemetry sampling rate. */
class AdaptiveMeasurementQuality {
	private val sensors = SensorStateManager()
	private val confidence = TrackerConfidenceEstimator()
	private val health = TrackerHealthEstimator()
	private val body = BodyEvidenceEstimator()
	private val independentMotion = IndependentArmMotion()
	private var identities: List<Pair<Tracker, TrackerPosition?>> = emptyList()
	var latestFrame: AdaptiveTelemetryFrame? = null
		private set
	val motion: AdaptiveMotionState get() = body.motion

	fun reset() {
		sensors.reset()
		confidence.reset()
		health.reset()
		body.reset()
		independentMotion.reset()
		identities = emptyList()
		latestFrame = null
	}

	fun observe(s: HumanSkeleton, now: Long): AdaptiveTelemetryFrame {
		val trackers = s.allHumanBones.mapNotNull { it.attachedTracker }.distinctBy { it.id }
		val current = trackers.map { it to it.trackerPosition }
		val cached = latestFrame
		if (cached != null && cached.timestampNanos == now && current == identities) return cached
		if (current != identities || (cached != null && now <= cached.timestampNanos)) reset()
		identities = current
		val chains = listOf(
			listOf(s.hipTracker, s.leftUpperLegTracker, s.leftLowerLegTracker, s.leftFootTracker),
			listOf(s.hipTracker, s.rightUpperLegTracker, s.rightLowerLegTracker, s.rightFootTracker),
			listOf(s.upperChestTracker ?: s.chestTracker, s.leftUpperArmTracker, s.leftLowerArmTracker, s.leftHandTracker),
			listOf(s.upperChestTracker ?: s.chestTracker, s.rightUpperArmTracker, s.rightLowerArmTracker, s.rightHandTracker),
		).map { chain -> chain.mapNotNull { it?.id } }
		val measured = confidence.observe(sensors.sample(trackers, emptyList(), now))
		val motionContexts = independentMotion.observe(s, measured)
		val frame = body.observe(health.observe(measured, motionContexts), chains)
		latestFrame = frame
		return frame
	}
}
