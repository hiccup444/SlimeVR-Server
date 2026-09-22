package dev.slimevr.tracking.processor.adaptive

import dev.slimevr.config.AdaptiveTrackingConfig
import dev.slimevr.tracking.trackers.Tracker

/** Observation-only entry point; never writes tracker or skeleton state. */
class AdaptiveTrackingTelemetry(private val config: AdaptiveTrackingConfig) : AutoCloseable {
	private val sensors = SensorStateManager()
	private val confidence = TrackerConfidenceEstimator()
	private var recorder: AdaptiveTelemetryRecorder? = null
	private var previousSampleNanos: Long? = null

	@Volatile var latestFrame: AdaptiveTelemetryFrame? = null
		private set

	fun update(trackers: List<Tracker>, outputs: List<Tracker>, nowNanos: Long = System.nanoTime(), footContacts: Map<String, FootContactSnapshot> = emptyMap(), driftDiagnostics: List<TrackerDriftDiagnostic> = emptyList(), poseSolver: AdaptivePoseSolver? = null, armCalibration: ArmCalibrationDiagnostic? = null, floorEstimate: AdaptiveFloorEstimate? = null) {
		if (!config.telemetryEnabled && !config.liveDiagnosticsEnabled) {
			close()
			return
		}
		val interval = 1_000_000_000L / config.telemetrySampleRateHz.coerceIn(1, 100)
		val previous = previousSampleNanos
		if (previous != null && nowNanos >= previous && nowNanos - previous < interval) return
		previousSampleNanos = nowNanos
		if (config.telemetryEnabled && recorder == null) recorder = AdaptiveTelemetryRecorder(config.telemetryDirectory)
		if (!config.telemetryEnabled) {
			recorder?.close()
			recorder = null
		}
		val captured = sensors.sample(trackers.filter { !it.isInternal }, outputs, nowNanos).copy(
			footContacts = footContacts,
			footAnchoringRequested = config.footAnchoringEnabled,
			driftDiagnostics = driftDiagnostics,
			poseDiagnostic = poseSolver?.diagnostic,
			rawPose = poseSolver?.rawPose ?: emptyMap(),
			predictedPose = poseSolver?.predictedPose ?: emptyMap(),
			poseInput = poseSolver?.replayInput,
			armCalibration = armCalibration,
			floorEstimate = floorEstimate,
			trackerPredictions = poseSolver?.trackerPredictions ?: emptyMap(),
			activity = poseSolver?.activity,
		)
		val frame = if (config.confidenceDiagnosticsEnabled) {
			confidence.observe(captured)
		} else {
			confidence.reset()
			captured
		}
		latestFrame = frame
		recorder?.offer(frame)
	}

	fun reset() {
		sensors.reset()
		confidence.reset()
		previousSampleNanos = null
		latestFrame = null
	}

	override fun close() {
		if (recorder == null && latestFrame == null && previousSampleNanos == null) return
		recorder?.close()
		recorder = null
		reset()
	}
}
