package dev.slimevr.tracking.processor.adaptive

import dev.slimevr.config.AdaptiveTrackingConfig
import dev.slimevr.tracking.trackers.Tracker

/** Observation-only entry point; never writes tracker or skeleton state. */
class AdaptiveTrackingTelemetry(private val config: AdaptiveTrackingConfig) : AutoCloseable {
	private val sensors = SensorStateManager()
	private val confidence = TrackerConfidenceEstimator()
	private val health = TrackerHealthEstimator()

	@Volatile private var recorder: AdaptiveTelemetryRecorder? = null

	@Volatile private var recentRecorder: AdaptiveTelemetryRecorder? = null
	private var previousSampleNanos: Long? = null

	@Volatile var latestFrame: AdaptiveTelemetryFrame? = null
		private set

	fun update(trackers: List<Tracker>, outputs: List<Tracker>, nowNanos: Long = System.nanoTime(), footContacts: Map<String, FootContactSnapshot> = emptyMap(), driftDiagnostics: List<TrackerDriftDiagnostic> = emptyList(), poseSolver: AdaptivePoseSolver? = null, armCalibration: ArmCalibrationDiagnostic? = null, floorEstimate: AdaptiveFloorEstimate? = null, qualityFrame: AdaptiveTelemetryFrame? = null) {
		if (!config.telemetryEnabled && !config.liveDiagnosticsEnabled) {
			close()
			return
		}
		if (config.telemetryEnabled && recorder == null) {
			recorder = AdaptiveTelemetryRecorder(config.telemetryDirectory).also { recentRecorder = it }
			previousSampleNanos = null
		} else if (!config.telemetryEnabled) {
			recorder?.close()
			recorder = null
		}
		val interval = 1_000_000_000L / config.telemetrySampleRateHz.coerceIn(1, 100)
		val previous = previousSampleNanos
		if (previous != null && nowNanos >= previous && nowNanos - previous < interval) return
		previousSampleNanos = nowNanos
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
		val live = qualityFrame?.takeIf { it.timestampNanos == nowNanos }?.samples?.associateBy { it.id }
		val aligned = if (live != null) {
			captured.copy(
				samples = captured.samples.map { sample -> live[sample.id] ?: sample },
			)
		} else {
			captured
		}
		val frame = if (config.confidenceDiagnosticsEnabled) {
			if (live != null) {
				aligned
			} else {
				health.observe(confidence.observe(aligned))
			}
		} else {
			confidence.reset()
			health.reset()
			aligned.copy(samples = aligned.samples.map { it.copy(confidence = null, health = null) })
		}
		latestFrame = frame
		recorder?.offer(frame)
	}

	fun recordingStatus(): Map<String, Any?> {
		val writer = recentRecorder
		return mapOf(
			"requested" to config.telemetryEnabled,
			"active" to (config.telemetryEnabled && recorder === writer && writer?.isRecording == true),
			"finalizing" to (writer?.isFinalizing == true || (!config.telemetryEnabled && writer?.isRecording == true)),
			"directory" to config.telemetryDirectory,
			"sampleRateHz" to config.telemetrySampleRateHz.coerceIn(1, 100),
			"files" to (writer?.outputPaths?.map { it.toAbsolutePath().toString() } ?: emptyList()),
			"writtenFrames" to (writer?.writtenFrames ?: 0L),
			"droppedFrames" to (writer?.droppedFrames ?: 0L),
			"sizeLimitReached" to (writer?.sizeLimitReached ?: false),
			"failure" to writer?.failure,
		)
	}

	fun reset() {
		if (!config.telemetryEnabled) {
			recorder?.close()
			recorder = null
		}
		sensors.reset()
		confidence.reset()
		health.reset()
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
