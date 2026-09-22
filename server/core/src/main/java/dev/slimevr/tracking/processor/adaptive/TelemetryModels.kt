package dev.slimevr.tracking.processor.adaptive

import dev.slimevr.tracking.trackers.TrackerRole
import dev.slimevr.tracking.trackers.TrackerStatus
import io.github.axisangles.ktmath.Quaternion
import io.github.axisangles.ktmath.Vector3

/** A read-only snapshot of all telemetry captured at one monotonic timestamp. */
data class AdaptiveTelemetryFrame(
	val timestampNanos: Long,
	val samples: List<TelemetrySample>,
	val computedSamples: List<TelemetrySample>,
	/** Increases whenever temporal continuity is intentionally broken. */
	val resetEpoch: Long,
	val footContacts: Map<String, FootContactSnapshot> = emptyMap(),
	val footAnchoringRequested: Boolean = false,
	val driftDiagnostics: List<TrackerDriftDiagnostic> = emptyList(),
	val poseDiagnostic: PoseSolverDiagnostic? = null,
	val rawPose: Map<String, Vector3> = emptyMap(),
	val predictedPose: Map<String, Vector3> = emptyMap(),
	val poseInput: PoseSolverInput? = null,
	val armCalibration: ArmCalibrationDiagnostic? = null,
	val floorEstimate: AdaptiveFloorEstimate? = null,
	val trackerPredictions: Map<Int, TrackerPosePrediction> = emptyMap(),
	val activity: AdaptiveActivityEstimate? = null,
)

/** Normalized sensor data. Derived fields are deliberately named as observations. */
data class TelemetrySample(
	val id: Int,
	val name: String,
	val role: TrackerRole?,
	val status: TrackerStatus,
	val rawRotation: Quaternion?,
	val adjustedRotation: Quaternion?,
	val position: Vector3?,
	val acceleration: Vector3?,
	val derivedLinearVelocity: Vector3?,
	val packetAgeNanos: Long?,
	val temperatureCelsius: Float?,
	val angularSpeedRadiansPerSecond: Float?,
	val rotationExpected: Boolean = rawRotation != null,
	val positionExpected: Boolean = position != null,
	val continuousObservation: Boolean = false,
	val confidence: TrackerConfidence? = null,
	val accelerationAgeNanos: Long? = null,
	val temperatureAgeNanos: Long? = null,
	val health: TrackerHealthDiagnostic? = null,
)

/** Diagnostic measurement quality, not a probability that the pose is correct. */
data class TrackerConfidence(
	val score: Float,
	val reasons: List<String>,
	val observationSeconds: Double,
)
