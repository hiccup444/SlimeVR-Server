package dev.slimevr.config

class AdaptiveTrackingConfig {
	var armCalibrationMode = "disabled"
	var poseOptimizerEnabled = false
	var temperatureLearningEnabled = false
	var calibrationDirectory = "adaptive-calibration"
	var telemetryEnabled = false
	var liveDiagnosticsEnabled = false
	var telemetryDirectory = "adaptive-telemetry"
	var telemetrySampleRateHz = 50
	var confidenceDiagnosticsEnabled = true
	var footContactDiagnosticsEnabled = true
	var floorEstimationEnabled = false
	var footAnchoringEnabled = false
	var footAnchorStrength = 0.8f
	var yawCorrectionEnabled = false
	var yawCorrectionStrength = 0.5f
}
