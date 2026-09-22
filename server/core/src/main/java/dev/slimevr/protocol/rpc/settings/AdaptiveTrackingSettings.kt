package dev.slimevr.protocol.rpc.settings

import dev.slimevr.config.AdaptiveTrackingConfig
import solarxr_protocol.rpc.AdaptiveBoolean
import solarxr_protocol.rpc.AdaptiveTrackingSettings

internal fun applyAdaptiveTrackingSettings(
	request: AdaptiveTrackingSettings,
	config: AdaptiveTrackingConfig,
) {
	if (request.hasTelemetryEnabled()) {
		config.telemetryEnabled = request.telemetryEnabled() == AdaptiveBoolean.TRUE
	}
	if (request.hasConfidenceDiagnosticsEnabled()) {
		config.confidenceDiagnosticsEnabled = request.confidenceDiagnosticsEnabled() == AdaptiveBoolean.TRUE
	}
	if (request.hasFootContactDiagnosticsEnabled()) {
		config.footContactDiagnosticsEnabled = request.footContactDiagnosticsEnabled() == AdaptiveBoolean.TRUE
	}
	if (request.hasFootAnchoringEnabled()) {
		config.footAnchoringEnabled = request.footAnchoringEnabled() == AdaptiveBoolean.TRUE
	}
	if (request.hasFootAnchorStrength()) {
		val strength = request.footAnchorStrength()
		if (strength.isFinite()) {
			config.footAnchorStrength = strength.coerceIn(0f, 1f)
		}
	}
	if (request.hasTelemetrySampleRateHz()) {
		config.telemetrySampleRateHz = request.telemetrySampleRateHz().coerceIn(1, 100)
	}
	if (request.hasYawCorrectionEnabled()) {
		config.yawCorrectionEnabled = request.yawCorrectionEnabled() == AdaptiveBoolean.TRUE
	}
	if (request.hasYawCorrectionStrength()) {
		val strength = request.yawCorrectionStrength()
		if (strength.isFinite()) {
			config.yawCorrectionStrength = strength.coerceIn(0f, 1f)
		}
	}
	if (request.hasPoseOptimizerEnabled()) {
		config.poseOptimizerEnabled = request.poseOptimizerEnabled() == AdaptiveBoolean.TRUE
	}
	if (request.hasTemperatureLearningEnabled()) {
		config.temperatureLearningEnabled = request.temperatureLearningEnabled() == AdaptiveBoolean.TRUE
	}
	if (request.hasLiveDiagnosticsEnabled()) {
		config.liveDiagnosticsEnabled = request.liveDiagnosticsEnabled() == AdaptiveBoolean.TRUE
	}
	if (request.hasFloorEstimationEnabled()) {
		config.floorEstimationEnabled = request.floorEstimationEnabled() == AdaptiveBoolean.TRUE
	}
	request.armCalibrationMode()?.let { mode ->
		if (mode in setOf("disabled", "proportions", "mounting")) {
			config.armCalibrationMode = mode
		}
	}
}
