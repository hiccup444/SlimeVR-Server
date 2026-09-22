package dev.slimevr.protocol.rpc.settings

import com.google.flatbuffers.FlatBufferBuilder
import dev.slimevr.config.AdaptiveTrackingConfig
import org.junit.jupiter.api.Test
import solarxr_protocol.rpc.AdaptiveBoolean
import solarxr_protocol.rpc.AdaptiveTrackingSettings
import kotlin.test.assertEquals
import kotlin.test.assertFalse
import kotlin.test.assertTrue

class AdaptiveTrackingSettingsTest {
	@Test
	fun nullableFieldsPreservePartialUpdatesAndExplicitFalse() {
		val builder = FlatBufferBuilder(64)
		AdaptiveTrackingSettings.startAdaptiveTrackingSettings(builder)
		AdaptiveTrackingSettings.addYawCorrectionEnabled(builder, AdaptiveBoolean.FALSE)
		val offset = AdaptiveTrackingSettings.endAdaptiveTrackingSettings(builder)
		builder.finish(offset)
		val request = AdaptiveTrackingSettings.getRootAsAdaptiveTrackingSettings(builder.dataBuffer())
		val config = AdaptiveTrackingConfig().apply {
			telemetryEnabled = true
			yawCorrectionEnabled = true
		}

		assertFalse(request.hasTelemetryEnabled())
		assertTrue(request.hasYawCorrectionEnabled())
		applyAdaptiveTrackingSettings(request, config)

		assertTrue(config.telemetryEnabled)
		assertFalse(config.yawCorrectionEnabled)
	}

	@Test
	fun settingsResponsePreservesZeroStrengthValues() {
		val config = AdaptiveTrackingConfig().apply {
			footAnchorStrength = 0f
			yawCorrectionStrength = 0f
		}
		val builder = FlatBufferBuilder(64)
		val offset = createAdaptiveTrackingSettings(builder, config)
		builder.finish(offset)
		val response = AdaptiveTrackingSettings.getRootAsAdaptiveTrackingSettings(builder.dataBuffer())

		assertTrue(response.hasFootAnchorStrength())
		assertEquals(0f, response.footAnchorStrength())
		assertTrue(response.hasYawCorrectionStrength())
		assertEquals(0f, response.yawCorrectionStrength())
	}

	@Test
	fun numericUpdatesRejectNonfiniteValuesAndClampRanges() {
		val builder = FlatBufferBuilder(64)
		AdaptiveTrackingSettings.startAdaptiveTrackingSettings(builder)
		AdaptiveTrackingSettings.addFootAnchorStrength(builder, -0.5f)
		AdaptiveTrackingSettings.addTelemetrySampleRateHz(builder, 200)
		AdaptiveTrackingSettings.addYawCorrectionStrength(builder, Float.NaN)
		AdaptiveTrackingSettings.addPoseOptimizerEnabled(builder, AdaptiveBoolean.TRUE)
		val offset = AdaptiveTrackingSettings.endAdaptiveTrackingSettings(builder)
		builder.finish(offset)
		val request = AdaptiveTrackingSettings.getRootAsAdaptiveTrackingSettings(builder.dataBuffer())
		val config = AdaptiveTrackingConfig().apply { yawCorrectionStrength = 0.25f }

		applyAdaptiveTrackingSettings(request, config)

		assertEquals(0f, config.footAnchorStrength)
		assertEquals(100, config.telemetrySampleRateHz)
		assertEquals(0.25f, config.yawCorrectionStrength)
		assertTrue(config.poseOptimizerEnabled)
	}

	@Test
	fun temperatureLearningIsAppliedAndClearActionIsNotReturned() {
		val requestBuilder = FlatBufferBuilder(64)
		AdaptiveTrackingSettings.startAdaptiveTrackingSettings(requestBuilder)
		AdaptiveTrackingSettings.addTemperatureLearningEnabled(requestBuilder, AdaptiveBoolean.TRUE)
		AdaptiveTrackingSettings.addClearLearnedCalibration(requestBuilder, AdaptiveBoolean.TRUE)
		val requestOffset = AdaptiveTrackingSettings.endAdaptiveTrackingSettings(requestBuilder)
		requestBuilder.finish(requestOffset)
		val request = AdaptiveTrackingSettings.getRootAsAdaptiveTrackingSettings(requestBuilder.dataBuffer())
		val config = AdaptiveTrackingConfig()

		applyAdaptiveTrackingSettings(request, config)

		assertTrue(config.temperatureLearningEnabled)
		val responseBuilder = FlatBufferBuilder(64)
		val responseOffset = createAdaptiveTrackingSettings(responseBuilder, config)
		responseBuilder.finish(responseOffset)
		val response = AdaptiveTrackingSettings.getRootAsAdaptiveTrackingSettings(responseBuilder.dataBuffer())
		assertTrue(response.hasTemperatureLearningEnabled())
		assertEquals(AdaptiveBoolean.TRUE, response.temperatureLearningEnabled())
		assertFalse(response.hasClearLearnedCalibration())
	}

	@Test
	fun armCalibrationModeIsOptionalAndRejectsUnknownValues() {
		val config = AdaptiveTrackingConfig().apply { armCalibrationMode = "mounting" }
		val absentBuilder = FlatBufferBuilder(64)
		AdaptiveTrackingSettings.startAdaptiveTrackingSettings(absentBuilder)
		val absentOffset = AdaptiveTrackingSettings.endAdaptiveTrackingSettings(absentBuilder)
		absentBuilder.finish(absentOffset)
		applyAdaptiveTrackingSettings(
			AdaptiveTrackingSettings.getRootAsAdaptiveTrackingSettings(absentBuilder.dataBuffer()),
			config,
		)
		assertEquals("mounting", config.armCalibrationMode)

		val unknownBuilder = FlatBufferBuilder(64)
		val unknownMode = unknownBuilder.createString("unsupported")
		AdaptiveTrackingSettings.startAdaptiveTrackingSettings(unknownBuilder)
		AdaptiveTrackingSettings.addArmCalibrationMode(unknownBuilder, unknownMode)
		val unknownOffset = AdaptiveTrackingSettings.endAdaptiveTrackingSettings(unknownBuilder)
		unknownBuilder.finish(unknownOffset)
		applyAdaptiveTrackingSettings(
			AdaptiveTrackingSettings.getRootAsAdaptiveTrackingSettings(unknownBuilder.dataBuffer()),
			config,
		)
		assertEquals("mounting", config.armCalibrationMode)

		val validBuilder = FlatBufferBuilder(64)
		val validMode = validBuilder.createString("proportions")
		AdaptiveTrackingSettings.startAdaptiveTrackingSettings(validBuilder)
		AdaptiveTrackingSettings.addArmCalibrationMode(validBuilder, validMode)
		val validOffset = AdaptiveTrackingSettings.endAdaptiveTrackingSettings(validBuilder)
		validBuilder.finish(validOffset)
		applyAdaptiveTrackingSettings(
			AdaptiveTrackingSettings.getRootAsAdaptiveTrackingSettings(validBuilder.dataBuffer()),
			config,
		)
		assertEquals("proportions", config.armCalibrationMode)
	}

	@Test
	fun floorEstimationPreservesAbsentAndExplicitFalseUpdates() {
		val config = AdaptiveTrackingConfig().apply { floorEstimationEnabled = true }
		val absentBuilder = FlatBufferBuilder(64)
		AdaptiveTrackingSettings.startAdaptiveTrackingSettings(absentBuilder)
		val absentOffset = AdaptiveTrackingSettings.endAdaptiveTrackingSettings(absentBuilder)
		absentBuilder.finish(absentOffset)
		val absent = AdaptiveTrackingSettings.getRootAsAdaptiveTrackingSettings(absentBuilder.dataBuffer())
		assertFalse(absent.hasFloorEstimationEnabled())
		applyAdaptiveTrackingSettings(absent, config)
		assertTrue(config.floorEstimationEnabled)

		val falseBuilder = FlatBufferBuilder(64)
		AdaptiveTrackingSettings.startAdaptiveTrackingSettings(falseBuilder)
		AdaptiveTrackingSettings.addFloorEstimationEnabled(falseBuilder, AdaptiveBoolean.FALSE)
		val falseOffset = AdaptiveTrackingSettings.endAdaptiveTrackingSettings(falseBuilder)
		falseBuilder.finish(falseOffset)
		val explicitFalse = AdaptiveTrackingSettings.getRootAsAdaptiveTrackingSettings(falseBuilder.dataBuffer())
		assertTrue(explicitFalse.hasFloorEstimationEnabled())
		applyAdaptiveTrackingSettings(explicitFalse, config)
		assertFalse(config.floorEstimationEnabled)

		val responseBuilder = FlatBufferBuilder(64)
		val responseOffset = createAdaptiveTrackingSettings(responseBuilder, config)
		responseBuilder.finish(responseOffset)
		val response = AdaptiveTrackingSettings.getRootAsAdaptiveTrackingSettings(responseBuilder.dataBuffer())
		assertTrue(response.hasFloorEstimationEnabled())
		assertEquals(AdaptiveBoolean.FALSE, response.floorEstimationEnabled())
	}
}
