package dev.slimevr.tracking.processor.adaptive

import com.fasterxml.jackson.databind.ObjectMapper
import dev.slimevr.config.AdaptiveTrackingConfig
import dev.slimevr.config.ConfigManager
import dev.slimevr.tracking.processor.HumanPoseManager
import dev.slimevr.unit.TestTrackerSet
import io.github.axisangles.ktmath.Quaternion
import io.github.axisangles.ktmath.Vector3
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.io.TempDir
import java.nio.file.Files
import java.nio.file.Path
import kotlin.test.assertEquals
import kotlin.test.assertFalse
import kotlin.test.assertNotNull
import kotlin.test.assertNull
import kotlin.test.assertTrue

class AdaptiveTelemetryRecorderTest {
	@TempDir lateinit var directory: Path

	@Test
	fun predictionRecordingUsesExplicitRotationsAndResidualVectors() {
		val measured = Quaternion.rotationAroundYAxis(0.1f)
		val residual = assertNotNull(PoseResidualMonitor().observe(7, measured, Quaternion.IDENTITY, 1L))
		val frame = AdaptiveTelemetryFrame(1L, emptyList(), emptyList(), 0L, trackerPredictions = mapOf(7 to TrackerPosePrediction(Quaternion.IDENTITY, measured, residual)))
		val mapper = ObjectMapper()
		val prediction = mapper.readTree(mapper.writeValueAsString(frame.toRecord()))["trackerPredictions"]["7"]
		assertEquals(1.0, prediction["expectedRotation"]["w"].asDouble())
		assertEquals(0.1, prediction["residual"]["residualVectorRadians"]["y"].asDouble(), 1e-6)
		assertFalse(prediction["residual"]["independentlyConstrained"].asBoolean())
	}

	@Test
	fun yawHoldoverStateIsAvailableInRecordedDiagnostics() {
		val diagnostic = TrackerDriftDiagnostic(7, 0.01f, DriftResidual(0f, 0f, 0.0, false, "ARM_MOTION"), 0.0005, holdoverActive = true)
		val frame = AdaptiveTelemetryFrame(1L, emptyList(), emptyList(), 0L, driftDiagnostics = listOf(diagnostic))
		val json = ObjectMapper().readTree(ObjectMapper().writeValueAsString(frame.toRecord()))
		assertTrue(json["driftDiagnostics"][0]["holdoverActive"].asBoolean())
		assertEquals(0.0005, json["driftDiagnostics"][0]["predictedRateRadiansPerSecond"].asDouble(), 1e-9)
	}

	@Test
	fun liveDiagnosticsDoNotCreateRecordingFiles() {
		val path = directory.resolve("live")
		val telemetry = AdaptiveTrackingTelemetry(
			AdaptiveTrackingConfig().apply {
				liveDiagnosticsEnabled = true
				telemetryDirectory = path.toString()
			},
		)
		telemetry.update(emptyList(), emptyList(), 1L)
		assertNotNull(telemetry.latestFrame)
		assertFalse(Files.exists(path))
		telemetry.close()
	}

	@Test
	fun recordedInputRotationMatchesThePoseTickBeforeNewBiasTakesEffect() {
		val tracker = TestTrackerSet().leftThigh
		val timestamp = 100_000_000L
		tracker.dataTick(timestamp)
		val poseInput = SensorStateManager().sample(listOf(tracker), emptyList(), timestamp)
		tracker.adaptiveYawBiasRadians = 0.1f
		val telemetry = AdaptiveTrackingTelemetry(
			AdaptiveTrackingConfig().apply {
				liveDiagnosticsEnabled = true
				confidenceDiagnosticsEnabled = false
			},
		)
		try {
			telemetry.update(listOf(tracker), emptyList(), timestamp, qualityFrame = poseInput)
			assertEquals(poseInput.samples.single().adjustedRotation, telemetry.latestFrame!!.samples.single().adjustedRotation)
			assertTrue(tracker.getRotation() != telemetry.latestFrame!!.samples.single().adjustedRotation)
		} finally {
			telemetry.close()
		}
	}

	@Test
	fun liveDiagnosticsExposeRecordingStatusForTheTestScreen() {
		val pose = HumanPoseManager(TestTrackerSet().allL)
		pose.adaptiveTrackingConfig.liveDiagnosticsEnabled = true
		pose.adaptiveTrackingConfig.telemetryDirectory = directory.toString()
		pose.adaptiveTelemetry.update(emptyList(), emptyList(), 1L)
		val idle = ObjectMapper().readTree(assertNotNull(pose.adaptiveDiagnosticsJson()))["recording"]
		assertFalse(idle["requested"].asBoolean())
		assertEquals(50, idle["sampleRateHz"].asInt())
		pose.adaptiveTrackingConfig.telemetryEnabled = true
		pose.adaptiveTelemetry.update(emptyList(), emptyList(), 21_000_001L)
		val active = ObjectMapper().readTree(assertNotNull(pose.adaptiveDiagnosticsJson()))["recording"]
		assertTrue(active["requested"].asBoolean())
		assertTrue(active["active"].asBoolean())
		pose.adaptiveTelemetry.close()
	}

	@Test
	fun contactRecordingIncludesAnchorCoordinatesAndRequestedMode() {
		val frame = AdaptiveTelemetryFrame(
			1L,
			emptyList(),
			emptyList(),
			0L,
			footContacts = mapOf("left" to FootContactSnapshot(FootContactState.PLANTED, Vector3(1f, 2f, 3f), 0.5f)),
			footAnchoringRequested = true,
		)
		val mapper = ObjectMapper()
		val json = mapper.readTree(mapper.writeValueAsString(frame.toRecord()))
		assertEquals(1.0, json["footContacts"]["left"]["plantPosition"]["x"].asDouble())
		assertEquals("PLANTED", json["footContacts"]["left"]["state"].asText())
		assertTrue(json["footAnchoringRequested"].asBoolean())
	}

	@Test
	fun recordsCompleteFramesAndCloses() {
		val trackers = TestTrackerSet()
		val frame = TrackerConfidenceEstimator().observe(SensorStateManager().sample(trackers.allL, emptyList(), 1L))
		val recorder = AdaptiveTelemetryRecorder(directory.toString())
		recorder.offer(frame)
		recorder.close()
		recorder.awaitClosed()
		assertNull(recorder.failure)
		val lines = Files.readAllLines(assertNotNull(recorder.outputPath))
		assertEquals(2, lines.size)
		val json = ObjectMapper().readTree(lines[1])
		assertEquals("frame", json["type"].asText())
		assertEquals(7, json["frame"]["samples"].size())
		assertTrue(json["frame"]["samples"][0].has("rawRotation"))
		assertEquals(1.0, json["frame"]["samples"][0]["rawRotation"]["w"].asDouble())
		assertEquals(0, json["droppedFrames"].asInt())
		assertEquals(0.6, json["frame"]["samples"][0]["confidence"]["score"].asDouble(), 0.0001)
		assertTrue(json["frame"]["samples"][0]["confidence"]["reasons"].any { it.asText() == "FRESHNESS_UNKNOWN" })
	}

	@Test
	fun configurationRoundTripRetainsOptInSettings() {
		val path = directory.resolve("vrconfig.yml")
		Files.writeString(path, "adaptiveTracking:\n  telemetryEnabled: true\n  telemetryDirectory: test-recordings\n  telemetrySampleRateHz: 25\n")
		val config = ConfigManager(path.toString())
		config.loadConfig()
		assertTrue(config.vrConfig.adaptiveTracking.telemetryEnabled)
		assertEquals(25, config.vrConfig.adaptiveTracking.telemetrySampleRateHz)
		config.saveConfig()
		val restored = ConfigManager(path.toString())
		restored.loadConfig()
		assertEquals("test-recordings", restored.vrConfig.adaptiveTracking.telemetryDirectory)
		assertTrue(restored.vrConfig.adaptiveTracking.telemetryEnabled)
	}

	@Test
	fun recordingFailureDoesNotReachCaller() {
		val file = Files.createFile(directory.resolve("not-a-directory"))
		val recorder = AdaptiveTelemetryRecorder(file.toString())
		recorder.close()
		recorder.awaitClosed()
		assertNotNull(recorder.failure)
	}

	@Test
	fun sizeLimitStopsRecording() {
		val recorder = AdaptiveTelemetryRecorder(directory.toString(), maxBytes = 1)
		recorder.offer(SensorStateManager().sample(emptyList(), emptyList(), 1L))
		recorder.close()
		recorder.awaitClosed()
		assertNull(recorder.failure)
		assertEquals(1, Files.readAllLines(assertNotNull(recorder.outputPath)).size)
	}

	@Test
	fun longRecordingRotatesFilesWithoutLosingFrames() {
		val frame = SensorStateManager().sample(emptyList(), emptyList(), 1L)
		val line = ObjectMapper().writeValueAsString(mapOf("type" to "frame", "droppedFrames" to 0L, "frame" to frame.toRecord()))
		val limit = 80L + 2L * (line.toByteArray().size + 2)
		val recorder = AdaptiveTelemetryRecorder(directory.toString(), maxBytes = limit, maxParts = 3)
		repeat(4) { recorder.offer(frame.copy(timestampNanos = it.toLong() + 1)) }
		recorder.close()
		recorder.awaitClosed()
		assertNull(recorder.failure)
		assertFalse(recorder.sizeLimitReached)
		assertEquals(4L, recorder.writtenFrames)
		assertEquals(2, recorder.outputPaths.size)
		assertEquals(3, Files.readAllLines(recorder.outputPaths[0]).size)
		assertEquals(3, Files.readAllLines(recorder.outputPaths[1]).size)
	}

	@Test
	fun disabledTelemetryDoesNotCreateRecordingDirectory() {
		val path = directory.resolve("disabled")
		val telemetry = AdaptiveTrackingTelemetry(AdaptiveTrackingConfig().apply { telemetryDirectory = path.toString() })
		telemetry.update(emptyList(), emptyList(), 1L)
		assertNull(telemetry.latestFrame)
		assertFalse(Files.exists(path))
	}

	@Test
	fun observingSolvedFramesDoesNotChangePoseOrResetResults() {
		val trackers = TestTrackerSet()
		val pose = HumanPoseManager(trackers.allL)
		val sensors = SensorStateManager()
		for (index in 0..20) {
			trackers.chest.setRotation(Quaternion.rotationAroundYAxis(index * 0.03f))
			pose.update()
			val before = pose.computedTrackers.map { it.position to it.getRotation() }
			val inputsBefore = trackers.allL.map { it.getRawRotation() }
			sensors.sample(trackers.allL, pose.computedTrackers, index * 20_000_000L)
			assertEquals(before, pose.computedTrackers.map { it.position to it.getRotation() })
			assertEquals(inputsBefore, trackers.allL.map { it.getRawRotation() })
		}
		pose.resetTrackersYaw("telemetry-test")
		pose.update()
		val before = pose.computedTrackers.map { it.position to it.getRotation() }
		sensors.reset()
		sensors.sample(trackers.allL, pose.computedTrackers, 500_000_000L)
		assertEquals(before, pose.computedTrackers.map { it.position to it.getRotation() })
	}
}
