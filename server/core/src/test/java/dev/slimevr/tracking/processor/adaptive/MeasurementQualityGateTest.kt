package dev.slimevr.tracking.processor.adaptive

import dev.slimevr.tracking.trackers.TrackerStatus
import io.github.axisangles.ktmath.Quaternion
import org.junit.jupiter.api.Test
import kotlin.test.assertFalse
import kotlin.test.assertTrue

class MeasurementQualityGateTest {
	private fun sample(score: Float, health: Float? = null, status: TrackerStatus = TrackerStatus.OK) = TelemetrySample(
		id = 1,
		name = "foot",
		role = null,
		status = status,
		rawRotation = Quaternion.IDENTITY,
		adjustedRotation = Quaternion.IDENTITY,
		position = null,
		acceleration = null,
		derivedLinearVelocity = null,
		packetAgeNanos = 0L,
		temperatureCelsius = null,
		angularSpeedRadiansPerSecond = 0f,
		confidence = TrackerConfidence(score, emptyList(), 2.0),
		health = health?.let { TrackerHealthDiagnostic(1, it, 0f, emptyList(), 2.0, false) },
	)

	@Test
	fun badSensorHealthOrConfidenceCannotSupportFootContact() {
		assertTrue(MeasurementQualityGate.trusted(sample(0.9f, 0.9f)))
		assertTrue(MeasurementQualityGate.trusted(sample(0.9f)))
		assertFalse(MeasurementQualityGate.trusted(sample(0.4f, 0.9f)))
		assertFalse(MeasurementQualityGate.trusted(sample(0.9f, 0.2f)))
		assertFalse(MeasurementQualityGate.trusted(sample(0.9f, 0.9f, TrackerStatus.DISCONNECTED)))
		assertFalse(MeasurementQualityGate.trusted(null))
	}
}
