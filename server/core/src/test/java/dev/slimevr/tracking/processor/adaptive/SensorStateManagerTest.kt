package dev.slimevr.tracking.processor.adaptive

import dev.slimevr.tracking.trackers.Tracker
import dev.slimevr.tracking.trackers.TrackerPosition
import dev.slimevr.tracking.trackers.TrackerStatus
import io.github.axisangles.ktmath.Quaternion
import io.github.axisangles.ktmath.Vector3
import org.junit.jupiter.api.Test
import kotlin.test.assertEquals
import kotlin.test.assertNotNull
import kotlin.test.assertNull
import kotlin.test.assertTrue

class SensorStateManagerTest {
	private fun tracker(id: Int = 1, positional: Boolean = true, timeout: Boolean = false) = Tracker(
		null, id, "sensor-$id", trackerPosition = TrackerPosition.WAIST,
		hasPosition = positional, hasRotation = true, hasAcceleration = true,
		trackRotDirection = false, usesTimeout = timeout,
	).apply { status = TrackerStatus.OK }

	@Test
	fun capturesSupportedFieldsWithoutMutation() {
		val tracker = tracker()
		val rotation = Quaternion.rotationAroundYAxis(0.25f)
		tracker.setRotation(rotation)
		tracker.position = Vector3(1f, 2f, 3f)
		tracker.setAcceleration(Vector3(4f, 5f, 6f))
		tracker.temperature = 20f
		val sample = SensorStateManager().sample(listOf(tracker), emptyList(), -100L).samples.single()
		assertEquals(rotation, sample.rawRotation)
		assertEquals(rotation, sample.adjustedRotation)
		assertEquals(tracker.position, sample.position)
		assertEquals(20f, sample.temperatureCelsius)
		assertNull(sample.angularSpeedRadiansPerSecond)
		assertNull(sample.packetAgeNanos)
		assertEquals(rotation, tracker.getRawRotation())
	}

	@Test
	fun simultaneousTranslationAndRotationUseSamePriorFrame() {
		val tracker = tracker()
		val manager = SensorStateManager()
		manager.sample(listOf(tracker), emptyList(), -100_000_000L)
		tracker.position = Vector3(0.1f, 0f, 0f)
		tracker.setRotation(Quaternion.rotationAroundYAxis(0.2f))
		val sample = manager.sample(listOf(tracker), emptyList(), 0L).samples.single()
		assertEquals(2f, assertNotNull(sample.angularSpeedRadiansPerSecond), 0.0001f)
		assertEquals(1f, assertNotNull(sample.derivedLinearVelocity).x, 0.0001f)
	}

	@Test
	fun quaternionSignDoesNotCreateAngularSpeed() {
		val tracker = tracker()
		val manager = SensorStateManager()
		tracker.setRotation(Quaternion.rotationAroundYAxis(0.5f))
		manager.sample(listOf(tracker), emptyList(), 0L)
		tracker.setRotation(-tracker.getRawRotation())
		val sample = manager.sample(listOf(tracker), emptyList(), 100_000_000L).samples.single()
		assertEquals(0f, assertNotNull(sample.angularSpeedRadiansPerSecond), 0.00001f)
	}

	@Test
	fun resetMissingSamplesAndNonForwardTimeClearDerivatives() {
		val tracker = tracker()
		val manager = SensorStateManager()
		fun sample(time: Long) = manager.sample(listOf(tracker), emptyList(), time)
		sample(0)
		assertNull(sample(0).samples.single().angularSpeedRadiansPerSecond)
		assertNull(sample(600_000_000).samples.single().angularSpeedRadiansPerSecond)
		manager.reset()
		assertEquals(1L, sample(700_000_000).resetEpoch)
		assertNull(sample(600_000_000).samples.single().derivedLinearVelocity)
		manager.sample(emptyList(), emptyList(), 700_000_000)
		assertNull(sample(800_000_000).samples.single().angularSpeedRadiansPerSecond)
	}

	@Test
	fun unavailableStatusesAndRecoveryDoNotProduceMotion() {
		for (status in listOf(TrackerStatus.DISCONNECTED, TrackerStatus.TIMED_OUT, TrackerStatus.ERROR, TrackerStatus.OCCLUDED)) {
			val tracker = tracker()
			val manager = SensorStateManager()
			manager.sample(listOf(tracker), emptyList(), 0L)
			tracker.status = status
			tracker.position = Vector3(10f, 0f, 0f)
			val lost = manager.sample(listOf(tracker), emptyList(), 100_000_000L).samples.single()
			assertNull(lost.angularSpeedRadiansPerSecond)
			assertNull(lost.derivedLinearVelocity)
			tracker.status = TrackerStatus.OK
			val recovered = manager.sample(listOf(tracker), emptyList(), 200_000_000L).samples.single()
			assertNull(recovered.angularSpeedRadiansPerSecond)
			assertNull(recovered.derivedLinearVelocity)
		}
	}

	@Test
	fun invalidDataIsUnknownAndDoesNotContaminateHistory() {
		val tracker = tracker()
		val manager = SensorStateManager()
		manager.sample(listOf(tracker), emptyList(), 0L)
		tracker.setRotation(Quaternion.NULL)
		tracker.temperature = Float.NaN
		val invalid = manager.sample(listOf(tracker), emptyList(), 100_000_000L).samples.single()
		assertNull(invalid.rawRotation)
		assertNull(invalid.adjustedRotation)
		assertNull(invalid.temperatureCelsius)
		tracker.setRotation(Quaternion.IDENTITY)
		assertNull(manager.sample(listOf(tracker), emptyList(), 200_000_000L).samples.single().angularSpeedRadiansPerSecond)
	}

	@Test
	fun staleRotationPacketsSuppressDerivativesUntilRecovery() {
		val tracker = tracker(timeout = true)
		tracker.dataTick()
		val received = assertNotNull(tracker.lastRotationUpdateNanos)
		val manager = SensorStateManager()
		manager.sample(listOf(tracker), emptyList(), received)
		val stale = manager.sample(listOf(tracker), emptyList(), received + 600_000_000).samples.single()
		assertEquals(600_000_000L, stale.packetAgeNanos)
		assertNull(stale.angularSpeedRadiansPerSecond)
		assertNull(stale.derivedLinearVelocity)
	}

	@Test
	fun reusedIdsAndChangedDesignationDoNotInheritHistory() {
		val manager = SensorStateManager()
		val original = tracker()
		manager.sample(listOf(original), emptyList(), 0)
		val replacement = tracker()
		assertNull(manager.sample(listOf(replacement), emptyList(), 100_000_000).samples.single().angularSpeedRadiansPerSecond)
		replacement.trackerPosition = TrackerPosition.LEFT_FOOT
		assertNull(manager.sample(listOf(replacement), emptyList(), 200_000_000).samples.single().angularSpeedRadiansPerSecond)
	}

	@Test
	fun historyIsBoundedAndComputedNamespaceIsSeparate() {
		val manager = SensorStateManager(maxHistory = 1)
		val a = tracker(1)
		val b = tracker(2)
		manager.sample(listOf(a, b), emptyList(), 0)
		assertNull(manager.sample(listOf(a), emptyList(), 100_000_000).samples.single().angularSpeedRadiansPerSecond)
		val split = SensorStateManager()
		val output = tracker(1)
		split.sample(listOf(a), listOf(output), 0)
		output.setRotation(Quaternion.rotationAroundYAxis(0.2f))
		val frame = split.sample(listOf(a), listOf(output), 100_000_000)
		assertEquals(0f, assertNotNull(frame.samples.single().angularSpeedRadiansPerSecond), 0.00001f)
		assertTrue(assertNotNull(frame.computedSamples.single().angularSpeedRadiansPerSecond) > 1f)
	}

	@Test
	fun unsupportedMeasurementsRemainNull() {
		val tracker = Tracker(null, 8, "unknown", trackerPosition = null, trackRotDirection = false)
		val sample = SensorStateManager().sample(listOf(tracker), emptyList(), 0).samples.single()
		assertNull(sample.rawRotation)
		assertNull(sample.position)
		assertNull(sample.acceleration)
		assertNull(sample.temperatureCelsius)
	}
}
