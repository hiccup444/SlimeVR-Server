package dev.slimevr.tracking.processor.adaptive

import io.github.axisangles.ktmath.Quaternion
import io.github.axisangles.ktmath.Vector3
import org.junit.jupiter.api.Test
import kotlin.test.assertEquals
import kotlin.test.assertNull
import kotlin.test.assertTrue

class FootContactDetectorTest {
	private fun update(detector: FootContactDetector, millis: Long, position: Vector3 = Vector3.NULL, rotation: Quaternion = Quaternion.IDENTITY, trusted: Boolean = true, acceleration: Vector3? = Vector3.NULL) = detector.update(position, rotation, 0f, acceleration, trusted, millis * 1_000_000)

	private fun planted(acceleration: Vector3? = Vector3.NULL): FootContactDetector {
		val detector = FootContactDetector()
		for (millis in 0L..400L step 20) update(detector, millis, acceleration = acceleration)
		assertEquals(FootContactState.PLANTED, detector.snapshot.state)
		return detector
	}

	@Test
	fun contactRequiresDwellAndRampsItsWeight() {
		val detector = FootContactDetector()
		assertEquals(FootContactState.AIRBORNE, update(detector, 0).state)
		assertEquals(FootContactState.CONTACT_CANDIDATE, update(detector, 20).state)
		for (time in 40L..160L step 20) assertEquals(0f, update(detector, time).weight)
		assertEquals(FootContactState.PLANTED, update(detector, 180).state)
		assertEquals(0f, detector.snapshot.weight)
		assertTrue(update(detector, 200).weight in 0.01f..0.94f)
		assertEquals(0.95f, update(detector, 280).weight)
	}

	@Test
	fun softCorrectionPreservesHeightAndCannotExceedItsBound() {
		val detector = planted()
		val input = Vector3(0.10f, 0.03f, 0f)
		val corrected = detector.correct(input, 1f)
		assertEquals(input.y, corrected.y)
		assertEquals(0.06f, (input - corrected).len(), 0.00001f)
		assertTrue(corrected.x > 0f)
		assertEquals(input, detector.correct(input, 0f))
		assertEquals(input, detector.correct(input, Float.NaN))
	}

	@Test
	fun liftingOrFastMotionReleasesWithinFiftyMilliseconds() {
		val detector = planted()
		val lifted = Vector3(0f, 0.2f, 0f)
		assertEquals(FootContactState.RELEASING, update(detector, 420, lifted).state)
		assertTrue(update(detector, 440, lifted).weight < 0.95f)
		assertEquals(FootContactState.AIRBORNE, update(detector, 480, lifted).state)
		assertNull(detector.snapshot.plantPosition)
		assertEquals(lifted, detector.correct(lifted, 1f))
	}

	@Test
	fun rotatingFootAndAccelerationCanReleaseWithoutTranslation() {
		val rotated = planted()
		assertEquals(FootContactState.RELEASING, update(rotated, 420, rotation = Quaternion.rotationAroundYAxis(0.3f)).state)
		val accelerated = planted()
		assertEquals(FootContactState.RELEASING, update(accelerated, 420, acceleration = Vector3(3f, 0f, 0f)).state)
	}

	@Test
	fun staleGapsInvalidDataAndTrackingLossDiscardAnchors() {
		val gap = planted()
		assertEquals(FootContactState.AIRBORNE, update(gap, 1000).state)
		assertNull(gap.snapshot.plantPosition)
		val lost = planted()
		assertEquals(FootContactState.AIRBORNE, update(lost, 420, trusted = false).state)
		assertEquals(0f, update(lost, 440).weight)
		val invalid = planted()
		assertEquals(0f, update(invalid, 420, rotation = Quaternion.NULL).weight)
		val reversed = planted()
		assertEquals(0f, update(reversed, 300).weight)
	}

	@Test
	fun missingAccelerationCapsWeightAndQuaternionSignIsIrrelevant() {
		val detector = planted(acceleration = null)
		assertEquals(0.65f, detector.snapshot.weight)
		assertEquals(FootContactState.PLANTED, update(detector, 420, rotation = -Quaternion.IDENTITY, acceleration = null).state)
	}

	@Test
	fun airborneMotionCannotAccumulateContactTime() {
		val detector = FootContactDetector()
		for (time in 0L..1000L step 20) {
			assertEquals(FootContactState.AIRBORNE, update(detector, time, Vector3(0f, 0.3f, 0f)).state)
		}
		detector.reset()
		assertEquals(0f, detector.snapshot.weight)
	}
}
