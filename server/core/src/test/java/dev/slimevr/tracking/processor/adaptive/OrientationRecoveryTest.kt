package dev.slimevr.tracking.processor.adaptive

import io.github.axisangles.ktmath.Quaternion
import org.junit.jupiter.api.Test
import kotlin.test.assertEquals
import kotlin.test.assertTrue

class OrientationRecoveryTest {
	@Test
	fun predictsBriefLossAndStopsExtrapolatingAfterTwoHundredMilliseconds() {
		val model = OrientationRecovery()
		model.update(Quaternion.IDENTITY, true, 0)
		val measured = Quaternion.rotationAroundYAxis(0.01f)
		model.update(measured, true, 20_000_000)
		val early = model.update(measured, false, 40_000_000)
		assertTrue(early.angleToR(Quaternion.IDENTITY) > measured.angleToR(Quaternion.IDENTITY))
		val bounded = model.update(measured, false, 220_000_000)
		assertEquals(bounded, model.update(measured, false, 420_000_000))
		val target = Quaternion.rotationAroundYAxis(0.3f)
		val recovered = model.update(target, true, 440_000_000)
		assertTrue(recovered.angleToR(target) > 0.02f)
		assertTrue(recovered.angleToR(target) < bounded.angleToR(target))
	}

	@Test
	fun gapsAndResetsDiscardPredictionAndHealthyMotionHasNoFiltering() {
		val model = OrientationRecovery()
		val rotation = Quaternion.rotationAroundYAxis(1f)
		assertEquals(rotation, model.update(rotation, true, 0))
		assertEquals(Quaternion.IDENTITY, model.update(Quaternion.IDENTITY, true, 10_000_000))
		assertEquals(rotation, model.update(rotation, true, 2_000_000_000))
		model.reset()
		assertEquals(Quaternion.IDENTITY, model.update(Quaternion.IDENTITY, false, 2_020_000_000))
	}
}
