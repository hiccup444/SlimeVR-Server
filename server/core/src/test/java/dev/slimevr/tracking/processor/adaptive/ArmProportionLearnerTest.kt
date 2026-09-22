package dev.slimevr.tracking.processor.adaptive

import io.github.axisangles.ktmath.Vector3
import org.junit.jupiter.api.Test
import kotlin.math.cos
import kotlin.math.sin
import kotlin.test.assertEquals
import kotlin.test.assertNull
import kotlin.test.assertTrue

class ArmProportionLearnerTest {
	private fun displacement(upper: Double, lower: Double, angle: Double): Vector3 = Vector3(
		(upper + lower * cos(angle)).toFloat(),
		(lower * sin(angle)).toFloat(),
		0f,
	)

	private fun feedSweep(
		learner: ArmProportionLearner,
		startTime: Long = 0L,
		count: Int = 351,
		corruptEvery: Int = 0,
		confidence: Float = 0.98f,
		upperDirection: (Int) -> Vector3 = { Vector3.POS_X },
	): ArmProportionLearner.ArmLengthEstimate? {
		var result: ArmProportionLearner.ArmLengthEstimate? = null
		for (index in 0 until count) {
			val phase = (index % 100) / 99.0
			val angle = 0.12 + phase * 2.6
			val d = if (corruptEvery > 0 && index % corruptEvery == 0) {
				Vector3(0.5f, 0.5f, 0f)
			} else {
				displacement(0.30, 0.27, angle)
			}
			result = learner.observe(d, upperDirection(index), confidence, startTime + index * 200_000_000L)
		}
		return result
	}

	@Test
	fun diverseElbowAnglesRecoverBothSegmentLengths() {
		val learner = ArmProportionLearner(0.30, 0.27)
		val estimate = feedSweep(learner)
		assertTrue(estimate != null)
		assertEquals(0.30, estimate!!.upperArmMeters, 0.005)
		assertEquals(0.27, estimate.lowerArmMeters, 0.005)
		assertTrue(estimate.observedSeconds >= 60.0)
		assertTrue(estimate.lowerArmDistanceRmsMeters <= 0.01)
		assertTrue(estimate.inlierCount >= 90)
		assertTrue(estimate.upperArmUncertaintyMeters >= 0.0)
		assertTrue(estimate.lowerArmUncertaintyMeters >= 0.0)
	}

	@Test
	fun fixedPoseDoesNotProvideObservableLengths() {
		val learner = ArmProportionLearner(0.30, 0.27)
		var result: ArmProportionLearner.ArmLengthEstimate? = null
		val pose = displacement(0.30, 0.27, 1.0)
		for (index in 0 until 351) {
			result = learner.observe(pose, Vector3.POS_X, 0.98f, index * 200_000_000L)
		}
		assertNull(result)
	}

	@Test
	fun robustFitRejectsSmallNumberOfCorruptedPoses() {
		val learner = ArmProportionLearner(0.30, 0.27)
		val estimate = feedSweep(learner, corruptEvery = 20)
		assertTrue(estimate != null)
		assertEquals(0.30, estimate!!.upperArmMeters, 0.01)
		assertEquals(0.27, estimate.lowerArmMeters, 0.01)
		assertTrue(estimate.inlierCount < 351)
		assertTrue(estimate.inlierCount.toDouble() / 351 >= 0.90)
	}

	@Test
	fun inconsistentUpperDirectionCannotFitTheKnownLengthGeometry() {
		val learner = ArmProportionLearner(0.30, 0.27)
		val estimate = feedSweep(learner, upperDirection = { Vector3.POS_Y })
		assertNull(estimate)
	}

	@Test
	fun lowConfidenceAndInvalidVectorsDoNotAddSamples() {
		val lowConfidence = ArmProportionLearner(0.30, 0.27)
		assertNull(feedSweep(lowConfidence, confidence = 0.89f))

		val badDirection = ArmProportionLearner(0.30, 0.27)
		assertNull(badDirection.observe(displacement(0.30, 0.27, 1.0), Vector3.NULL, 0.99f, 0L))
		assertNull(badDirection.observe(Vector3(Float.NaN, 0f, 0f), Vector3.POS_X, 0.99f, 200_000_000L))
	}

	@Test
	fun stationaryPhasesAccumulateOnlyAdjacentTrustedTimeAcrossMotionAndGaps() {
		val learner = ArmProportionLearner(0.30, 0.27)
		for (index in 0 until 175) {
			val phase = (index % 100) / 99.0
			learner.observe(displacement(0.30, 0.27, 0.12 + phase * 2.6), Vector3.POS_X, 0.98f, index * 200_000_000L)
		}
		val duringMotion = learner.observe(displacement(0.30, 0.27, 1.0), Vector3.POS_X, 0.5f, 35_000_000_000L)
		assertNull(duringMotion)
		var estimate: ArmProportionLearner.ArmLengthEstimate? = null
		for (index in 175 until 350) {
			val phase = (index % 100) / 99.0
			estimate = learner.observe(
				displacement(0.30, 0.27, 0.12 + phase * 2.6),
				Vector3.POS_X,
				0.98f,
				35_200_000_000L + (index - 175) * 200_000_000L,
			)
		}
		assertTrue(estimate != null)
		assertEquals(69.6, estimate!!.observedSeconds, 0.1)
	}

	@Test
	fun longGapsExcludeTimeButRetainRecentValidPoseSamples() {
		val learner = ArmProportionLearner(0.30, 0.27)
		for (index in 0 until 200) {
			learner.observe(displacement(0.30, 0.27, index * 0.01), Vector3.POS_X, 0.99f, index * 200_000_000L)
		}
		val estimate = feedSweep(learner, startTime = 50_000_000_000L)
		assertTrue(estimate != null)
		assertTrue(estimate!!.observedSeconds >= 60.0)
	}

	@Test
	fun samplingRateAndPhysiologicalBoundsAreEnforced() {
		val learner = ArmProportionLearner(0.30, 0.27)
		var result: ArmProportionLearner.ArmLengthEstimate? = null
		for (index in 0 until 1301) {
			val phase = (index % 100) / 99.0
			result = learner.observe(displacement(0.30, 0.27, phase * 3.0), Vector3.POS_X, 0.99f, index * 100_000_000L)
		}
		assertTrue(result != null)
		assertTrue(result!!.inlierCount <= 512)
		assertEquals(102.2, result.observedSeconds, 0.2)
		val impossible = ArmProportionLearner(0.15, 0.15)
		assertNull(feedSweep(impossible))
	}
}
