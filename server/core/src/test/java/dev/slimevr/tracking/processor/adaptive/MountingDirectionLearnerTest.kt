package dev.slimevr.tracking.processor.adaptive

import io.github.axisangles.ktmath.Quaternion
import io.github.axisangles.ktmath.Vector3
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertNotNull
import kotlin.test.assertNull
import kotlin.test.assertTrue

class MountingDirectionLearnerTest {
	private val upperLength = 0.30
	private val lowerLength = 0.27
	private val direction = normalized(Vector3(sin(Math.toRadians(2.0)).toFloat(), -cos(Math.toRadians(2.0)).toFloat(), 0f))

	@Test
	fun estimatesSmallDirectionOffsetAcrossDiversePoses() {
		val learner = MountingDirectionLearner(upperLength, lowerLength)
		var estimate: MountingDirectionEstimate? = null
		for (i in 0..350) estimate = learner.observe(rotation(i), displacement(i), 0.97f, i * 200_000_000L)

		val result = assertNotNull(estimate)
		assertEquals(351, result.sampleCount)
		assertEquals(351, result.inlierCount)
		assertTrue(result.trustedSeconds >= 60.0)
		assertTrue(result.conditionNumber <= 100.0)
		assertTrue(result.distanceConstraintRmsMeters < 1e-4)
		assertTrue((result.direction - direction).len() < 1e-3f)
		assertTrue(result.confidence > 0.9f)
	}

	@Test
	fun changingWorldOrientationWithoutChangingElbowGeometryIsRankDeficient() {
		val learner = MountingDirectionLearner(upperLength, lowerLength)
		val localForearm = Vector3(0.2f, 0.1f, 0.97f)
		val localDisplacement = sum(scale(direction, upperLength), scale(normalized(localForearm), lowerLength))
		for (i in 0..350) {
			val q = rotation(i)
			learner.observe(q, q.sandwich(localDisplacement), 0.95f, i * 200_000_000L)
		}
		assertNull(learner.observe(rotation(351), rotation(351).sandwich(localDisplacement), 0.95f, 351 * 200_000_000L))
	}

	@Test
	fun rejectsDirectionOutsideFiveDegreePrior() {
		val learner = MountingDirectionLearner(upperLength, lowerLength)
		val shifted = normalized(Vector3(sin(Math.toRadians(15.0)).toFloat(), -cos(Math.toRadians(15.0)).toFloat(), 0f))
		var result: MountingDirectionEstimate? = null
		for (i in 0..350) result = learner.observe(rotation(i), displacement(i, shifted), 0.99f, i * 200_000_000L)
		assertNull(result)
	}

	@Test
	fun trimsAtMostTenPercentAndRejectsCorruptedMajority() {
		val learner = MountingDirectionLearner(upperLength, lowerLength)
		var estimate: MountingDirectionEstimate? = null
		for (i in 0..350) {
			var d = displacement(i)
			if (i % 5 == 0) d = Vector3(d.x + 0.05f, d.y, d.z - 0.04f)
			estimate = learner.observe(rotation(i), d, 0.98f, i * 200_000_000L)
		}
		assertNull(estimate)
	}

	@Test
	fun gapsBreakTrustedIntervalsAndResetClearsEvidence() {
		val learner = MountingDirectionLearner(upperLength, lowerLength)
		var estimate: MountingDirectionEstimate? = null
		for (i in 0..200) estimate = learner.observe(rotation(i), displacement(i), 0.95f, i * 200_000_000L)
		val afterGap = 200 * 200_000_000L + 600_000_000L
		estimate = learner.observe(rotation(201), displacement(201), 0.95f, afterGap)
		assertNull(estimate)
		for (i in 202..300) estimate = learner.observe(rotation(i), displacement(i), 0.95f, afterGap + (i - 201) * 200_000_000L)
		assertNull(estimate)
		estimate = learner.observe(rotation(301), displacement(301), 0.95f, afterGap + 100 * 200_000_000L)
		assertNotNull(estimate)
		learner.reset()
		assertNull(learner.observe(rotation(0), displacement(0), 0.95f, 0L))
	}

	@Test
	fun lowConfidenceDoesNotAddEvidenceOrMutateInputs() {
		val learner = MountingDirectionLearner(upperLength, lowerLength)
		val q = rotation(2)
		val d = displacement(2)
		val qBefore = q
		val dBefore = d
		var estimate: MountingDirectionEstimate? = null
		for (i in 0..350) estimate = learner.observe(rotation(i), displacement(i), 0.95f, i * 200_000_000L)
		assertNotNull(estimate)
		assertNull(learner.observe(q, d, 0.89f, 352 * 200_000_000L))
		assertNull(learner.observe(q, d, 1.01f, 353 * 200_000_000L))
		assertEquals(qBefore, q)
		assertEquals(dBefore, d)
	}

	@Test
	fun boundedSampleWindowContinuesToRefreshEstimate() {
		val learner = MountingDirectionLearner(upperLength, lowerLength)
		var estimate: MountingDirectionEstimate? = null
		for (i in 0..700) estimate = learner.observe(rotation(i), displacement(i), 0.96f, i * 200_000_000L)
		val result = assertNotNull(estimate)
		assertEquals(MountingDirectionLearner.MAX_SAMPLES, result.sampleCount)
		assertTrue(result.trustedSeconds >= 60.0)
	}

	private fun displacement(index: Int, axis: Vector3 = direction): Vector3 {
		val y = -0.9 + 1.8 * ((index * 37) % 101) / 100.0
		val radius = sqrt(1.0 - y * y)
		val phase = index * 2.399963229728653
		val forearm = Vector3((radius * cos(phase)).toFloat(), y.toFloat(), (radius * sin(phase)).toFloat())
		val local = sum(scale(axis, upperLength), scale(forearm, lowerLength))
		return rotation(index).sandwich(local)
	}

	private fun rotation(index: Int): Quaternion {
		val yaw = axisAngle(Vector3(0f, 1f, 0f), index * 0.071)
		val pitch = axisAngle(Vector3(1f, 0f, 0f), index * 0.037)
		val roll = axisAngle(Vector3(0f, 0f, 1f), index * 0.053)
		return (yaw * pitch * roll).unit()
	}

	private fun axisAngle(axis: Vector3, radians: Double): Quaternion {
		val half = radians / 2.0
		val s = sin(half).toFloat()
		return Quaternion(cos(half).toFloat(), axis.x * s, axis.y * s, axis.z * s)
	}

	private fun scale(v: Vector3, amount: Double) = Vector3((v.x * amount).toFloat(), (v.y * amount).toFloat(), (v.z * amount).toFloat())
	private fun sum(a: Vector3, b: Vector3) = Vector3(a.x + b.x, a.y + b.y, a.z + b.z)
	private fun normalized(v: Vector3): Vector3 {
		val norm = v.len()
		return Vector3(v.x / norm, v.y / norm, v.z / norm)
	}
}
