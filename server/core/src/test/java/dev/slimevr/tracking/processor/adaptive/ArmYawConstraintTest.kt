package dev.slimevr.tracking.processor.adaptive

import io.github.axisangles.ktmath.Vector3
import org.junit.jupiter.api.Test
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt
import kotlin.test.assertEquals
import kotlin.test.assertNotNull
import kotlin.test.assertNull

class ArmYawConstraintTest {
	@Test
	fun recoversPositiveAndNegativeYawBiases() {
		val (displacement, upperLength, lowerLength, trueDirection) = sampleGeometry()

		for (biasDegrees in listOf(-8.0, 8.0)) {
			val measuredDirection = rotateYaw(trueDirection, Math.toRadians(biasDegrees))
			val estimate = ArmYawConstraint.estimate(displacement, measuredDirection, upperLength, lowerLength)

			assertNotNull(estimate)
			assertEquals(Math.toRadians(biasDegrees), estimate.residualBiasRadians, 1e-6)
			assertEquals(true, estimate.sensitivityMetersPerRadian >= 0.05)
			assertEquals(true, estimate.alternativeSeparationRadians > Math.toRadians(10.0))
		}
	}

	@Test
	fun rejectsVerticalUnobservableYaw() {
		val result = ArmYawConstraint.estimate(
			Vector3(0.2f, 0f, 0f),
			Vector3(0f, 1f, 0f),
			0.3,
			0.3,
		)

		assertNull(result)
	}

	@Test
	fun rejectsUnreachableArmGeometry() {
		val result = ArmYawConstraint.estimate(
			Vector3(1f, 0f, 0f),
			Vector3(1f, 0f, 0f),
			0.3,
			0.3,
		)

		assertNull(result)
	}

	@Test
	fun rejectsTwoEquallySmallYawSolutions() {
		val displacement = Vector3(0.4f, 0f, 0f)
		val upperLength = 0.3
		val target = displacement.x.toDouble() * cos(Math.toRadians(20.0))
		val lowerLength = sqrt(
			displacement.x.toDouble() *
				displacement.x +
				upperLength *
				upperLength -
				2.0 *
				upperLength *
				target,
		)

		val result = ArmYawConstraint.estimate(
			displacement,
			Vector3(1f, 0f, 0f),
			upperLength,
			lowerLength,
		)

		assertNull(result)
	}

	@Test
	fun yawEstimateIsInvariantToGlobalYawRotation() {
		val (displacement, upperLength, lowerLength, trueDirection) = sampleGeometry()
		val biased = rotateYaw(trueDirection, Math.toRadians(7.0))
		val baseline = ArmYawConstraint.estimate(displacement, biased, upperLength, lowerLength)
		val worldYaw = Math.toRadians(113.0)
		val rotated = ArmYawConstraint.estimate(
			rotateYaw(displacement, worldYaw),
			rotateYaw(biased, worldYaw),
			upperLength,
			lowerLength,
		)

		assertNotNull(baseline)
		assertNotNull(rotated)
		assertEquals(baseline.residualBiasRadians, rotated.residualBiasRadians, 1e-6)
		assertEquals(baseline.sensitivityMetersPerRadian, rotated.sensitivityMetersPerRadian, 1e-6)
	}

	private fun sampleGeometry(): SampleGeometry {
		val upper = 0.3
		val displacement = Vector3(0.3f, 0.1f, 0.2f)
		val direction = Vector3((1.0 / sqrt(2.0)).toFloat(), (1.0 / sqrt(2.0)).toFloat(), 0f)
		val dot = displacement.x * direction.x + displacement.y * direction.y + displacement.z * direction.z
		val lengthSquared = displacement.x * displacement.x + displacement.y * displacement.y + displacement.z * displacement.z
		val lower = sqrt(lengthSquared + upper * upper - 2.0 * upper * dot)
		return SampleGeometry(displacement, upper, lower, direction)
	}

	private fun rotateYaw(vector: Vector3, radians: Double) = Vector3(
		(cos(radians) * vector.x + sin(radians) * vector.z).toFloat(),
		vector.y,
		(-sin(radians) * vector.x + cos(radians) * vector.z).toFloat(),
	)

	private data class SampleGeometry(
		val displacement: Vector3,
		val upperLength: Double,
		val lowerLength: Double,
		val trueDirection: Vector3,
	)
}
