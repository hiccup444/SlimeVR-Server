package dev.slimevr.tracking.processor.adaptive

import io.github.axisangles.ktmath.Quaternion
import io.github.axisangles.ktmath.Vector3
import org.junit.jupiter.api.Test
import kotlin.math.cos
import kotlin.math.sin
import kotlin.test.assertEquals
import kotlin.test.assertFalse
import kotlin.test.assertNotNull
import kotlin.test.assertNull
import kotlin.test.assertTrue

class PoseResidualMonitorTest {
	@Test
	fun computesNormalizedShortestArcResidualAndKeepsSolverConditioningExplicit() {
		val monitor = PoseResidualMonitor()
		val expected = axisAngle(Vector3(0f, 1f, 0f), 0.7)
		val measured = (expected * axisAngle(Vector3(1f, 0f, 0f), Math.toRadians(8.0))).unit()

		val residual = assertNotNull(monitor.observe(3, measured, expected, 1_000_000_000L))

		assertEquals(Math.toRadians(8.0), residual.magnitudeRadians, 1e-6)
		assertEquals(Math.toRadians(8.0), residual.residualVectorRadians.x.toDouble(), 1e-6)
		assertEquals(0.0, residual.residualVectorRadians.y.toDouble(), 1e-6)
		assertFalse(residual.independentlyConstrained)
		assertEquals(1, residual.sampleCount)
		val signFlipped = assertNotNull(monitor.observe(3, negate(measured), expected, 1_020_000_000L, true))
		assertEquals(Math.toRadians(8.0), signFlipped.magnitudeRadians, 1e-6)
		assertTrue(signFlipped.independentlyConstrained)
		assertEquals(0.02, signFlipped.continuousSeconds, 1e-9)
	}

	@Test
	fun rollingHistoryReportsDirectionalConsistencyAndStaysBounded() {
		val monitor = PoseResidualMonitor(historyLimit = 2, discontinuityRadians = Math.PI)
		val expected = Quaternion.IDENTITY
		val x = axisAngle(Vector3(1f, 0f, 0f), Math.toRadians(10.0))
		val minusX = axisAngle(Vector3(-1f, 0f, 0f), Math.toRadians(10.0))
		monitor.observe(1, x, expected, 0)
		val opposing = assertNotNull(monitor.observe(1, minusX, expected, 20_000_000L))
		assertTrue(opposing.directionalConsistency!! < 1e-6)
		assertEquals(2, opposing.sampleCount)

		val bounded = assertNotNull(monitor.observe(1, x, expected, 40_000_000L))
		assertEquals(3, bounded.sampleCount)
		assertTrue(bounded.directionalConsistency!! < 1e-6)
		val aligned = assertNotNull(monitor.observe(1, x, expected, 60_000_000L))
		assertEquals(4, aligned.sampleCount)
		assertTrue(aligned.directionalConsistency!! > 0.99)
	}

	@Test
	fun gapsDiscontinuitiesAndInvalidInputsStartNewRuns() {
		val monitor = PoseResidualMonitor(maximumGapNanos = 100_000_000L)
		val expected = Quaternion.IDENTITY
		val small = axisAngle(Vector3(0f, 1f, 0f), Math.toRadians(2.0))
		val large = axisAngle(Vector3(0f, 1f, 0f), Math.toRadians(60.0))
		monitor.observe(5, small, expected, 0)
		val discontinuity = assertNotNull(monitor.observe(5, large, expected, 20_000_000L))
		assertEquals("RESIDUAL_DISCONTINUITY", discontinuity.reason)
		assertEquals(0.0, discontinuity.continuousSeconds)
		assertEquals(1, discontinuity.sampleCount)

		val gap = assertNotNull(monitor.observe(5, large, expected, 200_000_001L))
		assertEquals("TIME_GAP", gap.reason)
		assertEquals(0.0, gap.continuousSeconds)
		assertEquals(1, gap.sampleCount)

		assertNull(monitor.observe(5, Quaternion(Float.NaN, 0f, 0f, 0f), expected, 220_000_000L))
		val afterInvalid = assertNotNull(monitor.observe(5, small, expected, 240_000_000L))
		assertEquals(1, afterInvalid.sampleCount)
		assertEquals(0.0, afterInvalid.continuousSeconds)
		assertNull(monitor.observe(5, small, expected, 230_000_000L))
	}

	@Test
	fun resetRetainAndCapacityBoundTrackerState() {
		val monitor = PoseResidualMonitor(maximumTrackers = 2)
		val q = axisAngle(Vector3(0f, 0f, 1f), 0.1)
		monitor.observe(1, q, Quaternion.IDENTITY, 1)
		monitor.observe(2, q, Quaternion.IDENTITY, 1)
		monitor.observe(3, q, Quaternion.IDENTITY, 1)
		val readded = assertNotNull(monitor.observe(1, q, Quaternion.IDENTITY, 2))
		assertEquals(1, readded.sampleCount)
		monitor.retain(setOf(1))
		val retained = assertNotNull(monitor.observe(1, q, Quaternion.IDENTITY, 3))
		assertEquals(2, retained.sampleCount)
		monitor.reset(1)
		assertEquals(1, assertNotNull(monitor.observe(1, q, Quaternion.IDENTITY, 4)).sampleCount)
		monitor.reset()
		assertEquals(1, assertNotNull(monitor.observe(1, q, Quaternion.IDENTITY, 5)).sampleCount)
	}

	private fun axisAngle(axis: Vector3, radians: Double): Quaternion {
		val half = radians / 2.0
		val s = sin(half).toFloat()
		return Quaternion(cos(half).toFloat(), axis.x * s, axis.y * s, axis.z * s)
	}

	private fun negate(q: Quaternion) = Quaternion(-q.w, -q.x, -q.y, -q.z)
}
