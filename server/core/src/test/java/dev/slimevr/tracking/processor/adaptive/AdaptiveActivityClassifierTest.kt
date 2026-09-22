package dev.slimevr.tracking.processor.adaptive

import org.junit.jupiter.api.Test
import kotlin.test.assertEquals
import kotlin.test.assertTrue

class AdaptiveActivityClassifierTest {
	private val down = ActivityVector3(0.0, -1.0, 0.0)
	private val flat = ActivityVector3(1.0, 0.0, 0.0)
	private val bentThigh = ActivityVector3(0.0, -0.8, 0.6)
	private val bentShin = ActivityVector3(0.0, -0.8, -0.6)
	private val still = ActivityVector3(0.0, 0.0, 0.0)

	private fun input(
		time: Long,
		headHeight: Double = 1.0,
		torsoUp: Double = 1.0,
		leftThigh: ActivityVector3 = down,
		leftShin: ActivityVector3 = down,
		rightThigh: ActivityVector3 = down,
		rightShin: ActivityVector3 = down,
		headVelocity: ActivityVector3 = still,
		leftFootVelocity: ActivityVector3 = still,
		rightFootVelocity: ActivityVector3 = still,
		leftContact: Double? = 1.0,
		rightContact: Double? = 1.0,
		headTime: Long = time,
	): AdaptiveActivityObservation = AdaptiveActivityObservation(
		nowNanos = time,
		normalizedHeadHeight = headHeight,
		headTimeNanos = headTime,
		torsoUpY = torsoUp,
		torsoTimeNanos = time,
		leftThighDirection = leftThigh,
		leftShinDirection = leftShin,
		rightThighDirection = rightThigh,
		rightShinDirection = rightShin,
		legsTimeNanos = time,
		headVelocityMetersPerSecond = headVelocity,
		leftFootVelocityMetersPerSecond = leftFootVelocity,
		rightFootVelocityMetersPerSecond = rightFootVelocity,
		motionTimeNanos = time,
		leftFootContact = leftContact,
		rightFootContact = rightContact,
		contactTimeNanos = if (leftContact == null && rightContact == null) null else time,
	)

	private fun feed(classifier: AdaptiveActivityClassifier, from: Int, through: Int, makeInput: (Long) -> AdaptiveActivityObservation): AdaptiveActivityEstimate {
		var estimate = classifier.estimate
		for (index in from..through) estimate = classifier.update(makeInput(index * 100_000_000L))
		return estimate
	}

	@Test
	fun standingRequiresDwellAndGaitTransitionsHaveHysteresis() {
		val classifier = AdaptiveActivityClassifier()
		var result = feed(classifier, 0, 5) { input(it) }
		assertEquals(AdaptiveActivityState.STANDING, result.state)

		val walking: (Long) -> AdaptiveActivityObservation = { time ->
			input(time, leftFootVelocity = ActivityVector3(0.0, 0.0, 0.8), leftContact = 0.1)
		}
		result = feed(classifier, 6, 15, walking)
		assertEquals(AdaptiveActivityState.STANDING, result.state)
		result = feed(classifier, 16, 20, walking)
		assertEquals(AdaptiveActivityState.WALKING, result.state)

		val running: (Long) -> AdaptiveActivityObservation = { time ->
			input(time, headVelocity = ActivityVector3(0.9, 0.0, 0.0), leftFootVelocity = ActivityVector3(0.0, 0.0, 2.4), rightFootVelocity = ActivityVector3(0.0, 0.0, 0.7), leftContact = 0.1, rightContact = 0.1)
		}
		result = feed(classifier, 21, 34, running)
		assertEquals(AdaptiveActivityState.RUNNING, result.state)
		assertTrue(result.confidence in 0.0..1.0)
	}

	@Test
	fun conservativeGeometrySeparatesCrouchingSittingAndLying() {
		val crouching = AdaptiveActivityClassifier()
		var result = feed(crouching, 0, 12) { time ->
			input(time, headHeight = 0.68, leftThigh = bentThigh, leftShin = bentShin, rightThigh = bentThigh, rightShin = bentShin)
		}
		assertEquals(AdaptiveActivityState.CROUCHING, result.state)
		val crouchWithoutContacts = AdaptiveActivityClassifier()
		result = feed(crouchWithoutContacts, 0, 12) { time ->
			input(time, headHeight = 0.68, leftThigh = bentThigh, leftShin = bentShin, rightThigh = bentThigh, rightShin = bentShin, leftContact = null, rightContact = null)
		}
		assertEquals(AdaptiveActivityState.CROUCHING, result.state)

		val sitting = AdaptiveActivityClassifier()
		result = feed(sitting, 0, 12) { time ->
			input(time, headHeight = 0.72, leftThigh = flat, leftShin = down, rightThigh = flat, rightShin = down)
		}
		assertEquals(AdaptiveActivityState.SITTING, result.state)
		val sittingWithoutContacts = AdaptiveActivityClassifier()
		result = feed(sittingWithoutContacts, 0, 12) { time ->
			input(time, headHeight = 0.72, leftThigh = flat, leftShin = down, rightThigh = flat, rightShin = down, leftContact = null, rightContact = null)
		}
		assertEquals(AdaptiveActivityState.SITTING, result.state)

		val lying = AdaptiveActivityClassifier()
		result = feed(lying, 0, 12) { time ->
			input(time, headHeight = 0.55, torsoUp = 0.1, leftThigh = flat, leftShin = flat, rightThigh = flat, rightShin = flat, leftContact = null, rightContact = null)
		}
		assertEquals(AdaptiveActivityState.LYING, result.state)
	}

	@Test
	fun lowHeadHeightWithoutDistinguishingGeometryRemainsUnknown() {
		val classifier = AdaptiveActivityClassifier()
		val result = feed(classifier, 0, 20) { time -> input(time, headHeight = 0.62) }
		assertEquals(AdaptiveActivityState.UNKNOWN, result.state)
		assertEquals("AMBIGUOUS_POSTURE_OR_ACTIVITY", result.reason)
	}

	@Test
	fun missingStaleInvalidAndDiscontinuousEvidenceReturnsUnknownImmediately() {
		val classifier = AdaptiveActivityClassifier()
		feed(classifier, 0, 5) { input(it) }
		var result = classifier.update(input(600_000_000L, headTime = 200_000_000L))
		assertEquals(AdaptiveActivityState.UNKNOWN, result.state)
		assertEquals("MISSING_OR_STALE_EVIDENCE", result.reason)

		result = classifier.update(input(700_000_000L, torsoUp = 1.1))
		assertEquals("INVALID_TORSO_DIRECTION", result.reason)
		result = classifier.update(input(800_000_000L, leftContact = 1.1))
		assertEquals("INVALID_CONTACT_CONFIDENCE", result.reason)
		classifier.update(input(900_000_000L))
		result = classifier.update(input(1_500_000_000L))
		assertEquals("UPDATE_GAP", result.reason)
	}

	@Test
	fun singleFootContactOrAmbiguousBendCannotClaimStanding() {
		val classifier = AdaptiveActivityClassifier()
		val oneFoot = feed(classifier, 0, 12) { time -> input(time, rightContact = 0.0) }
		assertEquals(AdaptiveActivityState.UNKNOWN, oneFoot.state)

		val bent = AdaptiveActivityClassifier()
		val result = feed(bent, 0, 12) { time ->
			input(time, leftThigh = bentThigh, leftShin = bentShin, rightThigh = down, rightShin = down)
		}
		assertEquals(AdaptiveActivityState.UNKNOWN, result.state)
	}

	@Test
	fun standingRequiresLowHeadSpeedAndSeatedFootKicksAreNotWalking() {
		val movingHead = AdaptiveActivityClassifier()
		var result = feed(movingHead, 0, 10) { time ->
			input(time, headVelocity = ActivityVector3(0.2, 0.0, 0.0))
		}
		assertEquals(AdaptiveActivityState.UNKNOWN, result.state)

		val seatedKick = AdaptiveActivityClassifier()
		result = feed(seatedKick, 0, 10) { time ->
			input(time, headHeight = 0.72, leftThigh = flat, leftShin = down, rightThigh = flat, rightShin = down, leftFootVelocity = ActivityVector3(0.0, 0.0, 0.8), leftContact = 0.1)
		}
		assertEquals(AdaptiveActivityState.UNKNOWN, result.state)
	}

	@Test
	fun resetClearsPublishedStateAndCandidateDwell() {
		val classifier = AdaptiveActivityClassifier()
		feed(classifier, 0, 5) { input(it) }
		classifier.reset()
		assertEquals(AdaptiveActivityState.UNKNOWN, classifier.estimate.state)
		assertEquals("RESET", classifier.estimate.reason)
		assertEquals(AdaptiveActivityState.UNKNOWN, classifier.update(input(1_000_000_000L)).state)
	}
}
