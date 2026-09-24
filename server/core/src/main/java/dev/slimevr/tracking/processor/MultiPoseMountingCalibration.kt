package dev.slimevr.tracking.processor

import com.fasterxml.jackson.databind.ObjectMapper
import dev.slimevr.tracking.trackers.Tracker
import dev.slimevr.tracking.trackers.TrackerStatus
import dev.slimevr.tracking.trackers.udp.TrackerDataType
import io.eiren.util.logging.LogManager
import io.github.axisangles.ktmath.Quaternion
import io.github.axisangles.ktmath.Quaternion.Companion.IDENTITY
import solarxr_protocol.datatypes.BodyPart
import kotlin.math.PI
import kotlin.math.sqrt

class MultiPoseMountingCalibration(private val poseManager: HumanPoseManager) {
	private data class PoseSample(val reference: Quaternion, val rotations: Map<Int, Quaternion>)
	private data class Result(val tracker: Tracker, val candidate: Quaternion?, val repeatErrorDeg: Float, val movementDeg: Float, val reason: String)
	private val mapper = ObjectMapper()
	private val samples = mutableMapOf<Int, PoseSample>()
	private val pending = mutableMapOf<Int, MutableList<Quaternion>>()
	private val lastArrival = mutableMapOf<Int, Long>()
	private val previous = mutableMapOf<Int, Quaternion>()
	private var trackerRoles = emptyMap<Int, Int?>()
	private var captureStep = -1
	private var captureStarted = 0L
	private var captureDeadline = 0L
	private var phase = "idle"
	private var error = ""
	private var results = emptyList<Result>()
	private var applied = false
	private var sessionNumber = 0
	private val trackedRoles = setOf(
		BodyPart.CHEST, BodyPart.UPPER_CHEST, BodyPart.WAIST, BodyPart.HIP,
		BodyPart.LEFT_UPPER_ARM, BodyPart.RIGHT_UPPER_ARM,
		BodyPart.LEFT_UPPER_LEG, BodyPart.RIGHT_UPPER_LEG,
		BodyPart.LEFT_LOWER_LEG, BodyPart.RIGHT_LOWER_LEG,
		BodyPart.LEFT_FOOT, BodyPart.RIGHT_FOOT,
	)

	private fun trackers() = poseManager.server!!.allTrackers.filter {
		it.allowMounting && it.hasRotation && it.trackerDataType == TrackerDataType.ROTATION && it.trackerPosition?.bodyPart in trackedRoles
	}

	fun invalidateOnReset() {
		if (phase == "idle") return
		LogManager.info("[MultiPoseMounting] Session $sessionNumber invalidated by a tracker reset")
		phase = "invalid"
		captureStep = -1
		applied = false
		previous.clear()
		error = "A tracker reset occurred. Start the pose checks again."
	}

	fun command(command: String): String {
		when {
			command == "start" -> {
				if (applied) undo()
				previous.clear()
				trackerRoles = trackers().associate { it.id to it.trackerPosition?.bodyPart }
				sessionNumber++
				samples.clear()
				pending.clear()
				results = emptyList()
				captureStep = -1
				applied = false
				error = ""
				phase = "ready"
				LogManager.info("[MultiPoseMounting] Session $sessionNumber started")
			}
			command.startsWith("capture:") -> {
				val step = command.substringAfter(':').toIntOrNull()
				if (step == null || step !in 0..3 || step != samples.size || phase !in setOf("ready", "captured")) {
					error = "Complete the poses in order."
				} else if (trackers().isEmpty()) {
					error = "No active assigned mounting trackers were found."
				} else if (poseManager.skeleton.headTracker?.status?.sendData != true) {
					error = "A tracked headset is required for the mounting reference."
				} else if (trackers().associate { it.id to it.trackerPosition?.bodyPart } != trackerRoles) {
					error = "Tracker assignments changed. Restart calibration."
				} else {
					captureStep = step
					captureStarted = System.nanoTime()
					captureDeadline = captureStarted + 10_000_000_000L
					pending.clear()
					lastArrival.clear()
					error = ""
					phase = "capturing"
					LogManager.info("[MultiPoseMounting] Session $sessionNumber capturing pose $step")
				}
			}
			command == "apply" -> apply()
			command == "undo" -> undo()
			command == "cancel" -> {
				if (!applied) {
					phase = "idle"
					captureStep = -1
				}
			}
		}
		return stateJson()
	}

	fun update(now: Long) {
		if (phase != "capturing") return
		for (tracker in trackers()) {
			val arrival = tracker.lastRotationUpdateNanos ?: continue
			if (tracker.status != TrackerStatus.OK || now - arrival > 500_000_000L || lastArrival[tracker.id] == arrival) continue
			lastArrival[tracker.id] = arrival
			val rotation = tracker.getRawRotation()
			val readings = pending.getOrPut(tracker.id) { mutableListOf() }
			if (readings.isNotEmpty() && angleDeg(readings.first(), rotation) > 6f) {
				pending.values.forEach { it.clear() }
				captureStarted = now
			}
			readings.add(rotation)
		}
		if (now - captureStarted < 1_500_000_000L) {
			if (now > captureDeadline) {
				phase = "captured"
				error = "Could not find a stable pose. Hold still and retry."
				LogManager.info("[MultiPoseMounting] Session $sessionNumber pose $captureStep failed: $error")
			}
			return
		}
		val current = trackers()
		val usable = current.filter { tracker ->
			val rotations = pending[tracker.id].orEmpty()
			rotations.size >= 12 && rotations.all { angleDeg(rotations.first(), it) <= 6f }
		}
		if (usable.size < maxOf(1, (current.size * 3 + 3) / 4)) {
			if (now > captureDeadline) {
				phase = "captured"
				error = "Too few stable fresh tracker readings (${usable.size}/${current.size}). Hold the pose and retry."
				LogManager.info("[MultiPoseMounting] Session $sessionNumber pose $captureStep failed: $error")
			}
			return
		}
		val head = poseManager.skeleton.headTracker
		if (head?.status?.sendData != true) {
			phase = "captured"
			error = "Headset tracking was lost. Retry this pose."
			return
		}
		val reference = head.getRotation()
		samples[captureStep] = PoseSample(reference, usable.associate { it.id to mean(pending[it.id].orEmpty()) })
		LogManager.info("[MultiPoseMounting] Session $sessionNumber pose $captureStep captured: " + mapper.writeValueAsString(mapOf(
			"reference" to listOf(reference.w, reference.x, reference.y, reference.z),
			"trackers" to usable.associate { tracker ->
				tracker.name to mapOf(
					"role" to tracker.trackerPosition?.name,
					"samples" to pending[tracker.id]?.size,
					"rotation" to samples[captureStep]?.rotations?.get(tracker.id)?.let { listOf(it.w, it.x, it.y, it.z) },
				)
			},
		)))
		phase = "captured"
		error = ""
		if (captureStep == 3) review()
	}

	private fun review() {
		val a = samples[0] ?: return
		val upright = samples[1] ?: return
		val bend = samples[2] ?: return
		val repeated = samples[3] ?: return
		val initial = trackers().map { tracker ->
			val first = a.rotations[tracker.id]
			val second = upright.rotations[tracker.id]
			val third = bend.rotations[tracker.id]
			val last = repeated.rotations[tracker.id]
			if (first == null || second == null || third == null || last == null) {
				Result(tracker, null, 0f, 0f, "Missing or unstable readings in one pose")
			} else {
				val firstCandidate = tracker.resetsHandler.calculateMountingCandidate(a.reference, first)
				val lastCandidate = tracker.resetsHandler.calculateMountingCandidate(repeated.reference, last)
				val repeatError = angleDeg(firstCandidate, lastCandidate)
				val movement = maxOf(angleDeg(first, second), angleDeg(second, third))
				val isFoot = tracker.trackerPosition?.bodyPart in setOf(BodyPart.LEFT_FOOT, BodyPart.RIGHT_FOOT)
				val reason = when {
					repeatError > 10f -> "Ski pose did not repeat closely; check the strap and retry"
					!isFoot && movement < 8f -> "Too little movement to cross-check this tracker"
					isFoot && movement > 20f -> "Foot moved too much during the flat-foot poses"
					isFoot -> "Repeatable ski estimate; foot heading remains pose-dependent"
					else -> "Repeatable ski estimate with contrasting pose movement"
				}
				val accepted = repeatError <= 10f && (isFoot && movement <= 20f || !isFoot && movement >= 8f)
				Result(tracker, if (accepted) mean(listOf(firstCandidate, lastCandidate)) else null, repeatError, movement, reason)
			}
		}
		results = initial.toMutableList().also { checked ->
			for ((thighRole, shinRole) in listOf(
				BodyPart.LEFT_UPPER_LEG to BodyPart.LEFT_LOWER_LEG,
				BodyPart.RIGHT_UPPER_LEG to BodyPart.RIGHT_LOWER_LEG,
			)) {
				val thigh = checked.firstOrNull { it.tracker.trackerPosition?.bodyPart == thighRole } ?: continue
				val shin = checked.firstOrNull { it.tracker.trackerPosition?.bodyPart == shinRole } ?: continue
				val offAxis = kneeOffAxis(upright, bend, thigh, shin) ?: continue
				if (offAxis > 0.7f) {
					val reason = "Knee motion disagrees with the expected flexion axis; keep existing mounting"
					checked[checked.indexOf(thigh)] = thigh.copy(candidate = null, reason = reason)
					checked[checked.indexOf(shin)] = shin.copy(candidate = null, reason = reason)
				}
			}
		}
		phase = "review"
		LogManager.info("[MultiPoseMounting] Session $sessionNumber review: " + mapper.writeValueAsString(results.map {
			mapOf("tracker" to it.tracker.name, "role" to it.tracker.trackerPosition?.name, "accepted" to (it.candidate != null), "repeatErrorDeg" to it.repeatErrorDeg, "movementDeg" to it.movementDeg, "reason" to it.reason)
		}))
	}

	private fun kneeOffAxis(upright: PoseSample, bend: PoseSample, thigh: Result, shin: Result): Float? {
		val thighCandidate = thigh.candidate ?: return null
		val shinCandidate = shin.candidate ?: return null
		fun jointRotation(pose: PoseSample): Quaternion? {
			val thighRaw = pose.rotations[thigh.tracker.id] ?: return null
			val shinRaw = pose.rotations[shin.tracker.id] ?: return null
			val thighRotation = thigh.tracker.resetsHandler.previewReferenceRotation(thighRaw, thighCandidate)
			val shinRotation = shin.tracker.resetsHandler.previewReferenceRotation(shinRaw, shinCandidate)
			return thighRotation.inv() * shinRotation
		}
		val first = jointRotation(upright) ?: return null
		val second = jointRotation(bend) ?: return null
		if (angleDeg(first, second) < 12f) return null
		val delta = (first.inv() * second).unit()
		val total = delta.xyz.len()
		return if (total < 0.01f) null else sqrt(delta.y * delta.y + delta.z * delta.z) / total
	}

	private fun apply() {
		if (phase != "review" || results.none { it.candidate != null }) {
			error = "No reviewed mounting changes are ready to apply."
			return
		}
		if (trackers().associate { it.id to it.trackerPosition?.bodyPart } != trackerRoles) {
			error = "Tracker assignments changed. Restart calibration."
			return
		}
		val head = poseManager.skeleton.headTracker
		if (head?.status?.sendData != true) {
			error = "Headset tracking was lost. Restore it before applying."
			return
		}
		val reference = head.getRotation()
		previous.clear()
		for (result in results) {
			val candidate = result.candidate ?: continue
			previous[result.tracker.id] = result.tracker.resetsHandler.mountRotFix
			result.tracker.resetsHandler.applyMountingCandidate(candidate, reference)
		}
		poseManager.adaptiveEstimator.reset()
		poseManager.adaptivePoseSolver.reset()
		poseManager.adaptiveMeasurementQuality.reset()
		poseManager.adaptiveArmCalibration.reset()
		poseManager.adaptiveTelemetry.reset()
		poseManager.skeleton.legTweaks.resetBuffer()
		poseManager.server?.configManager?.saveConfig()
		applied = true
		phase = "applied"
		error = ""
		LogManager.info("[MultiPoseMounting] Session $sessionNumber applied ${previous.size} tracker mounting estimates")
	}

	private fun undo() {
		if (!applied) return
		val reference = poseManager.skeleton.headTracker?.getRotation() ?: IDENTITY
		for (tracker in trackers()) {
			previous[tracker.id]?.let { tracker.resetsHandler.applyMountingCandidate(it, reference) }
		}
		poseManager.adaptiveEstimator.reset()
		poseManager.adaptivePoseSolver.reset()
		poseManager.adaptiveMeasurementQuality.reset()
		poseManager.adaptiveArmCalibration.reset()
		poseManager.adaptiveTelemetry.reset()
		poseManager.skeleton.legTweaks.resetBuffer()
		poseManager.server?.configManager?.saveConfig()
		applied = false
		phase = "undone"
		LogManager.info("[MultiPoseMounting] Session $sessionNumber restored prior tracker mounting estimates")
	}

	private fun angleDeg(a: Quaternion, b: Quaternion) = (a.angleToR(b) * 180f / PI.toFloat())

	private fun mean(values: List<Quaternion>): Quaternion {
		val first = values.first()
		val aligned = values.map { if (first.dot(it) < 0f) -it else it }
		return Quaternion(
			aligned.map { it.w }.average().toFloat(),
			aligned.map { it.x }.average().toFloat(),
			aligned.map { it.y }.average().toFloat(),
			aligned.map { it.z }.average().toFloat(),
		).unit()
	}

	fun stateJson(): String = mapper.writeValueAsString(
		mapOf(
			"phase" to phase,
			"step" to captureStep,
			"completed" to samples.keys.sorted(),
			"error" to error,
			"applied" to applied,
			"stableTrackers" to if (phase == "capturing") pending.count { (_, readings) -> readings.size >= 12 && readings.all { angleDeg(readings.first(), it) <= 6f } } else 0,
			"totalTrackers" to trackers().size,
			"trackers" to results.map {
				mapOf(
					"id" to it.tracker.id,
					"name" to it.tracker.displayName,
					"role" to it.tracker.trackerPosition?.name,
					"eligible" to (it.candidate != null),
					"yawChangeDeg" to it.candidate?.let { candidate -> angleDeg(candidate, it.tracker.resetsHandler.mountRotFix) },
					"repeatErrorDeg" to it.repeatErrorDeg,
					"movementDeg" to it.movementDeg,
					"reason" to it.reason,
				)
			},
		),
	)
}
