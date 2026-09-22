package dev.slimevr.tracking.processor.adaptive

import dev.slimevr.config.AdaptiveTrackingConfig
import dev.slimevr.tracking.processor.Bone
import dev.slimevr.tracking.processor.skeleton.HumanSkeleton
import io.github.axisangles.ktmath.Quaternion
import io.github.axisangles.ktmath.Vector3

data class PoseSolverDiagnostic(val initialError: Float, val finalError: Float, val processingNanos: Long, val measurementConfidence: Float)

data class TrackerPosePrediction(val expectedRotation: Quaternion, val measuredRotation: Quaternion, val residual: PoseResidualDiagnostic)

/** Adapts the existing multi-root skeleton to a single weighted pose objective. */
class AdaptivePoseSolver(private val config: AdaptiveTrackingConfig) {
	private val optimizer = PoseOptimizer()
	private val sensors = SensorStateManager()
	private val confidence = TrackerConfidenceEstimator()
	private val bodyEvidence = BodyEvidenceEstimator()
	private val recovery = mutableMapOf<Bone, OrientationRecovery>()
	private val previous = mutableMapOf<Bone, Quaternion>()
	private val residualMonitor = PoseResidualMonitor()
	private val activityMonitor = AdaptiveActivityMonitor()
	var activity: AdaptiveActivityEstimate? = null
		private set
	var trackerPredictions: Map<Int, TrackerPosePrediction> = emptyMap()
		private set
	private var lastTime: Long? = null
	var replayInput: PoseSolverInput? = null
		private set
	var diagnostic: PoseSolverDiagnostic? = null
		private set
	var rawPose: Map<String, Vector3> = emptyMap()
		private set
	var rawRotations: Map<String, Quaternion> = emptyMap()
		private set
	var predictedPose: Map<String, Vector3> = emptyMap()
		private set

	fun reset() {
		previous.clear()
		residualMonitor.reset()
		activityMonitor.reset()
		activity = null
		trackerPredictions = emptyMap()
		lastTime = null
		sensors.reset()
		confidence.reset()
		bodyEvidence.reset()
		recovery.clear()
		diagnostic = null
		replayInput = null
		rawPose = emptyMap()
		rawRotations = emptyMap()
		predictedPose = emptyMap()
	}

	fun update(s: HumanSkeleton, now: Long) {
		if (!config.poseOptimizerEnabled || s.getPauseTracking() || s.localizer.getEnabled()) {
			if (lastTime != null) reset()
			predictedPose = emptyMap()
			rawRotations = emptyMap()
			rawPose = if (config.telemetryEnabled || config.liveDiagnosticsEnabled) {
				s.allHumanBones.associate { it.boneType.name to it.getTailPosition() }
			} else {
				emptyMap()
			}
			if (!s.getPauseTracking() && !s.localizer.getEnabled() && (config.telemetryEnabled || config.liveDiagnosticsEnabled)) {
				activity = activityMonitor.update(s, now)
				capturePredictions(s.allHumanBones.toList(), now)
			} else {
				activityMonitor.reset()
				activity = null
				residualMonitor.reset()
				trackerPredictions = emptyMap()
			}
			return
		}
		val started = System.nanoTime()
		activity = activityMonitor.update(s, now)
		val selected = setOf(
			s.headBone, s.neckBone, s.upperChestBone, s.chestBone, s.waistBone, s.hipBone,
			s.leftHipBone, s.rightHipBone, s.leftUpperLegBone, s.rightUpperLegBone, s.leftLowerLegBone, s.rightLowerLegBone,
			s.leftFootBone, s.rightFootBone, s.leftUpperShoulderBone, s.rightUpperShoulderBone, s.leftShoulderBone, s.rightShoulderBone,
			s.leftUpperArmBone, s.rightUpperArmBone, s.leftLowerArmBone, s.rightLowerArmBone, s.leftHandBone, s.rightHandBone,
			s.leftHandTrackerBone, s.rightHandTrackerBone, s.leftFootTrackerBone, s.rightFootTrackerBone,
		)
		val bones = mutableListOf<Bone>()
		fun visit(bone: Bone) {
			if (bone !in selected) return
			bones.add(bone)
			bone.children.forEach { visit(it) }
		}
		selected.filter { it.parent == null }.forEach { visit(it) }
		val indices = bones.withIndex().associate { it.value to it.index }
		val trackers = bones.mapNotNull { it.attachedTracker }.distinctBy { it.id }
		val chains = listOf(
			listOf(s.hipTracker, s.leftUpperLegTracker, s.leftLowerLegTracker, s.leftFootTracker),
			listOf(s.hipTracker, s.rightUpperLegTracker, s.rightLowerLegTracker, s.rightFootTracker),
			listOf(s.chestTracker, s.leftUpperArmTracker, s.leftLowerArmTracker, s.leftHandTracker),
			listOf(s.chestTracker, s.rightUpperArmTracker, s.rightLowerArmTracker, s.rightHandTracker),
		).map { chain -> chain.mapNotNull { it?.id } }
		val frame = bodyEvidence.observe(confidence.observe(sensors.sample(trackers, emptyList(), now)), chains)
		val samples = frame.samples.associateBy { it.id }
		val continuous = lastTime?.let { now - it in 1..250_000_000L } == true
		val highMotion = bodyEvidence.motion == AdaptiveMotionState.HIGH_MOTION || activity?.state == AdaptiveActivityState.HIGH_MOTION || activity?.state == AdaptiveActivityState.RUNNING
		val followers = mapOf(
			s.leftFootTrackerBone to s.leftFootBone,
			s.rightFootTrackerBone to s.rightFootBone,
			s.leftHipBone to s.hipBone,
			s.rightHipBone to s.hipBone,
			s.leftUpperShoulderBone to s.upperChestBone,
			s.rightUpperShoulderBone to s.upperChestBone,
		)
		val segments = bones.map { bone ->
			val tracker = bone.attachedTracker
			val sample = samples[tracker?.id]
			val measured = if (tracker != null && !tracker.hasPosition) recovery.getOrPut(bone) { OrientationRecovery() }.update(bone.getGlobalRotation(), (sample?.confidence?.score ?: 0f) > 0f, now) else bone.getGlobalRotation()
			val speed = sample?.angularSpeedRadiansPerSecond
			val temporal = if (continuous && !highMotion && (speed == null || speed < 0.5f)) 0.15f else 0f
			PoseSegment(
				indices[bone.parent] ?: -1,
				bone.length,
				bone.getPosition(),
				measured,
				sample?.confidence?.score ?: 0.35f,
				bone.parent == null || bone === s.neckBone || (tracker?.hasPosition == true && !tracker.isImu()) || bone.boneType.name.endsWith("TRACKER"),
				if (continuous) previous[bone] else null,
				temporal,
				indices[followers[bone]] ?: -1,
				followers[bone]?.let { it.rotationOffset.inv() * bone.rotationOffset } ?: Quaternion.IDENTITY,
			)
		}
		val anchors = mutableListOf<PoseAnchor>()
		fun elbow(upper: Bone, lower: Bone, output: Bone, reversed: Boolean, hand: dev.slimevr.tracking.trackers.Tracker?) {
			if (hand == null || !hand.hasPosition || !hand.status.sendData) return
			val age = hand.lastRotationUpdateNanos?.let { now - it }
			if ((age == null && hand.usesTimeout) || (age != null && age !in 0..250_000_000L)) return
			if (reversed) {
				anchors.add(PoseAnchor(indices.getValue(upper), Vector3.NULL, 40f, indices.getValue(lower)))
			} else {
				anchors.add(PoseAnchor(indices.getValue(output), hand.position, 80f))
			}
		}
		elbow(s.leftUpperArmBone, s.leftLowerArmBone, s.leftHandTrackerBone, s.isTrackingLeftArmFromController, s.leftHandTracker)
		elbow(s.rightUpperArmBone, s.rightLowerArmBone, s.rightHandTrackerBone, s.isTrackingRightArmFromController, s.rightHandTracker)
		fun foot(bone: Bone, contact: FootContactSnapshot, strength: Float) {
			val plant = contact.plantPosition ?: return
			if (contact.state != FootContactState.PLANTED) return
			anchors.add(PoseAnchor(indices.getValue(bone), Vector3(plant.x, bone.getTailPosition().y, plant.z), 30f * contact.weight * strength))
		}
		if (s.legTweaks.adaptiveAnchoringEligible) {
			val strength = config.footAnchorStrength.takeIf { it.isFinite() }?.coerceIn(0f, 1f) ?: 0f
			if (strength > 0f) {
				foot(s.leftFootTrackerBone, s.legTweaks.adaptiveLeftFoot.snapshot, strength)
				foot(s.rightFootTrackerBone, s.legTweaks.adaptiveRightFoot.snapshot, strength)
			}
		}
		val joints = (
			listOf(s.leftUpperLegBone to s.leftLowerLegBone, s.rightUpperLegBone to s.rightLowerLegBone).map {
				SoftJoint(indices.getValue(it.first), indices.getValue(it.second), 0.08f, true)
			} +
				listOf(
					SoftJoint(indices.getValue(s.leftUpperArmBone), indices.getValue(s.leftLowerArmBone), 0.02f, true, s.leftUpperArmBone.rotationOffset.inv(), s.leftLowerArmBone.rotationOffset.inv()),
					SoftJoint(indices.getValue(s.rightUpperArmBone), indices.getValue(s.rightLowerArmBone), 0.02f, true, s.rightUpperArmBone.rotationOffset.inv(), s.rightLowerArmBone.rotationOffset.inv()),
				)
			).toMutableList()
		// Reuse the skeleton's joint ranges as soft costs, with no cost inside each range.
		listOf(
			s.upperChestBone, s.chestBone, s.waistBone, s.hipBone,
			s.leftUpperLegBone, s.rightUpperLegBone, s.leftFootBone, s.rightFootBone,
			s.leftShoulderBone, s.rightShoulderBone, s.leftUpperArmBone, s.rightUpperArmBone,
		).forEach { bone ->
			val parent = bone.parent ?: return@forEach
			val first = indices[parent] ?: return@forEach
			val second = indices[bone] ?: return@forEach
			joints.add(SoftJoint(first, second, 0.04f, firstOffset = parent.rotationOffset.inv(), secondOffset = bone.rotationOffset.inv(), maxSwingRadians = bone.rotationConstraint.swingLimitRadians, maxTwistRadians = bone.rotationConstraint.twistLimitRadians))
		}
		rawPose = bones.associate { it.boneType.name to it.getTailPosition() }
		rawRotations = bones.associate { it.boneType.name to it.getGlobalRotation() }
		replayInput = PoseSolverInput(segments, anchors, joints)
		val result = try {
			optimizer.solve(segments, anchors, joints)
		} catch (_: IllegalArgumentException) {
			reset()
			return
		}
		bones.forEachIndexed { i, bone ->
			bone.setRotationRaw(result.rotations[i])
			previous[bone] = result.rotations[i]
		}
		// Output tracker offsets follow the optimized segment, preserving configured mounting geometry.
		listOf(
			s.leftKneeTrackerBone to s.leftUpperLegBone,
			s.rightKneeTrackerBone to s.rightUpperLegBone,
			s.leftFootTrackerBone to s.leftFootBone,
			s.rightFootTrackerBone to s.rightFootBone,
			s.chestTrackerBone to s.upperChestBone,
			s.hipTrackerBone to s.hipBone,
		).forEach { (output, bone) ->
			output.setRotation(bone.getLocalRotation() * bone.rotationOffset.inv())
		}
		s.updateBones()
		predictedPose = bones.associate { it.boneType.name to it.getTailPosition() }
		capturePredictions(bones, now)
		lastTime = now
		val global = frame.samples.mapNotNull { it.confidence?.score }.average().takeIf { it.isFinite() }?.toFloat() ?: 0f
		diagnostic = PoseSolverDiagnostic(result.initialError, result.finalError, System.nanoTime() - started, global)
	}

	private fun capturePredictions(bones: List<Bone>, now: Long) {
		trackerPredictions = bones.filter { it.attachedTracker?.isImu() == true }.distinctBy { it.attachedTracker!!.id }.mapNotNull { bone ->
			val tracker = bone.attachedTracker!!
			val age = tracker.lastRotationUpdateNanos?.let { now - it }
			if (!tracker.status.sendData || (age == null && tracker.usesTimeout) || (age != null && age !in 0..250_000_000L)) {
				residualMonitor.reset(tracker.id)
				return@mapNotNull null
			}
			val measured = tracker.getRotation()
			val expected = bone.getGlobalRotation() * bone.rotationOffset.inv()
			val residual = residualMonitor.observe(tracker.id, measured, expected, now) ?: return@mapNotNull null
			tracker.id to TrackerPosePrediction(expected, measured, residual)
		}.toMap()
		residualMonitor.retain(trackerPredictions.keys)
	}
}
