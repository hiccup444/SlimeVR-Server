package dev.slimevr.tracking.processor.adaptive

import dev.slimevr.tracking.trackers.TrackerRole
import kotlin.math.exp

/** Independent measurements used to summarize pose trust for one calibration context.
 * Residuals must come from evidence independent of the candidate calibration target. Solver
 * objective error and geometry priors are diagnostic or pose-shaping inputs, never evidence here.
 */
data class GlobalPoseConfidenceInput(
	val trackerConfidences: Map<TrackerRole, Float>,
	val requiredRoles: Set<TrackerRole>,
	val absoluteAnchorAvailable: Boolean,
	val absoluteAnchorQuality: Float,
	val independentChainDisagreementsRadians: List<Double>,
	/** Residual magnitudes from independent anchors or chains, excluding the tracker being calibrated. */
	val independentResidualsRadians: List<Double>,
	val contactReliability: Float? = null,
	val contactRequiredForLearning: Boolean = false,
	val motionUncertainty: Float,
	val minimumIndependentChains: Int = 2,
	val minimumIndependentResiduals: Int = 2,
)

data class GlobalPoseConfidence(
	val score: Float,
	val reasons: List<String>,
	val learningEligible: Boolean,
)

/** Combines measured confidence factors without allowing missing evidence to disappear in an average. */
class GlobalPoseConfidenceEstimator(
	private val minimumLearningScore: Float = 0.75f,
	private val minimumAnchorQuality: Float = 0.85f,
	private val maximumChainDisagreementRadians: Double = Math.toRadians(5.0),
	private val maximumIndependentResidualRadians: Double = Math.toRadians(25.0),
	private val maximumMotionUncertainty: Float = 0.2f,
) {
	init {
		require(minimumLearningScore.isFinite() && minimumLearningScore in 0f..1f)
		require(minimumAnchorQuality.isFinite() && minimumAnchorQuality in 0f..1f)
		require(maximumChainDisagreementRadians.isFinite() && maximumChainDisagreementRadians > 0.0)
		require(maximumIndependentResidualRadians.isFinite() && maximumIndependentResidualRadians > 0.0)
		require(maximumMotionUncertainty.isFinite() && maximumMotionUncertainty in 0f..1f)
	}

	fun estimate(input: GlobalPoseConfidenceInput): GlobalPoseConfidence {
		val reasons = linkedSetOf<String>()
		var eligible = true
		val required = input.requiredRoles
		val roleScores = required.mapNotNull { role ->
			input.trackerConfidences[role]?.let { score ->
				if (!score.isFinite() || score !in 0f..1f) {
					reasons.add("INVALID_TRACKER_CONFIDENCE")
					eligible = false
					null
				} else {
					score
				}
			}
		}
		val present = roleScores.size
		val roleCoverage = if (required.isEmpty()) 0f else present.toFloat() / required.size
		if (required.isEmpty() || present != required.size) {
			reasons.add("REQUIRED_TRACKER_ROLE_MISSING")
			eligible = false
		}
		val trackerQuality = if (present == 0) 0f else roleScores.average().toFloat()
		if (roleScores.any { it < minimumLearningScore }) {
			reasons.add("TRACKER_CONFIDENCE_LOW")
			eligible = false
		}

		val anchorQuality = if (input.absoluteAnchorAvailable && input.absoluteAnchorQuality.isFinite() && input.absoluteAnchorQuality in 0f..1f) input.absoluteAnchorQuality else 0f
		if (!input.absoluteAnchorAvailable) {
			reasons.add("ABSOLUTE_ANCHOR_MISSING")
			eligible = false
		} else if (!input.absoluteAnchorQuality.isFinite() || input.absoluteAnchorQuality !in 0f..1f) {
			reasons.add("ABSOLUTE_ANCHOR_QUALITY_INVALID")
			eligible = false
		} else if (input.absoluteAnchorQuality < minimumAnchorQuality) {
			reasons.add("ABSOLUTE_ANCHOR_QUALITY_LOW")
			eligible = false
		}

		val chains = input.independentChainDisagreementsRadians
		val validChains = chains.filter { it.isFinite() && it >= 0.0 }
		val chainCoverage = if (input.minimumIndependentChains <= 0) 0f else (validChains.size.toFloat() / input.minimumIndependentChains).coerceIn(0f, 1f)
		if (input.minimumIndependentChains <= 0 || validChains.size < input.minimumIndependentChains) {
			reasons.add("INDEPENDENT_CHAIN_COVERAGE_LOW")
			eligible = false
		}
		if (validChains.size != chains.size) {
			reasons.add("INVALID_CHAIN_DISAGREEMENT")
			eligible = false
		}
		val chainQuality = if (validChains.isEmpty()) 0f else exp(-validChains.average() / maximumChainDisagreementRadians).toFloat().coerceIn(0f, 1f)
		if (chainQuality < minimumLearningScore || validChains.any { it > maximumChainDisagreementRadians }) {
			reasons.add("INDEPENDENT_CHAINS_DISAGREE")
			eligible = false
		}

		val residuals = input.independentResidualsRadians
		val validResiduals = residuals.filter { it.isFinite() && it >= 0.0 }
		val residualCoverage = if (input.minimumIndependentResiduals <= 0) 0f else (validResiduals.size.toFloat() / input.minimumIndependentResiduals).coerceIn(0f, 1f)
		if (input.minimumIndependentResiduals <= 0 || validResiduals.size < input.minimumIndependentResiduals) {
			reasons.add("INDEPENDENT_RESIDUAL_COVERAGE_LOW")
			eligible = false
		}
		if (validResiduals.size != residuals.size) {
			reasons.add("INVALID_INDEPENDENT_RESIDUAL")
			eligible = false
		}
		val residualQuality = if (validResiduals.isEmpty()) 0f else exp(-validResiduals.average() / maximumIndependentResidualRadians).toFloat().coerceIn(0f, 1f)
		if (validResiduals.any { it > maximumIndependentResidualRadians }) {
			reasons.add("INDEPENDENT_RESIDUAL_TOO_LARGE")
			eligible = false
		}

		val contactQuality = when {
			input.contactReliability == null && !input.contactRequiredForLearning -> 1f

			input.contactReliability == null -> {
				reasons.add("CONTACT_EVIDENCE_MISSING")
				eligible = false
				0f
			}

			!input.contactReliability.isFinite() || input.contactReliability !in 0f..1f -> {
				reasons.add("CONTACT_EVIDENCE_INVALID")
				eligible = false
				0f
			}

			else -> input.contactReliability
		}
		if (input.contactRequiredForLearning && contactQuality < minimumLearningScore) {
			reasons.add("CONTACT_RELIABILITY_LOW")
			eligible = false
		}

		val motionQuality = if (input.motionUncertainty.isFinite()) (1f - input.motionUncertainty.coerceIn(0f, 1f)) else 0f
		if (!input.motionUncertainty.isFinite() || input.motionUncertainty !in 0f..maximumMotionUncertainty) {
			reasons.add("MOTION_UNCERTAINTY_HIGH")
			eligible = false
		}

		// The geometric mean keeps correlated quality factors interpretable; zero coverage remains zero.
		val factors = listOf(roleCoverage, trackerQuality, anchorQuality, chainCoverage, chainQuality, residualCoverage, residualQuality, contactQuality, motionQuality)
		val score = if (factors.any { it <= 0f }) 0f else kotlin.math.exp(factors.sumOf { kotlin.math.ln(it.toDouble()) } / factors.size).toFloat().coerceIn(0f, 1f)
		if (score < minimumLearningScore) {
			reasons.add("GLOBAL_CONFIDENCE_LOW")
			eligible = false
		}
		if (reasons.isEmpty()) reasons.add("SUPPORTED_INDEPENDENT_EVIDENCE")
		return GlobalPoseConfidence(score, reasons.toList(), eligible)
	}
}
