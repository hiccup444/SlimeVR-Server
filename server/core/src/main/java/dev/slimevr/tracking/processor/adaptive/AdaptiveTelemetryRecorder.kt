package dev.slimevr.tracking.processor.adaptive

import com.fasterxml.jackson.databind.ObjectMapper
import io.eiren.util.logging.LogManager
import io.github.axisangles.ktmath.Quaternion
import io.github.axisangles.ktmath.Vector3
import java.nio.file.Files
import java.nio.file.Path
import java.nio.file.Paths
import java.util.concurrent.ArrayBlockingQueue
import java.util.concurrent.TimeUnit
import java.util.concurrent.atomic.AtomicLong

/** Writes bounded diagnostic recordings without blocking the pose thread on disk I/O. */
class AdaptiveTelemetryRecorder(
	private val directory: String,
	private val maxBytes: Long = 64L * 1024 * 1024,
) : AutoCloseable {
	private val queue = ArrayBlockingQueue<AdaptiveTelemetryFrame>(128)
	private val dropped = AtomicLong()

	@Volatile private var running = true

	@Volatile var failure: String? = null
		private set

	@Volatile var outputPath: Path? = null
		private set
	private val worker = Thread({ writeFrames() }, "AdaptiveTelemetryWriter").apply {
		isDaemon = true
		start()
	}

	fun offer(frame: AdaptiveTelemetryFrame) {
		if (running && !queue.offer(frame)) dropped.incrementAndGet()
	}

	private fun writeFrames() {
		try {
			val folder = Paths.get(directory)
			Files.createDirectories(folder)
			val path = Files.createTempFile(folder, "session-", ".jsonl")
			outputPath = path
			val mapper = ObjectMapper()
			var bytes = 0L
			Files.newBufferedWriter(path).use { writer ->
				writer.write("{\"type\":\"header\",\"schemaVersion\":1,\"mode\":\"diagnostics\"}\n")
				while (running || queue.isNotEmpty()) {
					val frame = queue.poll(100, TimeUnit.MILLISECONDS) ?: continue
					val line = mapper.writeValueAsString(mapOf("type" to "frame", "droppedFrames" to dropped.get(), "frame" to frame.toRecord()))
					bytes += line.toByteArray(Charsets.UTF_8).size + 1
					if (bytes > maxBytes) {
						LogManager.info("[AdaptiveTracking] Recording size limit reached: $path")
						break
					}
					writer.write(line)
					writer.newLine()
					if (queue.isEmpty()) writer.flush()
				}
			}
		} catch (error: Exception) {
			failure = error.message ?: error.javaClass.simpleName
			LogManager.warning("[AdaptiveTracking] Recording stopped: $failure")
		} finally {
			running = false
			queue.clear()
		}
	}

	override fun close() {
		running = false
	}

	fun awaitClosed(timeoutMillis: Long = 2000) {
		worker.join(timeoutMillis)
	}
}

// Explicit fields keep Kotlin value-class implementation details out of the file format.
internal fun AdaptiveTelemetryFrame.toRecord(): Map<String, Any?> = mapOf(
	"timestampNanos" to timestampNanos,
	"resetEpoch" to resetEpoch,
	"footAnchoringRequested" to footAnchoringRequested,
	"driftDiagnostics" to driftDiagnostics,
	"poseDiagnostic" to poseDiagnostic,
	"armCalibration" to armCalibration,
	"floorEstimate" to floorEstimate,
	"activity" to activity,
	"trackerPredictions" to trackerPredictions.mapValues { (_, prediction) ->
		val residual = prediction.residual
		mapOf(
			"expectedRotation" to prediction.expectedRotation.toRecord(),
			"measuredRotation" to prediction.measuredRotation.toRecord(),
			"residual" to mapOf(
				"trackerId" to residual.trackerId,
				"residualVectorRadians" to residual.residualVectorRadians.toRecord(),
				"magnitudeRadians" to residual.magnitudeRadians,
				"directionalConsistency" to residual.directionalConsistency,
				"continuousSeconds" to residual.continuousSeconds,
				"meanMagnitudeRadians" to residual.meanMagnitudeRadians,
				"magnitudeVarianceRadiansSquared" to residual.magnitudeVarianceRadiansSquared,
				"sampleCount" to residual.sampleCount,
				"independentlyConstrained" to residual.independentlyConstrained,
				"reason" to residual.reason,
			),
		)
	},
	"rawPose" to rawPose.mapValues { it.value.toRecord() },
	"predictedPose" to predictedPose.mapValues { it.value.toRecord() },
	"poseInput" to poseInput?.toRecord(),
	"samples" to samples.map { it.toRecord() },
	"computedSamples" to computedSamples.map { it.toRecord() },
	"footContacts" to footContacts.mapValues { (_, contact) ->
		mapOf(
			"state" to contact.state.name,
			"plantPosition" to contact.plantPosition?.toRecord(),
			"weight" to contact.weight,
			"speedMetersPerSecond" to contact.speedMetersPerSecond,
			"angularSpeedRadiansPerSecond" to contact.angularSpeedRadiansPerSecond,
		)
	},
)

private fun TelemetrySample.toRecord(): Map<String, Any?> = mapOf(
	"id" to id,
	"name" to name,
	"role" to role?.name,
	"status" to status.name,
	"rawRotation" to rawRotation?.toRecord(),
	"adjustedRotation" to adjustedRotation?.toRecord(),
	"position" to position?.toRecord(),
	"acceleration" to acceleration?.toRecord(),
	"derivedLinearVelocity" to derivedLinearVelocity?.toRecord(),
	"packetAgeNanos" to packetAgeNanos,
	"accelerationAgeNanos" to accelerationAgeNanos,
	"temperatureAgeNanos" to temperatureAgeNanos,
	"temperatureCelsius" to temperatureCelsius,
	"angularSpeedRadiansPerSecond" to angularSpeedRadiansPerSecond,
	"rotationExpected" to rotationExpected,
	"positionExpected" to positionExpected,
	"continuousObservation" to continuousObservation,
	"confidence" to confidence,
	"health" to health,
)

internal fun PoseSolverInput.toRecord() = mapOf(
	"segments" to segments.map { s -> mapOf("parent" to s.parent, "length" to s.length, "rootPosition" to s.rootPosition.toRecord(), "measured" to s.measured.toRecord(), "confidence" to s.confidence, "fixed" to s.fixed, "previous" to s.previous?.toRecord(), "temporalWeight" to s.temporalWeight, "rotationSource" to s.rotationSource, "sourceOffset" to s.sourceOffset.toRecord()) },
	"anchors" to anchors.map { a -> mapOf("segment" to a.segment, "target" to a.target.toRecord(), "weight" to a.weight, "otherSegment" to a.otherSegment) },
	"joints" to joints.map { j -> mapOf("first" to j.first, "second" to j.second, "weight" to j.weight, "hinge" to j.hinge, "firstOffset" to j.firstOffset.toRecord(), "secondOffset" to j.secondOffset.toRecord(), "maxSwingRadians" to j.maxSwingRadians, "maxTwistRadians" to j.maxTwistRadians) },
)
internal fun Quaternion.toRecord() = mapOf("w" to w, "x" to x, "y" to y, "z" to z)
internal fun Vector3.toRecord() = mapOf("x" to x, "y" to y, "z" to z)
