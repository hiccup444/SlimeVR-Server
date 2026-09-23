package dev.slimevr.tracking.processor.adaptive

import com.fasterxml.jackson.databind.ObjectMapper
import io.eiren.util.logging.LogManager
import io.github.axisangles.ktmath.Quaternion
import io.github.axisangles.ktmath.Vector3
import java.nio.file.Files
import java.nio.file.Path
import java.nio.file.Paths
import java.nio.file.StandardOpenOption
import java.util.concurrent.ArrayBlockingQueue
import java.util.concurrent.TimeUnit
import java.util.concurrent.atomic.AtomicLong

/** Writes bounded diagnostic recordings without blocking the pose thread on disk I/O. */
class AdaptiveTelemetryRecorder(
	private val directory: String,
	private val maxBytes: Long = 64L * 1024 * 1024,
	private val maxParts: Int = 32,
) : AutoCloseable {
	private data class QueuedFrame(val frame: AdaptiveTelemetryFrame, val droppedAtOffer: Long)
	private val queue = ArrayBlockingQueue<QueuedFrame>(128)
	private val dropped = AtomicLong()

	@Volatile private var running = true

	@Volatile var failure: String? = null
		private set

	@Volatile var outputPath: Path? = null
		private set

	@Volatile var outputPaths: List<Path> = emptyList()
		private set

	@Volatile var writtenFrames = 0L
		private set

	@Volatile var sizeLimitReached = false
		private set
	val droppedFrames: Long get() = dropped.get()
	val isRecording: Boolean get() = running && failure == null && !sizeLimitReached
	val isFinalizing: Boolean get() = !running && worker.isAlive
	private val worker = Thread({ writeFrames() }, "AdaptiveTelemetryWriter").apply {
		isDaemon = true
		start()
	}

	fun offer(frame: AdaptiveTelemetryFrame) {
		if (running && !queue.offer(QueuedFrame(frame, dropped.get()))) dropped.incrementAndGet()
	}

	private fun writeFrames() {
		try {
			require(maxBytes > 0 && maxParts in 1..64)
			val folder = Paths.get(directory)
			Files.createDirectories(folder)
			var path = Files.createTempFile(folder, "session-", ".jsonl")
			val stem = path.fileName.toString().removeSuffix(".jsonl")
			outputPath = path
			outputPaths = listOf(path)
			val mapper = ObjectMapper()
			val header = "{\"type\":\"header\",\"schemaVersion\":1,\"mode\":\"diagnostics\"}\n"
			val headerBytes = header.toByteArray(Charsets.UTF_8).size.toLong()
			var bytes = headerBytes
			var part = 1
			var writer = Files.newBufferedWriter(path)
			try {
				writer.write(header)
				while (running || queue.isNotEmpty()) {
					val queued = queue.poll(100, TimeUnit.MILLISECONDS) ?: continue
					val line = mapper.writeValueAsString(mapOf("type" to "frame", "droppedFrames" to queued.droppedAtOffer, "frame" to queued.frame.toRecord()))
					val lineBytes = line.toByteArray(Charsets.UTF_8).size + 1L
					if (bytes + lineBytes > maxBytes) {
						if (bytes == headerBytes || part >= maxParts) {
							sizeLimitReached = true
							LogManager.info("[AdaptiveTracking] Recording size limit reached: $path")
							break
						}
						writer.close()
						part++
						path = folder.resolve("$stem-part$part.jsonl")
						writer = Files.newBufferedWriter(path, StandardOpenOption.CREATE_NEW, StandardOpenOption.WRITE)
						outputPath = path
						outputPaths = outputPaths + listOf(path)
						writer.write(header)
						bytes = headerBytes
					}
					writer.write(line)
					writer.newLine()
					bytes += lineBytes
					writtenFrames++
					if (queue.isEmpty()) writer.flush()
				}
			} finally {
				writer.close()
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
	"segments" to segments.map { s -> mapOf("parent" to s.parent, "length" to s.length, "rootPosition" to s.rootPosition.toRecord(), "measured" to s.measured.toRecord(), "confidence" to s.confidence, "fixed" to s.fixed, "previous" to s.previous?.toRecord(), "temporalWeight" to s.temporalWeight, "rotationSource" to s.rotationSource, "sourceOffset" to s.sourceOffset.toRecord(), "initialRotation" to s.initialRotation?.toRecord()) },
	"anchors" to anchors.map { a -> mapOf("segment" to a.segment, "target" to a.target.toRecord(), "weight" to a.weight, "otherSegment" to a.otherSegment) },
	"joints" to joints.map { j -> mapOf("first" to j.first, "second" to j.second, "weight" to j.weight, "hinge" to j.hinge, "firstOffset" to j.firstOffset.toRecord(), "secondOffset" to j.secondOffset.toRecord(), "maxSwingRadians" to j.maxSwingRadians, "maxTwistRadians" to j.maxTwistRadians) },
)
internal fun Quaternion.toRecord() = mapOf("w" to w, "x" to x, "y" to y, "z" to z)
internal fun Vector3.toRecord() = mapOf("x" to x, "y" to y, "z" to z)
