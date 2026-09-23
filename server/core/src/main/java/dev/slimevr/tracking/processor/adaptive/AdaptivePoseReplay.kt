package dev.slimevr.tracking.processor.adaptive

import com.fasterxml.jackson.databind.JsonNode
import com.fasterxml.jackson.databind.ObjectMapper
import io.github.axisangles.ktmath.Quaternion
import io.github.axisangles.ktmath.Vector3
import java.nio.file.Files
import java.nio.file.Paths
import java.nio.file.StandardOpenOption

/** Replays captured optimizer inputs without a VR server, wall clock, network, or sensor hardware. */
object AdaptivePoseReplay {
	fun decode(node: JsonNode): PoseSolverInput {
		fun number(n: JsonNode, name: String): Float = n.get(name)?.takeIf { it.isNumber }?.floatValue()?.takeIf { it.isFinite() } ?: error("Invalid $name")
		fun integer(n: JsonNode, name: String): Int = n.get(name)?.takeIf { it.isIntegralNumber && it.canConvertToInt() }?.intValue() ?: error("Invalid $name")
		fun vector(n: JsonNode) = Vector3(number(n, "x"), number(n, "y"), number(n, "z"))
		fun rotation(n: JsonNode) = Quaternion(number(n, "w"), number(n, "x"), number(n, "y"), number(n, "z"))
		fun array(name: String): List<JsonNode> = node.path(name).takeIf { it.isArray && it.size() <= 128 }?.toList() ?: error("Invalid $name")
		val segments = array("segments").map { s ->
			require(s.path("fixed").isBoolean)
			PoseSegment(
				integer(s, "parent"), number(s, "length"), vector(s.path("rootPosition")), rotation(s.path("measured")), number(s, "confidence"), s.path("fixed").booleanValue(),
				s.get("previous")?.takeUnless { it.isNull }?.let(::rotation), number(s, "temporalWeight"), integer(s, "rotationSource"), rotation(s.path("sourceOffset")),
				if (s.has("initialRotation")) s.get("initialRotation")?.takeUnless { it.isNull }?.let(::rotation) else s.get("previous")?.takeUnless { it.isNull }?.let(::rotation),
			)
		}
		val anchors = array("anchors").map { a -> PoseAnchor(integer(a, "segment"), vector(a.path("target")), number(a, "weight"), integer(a, "otherSegment")) }
		val joints = array("joints").map { j ->
			require(j.path("hinge").isBoolean)
			SoftJoint(
				integer(j, "first"),
				integer(j, "second"),
				number(j, "weight"),
				j.path("hinge").booleanValue(),
				j.get("firstOffset")?.let(::rotation) ?: Quaternion.IDENTITY,
				j.get("secondOffset")?.let(::rotation) ?: Quaternion.IDENTITY,
				j.get("maxSwingRadians")?.takeUnless { it.isNull }?.let { number(j, "maxSwingRadians") },
				j.get("maxTwistRadians")?.takeUnless { it.isNull }?.let { number(j, "maxTwistRadians") },
			)
		}
		return PoseSolverInput(segments, anchors, joints)
	}

	@JvmStatic
	fun main(args: Array<String>) {
		require(args.size == 2) { "Usage: AdaptivePoseReplay recording.jsonl results.jsonl" }
		val input = Paths.get(args[0])
		val output = Paths.get(args[1])
		require(Files.size(input) <= 128L * 1024 * 1024) { "Recording exceeds 128 MiB" }
		val mapper = ObjectMapper()
		val solver = PoseOptimizer()
		var count = 0
		Files.newBufferedWriter(output, StandardOpenOption.CREATE_NEW, StandardOpenOption.WRITE).use { writer ->
			Files.newBufferedReader(input).useLines { lines ->
				lines.forEachIndexed { index, line ->
					require(line.length <= 1024 * 1024) { "Oversized line ${index + 1}" }
					if (line.isBlank()) return@forEachIndexed
					val envelope = mapper.readTree(line)
					if (envelope.path("type").asText() == "header") {
						require(envelope.path("schemaVersion").asInt() == 1) { "Unsupported recording schema" }
						return@forEachIndexed
					}
					val frame = envelope.path("frame")
					val poseNode = frame.get("poseInput")?.takeUnless { it.isNull } ?: return@forEachIndexed
					require(++count <= 100_000) { "Too many optimizer frames" }
					val pose = decode(poseNode)
					val solved = solver.solve(pose.segments, pose.anchors, pose.joints)
					writer.write(mapper.writeValueAsString(mapOf("timestampNanos" to frame.path("timestampNanos").longValue(), "initialError" to solved.initialError, "finalError" to solved.finalError, "rotations" to solved.rotations.map { it.toRecord() }, "tails" to solved.tails.map { it.toRecord() })))
					writer.newLine()
				}
			}
		}
		require(count > 0) { "No optimizer inputs recorded. Enable the weighted pose solver and telemetry together." }
		println("Replayed $count optimizer frames into $output")
	}
}
