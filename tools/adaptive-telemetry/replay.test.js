"use strict";
const assert = require("node:assert/strict");
const fs = require("node:fs");
const os = require("node:os");
const path = require("node:path");
const { parseRecording, evaluate, fixtureFrames, writeFixtures } = require("./replay.js");

function rec(frames) { return { header: { schemaVersion: 1 }, frames }; }
function frame(t, options = {}) {
  const sample = { id: 1, name: "foot", role: "LEFT_FOOT", status: "OK", continuousObservation: true,
    position: { x: options.x ?? 0, y: 0, z: 0 }, derivedLinearVelocity: options.vx === undefined ? { x: 0, y: 0, z: 0 } : { x: options.vx, y: 0, z: 0 },
    angularSpeedRadiansPerSecond: options.omega ?? 0 };
  return { type: "frame", droppedFrames: options.dropped ?? 0, frame: { timestampNanos: t, resetEpoch: options.epoch ?? 0, samples: [], computedSamples: options.invalid ? [{ ...sample, continuousObservation: false }] : [sample], driftDiagnostics: options.drift ? [{ trackerId: 4, biasRadians: 0.1, residual: { errorRadians: options.drift, filteredErrorRadians: options.drift / 2, consistentSeconds: 1.5, eligibleForLearning: true, reason: "stable" } }] : [], footContacts: { left: { state: options.planted === false ? "AIRBORNE" : "PLANTED", plantPosition: { x: 0, y: 0, z: 0 } } } } };
}

const malformed = parseRecording('{"type":"header","schemaVersion":1}\nnope\n{"type":"other"}\n{"type":"frame","frame":{"timestampNanos":1,"samples":[],"computedSamples":[]}}\n');
assert.equal(malformed.frames.length, 1);
assert.equal(malformed.errors.length, 2);
assert.throws(() => parseRecording('{"type":"frame","frame":{"timestampNanos":1}}'), /missing required frame fields/);
assert.doesNotThrow(() => parseRecording('null\n7\n"not an object"\n{"type":"header","schemaVersion":1}\n{"type":"frame","frame":{"timestampNanos":1,"samples":[null,7],"computedSamples":[false]}}'));
assert.throws(() => parseRecording('{"type":"header","schemaVersion":2}'), /Unsupported telemetry schema version/);
assert.equal(parseRecording('{"type":"header","schemaVersion":1}\n{"type":"frame","frame":{"timestampNanos":1,"samples":[null],"computedSamples":[]}}').errors.length, 1);

const measured = evaluate(rec([frame(0, { planted: true }), frame(100_000_000, { x: 0.02 }), frame(200_000_000, { x: 0.04, epoch: 1 }), frame(300_000_000, { x: 0.06, epoch: 1 }), frame(400_000_000, { x: 0.08, epoch: 1, dropped: 1 }), frame(500_000_000, { x: 0.1, epoch: 1, dropped: 1, invalid: true }), frame(600_000_000, { x: 0.12, epoch: 1, dropped: 1 })]));
assert.equal(measured.resetCount, 1);
assert.equal(measured.droppedFrameTransitions, 1);
assert.equal(measured.comparedIntervals, 2, "only uninterrupted valid intervals are scored");
assert.ok(Math.abs(measured.usableSeconds - 0.2) < 1e-9);
assert.equal(measured.trackers[0].plantedIntervals, 2);
assert.ok(Math.abs(measured.trackers[0].footSlideMeters - 0.04) < 1e-9);
assert.ok(Math.abs(measured.trackers[0].meanFootSlideMetersPerSecond - 0.2) < 1e-9);

const dynamic = evaluate(rec([frame(0, { omega: 0, drift: 0.4 }), frame(100_000_000, { omega: 1, vx: 0.2, drift: 0.2 }), frame(200_000_000, { omega: 3, vx: 0.4, drift: 0.1 })]));
assert.equal(dynamic.trackers[0].meanLinearAcceleration, 2);
assert.ok(dynamic.trackers[0].meanAngularSpeedSecondDerivativeRadiansPerSecondCubed > 0);
assert.equal(dynamic.driftDiagnostics[0].diagnosticFrames, 3);
assert.ok(dynamic.driftDiagnostics[0].meanFilteredErrorRadians > 0);
assert.equal(dynamic.driftDiagnostics[0].meanConsistentSeconds, 1.5);
assert.deepEqual(dynamic.driftDiagnostics[0].reasons, { stable: 3 });
assert.match(dynamic.evaluationScope, /not a full solver rerun or raw-packet replay/);

for (const [scenario, expected] of [["static-standing", 100], ["turn360", 100], ["walking", 160], ["disconnect", 100]]) {
  assert.equal(fixtureFrames(scenario).length, expected);
}
const staticMetrics=evaluate(rec(fixtureFrames("static-standing")));
assert.equal(staticMetrics.trackers[0].meanLinearAcceleration, 0);
assert.equal(staticMetrics.trackers[0].meanAngularSpeedSecondDerivativeRadiansPerSecondCubed, 0);
const turnMetrics=evaluate(rec(fixtureFrames("turn360")));
assert.ok(turnMetrics.trackers[0].validAngularSamples > 0);
const disconnectMetrics=evaluate(rec(fixtureFrames("disconnect")));
assert.equal(disconnectMetrics.resetCount, 1);
assert.ok(disconnectMetrics.transitions.some(item=>item.event==="observation-start"));
const fixtureDir = fs.mkdtempSync(path.join(os.tmpdir(), "adaptive-telemetry-"));
writeFixtures(fixtureDir);
assert.equal(fs.readdirSync(fixtureDir).length, 4);
for (const file of fs.readdirSync(fixtureDir)) assert.ok(parseRecording(fs.readFileSync(path.join(fixtureDir, file), "utf8")).frames.length > 0);
fs.rmSync(fixtureDir, { recursive: true, force: true });
console.log("recorded telemetry replay tests passed");
