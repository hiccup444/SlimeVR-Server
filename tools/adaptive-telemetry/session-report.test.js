"use strict";

const assert = require("node:assert/strict");
const fs = require("node:fs");
const os = require("node:os");
const path = require("node:path");
const { execFileSync } = require("node:child_process");
const { fixtureFrames } = require("./replay.js");
const { readNotes, integrity, report, compare } = require("./session-report.js");

const directory = fs.mkdtempSync(path.join(os.tmpdir(), "adaptive-session-report-"));
try {
  const frames = fixtureFrames("static-standing");
  const header = { type: "header", schemaVersion: 1, mode: "diagnostics" };
  const first = path.join(directory, "session-test.jsonl");
  const second = path.join(directory, "session-test-part2.jsonl");
  fs.writeFileSync(first, [header, ...frames.slice(0, 50)].map(JSON.stringify).join("\n") + "\n");
  fs.writeFileSync(second, [header, ...frames.slice(50)].map(JSON.stringify).join("\n") + "\n");
  const notesFile = path.join(directory, "notes.jsonl");
  fs.writeFileSync(notesFile, [
    { type: "header", schemaVersion: 1, mode: "live-debug-snapshots", session: { scenario: "standing", condition: "Baseline", trial: "1" }, settings: { telemetrySampleRateHz: 50 }, omittedEntries: 0 },
    { type: "frame", receivedAt: "2026-09-22T00:00:00Z", frame: frames[30].frame },
    { type: "marker", message: "foot shifted", receivedAt: "2026-09-22T00:00:01Z", timestampNanos: frames[30].frame.timestampNanos, resetEpoch: 0 },
    { type: "marker", message: "outside", timestampNanos: 10_000_000_000, resetEpoch: 0 },
  ].map(JSON.stringify).join("\n") + "\n");
  const result = report([second, first], notesFile);
  assert.equal(result.metrics.frameCount, 100);
  assert.equal(result.sourceFiles[0].path, first);
  assert.equal(result.integrity.nonIncreasingTimestamps, 0);
  assert.equal(result.integrity.gaps.length, 0);
  assert.equal(result.integrity.partSequenceIssue, false);
  assert.equal(result.markers[0].alignment, "NEAREST_RECORDED_FRAME");
  assert.equal(result.markers[0].contactStates.left, "PLANTED");
  assert.equal(result.markers[0].markerTiming, "STALE_UI_SNAPSHOT");
  assert.equal(result.markers[0].snapshotAgeMs, 1000);
  assert.equal(result.markers[1].alignment, "OUTSIDE_RECORDED_WINDOW_OR_EPOCH");
  assert.equal(result.metrics.trackers[0].role, "LEFT_FOOT");
  assert.deepEqual(result.metrics.poseResiduals, []);
  assert.equal(result.solverTiming.samples, 0);
  assert.equal(integrity([frames[0], frames[20]], 50).gaps.length, 1);
  assert.equal(integrity([frames[1], frames[0]], 50).nonIncreasingTimestamps, 1);
  assert.equal(readNotes(notesFile).records.length, 3);
  const feature = { ...result, condition: "Feature on", session: { ...result.session, condition: "Feature on" } };
  assert.equal(compare(result, feature).baseline.plantedFootSlideMetersPerSecond, 0);
  assert.throws(() => compare(result, { ...feature, session: { scenario: "walking" } }), /same scenario/);
  const cli = JSON.parse(execFileSync(process.execPath, [path.join(__dirname, "session-report.js"), "--notes", notesFile, second, first], { encoding: "utf8" }));
  assert.equal(cli.markers.length, 2);
  assert.throws(() => readNotes(first), /Unsupported UI debug log/);
} finally {
  fs.rmSync(directory, { recursive: true, force: true });
}
console.log("adaptive session report tests passed");
