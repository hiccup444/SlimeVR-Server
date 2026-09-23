#!/usr/bin/env node
"use strict";

const fs = require("node:fs");
const path = require("node:path");
const { parseRecording, evaluate } = require("./replay.js");

const MAX_BYTES = 128 * 1024 * 1024;
const MAX_FRAMES = 100000;
const MAX_NOTES = 10000;

function finite(value) { return typeof value === "number" && Number.isFinite(value); }
function fileName(value) { return value.split(/[\\/]/).pop().toLowerCase(); }
function readBounded(file) {
  if (fs.statSync(file).size > MAX_BYTES) throw new Error(`${file} exceeds the 128 MiB limit`);
  return fs.readFileSync(file, "utf8");
}
function readNotes(file) {
  const lines = readBounded(file).split(/\r?\n/);
  const errors = [];
  const records = [];
  let header = null;
  for (let index = 0; index < lines.length; index++) {
    if (!lines[index].trim()) continue;
    if (Buffer.byteLength(lines[index], "utf8") > 1024 * 1024) { errors.push(`Line ${index + 1}: exceeds 1 MiB`); continue; }
    let value;
    try { value = JSON.parse(lines[index]); } catch (error) { errors.push(`Line ${index + 1}: ${error.message}`); continue; }
    if (!value || typeof value !== "object" || Array.isArray(value)) { errors.push(`Line ${index + 1}: invalid record`); continue; }
    if (value.type === "header") {
      if (value.schemaVersion !== 1 || value.mode !== "live-debug-snapshots") throw new Error("Unsupported UI debug log");
      header = value;
    } else if (records.length < MAX_NOTES) records.push(value);
    else { errors.push(`Record limit reached (${MAX_NOTES})`); break; }
  }
  if (!header) throw new Error("UI debug log header is missing");
  return { header, records, errors };
}
function readParts(files) {
  if (!files.length) throw new Error("At least one server recording part is required");
  const parts = files.map(file => ({ file, data: parseRecording(readBounded(file)) }))
    .sort((a, b) => a.data.frames[0].frame.timestampNanos - b.data.frames[0].frame.timestampNanos);
  if (parts.some(part => part.data.header?.mode !== "diagnostics")) throw new Error("Expected server diagnostics recordings");
  const frames = parts.flatMap(part => part.data.frames);
  if (frames.length > MAX_FRAMES) throw new Error(`Combined recording exceeds ${MAX_FRAMES} frames`);
  const names = parts.map(part => fileName(part.file).match(/^(session-.+?)(?:-part(\d+))?\.jsonl$/));
  const partSequenceIssue = names.every(Boolean) &&
    (new Set(names.map(match => match[1])).size !== 1 || names.some((match, index) => (match[2] ? Number(match[2]) : 1) !== index + 1));
  return { parts, frames, partSequenceIssue, parseErrors: parts.flatMap(part => part.data.errors.map(error => `${part.file}: ${error}`)) };
}
function nearestFrame(frames, timestamp) {
  let low = 0; let high = frames.length;
  while (low < high) {
    const middle = (low + high) >> 1;
    if (frames[middle].frame.timestampNanos < timestamp) low = middle + 1;
    else high = middle;
  }
  if (low === 0) return frames[0];
  if (low === frames.length) return frames[frames.length - 1];
  return timestamp - frames[low - 1].frame.timestampNanos <= frames[low].frame.timestampNanos - timestamp ? frames[low - 1] : frames[low];
}
function summarizeMarker(record, frames, snapshotAgeMs = null) {
  const timestamp = record.timestampNanos;
  if (!finite(timestamp)) return { message: record.message ?? "", receivedAt: record.receivedAt ?? null, alignment: "NO_SNAPSHOT_TIMESTAMP", snapshotAgeMs };
  const envelope = nearestFrame(frames, timestamp);
  const frame = envelope.frame;
  const distanceMs = Math.abs(frame.timestampNanos - timestamp) / 1e6;
  const sameEpoch = !finite(record.resetEpoch) || record.resetEpoch === frame.resetEpoch;
  const aligned = sameEpoch && distanceMs <= 500;
  return {
    message: record.message ?? "",
    receivedAt: record.receivedAt ?? null,
    snapshotTimestampNanos: timestamp,
    nearestFrameTimestampNanos: frame.timestampNanos,
    distanceMs,
    alignment: aligned ? "NEAREST_RECORDED_FRAME" : "OUTSIDE_RECORDED_WINDOW_OR_EPOCH",
    snapshotAgeMs,
    markerTiming: snapshotAgeMs == null ? "UNKNOWN" : snapshotAgeMs > 500 ? "STALE_UI_SNAPSHOT" : "RECENT_UI_SNAPSHOT",
    ...(aligned ? {
      resetEpoch: frame.resetEpoch,
      contactStates: Object.fromEntries(Object.entries(frame.footContacts || {}).map(([side, contact]) => [side, contact?.state ?? null])),
      driftReasons: (frame.driftDiagnostics || []).map(item => ({ trackerId: item.trackerId, reason: item.residual?.reason ?? null, learningEligible: item.residual?.eligibleForLearning === true, holdoverActive: item.holdoverActive === true, temperatureModelStatus: item.temperatureModelStatus ?? null })),
      recoveryTrackerIds: frame.poseDiagnostic?.recoveryTrackerIds ?? [],
      unhealthyTrackers: (frame.samples || []).filter(sample => sample.health?.reasons?.some(reason => reason !== "HEALTHY" && reason !== "WARMING_UP")).map(sample => ({ id: sample.id, name: sample.name, reasons: sample.health.reasons })),
      lowConfidenceTrackers: (frame.samples || []).filter(sample => finite(sample.confidence?.score) && sample.confidence.score < 0.5).map(sample => ({ id: sample.id, name: sample.name, score: sample.confidence.score, reasons: sample.confidence.reasons ?? [] })),
    } : {}),
  };
}
function integrity(frames, expectedHz) {
  const positive = [];
  let reversals = 0;
  let resetTransitions = 0;
  let droppedFrames = Math.max(0, frames[0]?.droppedFrames || 0);
  for (let index = 1; index < frames.length; index++) {
    const previous = frames[index - 1]; const current = frames[index];
    const dt = current.frame.timestampNanos - previous.frame.timestampNanos;
    if (dt <= 0) reversals++;
    else if (current.frame.resetEpoch === previous.frame.resetEpoch) positive.push(dt);
    if (current.frame.resetEpoch !== previous.frame.resetEpoch) resetTransitions++;
    droppedFrames += Math.max(0, (current.droppedFrames || 0) - (previous.droppedFrames || 0));
  }
  positive.sort((a, b) => a - b);
  const median = positive.length ? positive[Math.floor(positive.length / 2)] : null;
  const expectedNanos = finite(expectedHz) && expectedHz > 0 && expectedHz <= 200 ? 1e9 / expectedHz : null;
  const gapThreshold = Math.max(100_000_000, 3 * (expectedNanos || median || 0));
  const gaps = [];
  for (let index = 1; index < frames.length; index++) {
    const before = frames[index - 1].frame; const after = frames[index].frame;
    const dt = after.timestampNanos - before.timestampNanos;
    if (after.resetEpoch === before.resetEpoch && dt > gapThreshold) gaps.push({ beforeTimestampNanos: before.timestampNanos, afterTimestampNanos: after.timestampNanos, gapMs: dt / 1e6 });
  }
  return { medianIntervalMs: median == null ? null : median / 1e6, expectedSampleRateHz: expectedNanos ? expectedHz : null, gapThresholdMs: gapThreshold / 1e6, gaps: gaps.slice(0, 100), omittedGaps: Math.max(0, gaps.length - 100), nonIncreasingTimestamps: reversals, resetTransitions, droppedFrames };
}
function solverTiming(frames) {
  const milliseconds = frames.map(item => item.frame.poseDiagnostic?.processingNanos)
    .filter(value => finite(value) && value >= 0).map(value => value / 1e6).sort((a, b) => a - b);
  const percentile = fraction => milliseconds.length ? milliseconds[Math.ceil(fraction * milliseconds.length) - 1] : null;
  return { samples: milliseconds.length, medianMs: percentile(0.5), p95Ms: percentile(0.95), maxMs: percentile(1), scope: "Pose optimizer processing only, not total input-to-output latency" };
}
function report(files, notesFile = null) {
  const { parts, frames, partSequenceIssue, parseErrors } = readParts(files);
  const notes = notesFile ? readNotes(notesFile) : null;
  const metrics = evaluate({ frames });
  const expectedHz = notes?.header.settings?.telemetrySampleRateHz;
  const condition = notes?.header.session?.condition ?? null;
  const marks = notes?.records.filter(record => record.type === "marker") ?? [];
  const snapshotReceipts = new Map();
  for (const record of notes?.records ?? []) {
    if (record.type !== "frame" || !finite(record.frame?.timestampNanos)) continue;
    const received = Date.parse(record.receivedAt);
    if (finite(received)) snapshotReceipts.set(`${record.frame.resetEpoch}:${record.frame.timestampNanos}`, received);
  }
  const recordedNames = new Set(parts.map(part => fileName(part.file)));
  const listedParts = Array.isArray(notes?.header.serverRecordingFiles) ? notes.header.serverRecordingFiles : [];
  const missingListedParts = listedParts.filter(file => typeof file === "string" && !recordedNames.has(fileName(file)));
  const orderedFrames = [...frames].sort((a, b) => a.frame.timestampNanos - b.frame.timestampNanos);
  return {
    reportVersion: 1,
    scope: "Recorded telemetry and sampled UI snapshots; no raw packet replay or ground-truth accuracy measurement.",
    session: notes?.header.session ?? null,
    build: notes?.header.build ?? null,
    modifiedBuild: notes?.header.modifiedBuild ?? null,
    condition,
    sourceFiles: parts.map(part => ({ path: path.resolve(part.file), frames: part.data.frames.length })),
    notesFile: notesFile ? path.resolve(notesFile) : null,
    uiEntriesOmitted: notes?.header.omittedEntries ?? null,
    integrity: { ...integrity(frames, expectedHz), partSequenceIssue, missingListedParts },
    solverTiming: solverTiming(frames),
    markers: marks.map(mark => {
      const snapshotReceipt = snapshotReceipts.get(`${mark.resetEpoch}:${mark.timestampNanos}`);
      const markerReceipt = Date.parse(mark.receivedAt);
      const age = finite(snapshotReceipt) && finite(markerReceipt) && markerReceipt >= snapshotReceipt ? markerReceipt - snapshotReceipt : null;
      return summarizeMarker(mark, orderedFrames, age);
    }),
    parseErrors: [...parseErrors, ...(notes?.errors ?? []).map(error => `${notesFile}: ${error}`)],
    metrics: { metricNotes: metrics.metricNotes, frameCount: metrics.frameCount, usableSeconds: metrics.usableSeconds, comparedIntervals: metrics.comparedIntervals, resetCount: metrics.resetCount, droppedFrameTransitions: metrics.droppedFrameTransitions, trackers: metrics.trackers, driftDiagnostics: metrics.driftDiagnostics, poseResiduals: metrics.poseResiduals },
  };
}
function compare(baseline, feature) {
  if (baseline.session?.scenario !== feature.session?.scenario || !baseline.session?.scenario) throw new Error("Both reports need UI logs for the same scenario");
  if (baseline.condition !== "Baseline" || feature.condition !== "Feature on") throw new Error("Expected Baseline then Feature on report");
  const footSlide = item => {
    const feet = item.metrics.trackers.filter(tracker => tracker.group === "output" && ["LEFT_FOOT", "RIGHT_FOOT"].includes(tracker.role) && tracker.plantedSeconds > 0);
    const seconds = feet.reduce((sum, tracker) => sum + tracker.plantedSeconds, 0);
    return seconds ? feet.reduce((sum, tracker) => sum + tracker.footSlideMeters, 0) / seconds : null;
  };
  const values = item => {
    const drift = item.metrics.driftDiagnostics;
    const outputs = item.metrics.trackers.filter(tracker => tracker.group === "output");
    const largestStep = key => {
      const observed = outputs.map(tracker => tracker[key]).filter(finite);
      return observed.length ? Math.max(...observed) : null;
    };
    const change = drift.reduce((sum, tracker) => sum + (tracker.totalAbsoluteBiasChangeRadians || 0), 0);
    const seconds = drift.reduce((sum, tracker) => sum + (tracker.correctionObservedSeconds || 0), 0);
    return { recordedFrames: item.metrics.frameCount, usableSeconds: item.metrics.usableSeconds, resetCount: item.metrics.resetCount, droppedFrames: item.integrity.droppedFrames, recordingGaps: item.integrity.gaps.length + item.integrity.omittedGaps, plantedFootSlideMetersPerSecond: footSlide(item), largestOutputPositionStepMeters: largestStep("maxPositionStepMeters"), largestOutputOrientationStepRadians: largestStep("maxOrientationStepRadians"), yawLearningEligibleTrackerFrames: drift.reduce((sum, tracker) => sum + tracker.eligibleFrames, 0), yawHoldoverTrackerFrames: drift.reduce((sum, tracker) => sum + (tracker.holdoverFrames || 0), 0), totalAbsoluteYawCorrectionRadians: change, meanAbsoluteYawCorrectionRateRadiansPerSecond: seconds ? change / seconds : null, optimizerMedianMs: item.solverTiming?.medianMs ?? null, optimizerP95Ms: item.solverTiming?.p95Ms ?? null };
  };
  return { scenario: baseline.session.scenario, baseline: values(baseline), featureOn: values(feature), note: "Descriptive recorded-output comparison only. Match trial motion and inspect markers; these numbers do not establish absolute yaw accuracy or reduced drift." };
}

if (require.main === module) {
  try {
    const args = process.argv.slice(2);
    if (args[0] === "--compare" && args.length === 3) {
      process.stdout.write(`${JSON.stringify(compare(JSON.parse(readBounded(args[1])), JSON.parse(readBounded(args[2]))), null, 2)}\n`);
    } else {
      const noteIndex = args.indexOf("--notes");
      let noteFile = null;
      if (noteIndex >= 0) {
        if (!args[noteIndex + 1]) throw new Error("--notes requires a UI debug log");
        noteFile = args.splice(noteIndex, 2)[1];
      }
      if (!args.length) throw new Error("Usage: node session-report.js [--notes ui-debug.jsonl] server-part.jsonl ... | --compare baseline-report.json feature-report.json");
      process.stdout.write(`${JSON.stringify(report(args, noteFile), null, 2)}\n`);
    }
  } catch (error) { process.stderr.write(`${error.message}\n`); process.exitCode = 1; }
}

module.exports = { readNotes, readParts, nearestFrame, summarizeMarker, integrity, report, compare };
