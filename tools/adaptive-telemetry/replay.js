#!/usr/bin/env node
"use strict";

const fs = require("node:fs");
const path = require("node:path");

const MAX_BYTES = 128 * 1024 * 1024;
const MAX_FRAMES = 100000;
const MAX_LINE = 1024 * 1024;

function finite(value) { return typeof value === "number" && Number.isFinite(value); }
function vec(value) { return value && finite(value.x) && finite(value.y) && finite(value.z) ? value : null; }
function orientationStep(a, b) {
  if (![a?.w, a?.x, a?.y, a?.z, b?.w, b?.x, b?.y, b?.z].every(finite)) return null;
  const aNorm = Math.hypot(a.w, a.x, a.y, a.z); const bNorm = Math.hypot(b.w, b.x, b.y, b.z);
  if (!finite(aNorm) || !finite(bNorm) || aNorm < 1e-6 || bNorm < 1e-6) return null;
  const dot = Math.abs(a.w*b.w + a.x*b.x + a.y*b.y + a.z*b.z) / (aNorm*bNorm);
  if (!finite(dot)) return null;
  return 2 * Math.acos(Math.min(1, Math.max(0, dot)));
}
function trackerSamples(frame) {
  return [...(Array.isArray(frame.samples) ? frame.samples : []), ...(Array.isArray(frame.computedSamples) ? frame.computedSamples : [])];
}
function parseRecording(text) {
  const parsed = [];
  const errors = [];
  let header = null;
  if (Buffer.byteLength(text, "utf8") > MAX_BYTES) throw new Error(`Input exceeds the ${MAX_BYTES} byte limit`);
  const lines = text.split(/\r?\n/);
  for (let index = 0; index < lines.length; index++) {
    const line = lines[index].trim();
    if (!line) continue;
    if (Buffer.byteLength(line, "utf8") > MAX_LINE) { errors.push(`Line ${index + 1}: line exceeds 1 MiB`); continue; }
    let value;
    try { value = JSON.parse(line); } catch (error) { errors.push(`Line ${index + 1}: ${error.message}`); continue; }
    if (!value || typeof value !== "object" || Array.isArray(value)) { errors.push(`Line ${index + 1}: JSON record must be an object`); continue; }
    if (value.type === "header") {
      if (value.schemaVersion !== 1) throw new Error(`Unsupported telemetry schema version: ${String(value.schemaVersion)}`);
      header = value; continue;
    }
    if (value.type !== "frame" || !value.frame || typeof value.frame !== "object" || !finite(value.frame.timestampNanos) || !Array.isArray(value.frame.samples) || !Array.isArray(value.frame.computedSamples)) {
      errors.push(`Line ${index + 1}: unknown record or missing required frame fields`); continue;
    }
    if (parsed.length >= MAX_FRAMES) { errors.push(`Frame limit reached (${MAX_FRAMES})`); break; }
    const cleanFrame = { ...value.frame };
    for (const field of ["samples", "computedSamples"]) {
      const records = [];
      for (let item = 0; item < cleanFrame[field].length; item++) {
        const sample = cleanFrame[field][item];
        if (!sample || typeof sample !== "object" || Array.isArray(sample)) { errors.push(`Line ${index + 1}: ${field}[${item}] must be an object`); continue; }
        records.push(sample);
      }
      cleanFrame[field] = records;
    }
    parsed.push({ ...value, frame: cleanFrame });
  }
  if (!parsed.length) throw new Error(errors.length ? errors.join("\n") : "No telemetry frames found");
  return { header, frames: parsed, errors };
}
function evaluate(recording) {
  const trackerStats = new Map();
  const driftStats = new Map();
  const poseResidualStats = new Map();
  const transitions = [];
  let usableSeconds = 0;
  let comparedIntervals = 0;
  let resetCount = 0;
  let previousFrame = null;
  let previousSamples = new Map();
  for (const envelope of recording.frames) {
    const frame = envelope.frame;
    const epoch = finite(frame.resetEpoch) ? frame.resetEpoch : 0;
    const time = frame.timestampNanos * 1e-9;
    const dropped = finite(envelope.droppedFrames) ? envelope.droppedFrames : 0;
    const frameBreak = previousFrame !== null && (epoch !== previousFrame.epoch || dropped > previousFrame.dropped || !(time > previousFrame.time));
    const hasValidObservation = trackerSamples(frame).some(sample => sample.status === "OK" && sample.continuousObservation === true);
    const frameInterval = previousFrame !== null && previousFrame.hasValidObservation && hasValidObservation && !frameBreak && time > previousFrame.time && time - previousFrame.time <= 0.5;
    if (frameInterval) usableSeconds += time - previousFrame.time;
    if (previousFrame !== null && epoch !== previousFrame.epoch) resetCount++;
    const current = new Map();
    for (const sample of trackerSamples(frame)) {
      const group = frame.samples.includes(sample) ? "input" : "output";
      const id = `${group}:${String(sample.id)}`;
      if (!trackerStats.has(id)) trackerStats.set(id, { id, name: String(sample.name ?? sample.id), role: sample.role ?? null, group, footSlideMeters: 0, plantedSeconds: 0, plantedIntervals: 0, validAngularSamples: 0, lowConfidenceFrames: 0, healthReasons: new Map(), maxPositionStepMeters: null, maxPositionStepTimestampNanos: null, maxOrientationStepRadians: null, maxOrientationStepTimestampNanos: null, linearAccelerations: [], angularSpeedSecondDerivatives: [], previousAngularAcceleration: null });
      const stats = trackerStats.get(id);
      if (finite(sample.confidence?.score) && sample.confidence.score < 0.5) stats.lowConfidenceFrames++;
      for (const reason of Array.isArray(sample.health?.reasons) ? sample.health.reasons : []) if (typeof reason === "string" && reason !== "HEALTHY") stats.healthReasons.set(reason, (stats.healthReasons.get(reason) || 0) + 1);
      const valid = sample.status === "OK" && sample.continuousObservation === true;
      const p = vec(sample.position);
      const velocity = vec(sample.derivedLinearVelocity);
      const angular = finite(sample.angularSpeedRadiansPerSecond) ? sample.angularSpeedRadiansPerSecond : null;
      current.set(id, { sample, p, velocity, angular, time, epoch, dropped, valid });
      if (valid && angular !== null) stats.validAngularSamples++;
      const prior = previousSamples.get(id);
      const intervalOk = !frameBreak && prior?.valid && valid && prior.epoch === epoch && prior.dropped === dropped && time > prior.time;
      if (intervalOk) {
        const dt = time - prior.time;
        if (dt <= 0.5) {
          comparedIntervals++;
          if (group === "output") {
            if (p && prior.p) {
              const step = Math.hypot(p.x-prior.p.x, p.y-prior.p.y, p.z-prior.p.z);
              if (stats.maxPositionStepMeters === null || step > stats.maxPositionStepMeters) { stats.maxPositionStepMeters = step; stats.maxPositionStepTimestampNanos = frame.timestampNanos; }
            }
            const step = orientationStep(sample.adjustedRotation, prior.sample.adjustedRotation);
            if (step !== null && (stats.maxOrientationStepRadians === null || step > stats.maxOrientationStepRadians)) { stats.maxOrientationStepRadians = step; stats.maxOrientationStepTimestampNanos = frame.timestampNanos; }
          }
          if (prior.velocity && velocity) stats.linearAccelerations.push(Math.hypot(velocity.x-prior.velocity.x, velocity.y-prior.velocity.y, velocity.z-prior.velocity.z) / dt);
          if (prior.angular !== null && angular !== null) {
            const accel = (angular-prior.angular)/dt;
            if (stats.previousAngularAcceleration !== null) stats.angularSpeedSecondDerivatives.push(Math.abs(accel-stats.previousAngularAcceleration)/dt);
            stats.previousAngularAcceleration = accel;
          } else stats.previousAngularAcceleration = null;
        } else stats.previousAngularAcceleration = null;
      } else stats.previousAngularAcceleration = null;
    }
    for (const diagnostic of Array.isArray(frame.driftDiagnostics) ? frame.driftDiagnostics : []) {
      if (!diagnostic || diagnostic.trackerId == null) continue;
      const id = String(diagnostic.trackerId);
      if (!driftStats.has(id)) driftStats.set(id, { trackerId: id, count: 0, eligibleFrames: 0, holdoverFrames: 0, reasons: new Map(), temperatureModelStatuses: new Map(), errors: [], filteredErrors: [], consistentSeconds: [], totalAbsoluteBiasChangeRadians: 0, correctionObservedSeconds: 0, maxCorrectionRateRadiansPerSecond: null, priorBias: null });
      const stat = driftStats.get(id);
      stat.count++;
      if (diagnostic.residual?.eligibleForLearning === true) stat.eligibleFrames++;
      if (diagnostic.holdoverActive === true) stat.holdoverFrames++;
      if (typeof diagnostic.residual?.reason === "string") stat.reasons.set(diagnostic.residual.reason, (stat.reasons.get(diagnostic.residual.reason) || 0) + 1);
      if (typeof diagnostic.temperatureModelStatus === "string") stat.temperatureModelStatuses.set(diagnostic.temperatureModelStatus, (stat.temperatureModelStatuses.get(diagnostic.temperatureModelStatus) || 0) + 1);
      if (finite(diagnostic.residual?.consistentSeconds)) stat.consistentSeconds.push(diagnostic.residual.consistentSeconds);
      if (finite(diagnostic.residual?.errorRadians)) stat.errors.push(diagnostic.residual.errorRadians);
      if (finite(diagnostic.residual?.filteredErrorRadians)) stat.filteredErrors.push(diagnostic.residual.filteredErrorRadians);
      if (finite(diagnostic.biasRadians)) {
        const prior = stat.priorBias;
        const dt = prior === null ? null : time - prior.time;
        if (prior !== null && prior.epoch === epoch && prior.dropped === dropped && dt > 0 && dt <= 0.5) {
          const change = Math.abs(diagnostic.biasRadians - prior.bias);
          stat.totalAbsoluteBiasChangeRadians += change;
          stat.correctionObservedSeconds += dt;
          stat.maxCorrectionRateRadiansPerSecond = Math.max(stat.maxCorrectionRateRadiansPerSecond ?? 0, change / dt);
        }
        stat.priorBias = { bias: diagnostic.biasRadians, time, epoch, dropped };
      } else stat.priorBias = null;
    }
    for (const [id, prediction] of Object.entries(frame.trackerPredictions || {})) {
      const residual = prediction?.residual;
      if (!finite(residual?.magnitudeRadians) || residual.magnitudeRadians < 0) continue;
      if (!poseResidualStats.has(id)) poseResidualStats.set(id, { trackerId: id, count: 0, sum: 0, max: 0, independentFrames: 0, reasons: new Map() });
      const stat = poseResidualStats.get(id);
      stat.count++;
      stat.sum += residual.magnitudeRadians;
      stat.max = Math.max(stat.max, residual.magnitudeRadians);
      if (residual.independentlyConstrained === true) stat.independentFrames++;
      if (typeof residual.reason === "string") stat.reasons.set(residual.reason, (stat.reasons.get(residual.reason) || 0) + 1);
    }
    if (frameInterval) {
      for (const [id, sample] of current) if (sample.valid && sample.sample.status === "OK") {
        const prior = previousSamples.get(id);
        if (prior?.valid && prior.epoch === epoch && prior.dropped === dropped) {
          const stat = trackerStats.get(id);
          const contactNow = frame.footContacts?.[sample.sample.role === "LEFT_FOOT" ? "left" : sample.sample.role === "RIGHT_FOOT" ? "right" : ""];
          const contactBefore = previousFrame.envelope.frame.footContacts?.[sample.sample.role === "LEFT_FOOT" ? "left" : sample.sample.role === "RIGHT_FOOT" ? "right" : ""];
          const plantNow = vec(contactNow?.plantPosition); const plantBefore = vec(contactBefore?.plantPosition);
          if (contactNow?.state === "PLANTED" && contactBefore?.state === "PLANTED" && plantNow && plantBefore && sample.p && prior.p && Math.hypot(plantNow.x-plantBefore.x, plantNow.z-plantBefore.z) < 0.005) {
            const dt = time - previousFrame.time;
            stat.footSlideMeters += Math.hypot(sample.p.x-prior.p.x, sample.p.z-prior.p.z);
            stat.plantedSeconds += dt;
            stat.plantedIntervals++;
          }
        }
      }
    }
    for (const [id, prior] of previousSamples) if (!current.has(id) && prior.valid) transitions.push({ id, time, event: "missing-sample" });
    for (const [id, item] of current) if (item.valid && !previousSamples.has(id)) transitions.push({ id, time, event: "observation-start" });
    previousFrame = { time, epoch, dropped, envelope, hasValidObservation };
    previousSamples = current;
  }
  const trackers = [...trackerStats.values()].map((stats) => ({
    ...stats,
    meanLinearAcceleration: mean(stats.linearAccelerations),
    maxLinearAcceleration: max(stats.linearAccelerations),
    meanAngularSpeedSecondDerivativeRadiansPerSecondCubed: mean(stats.angularSpeedSecondDerivatives),
    maxAngularSpeedSecondDerivativeRadiansPerSecondCubed: max(stats.angularSpeedSecondDerivatives),
    meanFootSlideMetersPerSecond: stats.plantedSeconds ? stats.footSlideMeters / stats.plantedSeconds : null,
    healthReasons: Object.fromEntries(stats.healthReasons),
    linearAccelerations: undefined,
    angularSpeedSecondDerivatives: undefined,
    previousAngularAcceleration: undefined,
  }));
  const driftDiagnostics = [...driftStats.values()].map((stat) => ({ trackerId: stat.trackerId, diagnosticFrames: stat.count, eligibleFrames: stat.eligibleFrames, holdoverFrames: stat.holdoverFrames, totalAbsoluteBiasChangeRadians: stat.totalAbsoluteBiasChangeRadians, correctionObservedSeconds: stat.correctionObservedSeconds, meanAbsoluteCorrectionRateRadiansPerSecond: stat.correctionObservedSeconds ? stat.totalAbsoluteBiasChangeRadians / stat.correctionObservedSeconds : null, maxCorrectionRateRadiansPerSecond: stat.maxCorrectionRateRadiansPerSecond, meanErrorRadians: mean(stat.errors), meanFilteredErrorRadians: mean(stat.filteredErrors), meanConsistentSeconds: mean(stat.consistentSeconds), reasons: Object.fromEntries(stat.reasons), temperatureModelStatuses: Object.fromEntries(stat.temperatureModelStatuses) }));
  const poseResiduals = [...poseResidualStats.values()].map(stat => ({ trackerId: stat.trackerId, diagnosticFrames: stat.count, meanMagnitudeRadians: stat.sum / stat.count, maxMagnitudeRadians: stat.max, independentlyConstrainedFrames: stat.independentFrames, reasons: Object.fromEntries(stat.reasons) }));
  return { evaluationScope: "recorded-output telemetry; this is not a full solver rerun or raw-packet replay", metricNotes: ["Foot slide is horizontal computed-position distance per second only across adjacent valid PLANTED samples with the same plant position.", "Linear acceleration is derived from changes in recorded derivedLinearVelocity.", "Angular-speed second derivative is based on scalar angular-speed telemetry; it is not a 3D angular jerk vector.", "Yaw correction movement excludes reset epochs, dropped-frame changes, non-forward timestamps, and gaps over 0.5 seconds. It measures applied bias changes, not physical drift accuracy.", "Pose residuals compare recorded solver expectations with measured rotations; the expectations can use the same trackers and do not establish independent tracking error.", "Largest output steps use adjacent valid samples within one reset epoch and at most 0.5 seconds apart. They describe movement, not confirmed pose snaps."], frameCount: recording.frames.length, usableSeconds, comparedIntervals, resetCount, droppedFrameTransitions: recording.frames.reduce((n, f, i, all) => n + (i > 0 && (f.droppedFrames || 0) > (all[i-1].droppedFrames || 0) ? 1 : 0), 0), trackers, driftDiagnostics, poseResiduals, transitions };
}
function mean(values) { return values.length ? values.reduce((sum, value) => sum + value, 0) / values.length : null; }
function max(values) { let result = null; for (const value of values) if (result === null || value > result) result = value; return result; }

function fixtureFrames(scenario) {
  const frames = [];
  const total = scenario === "walking" ? 160 : 100;
  let epoch = 0;
  for (let i = 0; i < total; i++) {
    const t = 1_000_000_000 + i * 20_000_000;
    if (scenario === "disconnect" && i === 45) epoch++;
    const angle = scenario === "turn360" ? 2 * Math.PI * i / (total - 1) : 0;
    const x = scenario === "walking" ? 0.025 * Math.sin(i * Math.PI / 12) : 0;
    const status = scenario === "disconnect" && i >= 45 && i < 58 ? "DISCONNECTED" : "OK";
    const continuous = status === "OK" && !(scenario === "disconnect" && i === 58);
    const foot = { id: 1, name: "left foot", role: "LEFT_FOOT", status, continuousObservation: continuous, position: { x, y: 0, z: 0 }, derivedLinearVelocity: { x: scenario === "walking" ? 0.15 * Math.cos(i*Math.PI/12) : 0, y: 0, z: 0 }, rawRotation: { x: 0, y: Math.sin(angle/2), z: 0, w: Math.cos(angle/2) }, adjustedRotation: { x: 0, y: Math.sin(angle/2), z: 0, w: Math.cos(angle/2) }, angularSpeedRadiansPerSecond: scenario === "turn360" ? 2*Math.PI/(total*0.02) : 0 };
    if (scenario === "disconnect" && i >= 45 && i < 58) foot.position = null;
    const contact = status === "OK" && scenario !== "walking" ? { state: "PLANTED", plantPosition: { x: 0, y: 0, z: 0 }, weight: 0.8 } : { state: "AIRBORNE", plantPosition: null, weight: 0 };
    frames.push({ type: "frame", droppedFrames: 0, frame: { timestampNanos: t, resetEpoch: epoch, samples: [], computedSamples: [foot], footContacts: { left: contact, right: { state: "AIRBORNE", plantPosition: null, weight: 0 } } } });
  }
  return frames;
}
function writeFixtures(directory) {
  fs.mkdirSync(directory, { recursive: true });
  for (const scenario of ["static-standing", "turn360", "walking", "disconnect"]) {
    const name = scenario === "static-standing" ? "static-standing" : scenario;
    const lines = [{ type: "header", schemaVersion: 1, mode: "diagnostics" }, ...fixtureFrames(name === "static-standing" ? "static" : name)].map((value) => JSON.stringify(value));
    fs.writeFileSync(path.join(directory, `${name}.jsonl`), lines.join("\n") + "\n", "utf8");
  }
}

if (require.main === module) {
  try {
    if (process.argv[2] === "--generate-fixtures") { writeFixtures(process.argv[3] || path.join(__dirname, "fixtures")); process.stdout.write("Wrote four deterministic telemetry fixtures.\n"); }
    else if (!process.argv[2]) { process.stderr.write("Usage: node replay.js <recording.jsonl> [more-parts.jsonl ...] | --generate-fixtures [directory]\n"); process.exitCode = 2; }
    else {
      const parts = process.argv.slice(2).map(file => {
        const stat = fs.statSync(file); if (stat.size > MAX_BYTES) throw new Error(`${file} exceeds the ${MAX_BYTES} byte limit`);
        const recording = parseRecording(fs.readFileSync(file, "utf8"));
        return { file, recording };
      }).sort((a, b) => a.recording.frames[0].frame.timestampNanos - b.recording.frames[0].frame.timestampNanos);
      const frames = parts.flatMap(part => part.recording.frames);
      if (frames.length > MAX_FRAMES) throw new Error(`Combined recording exceeds ${MAX_FRAMES} frames`);
      const parseErrors = parts.flatMap(part => part.recording.errors.map(error => `${part.file}: ${error}`));
      process.stdout.write(`${JSON.stringify({ ...evaluate({ frames }), sourceFiles: parts.map(part => part.file), parseErrors }, null, 2)}\n`);
    }
  } catch (error) { process.stderr.write(`${error.message}\n`); process.exitCode = 1; }
}

module.exports = { parseRecording, evaluate, fixtureFrames, writeFixtures };
