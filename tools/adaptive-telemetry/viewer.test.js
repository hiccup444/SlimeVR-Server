const assert = require("node:assert/strict");
const fs = require("node:fs");
const vm = require("node:vm");

const html = fs.readFileSync(__dirname + "/viewer.html", "utf8");
const script = html.match(/<script>([\s\S]*?)<\/script>/)[1];
function element() {
  return {
    value: "", textContent: "", className: "", disabled: false, options: [], children: [], firstChild: null,
    addEventListener() {}, getContext() { return { clearRect() {}, fillRect() {}, beginPath() {}, moveTo() {}, lineTo() {}, stroke() {}, fillText() {}, arc() {}, fill() {} }; },
    appendChild(child) { this.options.push(child); this.children.push(child); this.firstChild = child; }, remove() { this.options.shift(); this.firstChild = this.options[0] || null; }, removeChild(child) { const index=this.children.indexOf(child); if(index>=0)this.children.splice(index,1); this.firstChild=this.children[0]||null; },
  };
}
const elements = new Map(["file", "frame", "frameLabel", "tracker", "play", "message", "meta", "plot", "rows", "contacts", "driftRows", "predictionRows", "poseDiagnostic", "activityDiagnostic", "armConfidenceDiagnostic", "posePlot", "poseProjection"].map((id) => [id, element()]));
const context = { document: { getElementById: (id) => elements.get(id), createElement: () => element() }, window: {}, console, setTimeout, clearTimeout };
vm.runInNewContext(script, context, { filename: "viewer.html" });
const normalize = context.window.__adaptiveTelemetryTest.normalizeFrames;
const renderDiagnostics = context.window.__adaptiveTelemetryTest.renderDiagnostics;
const projectPoint = context.window.__adaptiveTelemetryTest.projectPoint;
const sample = (id, name, angular, velocity, status = "OK", confidence) => ({
  id, name, status, rawRotation: { x: 0, y: 0, z: 0, w: 1 }, adjustedRotation: { x: 0, y: 0, z: 0, w: 1 },
  derivedLinearVelocity: velocity, temperatureCelsius: 25, angularSpeedRadiansPerSecond: angular, confidence,
});
const frames = normalize([
  { type: "frame", droppedFrames: 0, frame: { timestampNanos: 1_000_000_000, resetEpoch: 0, footContacts: { left: { state: "PLANTED", plantPosition: { x: 1, y: 0, z: 2 }, weight: 0.8, speedMetersPerSecond: 0.01, angularSpeedRadiansPerSecond: 0.02 }, right: { state: "CONTACT_CANDIDATE", plantPosition: null, weight: 0, speedMetersPerSecond: null, angularSpeedRadiansPerSecond: null } }, driftDiagnostics: [{ trackerId: 7, biasRadians: 0.1, residual: { errorRadians: 0.2, filteredErrorRadians: 0.15, consistentSeconds: 2, eligibleForLearning: true, reason: "steady <test>" }, poseConfidence: { score: 0.9, reasons: ["independent chain"], learningEligible: true } }], trackerPredictions: { "7": { expectedRotation: { x: 0, y: 0, z: 0, w: 1 }, measuredRotation: { x: 0, y: 0.01, z: 0, w: 0.99995 }, residual: { trackerId: 7, residualVectorRadians: { x: 0, y: 0.02, z: 0 }, magnitudeRadians: 0.02, directionalConsistency: 0.9, continuousSeconds: 3, meanMagnitudeRadians: 0.018, magnitudeVarianceRadiansSquared: 0.001, sampleCount: 5, independentlyConstrained: false, reason: "SOLVER_CONDITIONED_DIAGNOSTIC" } }, "8": null }, activity: { state: "STANDING", confidence: 0.88, reason: "stable upright", candidateSeconds: 4 }, armCalibration: { sides: [{ side: "left", poseConfidence: { score: 0.92, reasons: ["controllers stable"], learningEligible: true } }] }, poseDiagnostic: { initialError: 0.5, finalError: 0.2, processingNanos: 1200, measurementConfidence: 0.81 }, rawPose: { hip: { x: 0, y: 1, z: 0 } }, predictedPose: { hip: { x: 0, y: 0.9, z: 0 } }, samples: [sample(7, "left", 2, { x: 3, y: 4, z: 0 }, "OK", { score: 0.75, reasons: ["steady", "safe <tag> text"], observationSeconds: 4.5 })], computedSamples: [] } },
  { type: "frame", droppedFrames: 2, frame: { timestampNanos: 1_100_000_000, resetEpoch: 0, samples: [], computedSamples: [] } },
  { type: "frame", droppedFrames: 2, frame: { timestampNanos: 1_200_000_000, resetEpoch: 1, samples: [sample(7, "left", null, null, "RESET", { score: 0, reasons: [], observationSeconds: 0 })], computedSamples: [], poseDiagnostic: { globalConfidence: 0.44 } } },
]);
assert.equal(frames.length, 3);
assert.equal(frames[0].trackers[0].angularSpeed, 2);
assert.equal(frames[0].trackers[0].linearSpeed, 5);
assert.equal(frames[0].footContacts.left.state, "PLANTED");
assert.equal(frames[0].footContacts.left.weight, 0.8);
assert.equal(frames[0].footContacts.right.weight, 0);
assert.equal(frames[0].driftDiagnostics[0].residual.filteredErrorRadians, 0.15);
assert.equal(frames[0].poseDiagnostic.measurementConfidence, 0.81);
assert.equal(frames[0].trackerPredictions["7"].residual.magnitudeRadians, 0.02);
assert.equal(frames[0].trackerPredictions["8"], null);
assert.equal(frames[0].rawPose.hip.y, 1);
const yawTurn90 = { x: 0, y: 0, z: -1 };
assert.equal(projectPoint(yawTurn90, "front").y, 0);
assert.equal(projectPoint(yawTurn90, "top").y, -1);
renderDiagnostics(frames[0]);
assert.equal(elements.get("driftRows").children[0].children[6].textContent, "steady <test>");
assert.equal(elements.get("driftRows").children[0].children[4].textContent, "2.0");
assert.equal(elements.get("driftRows").children[0].children[5].textContent, "yes");
assert.equal(elements.get("poseDiagnostic").textContent.includes("Processing: 1200 ns"), true);
assert.equal(elements.get("poseDiagnostic").textContent.includes("Measurement confidence: 0.810"), true);
assert.equal(elements.get("predictionRows").children.length, 1);
assert.equal(elements.get("predictionRows").children[0].children[3].textContent, "(0.000, 0.020, 0.000)");
assert.equal(elements.get("predictionRows").children[0].children[7].textContent, "no");
assert.equal(elements.get("predictionRows").children[0].children[8].textContent, "SOLVER_CONDITIONED_DIAGNOSTIC");
assert.equal(elements.get("driftRows").children[0].children[7].textContent, "0.900");
assert.equal(elements.get("driftRows").children[0].children[8].textContent, "eligible");
assert.equal(elements.get("driftRows").children[0].children[9].textContent, "independent chain");
assert.equal(elements.get("activityDiagnostic").textContent.includes("STANDING"), true);
assert.equal(elements.get("armConfidenceDiagnostic").textContent.includes("controllers stable"), true);
assert.equal(frames[0].trackers[0].confidence.score, 0.75);
assert.deepEqual(frames[0].trackers[0].confidence.reasons, ["steady", "safe <tag> text"]);
assert.equal(frames[1].trackers.length, 0);
assert.equal(frames[1].footContacts, null);
assert.equal(frames[2].resetEpoch, 1);
assert.equal(frames[2].trackers[0].angularSpeed, null);
assert.equal(frames[2].trackers[0].linearSpeed, null);
assert.equal(frames[2].trackers[0].confidence.score, 0);
renderDiagnostics(frames[2]);
assert.equal(elements.get("poseDiagnostic").textContent.includes("Legacy measurement confidence: 0.440"), true);
assert.equal(elements.get("predictionRows").children.length, 0);
assert.equal(normalize([{ type: "frame", frame: { timestampNanos: 2, resetEpoch: 0, samples: [sample(8, "unknown", null, null)], computedSamples: [] } }])[0].trackers[0].confidence, null);
console.log("viewer telemetry normalization tests passed");
