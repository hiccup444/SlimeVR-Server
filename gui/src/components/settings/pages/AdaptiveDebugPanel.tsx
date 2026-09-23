import { useEffect, useRef, useState } from 'react';
import { Button } from '@/components/commons/Button';

type RecordValue = Record<string, unknown>;
const object = (value: unknown): RecordValue =>
  value !== null && typeof value === 'object' && !Array.isArray(value)
    ? (value as RecordValue)
    : {};
const numeric = (value: unknown): number | null =>
  typeof value === 'number' && Number.isFinite(value) ? value : null;
const label = (value: unknown) =>
  typeof value === 'string'
    ? value.replaceAll('_', ' ').toLowerCase()
    : 'unavailable';
const degrees = (value: unknown) => {
  const number = numeric(value);
  return number === null
    ? 'unavailable'
    : `${((number * 180) / Math.PI).toFixed(2)}°`;
};
const heading = (value: unknown) => {
  const rotation = object(value);
  const w = numeric(rotation.w);
  const x = numeric(rotation.x);
  const y = numeric(rotation.y);
  const z = numeric(rotation.z);
  if (w === null || x === null || y === null || z === null)
    return 'unavailable';
  const norm = w * w + x * x + y * y + z * z;
  if (!Number.isFinite(norm) || norm < 1e-12) return 'unavailable';
  const horizontalX = (2 * (x * z + w * y)) / norm;
  const horizontalZ = 1 - (2 * (x * x + y * y)) / norm;
  if (
    !Number.isFinite(horizontalX) ||
    !Number.isFinite(horizontalZ) ||
    horizontalX * horizontalX + horizontalZ * horizontalZ < 0.1
  )
    return 'unavailable';
  return `${((Math.atan2(horizontalX, horizontalZ) * 180) / Math.PI).toFixed(1)}°`;
};

const scenarios = [
  {
    id: 'standing',
    name: 'Quiet standing',
    steps:
      'Stand still for up to 30 minutes. Mark any visible drift or yaw reset.',
    expect:
      'Both feet should usually show planted; yaw bias should change slowly or stay still.',
  },
  {
    id: 'turns',
    name: 'Full turns',
    steps:
      'Turn slowly and quickly in both directions, then stop after each turn.',
    expect:
      'Tracking should stay continuous; yaw learning should pause during motion.',
  },
  {
    id: 'walking',
    name: 'Walking and pivots',
    steps: 'Walk at several speeds, pivot, and take quick steps.',
    expect:
      'The moving foot should release promptly; a planted foot should not slide visibly.',
  },
  {
    id: 'poses',
    name: 'Crouch, sit, lie, kneel',
    steps: 'Repeat each transition and try cross-legged sitting.',
    expect:
      'The solver should permit unusual poses without false floor or foot locks.',
  },
  {
    id: 'rapid',
    name: 'Rapid movement',
    steps: 'Dance or change direction repeatedly for several minutes.',
    expect:
      'Corrections should pause under high motion; output should remain responsive.',
  },
  {
    id: 'disturbance',
    name: 'Tracker disturbance',
    steps:
      'Mark the moment you rotate one tracker on its strap, then let it recover.',
    expect:
      'Confidence should fall or learning should pause. Note any pose snap.',
  },
  {
    id: 'disconnect',
    name: 'Disconnect and reconnect',
    steps:
      'Disconnect one tracker briefly, reconnect it, and mark both moments.',
    expect: 'The log should show stale or missing input and a smooth return.',
  },
  {
    id: 'temperature',
    name: 'Warm-up comparison',
    steps:
      'Compare a cold start with a session after the trackers have warmed.',
    expect:
      'Temperature is recorded when available; unsupported yaw roles should not learn bias.',
  },
] as const;

type TestSession = {
  id: string;
  scenario: string;
  condition: string;
  trial: string;
  startedAt: string;
  startedAtMs: number;
};

export function AdaptiveDebugPanel({
  frame,
  connected,
  enabled,
  settings,
  activeFeatures,
  onEnable,
  onStartRecording,
  onStopRecording,
}: {
  frame: RecordValue | null;
  connected: boolean;
  enabled: boolean;
  settings: unknown;
  activeFeatures: string[];
  onEnable: () => void;
  onStartRecording: () => void;
  onStopRecording: () => void;
}) {
  const buffer = useRef<{ line: string; bytes: number; type: string }[]>([]);
  const bytes = useRef(0);
  const removed = useRef(0);
  const lastFrame = useRef<RecordValue | null>(null);
  const lastTimestamp = useRef<string | null>(null);
  const lastFrameReceivedAt = useRef<string | null>(null);
  const lastSnapshotLoggedMs = useRef(0);
  const [history, setHistory] = useState<number[]>([]);
  const [events, setEvents] = useState<string[]>([]);
  const [lastReceived, setLastReceived] = useState(0);
  const [clock, setClock] = useState(Date.now());
  const [note, setNote] = useState('');
  const [exportError, setExportError] = useState('');
  const [scenarioId, setScenarioId] = useState<string>('standing');
  const [condition, setCondition] = useState('Baseline');
  const [trial, setTrial] = useState('1');
  const [session, setSession] = useState<TestSession | null>(null);
  const [stopping, setStopping] = useState(false);
  const sessionRef = useRef<TestSession | null>(null);
  const completedSessionRef = useRef<TestSession | null>(null);
  const completedRecordingFilesRef = useRef<string[]>([]);
  const lastRecordingFilesRef = useRef<string[]>([]);
  const previousState = useRef('');
  const previousRecordingState = useRef('');

  function append(value: unknown) {
    const line = JSON.stringify(value);
    const size = new TextEncoder().encode(line).length + 1;
    if (size > 64 * 1024 * 1024) return;
    buffer.current.push({
      line,
      bytes: size,
      type: String(object(value).type ?? ''),
    });
    bytes.current += size;
    while (bytes.current > 64 * 1024 * 1024 || buffer.current.length > 10000) {
      const oldestFrame = buffer.current.findIndex(
        (entry) => entry.type === 'frame'
      );
      const removedEntry = buffer.current.splice(
        oldestFrame >= 0 ? oldestFrame : 0,
        1
      )[0];
      bytes.current -= removedEntry.bytes;
      removed.current++;
    }
  }

  useEffect(() => {
    const timer = setInterval(() => setClock(Date.now()), 1000);
    return () => clearInterval(timer);
  }, []);

  useEffect(() => {
    append({
      type: 'settings',
      receivedAt: new Date().toISOString(),
      settings,
    });
  }, [settings]);

  useEffect(() => {
    if (!connected || !enabled) {
      lastTimestamp.current = null;
      lastFrame.current = null;
      lastFrameReceivedAt.current = null;
      setLastReceived(0);
    }
    append({
      type: 'connection',
      receivedAt: new Date().toISOString(),
      connected,
      enabled,
    });
  }, [connected, enabled]);

  useEffect(() => {
    if (!frame || !enabled || !connected || lastFrame.current === frame) return;
    lastFrame.current = frame;
    const timestamp = `${String(frame.resetEpoch)}:${String(frame.timestampNanos)}`;
    if (timestamp === lastTimestamp.current) return;
    lastTimestamp.current = timestamp;
    const receivedAt = new Date().toISOString();
    const receivedAtMs = Date.now();
    setLastReceived(receivedAtMs);
    lastFrameReceivedAt.current = receivedAt;
    if (receivedAtMs - lastSnapshotLoggedMs.current >= 5000) {
      append({ type: 'frame', receivedAt, frame });
      lastSnapshotLoggedMs.current = receivedAtMs;
    }
    const samples = Array.isArray(frame.samples)
      ? frame.samples.map(object)
      : [];
    const scores = samples
      .map((sample) => numeric(object(sample.confidence).score))
      .filter((score): score is number => score !== null);
    if (scores.length)
      setHistory((previous) => [...previous.slice(-119), Math.min(...scores)]);
    const contacts = Object.entries(object(frame.footContacts)).map(
      ([side, value]) => `${side}: ${label(object(value).state)}`
    );
    const drift = (
      Array.isArray(frame.driftDiagnostics) ? frame.driftDiagnostics : []
    ).map((value) => {
      const item = object(value);
      return `Tracker ${String(item.trackerId)}: ${label(object(item.residual).reason)}`;
    });
    const problems = samples.flatMap((sample) => {
      const health = object(sample.health);
      const confidence = object(sample.confidence);
      const reasons = [
        ...(Array.isArray(health.reasons) ? health.reasons : []),
        ...(Array.isArray(confidence.reasons) ? confidence.reasons : []),
      ].filter(
        (reason, index, all): reason is string =>
          typeof reason === 'string' &&
          reason !== 'HEALTHY' &&
          reason !== 'INDEPENDENT_CONSTRAINTS_UNAVAILABLE' &&
          all.indexOf(reason) === index
      );
      return reasons.length
        ? [`${String(sample.name)}: ${reasons.map(label).join(', ')}`]
        : [];
    });
    const recovery = object(frame.poseDiagnostic).recoveryTrackerIds;
    const recovering = Array.isArray(recovery)
      ? recovery.map((id) => `Tracker ${String(id)}: pose recovery active`)
      : [];
    const state = [...contacts, ...drift, ...problems, ...recovering].join(
      ' | '
    );
    if (state && state !== previousState.current) {
      previousState.current = state;
      append({
        type: 'state-change',
        receivedAt,
        timestampNanos: frame.timestampNanos,
        state,
      });
      setEvents((previous) =>
        [`${new Date().toLocaleTimeString()} ${state}`, ...previous].slice(
          0,
          30
        )
      );
    }
  }, [frame, enabled, connected]);

  const recording = object(frame?.recording);
  const recordingFiles = Array.isArray(recording.files)
    ? recording.files.filter((file): file is string => typeof file === 'string')
    : [];
  const recordingActive = recording.active === true;
  const recordingRequested =
    object(settings).telemetryEnabled === true ||
    recording.requested === true ||
    recording.active === true ||
    recording.finalizing === true;
  const writtenFrames = numeric(recording.writtenFrames) ?? 0;
  const droppedFrames = numeric(recording.droppedFrames) ?? 0;

  useEffect(() => {
    if (recordingFiles.length > 0)
      lastRecordingFilesRef.current = recordingFiles;
  }, [frame]);

  useEffect(() => {
    if (!frame) return;
    const state = JSON.stringify({
      active: recording.active,
      finalizing: recording.finalizing,
      sizeLimitReached: recording.sizeLimitReached,
      failure: recording.failure,
      files: recordingFiles.length,
    });
    if (state === previousRecordingState.current) return;
    previousRecordingState.current = state;
    const receivedAt = new Date().toISOString();
    append({
      type: 'recording-state',
      receivedAt,
      timestampNanos: frame.timestampNanos,
      recording,
    });
    setEvents((previous) =>
      [
        `${new Date().toLocaleTimeString()} Recording: ${recordingActive ? 'active' : 'stopped'}, ${recordingFiles.length} file(s)`,
        ...previous,
      ].slice(0, 30)
    );
  }, [frame]);

  function startSession() {
    const next: TestSession = {
      id: `${Date.now()}`,
      scenario: scenarioId,
      condition,
      trial: trial.trim() || '1',
      startedAt: new Date().toISOString(),
      startedAtMs: Date.now(),
    };
    buffer.current = [];
    bytes.current = 0;
    removed.current = 0;
    lastSnapshotLoggedMs.current = 0;
    previousState.current = '';
    previousRecordingState.current = '';
    setHistory([]);
    setEvents([]);
    sessionRef.current = next;
    completedSessionRef.current = null;
    completedRecordingFilesRef.current = [];
    lastRecordingFilesRef.current = [];
    setSession(next);
    setStopping(false);
    append({ type: 'session-start', ...next, settings });
    onStartRecording();
  }

  function stopSession() {
    append({
      type: 'session-end',
      receivedAt: new Date().toISOString(),
      timestampNanos: frame?.timestampNanos ?? null,
      sessionId: sessionRef.current?.id,
    });
    onStopRecording();
    setStopping(true);
  }

  useEffect(() => {
    if (!stopping) return;
    if (
      connected &&
      (!frame ||
        recording.requested === true ||
        recording.active === true ||
        recording.finalizing === true)
    )
      return;
    completedSessionRef.current = sessionRef.current;
    completedRecordingFilesRef.current =
      recordingFiles.length > 0
        ? recordingFiles
        : lastRecordingFilesRef.current;
    sessionRef.current = null;
    download();
    setSession(null);
    setStopping(false);
  }, [frame, stopping, connected]);

  function mark() {
    const message = note.trim() || 'Visible tracking problem';
    const snapshotReceivedAt =
      lastFrame.current === frame ? lastFrameReceivedAt.current : null;
    if (frame && snapshotReceivedAt) {
      append({
        type: 'frame',
        receivedAt: snapshotReceivedAt,
        frame,
      });
    }
    append({
      type: 'marker',
      receivedAt: new Date().toISOString(),
      timestampNanos: frame?.timestampNanos ?? null,
      resetEpoch: frame?.resetEpoch ?? null,
      sessionId: sessionRef.current?.id ?? null,
      message,
    });
    setEvents((previous) =>
      [
        `${new Date().toLocaleTimeString()} MARK: ${message}`,
        ...previous,
      ].slice(0, 30)
    );
    setNote('');
  }

  function download() {
    try {
      const exportedSession = sessionRef.current ?? completedSessionRef.current;
      const header = JSON.stringify({
        type: 'header',
        schemaVersion: 1,
        mode: 'live-debug-snapshots',
        build: __COMMIT_HASH__,
        modifiedBuild: !__GIT_CLEAN__,
        exportedAt: new Date().toISOString(),
        omittedEntries: removed.current,
        session: exportedSession,
        serverRecordingFiles:
          sessionRef.current || !completedSessionRef.current
            ? recordingFiles.length > 0
              ? recordingFiles
              : lastRecordingFilesRef.current
            : completedRecordingFilesRef.current,
        settings,
        note: 'UI diagnostics sampled every five seconds and at each marker. Send this log together with every server recording part. Neither contains raw IMU packets.',
      });
      const url = URL.createObjectURL(
        new Blob(
          [header, '\n', ...buffer.current.map((entry) => entry.line + '\n')],
          { type: 'application/x-ndjson' }
        )
      );
      const anchor = document.createElement('a');
      anchor.href = url;
      anchor.download = `slimevr-test-${exportedSession?.scenario ?? 'debug'}-${new Date().toISOString().replaceAll(':', '-')}.jsonl`;
      document.body.appendChild(anchor);
      anchor.click();
      anchor.remove();
      setTimeout(() => URL.revokeObjectURL(url), 10000);
      setExportError('');
    } catch {
      setExportError('Could not export the debug log. Try again.');
    }
  }

  const stale = !lastReceived || clock - lastReceived > 3000;
  const samples =
    frame && Array.isArray(frame.samples) ? frame.samples.map(object) : [];
  const drift =
    frame && Array.isArray(frame.driftDiagnostics)
      ? frame.driftDiagnostics.map(object)
      : [];
  const activity = object(frame?.activity);
  const solver = object(frame?.poseDiagnostic);
  const recoveryIds = Array.isArray(solver.recoveryTrackerIds)
    ? solver.recoveryTrackerIds
    : [];
  const solverMilliseconds = numeric(solver.processingNanos);
  const scenario =
    scenarios.find((item) => item.id === scenarioId) ?? scenarios[0];
  return (
    <section
      className="flex flex-col gap-3 rounded-lg bg-background-60 p-4 text-background-10"
      aria-label="Live tracking debug"
    >
      <h2 className="text-xl font-bold">Tracking test and debug</h2>
      <p role="status">
        {!connected
          ? 'Disconnected from server'
          : !enabled
            ? 'Debug view is off'
            : stale
              ? 'Waiting for fresh tracking data'
              : `Live · ${samples.length} trackers · updating up to 4 times per second`}
      </p>
      <div className="grid gap-2 sm:grid-cols-3">
        <label className="flex flex-col gap-1 text-sm">
          Movement
          <select
            className="rounded bg-background-80 p-2"
            value={scenarioId}
            disabled={session !== null}
            onChange={(event) => setScenarioId(event.target.value)}
          >
            {scenarios.map((item) => (
              <option key={item.id} value={item.id}>
                {item.name}
              </option>
            ))}
          </select>
        </label>
        <label className="flex flex-col gap-1 text-sm">
          Comparison
          <select
            className="rounded bg-background-80 p-2"
            value={condition}
            disabled={session !== null}
            onChange={(event) => setCondition(event.target.value)}
          >
            <option>Baseline</option>
            <option>Feature on</option>
          </select>
        </label>
        <label className="flex flex-col gap-1 text-sm">
          Trial number
          <input
            className="rounded bg-background-80 p-2"
            value={trial}
            maxLength={20}
            disabled={session !== null}
            onChange={(event) => setTrial(event.target.value)}
          />
        </label>
      </div>
      <p className="text-sm">
        Do: {scenario.steps} Expected: {scenario.expect}
      </p>
      <p className="text-sm">
        The comparison label names the log. Choose and save correction settings
        below before a feature-on trial.
      </p>
      <p className="text-sm">
        Confirmed adaptive features:{' '}
        {settings === null
          ? 'loading settings'
          : activeFeatures.length
            ? activeFeatures.join(', ')
            : 'none'}
      </p>
      {settings !== null &&
        condition === 'Baseline' &&
        activeFeatures.length > 0 && (
          <p role="alert">
            Baseline is selected while adaptive features are on. Save them off
            below for a clean comparison.
          </p>
        )}
      {settings !== null &&
        condition === 'Feature on' &&
        activeFeatures.length === 0 && (
          <p role="alert">
            Feature on is selected, but no adaptive feature is enabled yet.
          </p>
        )}
      <div className="flex flex-wrap gap-2">
        <Button
          variant="primary"
          disabled={!connected || enabled}
          onClick={onEnable}
        >
          Enable live debug
        </Button>
        <Button
          variant="primary"
          disabled={
            !connected ||
            settings === null ||
            session !== null ||
            recordingRequested
          }
          onClick={startSession}
        >
          Start recorded test
        </Button>
        <Button
          variant="secondary"
          disabled={session === null || stopping}
          onClick={stopSession}
        >
          {stopping ? 'Finalizing recording' : 'Stop and export notes'}
        </Button>
        <Button variant="secondary" onClick={download}>
          Export notes again
        </Button>
      </div>
      {session === null && recordingRequested && (
        <p role="alert" className="text-sm">
          A server recording is running or finalizing. Stop it in the settings
          below and wait for it to finish before starting a separate test trial.
        </p>
      )}
      <p className="text-sm">
        A recorded test saves server telemetry at the selected sample rate and a
        separate UI note log. The UI log holds up to 64 MiB or 10,000 entries
        while this page is open; its header reports omitted entries. Send the
        exported notes and every server recording file together.
      </p>
      <p role="status" className="text-sm">
        {session
          ? `${session.condition} · ${scenario.name} · trial ${session.trial} · ${Math.floor((clock - session.startedAtMs) / 1000)} seconds · `
          : ''}
        {recordingActive
          ? `Server recording: ${writtenFrames} frames in ${recordingFiles.length} file(s), ${droppedFrames} dropped`
          : recordingRequested
            ? 'Server recording is starting or finalizing'
            : 'Server recording is off'}
      </p>
      {recording.sizeLimitReached === true && (
        <p role="alert">
          Recording reached its file limit. Stop and export; this test is
          incomplete.
        </p>
      )}
      {typeof recording.failure === 'string' && (
        <p role="alert">Server recording failed: {recording.failure}</p>
      )}
      {recordingFiles.length > 0 && (
        <div className="text-sm">
          <p>Server files to keep:</p>
          <ul className="list-disc pl-5">
            {recordingFiles.map((file) => (
              <li key={file}>{file}</li>
            ))}
          </ul>
          {typeof window.electronAPI !== 'undefined' && (
            <Button
              variant="secondary"
              onClick={() =>
                window.electronAPI.openAdaptiveRecording(recordingFiles[0])
              }
            >
              Show recording in folder
            </Button>
          )}
        </div>
      )}
      {exportError && <p role="alert">{exportError}</p>}
      <div className="flex flex-wrap gap-2">
        <input
          aria-label="Debug marker note"
          className="min-w-0 flex-1 rounded bg-background-80 p-2"
          maxLength={300}
          placeholder="What did you notice? e.g. left foot stuck while stepping"
          value={note}
          onChange={(event) => setNote(event.target.value)}
        />
        <Button
          variant="secondary"
          disabled={!enabled || !connected || stale}
          onClick={mark}
        >
          Mark this moment
        </Button>
      </div>
      {enabled && frame && (
        <div className={stale || !connected ? 'opacity-50' : ''}>
          <p className="text-sm">
            Activity: {label(activity.state)} · optimizer time:{' '}
            {solverMilliseconds === null
              ? 'unavailable'
              : `${(solverMilliseconds / 1e6).toFixed(2)} ms`}
          </p>
          <p>Lowest tracker confidence over the last 120 samples</p>
          <svg
            viewBox="0 0 400 65"
            className="h-20 w-full"
            role="img"
            aria-label="Lowest tracker confidence history from zero to one"
          >
            <line
              x1="0"
              y1="60"
              x2="400"
              y2="60"
              stroke="currentColor"
              opacity="0.3"
            />
            <polyline
              fill="none"
              stroke="#60a5fa"
              strokeWidth="2"
              points={history
                .map(
                  (score, index) =>
                    `${(index * 400) / 119},${60 - Math.max(0, Math.min(1, score)) * 55}`
                )
                .join(' ')}
            />
          </svg>
          <details className="my-2">
            <summary>Tracker health and freshness ({samples.length})</summary>
            <div className="grid gap-2 py-2 sm:grid-cols-2">
              {samples.map((sample) => {
                const confidence = object(sample.confidence);
                const age = numeric(sample.packetAgeNanos);
                const temperature = numeric(sample.temperatureCelsius);
                const health = object(sample.health);
                const prediction = object(
                  object(frame.trackerPredictions)[String(sample.id)]
                );
                const residual = object(prediction.residual);
                const healthReasons = Array.isArray(health.reasons)
                  ? health.reasons.filter(
                      (reason): reason is string =>
                        typeof reason === 'string' && reason !== 'HEALTHY'
                    )
                  : [];
                const confidenceReasons = Array.isArray(confidence.reasons)
                  ? confidence.reasons.filter(
                      (reason): reason is string =>
                        typeof reason === 'string' &&
                        reason !== 'INDEPENDENT_CONSTRAINTS_UNAVAILABLE' &&
                        !healthReasons.includes(reason)
                    )
                  : [];
                return (
                  <div
                    key={String(sample.id)}
                    className="rounded bg-background-80 p-2 text-sm"
                  >
                    <strong>
                      {String(sample.name ?? sample.role ?? sample.id)}
                    </strong>{' '}
                    · {label(sample.status)}
                    <p>
                      Confidence:{' '}
                      {numeric(confidence.score)?.toFixed(2) ?? 'unavailable'} ·
                      packet age:{' '}
                      {age === null
                        ? 'unavailable'
                        : `${(age / 1e6).toFixed(0)} ms`}{' '}
                      · temperature:{' '}
                      {temperature === null
                        ? 'unavailable'
                        : `${temperature.toFixed(1)} °C`}
                    </p>
                    <p>
                      Stationary confidence:{' '}
                      {numeric(health.stationaryConfidence)?.toFixed(2) ??
                        'unavailable'}
                    </p>
                    {health.suspectedFrozen === true && (
                      <p>Suspected frozen orientation</p>
                    )}
                    {recoveryIds.includes(sample.id) && (
                      <p>Pose recovery active</p>
                    )}
                    {healthReasons.length > 0 && (
                      <p>Health: {healthReasons.map(label).join(', ')}</p>
                    )}
                    {confidenceReasons.length > 0 && (
                      <p>Quality: {confidenceReasons.map(label).join(', ')}</p>
                    )}
                    {prediction.expectedRotation !== undefined && (
                      <details className="mt-1">
                        <summary>Orientation and pose residual</summary>
                        <p>Raw sensor heading: {heading(sample.rawRotation)}</p>
                        <p>
                          Mounted input heading:{' '}
                          {heading(sample.adjustedRotation)}
                        </p>
                        <p>
                          Pose input heading:{' '}
                          {heading(prediction.measuredRotation)}
                        </p>
                        <p>
                          Solver expected heading:{' '}
                          {heading(prediction.expectedRotation)}
                        </p>
                        <p>
                          Orientation difference:{' '}
                          {degrees(residual.magnitudeRadians)}
                        </p>
                        <p>
                          Difference axes (x, y, z):{' '}
                          {['x', 'y', 'z']
                            .map((axis) =>
                              degrees(
                                object(residual.residualVectorRadians)[axis]
                              )
                            )
                            .join(', ')}
                        </p>
                        <p>
                          Raw heading uses sensor axes. Expected heading is
                          solver inferred and may include this tracker’s own
                          input.
                        </p>
                      </details>
                    )}
                  </div>
                );
              })}
            </div>
          </details>
          <div className="grid gap-2 sm:grid-cols-2">
            {Object.entries(object(frame.footContacts)).map(
              ([side, contact]) => (
                <div key={side} className="rounded bg-background-80 p-2">
                  {side} foot: <strong>{label(object(contact).state)}</strong>
                </div>
              )
            )}
            {drift.map((item) => (
              <div
                key={String(item.trackerId)}
                className="rounded bg-background-80 p-2"
              >
                <strong>
                  {String(
                    samples.find((sample) => sample.id === item.trackerId)
                      ?.name ?? `Tracker ${String(item.trackerId)}`
                  )}
                </strong>
                <p>
                  Yaw bias: {degrees(item.biasRadians)} · residual:{' '}
                  {degrees(object(item.residual).errorRadians)}
                </p>
                <p>{label(object(item.residual).reason)}</p>
                {item.holdoverActive === true && (
                  <p className="text-sm">
                    Learned temperature drift rate is carrying this correction
                    while fresh evidence is unavailable.
                  </p>
                )}
                {numeric(item.predictedRateRadiansPerSecond) !== null && (
                  <p className="text-sm">
                    Learned drift rate:{' '}
                    {(
                      (Number(item.predictedRateRadiansPerSecond) * 180 * 60) /
                      Math.PI
                    ).toFixed(3)}{' '}
                    °/min
                  </p>
                )}
                <p className="text-sm">
                  Temperature model: {label(item.temperatureModelStatus)}
                </p>
                {numeric(item.historicalConfidenceMultiplier) !== null &&
                  Number(item.historicalConfidenceMultiplier) < 1 && (
                    <p className="text-sm">
                      Learned reliability weight:{' '}
                      {Number(item.historicalConfidenceMultiplier).toFixed(2)}
                    </p>
                  )}
                <p className="text-sm">
                  Learning:{' '}
                  {object(item.residual).eligibleForLearning === true
                    ? 'eligible'
                    : 'paused'}
                  {' · '}Pose confidence:{' '}
                  {numeric(object(item.poseConfidence).score)?.toFixed(2) ??
                    'unavailable'}
                </p>
                {Array.isArray(object(item.poseConfidence).reasons) && (
                  <p className="text-sm">
                    Evidence:{' '}
                    {(object(item.poseConfidence).reasons as unknown[])
                      .map(label)
                      .join(', ')}
                  </p>
                )}
                <p className="text-sm">
                  Mode: {label(item.correctionMode)} · learning duty:{' '}
                  {numeric(object(item.learning).learningDutyCycle) === null
                    ? 'unavailable'
                    : `${(Number(object(item.learning).learningDutyCycle) * 100).toFixed(1)}%`}
                </p>
                <p className="text-sm">
                  Context restarts:{' '}
                  {String(object(item.learning).contextRestarts ?? 0)}
                  {item.correctionMode === 'PLANTED_REFERENCE_INCREMENTAL_ONLY'
                    ? ' · Resists new drift during contact; does not establish absolute foot heading.'
                    : ''}
                </p>
              </div>
            ))}
          </div>
          <p className="mt-2 text-sm">
            Confidence describes measurement quality, not physical pose
            accuracy. Bias is the current correction, not a correction rate.
          </p>
        </div>
      )}
      <details>
        <summary>Recent state changes and markers ({events.length})</summary>
        <ol className="max-h-48 overflow-auto text-sm">
          {events.map((event, index) => (
            <li key={index} className="py-1">
              {event}
            </li>
          ))}
        </ol>
      </details>
    </section>
  );
}
