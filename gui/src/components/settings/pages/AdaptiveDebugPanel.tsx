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

export function AdaptiveDebugPanel({
  frame,
  connected,
  enabled,
  settings,
  onEnable,
}: {
  frame: RecordValue | null;
  connected: boolean;
  enabled: boolean;
  settings: unknown;
  onEnable: () => void;
}) {
  const buffer = useRef<{ line: string; bytes: number }[]>([]);
  const bytes = useRef(0);
  const removed = useRef(0);
  const lastFrame = useRef<RecordValue | null>(null);
  const lastTimestamp = useRef<string | null>(null);
  const [history, setHistory] = useState<number[]>([]);
  const [events, setEvents] = useState<string[]>([]);
  const [lastReceived, setLastReceived] = useState(0);
  const [clock, setClock] = useState(Date.now());
  const [note, setNote] = useState('');
  const [exportError, setExportError] = useState('');
  const previousState = useRef('');

  function append(value: unknown) {
    const line = JSON.stringify(value);
    const size = new TextEncoder().encode(line).length + 1;
    if (size > 16 * 1024 * 1024) return;
    buffer.current.push({ line, bytes: size });
    bytes.current += size;
    while (bytes.current > 16 * 1024 * 1024 || buffer.current.length > 2400) {
      bytes.current -= buffer.current.shift()!.bytes;
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
    setLastReceived(Date.now());
    append({ type: 'frame', receivedAt, droppedFrames: 0, frame, settings });
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
      const reasons = Array.isArray(health.reasons)
        ? health.reasons.filter((reason) => reason !== 'HEALTHY')
        : [];
      return reasons.length
        ? [`${String(sample.name)}: ${reasons.map(label).join(', ')}`]
        : [];
    });
    const state = [...contacts, ...drift, ...problems].join(' | ');
    if (state && state !== previousState.current) {
      previousState.current = state;
      setEvents((previous) =>
        [`${new Date().toLocaleTimeString()} ${state}`, ...previous].slice(
          0,
          30
        )
      );
    }
  }, [frame, enabled, connected, settings]);

  function mark() {
    const message = note.trim() || 'Visible tracking problem';
    append({
      type: 'marker',
      receivedAt: new Date().toISOString(),
      timestampNanos: frame?.timestampNanos ?? null,
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
      const header = JSON.stringify({
        type: 'header',
        schemaVersion: 1,
        mode: 'live-debug-snapshots',
        build: __COMMIT_HASH__,
        modifiedBuild: !__GIT_CLEAN__,
        exportedAt: new Date().toISOString(),
        omittedEntries: removed.current,
        settings,
        note: 'Sampled UI diagnostics at up to 4 Hz. Not raw packets or full-rate telemetry.',
      });
      const url = URL.createObjectURL(
        new Blob(
          [header, '\n', ...buffer.current.map((entry) => entry.line + '\n')],
          { type: 'application/x-ndjson' }
        )
      );
      const anchor = document.createElement('a');
      anchor.href = url;
      anchor.download = `slimevr-debug-${new Date().toISOString().replaceAll(':', '-')}.jsonl`;
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
  return (
    <section
      className="flex flex-col gap-3 rounded-lg bg-background-60 p-4 text-background-10"
      aria-label="Live tracking debug"
    >
      <h2 className="text-xl font-bold">Live tracking debug</h2>
      <p role="status">
        {!connected
          ? 'Disconnected from server'
          : !enabled
            ? 'Debug view is off'
            : stale
              ? 'Waiting for fresh tracking data'
              : `Live · ${samples.length} trackers · updating up to 4 times per second`}
      </p>
      <div className="flex flex-wrap gap-2">
        <Button
          variant="primary"
          disabled={!connected || enabled}
          onClick={onEnable}
        >
          Enable live debug
        </Button>
        <Button variant="secondary" onClick={download}>
          Export debug log
        </Button>
      </div>
      <p className="text-sm">
        The log keeps recent snapshots and your markers while this page is open,
        up to 16 MiB. Export before leaving this page. Older entries are dropped
        when full. Enable Record adaptive telemetry below for a separate
        full-rate server recording.
      </p>
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
