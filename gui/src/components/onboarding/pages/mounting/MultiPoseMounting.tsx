import { useEffect, useRef, useState } from 'react';
import {
  MultiPoseMountingRequestT,
  MultiPoseMountingResponseT,
  RpcMessage,
  ResetType,
} from 'solarxr-protocol';
import { Button } from '@/components/commons/Button';
import { Typography } from '@/components/commons/Typography';
import { ResetButton } from '@/components/home/ResetButton';
import { useWebsocketAPI } from '@/hooks/websocket-api';
import { useOnboarding } from '@/hooks/onboarding';

type TrackerResult = {
  id: number;
  name: string;
  role: string;
  eligible: boolean;
  yawChangeDeg: number | null;
  repeatErrorDeg: number;
  movementDeg: number;
  reason: string;
};
type CalibrationState = {
  phase: string;
  step: number;
  completed: number[];
  error: string;
  applied: boolean;
  stableTrackers: number;
  totalTrackers: number;
  trackers: TrackerResult[];
};

const poses = [
  {
    title: 'Ski pose',
    instruction:
      'Face a fixed object. Hold the familiar ski pose with your feet planted. Keep your head facing forward.',
  },
  {
    title: 'Comfortable upright',
    instruction:
      'Stand naturally with feet flat and arms relaxed. Keep your knees comfortable; do not force them straight.',
  },
  {
    title: 'Shallow bend, arms out',
    instruction:
      'Make a small supported knee bend with both feet flat. Lift your arms out to the sides and keep your torso comfortable.',
  },
  {
    title: 'Repeat ski pose',
    instruction:
      'Return to the same ski pose and facing direction. This checks whether each tracker gives the same answer again.',
  },
];

export function MultiPoseMountingPage() {
  const { state: onboarding } = useOnboarding();
  const { isConnected, sendRPCPacket, useRPCPacket } = useWebsocketAPI();
  const sendRef = useRef(sendRPCPacket);
  sendRef.current = sendRPCPacket;
  const [resetDone, setResetDone] = useState(false);
  const [calibration, setCalibration] = useState<CalibrationState | null>(null);
  const [connectionError, setConnectionError] = useState('');

  const send = (command: string) => {
    const request = new MultiPoseMountingRequestT();
    request.command = command;
    sendRef.current(RpcMessage.MultiPoseMountingRequest, request);
  };

  useRPCPacket(
    RpcMessage.MultiPoseMountingResponse,
    (response: MultiPoseMountingResponseT) => {
      try {
        const json =
          typeof response.stateJson === 'string'
            ? response.stateJson
            : new TextDecoder().decode(response.stateJson || new Uint8Array());
        setCalibration(JSON.parse(json) as CalibrationState);
        setConnectionError('');
      } catch {
        setConnectionError('Could not read the calibration result.');
      }
    }
  );

  useEffect(() => {
    if (!isConnected) return;
    const poll = () => {
      const request = new MultiPoseMountingRequestT();
      request.command = 'status';
      sendRef.current(RpcMessage.MultiPoseMountingRequest, request);
    };
    poll();
    const timer = setInterval(poll, 300);
    return () => clearInterval(timer);
  }, [isConnected]);

  const next = calibration?.completed.length ?? 0;
  const ready =
    calibration?.phase === 'ready' || calibration?.phase === 'captured';
  const reviewing = calibration?.phase === 'review';
  const applied = calibration?.phase === 'applied';
  const eligible =
    calibration?.trackers.filter((tracker) => tracker.eligible).length ?? 0;

  return (
    <div className="flex flex-col gap-5 w-full h-full overflow-y-auto px-4 pb-8 items-center">
      <div className="flex flex-col gap-4 w-full max-w-3xl">
        <Typography variant="main-title">
          Guided mounting calibration
        </Typography>
        <Typography>
          Start with the existing ski pose, then use two other poses and a
          repeated ski pose to check each tracker. Changes are shown before they
          are applied.
        </Typography>
        {!isConnected && (
          <Typography>Waiting for the server connection.</Typography>
        )}
        {connectionError && <Typography>{connectionError}</Typography>}
        {!resetDone && calibration?.phase !== 'review' && !applied && (
          <div className="rounded-lg bg-background-70 p-4 flex flex-col gap-3">
            <Typography variant="section-title">1. Full reset</Typography>
            <Typography>
              Stand in the normal full reset pose, then perform a full reset.
              Keep all straps in place through the remaining steps.
            </Typography>
            <div className="self-start">
              <ResetButton
                type={ResetType.Full}
                onReseted={() => setResetDone(true)}
              />
            </div>
          </div>
        )}
        {resetDone &&
          (!calibration ||
            calibration.phase === 'idle' ||
            calibration.phase === 'invalid') && (
            <Button
              variant="primary"
              className="self-start"
              onClick={() => send('start')}
            >
              Start pose checks
            </Button>
          )}
        {calibration &&
          !['idle', 'invalid'].includes(calibration.phase) &&
          next < poses.length && (
            <div className="rounded-lg bg-background-70 p-4 flex flex-col gap-4">
              <Typography variant="section-title">
                {next + 1} of {poses.length}: {poses[next].title}
              </Typography>
              <Typography>{poses[next].instruction}</Typography>
              {next === 0 && (
                <img
                  src="/images/mounting-reset-pose.webp"
                  width={400}
                  alt="Ski pose"
                />
              )}
              <Typography>
                {calibration.phase === 'capturing'
                  ? `Hold still while fresh tracker readings are collected (${calibration.stableTrackers}/${calibration.totalTrackers} stable).`
                  : 'Move into the pose, then press Capture. Hold still until the step completes.'}
              </Typography>
              <Button
                variant="primary"
                className="self-start"
                disabled={!ready || !resetDone}
                onClick={() => send(`capture:${next}`)}
              >
                {calibration.phase === 'capturing'
                  ? 'Capturing...'
                  : 'Capture this pose'}
              </Button>
            </div>
          )}
        {calibration?.error && (
          <div className="rounded-lg bg-status-critical p-3">
            <Typography>{calibration.error}</Typography>
          </div>
        )}
        {(reviewing || applied || calibration?.phase === 'undone') && (
          <div className="rounded-lg bg-background-70 p-4 flex flex-col gap-4">
            <Typography variant="section-title">Tracker review</Typography>
            <Typography>
              {eligible} of {calibration.trackers.length} trackers have
              repeatable evidence. Unresolved trackers keep their existing
              mounting.
            </Typography>
            <div className="flex flex-col gap-2 max-h-80 overflow-y-auto">
              {calibration.trackers.map((tracker) => (
                <div key={tracker.id} className="rounded bg-background-60 p-2">
                  <Typography bold>
                    {tracker.role?.replaceAll('_', ' ') || tracker.name}:{' '}
                    {tracker.eligible ? 'Ready' : 'Needs retry'}
                  </Typography>
                  <Typography>{tracker.reason}</Typography>
                  <Typography>
                    Ski repeat difference: {tracker.repeatErrorDeg.toFixed(1)}°;
                    pose movement: {tracker.movementDeg.toFixed(1)}°
                  </Typography>
                  {tracker.yawChangeDeg !== null && (
                    <Typography>
                      Proposed mounting yaw change:{' '}
                      {tracker.yawChangeDeg.toFixed(1)}°
                    </Typography>
                  )}
                </div>
              ))}
            </div>
            <div className="flex gap-2 flex-wrap">
              {reviewing && eligible > 0 && (
                <Button variant="primary" onClick={() => send('apply')}>
                  Apply ready trackers
                </Button>
              )}
              {applied && (
                <Button variant="secondary" onClick={() => send('undo')}>
                  Undo changes
                </Button>
              )}
              {applied && !onboarding.alonePage && (
                <Button
                  variant="primary"
                  to="/onboarding/body-proportions/scaled"
                >
                  Continue setup
                </Button>
              )}
              <Button
                variant="secondary"
                onClick={() => {
                  setResetDone(false);
                  send('start');
                }}
              >
                Retry all poses
              </Button>
            </div>
          </div>
        )}
        <Button
          variant="tertiary"
          className="self-start"
          to={onboarding.alonePage ? '/' : '/onboarding/mounting/choose'}
          state={onboarding}
        >
          {onboarding.alonePage ? 'Return home' : 'Back to mounting choices'}
        </Button>
      </div>
    </div>
  );
}
