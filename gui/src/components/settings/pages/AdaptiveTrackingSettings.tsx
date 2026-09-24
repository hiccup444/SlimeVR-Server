import { useEffect, useRef, useState } from 'react';
import { useForm } from 'react-hook-form';
import { Builder } from 'flatbuffers';
import {
  AdaptiveBoolean,
  AdaptiveTrackingSettings as AdaptiveTrackingSettingsWire,
  AdaptiveTrackingSettingsT,
  AdaptiveDiagnosticsRequestT,
  AdaptiveDiagnosticsResponseT,
  ChangeSettingsRequestT,
  RpcMessage,
  SettingsRequestT,
  SettingsResponseT,
} from 'solarxr-protocol';
import { useLocalization } from '@fluent/react';
import { Button } from '@/components/commons/Button';
import { CheckBox } from '@/components/commons/Checkbox';
import { Dropdown } from '@/components/commons/Dropdown';
import { Input } from '@/components/commons/Input';
import { Typography } from '@/components/commons/Typography';
import { BugIcon } from '@/components/commons/icon/BugIcon';
import { SettingsPagePaneLayout } from '@/components/settings/SettingsPageLayout';
import { useWebsocketAPI } from '@/hooks/websocket-api';
import { AdaptiveDebugPanel } from './AdaptiveDebugPanel';

interface AdaptiveTrackingForm {
  telemetryEnabled: boolean;
  confidenceDiagnosticsEnabled: boolean;
  footContactDiagnosticsEnabled: boolean;
  footAnchoringEnabled: boolean;
  footAnchorStrength: number;
  telemetrySampleRateHz: number;
  yawCorrectionEnabled: boolean;
  yawCorrectionStrength: number;
  poseOptimizerEnabled: boolean;
  temperatureLearningEnabled: boolean;
  liveDiagnosticsEnabled: boolean;
  floorEstimationEnabled: boolean;
  armCalibrationMode: string;
}

class AdaptiveTrackingSettingsPatchT extends AdaptiveTrackingSettingsT {
  override pack(builder: Builder) {
    const armCalibrationModeOffset =
      typeof this.armCalibrationMode === 'string'
        ? builder.createString(this.armCalibrationMode)
        : 0;
    AdaptiveTrackingSettingsWire.startAdaptiveTrackingSettings(builder);
    if (this.telemetryEnabled !== null)
      AdaptiveTrackingSettingsWire.addTelemetryEnabled(
        builder,
        this.telemetryEnabled
      );
    if (this.confidenceDiagnosticsEnabled !== null)
      AdaptiveTrackingSettingsWire.addConfidenceDiagnosticsEnabled(
        builder,
        this.confidenceDiagnosticsEnabled
      );
    if (this.footContactDiagnosticsEnabled !== null)
      AdaptiveTrackingSettingsWire.addFootContactDiagnosticsEnabled(
        builder,
        this.footContactDiagnosticsEnabled
      );
    if (this.footAnchoringEnabled !== null)
      AdaptiveTrackingSettingsWire.addFootAnchoringEnabled(
        builder,
        this.footAnchoringEnabled
      );
    if (this.footAnchorStrength !== null) {
      if (this.footAnchorStrength === 0) builder.addFieldFloat32(4, 0, 1);
      else
        AdaptiveTrackingSettingsWire.addFootAnchorStrength(
          builder,
          this.footAnchorStrength
        );
    }
    if (this.telemetrySampleRateHz !== null)
      AdaptiveTrackingSettingsWire.addTelemetrySampleRateHz(
        builder,
        this.telemetrySampleRateHz
      );
    if (this.yawCorrectionEnabled !== null)
      AdaptiveTrackingSettingsWire.addYawCorrectionEnabled(
        builder,
        this.yawCorrectionEnabled
      );
    if (this.yawCorrectionStrength !== null) {
      if (this.yawCorrectionStrength === 0) builder.addFieldFloat32(7, 0, 1);
      else
        AdaptiveTrackingSettingsWire.addYawCorrectionStrength(
          builder,
          this.yawCorrectionStrength
        );
    }
    if (this.poseOptimizerEnabled !== null)
      AdaptiveTrackingSettingsWire.addPoseOptimizerEnabled(
        builder,
        this.poseOptimizerEnabled
      );
    if (this.temperatureLearningEnabled !== null)
      AdaptiveTrackingSettingsWire.addTemperatureLearningEnabled(
        builder,
        this.temperatureLearningEnabled
      );
    if (this.clearLearnedCalibration !== null)
      AdaptiveTrackingSettingsWire.addClearLearnedCalibration(
        builder,
        this.clearLearnedCalibration
      );
    if (this.liveDiagnosticsEnabled !== null)
      AdaptiveTrackingSettingsWire.addLiveDiagnosticsEnabled(
        builder,
        this.liveDiagnosticsEnabled
      );
    if (typeof this.armCalibrationMode === 'string')
      AdaptiveTrackingSettingsWire.addArmCalibrationMode(
        builder,
        armCalibrationModeOffset
      );
    if (this.floorEstimationEnabled !== null)
      AdaptiveTrackingSettingsWire.addFloorEstimationEnabled(
        builder,
        this.floorEstimationEnabled
      );
    return AdaptiveTrackingSettingsWire.endAdaptiveTrackingSettings(builder);
  }
}

const defaults: AdaptiveTrackingForm = {
  telemetryEnabled: false,
  confidenceDiagnosticsEnabled: true,
  footContactDiagnosticsEnabled: true,
  footAnchoringEnabled: false,
  footAnchorStrength: 0.8,
  telemetrySampleRateHz: 50,
  yawCorrectionEnabled: false,
  yawCorrectionStrength: 0.5,
  poseOptimizerEnabled: false,
  temperatureLearningEnabled: false,
  liveDiagnosticsEnabled: false,
  floorEstimationEnabled: false,
  armCalibrationMode: 'disabled',
};

type DiagnosticObject = Record<string, unknown>;

function asObject(value: unknown): DiagnosticObject | null {
  return value !== null && typeof value === 'object' && !Array.isArray(value)
    ? (value as DiagnosticObject)
    : null;
}

function finite(value: unknown): number | null {
  return typeof value === 'number' && Number.isFinite(value) ? value : null;
}

function format(value: unknown, digits = 2): string {
  const numeric = finite(value);
  return numeric === null ? '—' : numeric.toFixed(digits);
}

function displayStrength(value: number | null, fallback: number): number {
  const strength = value ?? fallback;
  return Number.isFinite(strength)
    ? Math.round(strength * 1_000_000) / 1_000_000
    : fallback;
}

function parseDiagnostics(frameJson: string): DiagnosticObject {
  if (frameJson.length > 2 * 1024 * 1024)
    throw new Error('Diagnostics snapshot exceeds size limit');
  const parsed: unknown = JSON.parse(frameJson);
  const frame = asObject(parsed);
  if (!frame || !finite(frame.timestampNanos) || !Array.isArray(frame.samples))
    throw new Error('Unsupported diagnostics schema');
  return frame;
}

function DiagnosticsView({ frame }: { frame: DiagnosticObject }) {
  const samples = (Array.isArray(frame.samples) ? frame.samples : [])
    .map(asObject)
    .filter((sample): sample is DiagnosticObject => sample !== null);
  const contacts = asObject(frame.footContacts) ?? {};
  const pose = asObject(frame.poseDiagnostic);
  const armCalibration = asObject(frame.armCalibration);
  const floorEstimate = asObject(frame.floorEstimate);
  const activity = asObject(frame.activity);
  const trackerPredictions = Object.entries(
    asObject(frame.trackerPredictions) ?? {}
  )
    .slice(0, 32)
    .flatMap(([key, value]) => {
      const prediction = asObject(value);
      const residual = asObject(prediction?.residual);
      if (!prediction || !residual) return [];
      const direction = asObject(residual.residualVectorRadians);
      const quaternion = (candidate: unknown) => {
        const q = asObject(candidate);
        return q
          ? `(${format(q.w, 3)}, ${format(q.x, 3)}, ${format(q.y, 3)}, ${format(q.z, 3)})`
          : '-';
      };
      return [
        {
          key,
          id: String(residual.trackerId ?? key),
          expected: quaternion(prediction.expectedRotation),
          measured: quaternion(prediction.measuredRotation),
          direction: direction
            ? `(${format(direction.x, 3)}, ${format(direction.y, 3)}, ${format(direction.z, 3)})`
            : '-',
          magnitude: format(residual.magnitudeRadians, 3),
          consistency: format(residual.directionalConsistency, 3),
          seconds: format(residual.continuousSeconds, 1),
          independent:
            residual.independentlyConstrained === true
              ? 'yes, solver-conditioned'
              : residual.independentlyConstrained === false
                ? 'no'
                : 'unknown',
          reason:
            typeof residual.reason === 'string' ? residual.reason : 'unknown',
        },
      ];
    });
  const armSides = (
    Array.isArray(armCalibration?.sides) ? armCalibration.sides : []
  )
    .map(asObject)
    .filter((side): side is DiagnosticObject => side !== null);
  const drift = (
    Array.isArray(frame.driftDiagnostics) ? frame.driftDiagnostics : []
  )
    .map(asObject)
    .filter((item): item is DiagnosticObject => item !== null);

  return (
    <div className="flex flex-col gap-3" aria-live="polite">
      <div className="grid grid-cols-1 gap-2 sm:grid-cols-2">
        {samples.map((sample, index) => {
          const confidence = asObject(sample.confidence);
          const health = asObject(sample.health);
          return (
            <div
              className="rounded-md bg-background-60 p-3"
              key={`${String(sample.id ?? index)}`}
            >
              <Typography>
                {String(sample.name ?? sample.role ?? `Tracker ${index + 1}`)} ·
                tracker confidence {format(confidence?.score)} · packet age{' '}
                {format(
                  finite(sample.packetAgeNanos) === null
                    ? null
                    : Number(sample.packetAgeNanos) / 1e6,
                  0
                )}{' '}
                ms · temperature {format(sample.temperatureCelsius, 1)} °C
              </Typography>
              {health ? (
                <Typography>
                  {`Sensor health ${format(health.qualityMultiplier)} · derived stationary confidence ${format(health.stationaryConfidence)}${health.suspectedFrozen === true ? ' · suspected frozen orientation' : ''}`}
                </Typography>
              ) : null}
              <Typography>
                {Array.isArray(confidence?.reasons)
                  ? confidence.reasons
                      .filter(
                        (reason): reason is string => typeof reason === 'string'
                      )
                      .join(', ')
                  : ''}
              </Typography>
            </div>
          );
        })}
      </div>
      <Typography>
        {`Contacts: ${
          Object.entries(contacts)
            .map(
              ([name, value]) =>
                `${name} ${String(asObject(value)?.state ?? 'unknown')}`
            )
            .join(' · ') || 'none'
        }`}
      </Typography>
      <Typography>
        {`Yaw drift: ${
          drift
            .map((item) => {
              const residual = asObject(item.residual);
              const gate = asObject(item.poseConfidence);
              const gateReasons = Array.isArray(gate?.reasons)
                ? gate.reasons
                    .filter(
                      (reason): reason is string => typeof reason === 'string'
                    )
                    .join(', ')
                : '';
              const gateState =
                gate?.learningEligible === true
                  ? 'eligible'
                  : gate?.learningEligible === false
                    ? 'paused'
                    : 'unknown';
              return `${String(item.trackerId ?? '?')} bias ${format(item.biasRadians, 3)} rad residual ${format(residual?.errorRadians, 3)} rad, ${String(residual?.reason ?? 'no learner state')}, predicted rate ${format(item.predictedRateRadiansPerSecond, 3)} rad/s; learning gate ${gate ? `${format(gate.score)} ${gateState} ${gateReasons}` : 'unavailable'}`;
            })
            .join(' · ') || 'none'
        }`}
      </Typography>
      <Typography>
        Solver-conditioned expected rotations and residuals below are
        diagnostics, not independent truth. The independent-constraint flag
        reports the evidence label passed by the backend.
      </Typography>
      {trackerPredictions.length > 0 ? (
        <div className="table-wrap overflow-auto">
          <table className="w-full border-collapse text-left">
            <thead>
              <tr>
                <th>Tracker</th>
                <th>Expected (w, x, y, z)</th>
                <th>Measured (w, x, y, z)</th>
                <th>Residual vector (rad)</th>
                <th>Magnitude (rad)</th>
                <th>Direction consistency</th>
                <th>Continuous (s)</th>
                <th>Independent constraint</th>
                <th>Reason</th>
              </tr>
            </thead>
            <tbody>
              {trackerPredictions.map((prediction) => (
                <tr key={prediction.key}>
                  <td>{prediction.id}</td>
                  <td>{prediction.expected}</td>
                  <td>{prediction.measured}</td>
                  <td>{prediction.direction}</td>
                  <td>{prediction.magnitude}</td>
                  <td>{prediction.consistency}</td>
                  <td>{prediction.seconds}</td>
                  <td>{prediction.independent}</td>
                  <td>{prediction.reason}</td>
                </tr>
              ))}
            </tbody>
          </table>
        </div>
      ) : (
        <Typography>Per-tracker pose residuals are unavailable.</Typography>
      )}
      <Typography>
        {pose
          ? `Solver error ${format(pose.initialError)} → ${format(pose.finalError)} · cost ${format(pose.processingNanos, 0)} ns · measurement confidence ${format(pose.measurementConfidence ?? pose.globalConfidence)}${pose.measurementConfidence == null && pose.globalConfidence != null ? ' (legacy field)' : ''}`
          : 'Pose solver has no diagnostic sample yet'}
      </Typography>
      <Typography>
        {activity
          ? `Activity estimate: ${String(activity.state ?? 'unknown')} · confidence ${format(activity.confidence)} · ${typeof activity.reason === 'string' ? activity.reason : 'unknown'} · candidate ${format(activity.candidateSeconds, 1)} s`
          : 'Activity estimate unavailable.'}
      </Typography>
      {armCalibration && (
        <div className="flex flex-col gap-1">
          <Typography>
            {`Arm calibration: ${String(armCalibration.mode ?? 'unknown')} · upper arm ${format(finite(armCalibration.upperArmMeters) === null ? null : Number(armCalibration.upperArmMeters) * 100, 1)} cm · lower arm ${format(finite(armCalibration.lowerArmMeters) === null ? null : Number(armCalibration.lowerArmMeters) * 100, 1)} cm`}
          </Typography>
          {armSides.map((side, index) => (
            <Typography key={`${String(side.side ?? index)}`}>
              {`${String(side.side ?? `Side ${index + 1}`)}: ${String(side.status ?? 'unknown')} · trusted ${format(side.trustedSeconds, 1)} s · mounting ${format(side.mountingDegrees, 1)}°`}
              {(() => {
                const confidence = asObject(side.poseConfidence);
                if (!confidence) return ' · confidence unavailable';
                const reasons = Array.isArray(confidence.reasons)
                  ? confidence.reasons
                      .filter(
                        (reason): reason is string => typeof reason === 'string'
                      )
                      .join(', ')
                  : '';
                const gateState =
                  confidence.learningEligible === true
                    ? 'eligible'
                    : confidence.learningEligible === false
                      ? 'paused'
                      : 'unknown';
                return ` · learning gate ${format(confidence.score)} ${gateState} ${reasons}`;
              })()}
            </Typography>
          ))}
        </div>
      )}
      <Typography>
        {floorEstimate
          ? `Floor height: calibrated ${format(floorEstimate.calibratedHeightMeters, 3)} m · estimated ${format(floorEstimate.estimatedHeightMeters, 3)} m · trusted ${format(floorEstimate.trustedSeconds, 1)} s · ${String(floorEstimate.reason ?? 'unknown')}`
          : 'Floor height estimate: unavailable'}
      </Typography>
      <PoseComparison frame={frame} />
    </div>
  );
}

function PoseComparison({ frame }: { frame: DiagnosticObject }) {
  const { l10n } = useLocalization();
  const [projection, setProjection] = useState<'front' | 'top'>('front');
  const raw = asObject(frame.rawPose) ?? {};
  const predicted = asObject(frame.predictedPose) ?? {};
  const entries = Object.keys(raw)
    .slice(0, 32)
    .flatMap((key) => {
      const before = asObject(raw[key]);
      const after = asObject(predicted[key]);
      const x1 = finite(before?.x);
      const y1 = finite(projection === 'front' ? before?.y : before?.z);
      const x2 = finite(after?.x);
      const y2 = finite(projection === 'front' ? after?.y : after?.z);
      if (x1 === null || y1 === null || x2 === null || y2 === null) return [];
      return [{ key, x1, y1, x2, y2 }];
    });
  if (entries.length === 0)
    return <Typography>No comparable pose points are available.</Typography>;
  const xs = entries.flatMap((item) => [item.x1, item.x2]);
  const ys = entries.flatMap((item) => [item.y1, item.y2]);
  const minX = Math.min(...xs);
  const maxX = Math.max(...xs);
  const minY = Math.min(...ys);
  const maxY = Math.max(...ys);
  const px = (x: number) => 10 + ((x - minX) / (maxX - minX || 1)) * 180;
  const py = (y: number) => 10 + ((maxY - y) / (maxY - minY || 1)) * 180;
  return (
    <div>
      <label className="mb-2 flex items-center gap-2">
        <Typography>
          {l10n.getString('settings-adaptive-tracking-pose-view')}
        </Typography>
        <select
          aria-label="Pose view"
          className="rounded bg-background-60 px-2 py-1"
          value={projection}
          onChange={(event) =>
            setProjection(event.target.value === 'top' ? 'top' : 'front')
          }
        >
          <option value="front">
            {l10n.getString('settings-adaptive-tracking-pose-front')}
          </option>
          <option value="top">
            {l10n.getString('settings-adaptive-tracking-pose-top')}
          </option>
        </select>
      </label>
      <Typography>Pose points: observed (gray), predicted (accent)</Typography>
      <svg
        viewBox="0 0 200 200"
        role="img"
        aria-label="Observed and predicted body pose points"
        className="h-48 w-48 rounded bg-background-60"
      >
        {entries.map((item) => (
          <g key={item.key}>
            <circle cx={px(item.x1)} cy={py(item.y1)} r="3" fill="#999" />
            <circle
              cx={px(item.x2)}
              cy={py(item.y2)}
              r="3"
              fill="currentColor"
            />
          </g>
        ))}
      </svg>
    </div>
  );
}

export function AdaptiveTrackingSettings() {
  const { l10n } = useLocalization();
  const { sendRPCPacket, useRPCPacket, isConnected } = useWebsocketAPI();
  const sendRPCPacketRef = useRef(sendRPCPacket);
  const pendingSaveTimer = useRef<ReturnType<typeof setTimeout> | null>(null);
  const pendingClearTimer = useRef<ReturnType<typeof setTimeout> | null>(null);
  const diagnosticsTimer = useRef<ReturnType<typeof setInterval> | null>(null);
  const [diagnostics, setDiagnostics] = useState<DiagnosticObject | null>(null);
  const [diagnosticsError, setDiagnosticsError] = useState('');
  const { control, handleSubmit, reset } = useForm<AdaptiveTrackingForm>({
    defaultValues: defaults,
  });
  const [confirmedSettings, setConfirmedSettings] =
    useState<AdaptiveTrackingSettingsT | null>(null);
  const liveDiagnosticsEnabled =
    confirmedSettings?.liveDiagnosticsEnabled === AdaptiveBoolean.TRUE;
  const confirmedArmMode =
    typeof confirmedSettings?.armCalibrationMode === 'string'
      ? confirmedSettings.armCalibrationMode
      : confirmedSettings?.armCalibrationMode instanceof Uint8Array
        ? new TextDecoder().decode(confirmedSettings.armCalibrationMode)
        : 'disabled';
  const activeFeatures = [
    confirmedSettings?.yawCorrectionEnabled === AdaptiveBoolean.TRUE &&
      'Yaw correction',
    confirmedSettings?.footAnchoringEnabled === AdaptiveBoolean.TRUE &&
      'Foot anchoring',
    confirmedSettings?.poseOptimizerEnabled === AdaptiveBoolean.TRUE &&
      'Pose optimizer',
    confirmedSettings?.floorEstimationEnabled === AdaptiveBoolean.TRUE &&
      'Floor estimation',
    confirmedSettings?.temperatureLearningEnabled === AdaptiveBoolean.TRUE &&
      'Temperature learning',
    confirmedArmMode !== 'disabled' && 'Arm calibration',
  ].filter((name): name is string => typeof name === 'string');
  const [saveState, setSaveState] = useState<
    'idle' | 'saving' | 'saved' | 'error'
  >('idle');
  const [clearState, setClearState] = useState<
    'idle' | 'clearing' | 'cleared' | 'error'
  >('idle');

  useEffect(() => {
    sendRPCPacketRef.current = sendRPCPacket;
  }, [sendRPCPacket]);

  useEffect(() => {
    if (isConnected) {
      sendRPCPacketRef.current(
        RpcMessage.SettingsRequest,
        new SettingsRequestT()
      );
    } else {
      setConfirmedSettings(null);
      setSaveState((current) => (current === 'saving' ? 'error' : current));
      setClearState((current) => (current === 'clearing' ? 'error' : current));
    }
  }, [isConnected]);

  useEffect(() => {
    if (pendingSaveTimer.current) clearTimeout(pendingSaveTimer.current);
    pendingSaveTimer.current = null;
    if (saveState === 'saving') {
      pendingSaveTimer.current = setTimeout(() => {
        setSaveState((current) => (current === 'saving' ? 'error' : current));
      }, 10000);
    }
    return () => {
      if (pendingSaveTimer.current) clearTimeout(pendingSaveTimer.current);
    };
  }, [saveState]);

  useEffect(() => {
    if (pendingClearTimer.current) clearTimeout(pendingClearTimer.current);
    pendingClearTimer.current = null;
    if (clearState === 'clearing') {
      pendingClearTimer.current = setTimeout(() => {
        setClearState((current) =>
          current === 'clearing' ? 'error' : current
        );
      }, 10000);
    }
    return () => {
      if (pendingClearTimer.current) clearTimeout(pendingClearTimer.current);
    };
  }, [clearState]);

  useRPCPacket(RpcMessage.SettingsResponse, (settings: SettingsResponseT) => {
    const adaptive = settings.adaptiveTracking;
    if (!adaptive) return;
    setConfirmedSettings(adaptive);
    reset({
      telemetryEnabled: adaptive.telemetryEnabled === AdaptiveBoolean.TRUE,
      confidenceDiagnosticsEnabled:
        adaptive.confidenceDiagnosticsEnabled === AdaptiveBoolean.TRUE,
      footContactDiagnosticsEnabled:
        adaptive.footContactDiagnosticsEnabled === AdaptiveBoolean.TRUE,
      footAnchoringEnabled:
        adaptive.footAnchoringEnabled === AdaptiveBoolean.TRUE,
      footAnchorStrength: displayStrength(
        adaptive.footAnchorStrength,
        defaults.footAnchorStrength
      ),
      telemetrySampleRateHz:
        adaptive.telemetrySampleRateHz ?? defaults.telemetrySampleRateHz,
      yawCorrectionEnabled:
        adaptive.yawCorrectionEnabled === AdaptiveBoolean.TRUE,
      yawCorrectionStrength: displayStrength(
        adaptive.yawCorrectionStrength,
        defaults.yawCorrectionStrength
      ),
      poseOptimizerEnabled:
        adaptive.poseOptimizerEnabled === AdaptiveBoolean.TRUE,
      temperatureLearningEnabled:
        adaptive.temperatureLearningEnabled === AdaptiveBoolean.TRUE,
      liveDiagnosticsEnabled:
        adaptive.liveDiagnosticsEnabled === AdaptiveBoolean.TRUE,
      floorEstimationEnabled:
        adaptive.floorEstimationEnabled === AdaptiveBoolean.TRUE,
      armCalibrationMode:
        typeof adaptive.armCalibrationMode === 'string'
          ? adaptive.armCalibrationMode
          : adaptive.armCalibrationMode instanceof Uint8Array
            ? new TextDecoder().decode(adaptive.armCalibrationMode)
            : 'disabled',
    });
    setSaveState((current) => (current === 'saving' ? 'saved' : current));
    setClearState((current) => (current === 'clearing' ? 'cleared' : current));
  });

  useRPCPacket(
    RpcMessage.AdaptiveDiagnosticsResponse,
    (response: AdaptiveDiagnosticsResponseT) => {
      try {
        if (!response.frameJson) {
          setDiagnostics(null);
          setDiagnosticsError('Waiting for the first live snapshot');
          return;
        }
        const frameJson =
          typeof response.frameJson === 'string'
            ? response.frameJson
            : new TextDecoder().decode(response.frameJson);
        setDiagnostics(parseDiagnostics(frameJson));
        setDiagnosticsError('');
      } catch (error) {
        setDiagnostics(null);
        setDiagnosticsError(
          error instanceof Error
            ? error.message
            : 'Unsupported diagnostics schema'
        );
      }
    }
  );

  useEffect(() => {
    if (diagnosticsTimer.current) clearInterval(diagnosticsTimer.current);
    diagnosticsTimer.current = null;
    setDiagnostics(null);
    if (!liveDiagnosticsEnabled || !isConnected) return;
    const request = () =>
      sendRPCPacketRef.current(
        RpcMessage.AdaptiveDiagnosticsRequest,
        new AdaptiveDiagnosticsRequestT()
      );
    request();
    diagnosticsTimer.current = setInterval(request, 250);
    return () => {
      if (diagnosticsTimer.current) clearInterval(diagnosticsTimer.current);
      diagnosticsTimer.current = null;
    };
  }, [liveDiagnosticsEnabled, isConnected]);

  const saveSettings = (values: AdaptiveTrackingForm) => {
    const strength = Number(values.footAnchorStrength);
    const rate = Number(values.telemetrySampleRateHz);
    const yawStrength = Number(values.yawCorrectionStrength);
    if (
      !Number.isFinite(strength) ||
      strength < 0 ||
      strength > 1 ||
      !Number.isInteger(rate) ||
      rate < 1 ||
      rate > 100 ||
      !Number.isFinite(yawStrength) ||
      yawStrength < 0 ||
      yawStrength > 1
    ) {
      setSaveState('error');
      return;
    }

    if (!isConnected) {
      setSaveState('error');
      return;
    }

    const adaptive = new AdaptiveTrackingSettingsPatchT();
    adaptive.telemetryEnabled = values.telemetryEnabled
      ? AdaptiveBoolean.TRUE
      : AdaptiveBoolean.FALSE;
    adaptive.confidenceDiagnosticsEnabled = values.confidenceDiagnosticsEnabled
      ? AdaptiveBoolean.TRUE
      : AdaptiveBoolean.FALSE;
    adaptive.footContactDiagnosticsEnabled =
      values.footContactDiagnosticsEnabled
        ? AdaptiveBoolean.TRUE
        : AdaptiveBoolean.FALSE;
    adaptive.footAnchoringEnabled = values.footAnchoringEnabled
      ? AdaptiveBoolean.TRUE
      : AdaptiveBoolean.FALSE;
    adaptive.footAnchorStrength = strength;
    adaptive.telemetrySampleRateHz = rate;
    adaptive.yawCorrectionEnabled = values.yawCorrectionEnabled
      ? AdaptiveBoolean.TRUE
      : AdaptiveBoolean.FALSE;
    adaptive.yawCorrectionStrength = yawStrength;
    adaptive.poseOptimizerEnabled = values.poseOptimizerEnabled
      ? AdaptiveBoolean.TRUE
      : AdaptiveBoolean.FALSE;
    adaptive.temperatureLearningEnabled = values.temperatureLearningEnabled
      ? AdaptiveBoolean.TRUE
      : AdaptiveBoolean.FALSE;
    adaptive.liveDiagnosticsEnabled = values.liveDiagnosticsEnabled
      ? AdaptiveBoolean.TRUE
      : AdaptiveBoolean.FALSE;
    adaptive.floorEstimationEnabled = values.floorEstimationEnabled
      ? AdaptiveBoolean.TRUE
      : AdaptiveBoolean.FALSE;
    adaptive.armCalibrationMode = values.armCalibrationMode;

    const request = new ChangeSettingsRequestT();
    request.adaptiveTracking = adaptive;
    setSaveState('saving');
    sendRPCPacket(RpcMessage.ChangeSettingsRequest, request);
  };

  const clearLearnedCalibration = () => {
    if (!isConnected) {
      setClearState('error');
      return;
    }
    const adaptive = new AdaptiveTrackingSettingsPatchT();
    adaptive.clearLearnedCalibration = AdaptiveBoolean.TRUE;
    const request = new ChangeSettingsRequestT();
    request.adaptiveTracking = adaptive;
    setClearState('clearing');
    sendRPCPacket(RpcMessage.ChangeSettingsRequest, request);
  };

  const setYawComparison = (enableYaw: boolean) => {
    if (!isConnected) {
      setSaveState('error');
      return;
    }
    const adaptive = new AdaptiveTrackingSettingsPatchT();
    adaptive.footAnchoringEnabled = AdaptiveBoolean.FALSE;
    adaptive.yawCorrectionEnabled = enableYaw
      ? AdaptiveBoolean.TRUE
      : AdaptiveBoolean.FALSE;
    adaptive.poseOptimizerEnabled = AdaptiveBoolean.FALSE;
    adaptive.temperatureLearningEnabled = AdaptiveBoolean.FALSE;
    adaptive.floorEstimationEnabled = AdaptiveBoolean.FALSE;
    adaptive.armCalibrationMode = 'disabled';
    const request = new ChangeSettingsRequestT();
    request.adaptiveTracking = adaptive;
    setSaveState('saving');
    sendRPCPacket(RpcMessage.ChangeSettingsRequest, request);
  };

  const setTestRecording = (record: boolean) => {
    if (!isConnected) {
      setSaveState('error');
      return;
    }
    const adaptive = new AdaptiveTrackingSettingsPatchT();
    adaptive.telemetryEnabled = record
      ? AdaptiveBoolean.TRUE
      : AdaptiveBoolean.FALSE;
    if (record) {
      adaptive.liveDiagnosticsEnabled = AdaptiveBoolean.TRUE;
      adaptive.confidenceDiagnosticsEnabled = AdaptiveBoolean.TRUE;
      adaptive.footContactDiagnosticsEnabled = AdaptiveBoolean.TRUE;
    }
    const request = new ChangeSettingsRequestT();
    request.adaptiveTracking = adaptive;
    setSaveState('saving');
    sendRPCPacket(RpcMessage.ChangeSettingsRequest, request);
  };

  return (
    <SettingsPagePaneLayout icon={<BugIcon />} id="adaptive-tracking">
      <div className="flex flex-col gap-4">
        <AdaptiveDebugPanel
          frame={diagnostics}
          connected={isConnected}
          enabled={liveDiagnosticsEnabled}
          settings={confirmedSettings}
          activeFeatures={activeFeatures}
          onEnable={() => {
            const adaptive = new AdaptiveTrackingSettingsPatchT();
            adaptive.liveDiagnosticsEnabled = AdaptiveBoolean.TRUE;
            adaptive.confidenceDiagnosticsEnabled = AdaptiveBoolean.TRUE;
            adaptive.footContactDiagnosticsEnabled = AdaptiveBoolean.TRUE;
            const request = new ChangeSettingsRequestT();
            request.adaptiveTracking = adaptive;
            setSaveState('saving');
            sendRPCPacket(RpcMessage.ChangeSettingsRequest, request);
          }}
          onSetYawComparison={setYawComparison}
          onStartRecording={() => setTestRecording(true)}
          onStopRecording={() => setTestRecording(false)}
        />
        <div>
          <Typography variant="main-title">
            {l10n.getString('settings-adaptive-tracking-title')}
          </Typography>
          <Typography>
            {l10n.getString('settings-adaptive-tracking-description')}
          </Typography>
        </div>

        <div className="grid grid-cols-1 gap-2 sm:grid-cols-2">
          <CheckBox
            variant="toggle"
            outlined
            control={control}
            name="telemetryEnabled"
            label={l10n.getString('settings-adaptive-tracking-telemetry')}
          />
          <CheckBox
            variant="toggle"
            outlined
            control={control}
            name="confidenceDiagnosticsEnabled"
            label={l10n.getString('settings-adaptive-tracking-confidence')}
          />
          <CheckBox
            variant="toggle"
            outlined
            control={control}
            name="footContactDiagnosticsEnabled"
            label={l10n.getString('settings-adaptive-tracking-foot-contact')}
          />
          <CheckBox
            variant="toggle"
            outlined
            control={control}
            name="footAnchoringEnabled"
            label={l10n.getString('settings-adaptive-tracking-foot-anchoring')}
          />
          <CheckBox
            variant="toggle"
            outlined
            control={control}
            name="yawCorrectionEnabled"
            label={l10n.getString('settings-adaptive-tracking-yaw-correction')}
          />
          <CheckBox
            variant="toggle"
            outlined
            control={control}
            name="poseOptimizerEnabled"
            label={l10n.getString('settings-adaptive-tracking-pose-optimizer')}
          />
          <CheckBox
            variant="toggle"
            outlined
            control={control}
            name="temperatureLearningEnabled"
            label={l10n.getString(
              'settings-adaptive-tracking-temperature-learning'
            )}
          />
          <CheckBox
            variant="toggle"
            outlined
            control={control}
            name="liveDiagnosticsEnabled"
            label={l10n.getString(
              'settings-adaptive-tracking-live-diagnostics'
            )}
          />
          <CheckBox
            variant="toggle"
            outlined
            control={control}
            name="floorEstimationEnabled"
            label={l10n.getString(
              'settings-adaptive-tracking-floor-estimation'
            )}
          />
        </div>

        <Typography>
          {l10n.getString(
            'settings-adaptive-tracking-temperature-learning-description'
          )}
        </Typography>
        <Typography>
          {l10n.getString(
            'settings-adaptive-tracking-yaw-correction-description'
          )}
        </Typography>
        <Typography>
          {l10n.getString('settings-adaptive-tracking-floor-estimation-help')}
        </Typography>

        <div className="flex flex-col gap-2">
          <Typography variant="section-title">
            {l10n.getString('settings-adaptive-tracking-arm-calibration')}
          </Typography>
          <Dropdown
            control={control}
            name="armCalibrationMode"
            placeholder={l10n.getString(
              'settings-adaptive-tracking-arm-calibration-placeholder'
            )}
            items={[
              {
                value: 'disabled',
                label: l10n.getString(
                  'settings-adaptive-tracking-arm-calibration-disabled'
                ),
              },
              {
                value: 'proportions',
                label: l10n.getString(
                  'settings-adaptive-tracking-arm-calibration-proportions'
                ),
              },
              {
                value: 'mounting',
                label: l10n.getString(
                  'settings-adaptive-tracking-arm-calibration-mounting'
                ),
              },
            ]}
            alignment="left"
            display="block"
          />
          <Typography>
            {l10n.getString('settings-adaptive-tracking-arm-calibration-help')}
          </Typography>
        </div>

        <div className="grid grid-cols-1 gap-3 sm:grid-cols-2">
          <Input
            type="number"
            min={0}
            max={1}
            step={0.05}
            control={control}
            name="footAnchorStrength"
            label={l10n.getString('settings-adaptive-tracking-anchor-strength')}
            rules={{ required: true, min: 0, max: 1 }}
          />
          <Input
            type="number"
            min={1}
            max={100}
            step={1}
            control={control}
            name="telemetrySampleRateHz"
            label={l10n.getString('settings-adaptive-tracking-sample-rate')}
            rules={{
              required: true,
              min: 1,
              max: 100,
              validate: (value) => Number.isInteger(Number(value)),
            }}
          />
          <Input
            type="number"
            min={0}
            max={1}
            step={0.05}
            control={control}
            name="yawCorrectionStrength"
            label={l10n.getString('settings-adaptive-tracking-yaw-strength')}
            rules={{ required: true, min: 0, max: 1 }}
          />
        </div>

        <div className="flex items-center gap-3">
          <Button
            type="button"
            variant="secondary"
            onClick={handleSubmit(saveSettings, () => setSaveState('error'))}
          >
            {l10n.getString('settings-adaptive-tracking-save')}
          </Button>
          <div role="status" aria-live="polite">
            <Typography>
              {saveState === 'saved' &&
                l10n.getString('settings-adaptive-tracking-saved')}
              {saveState === 'saving' &&
                l10n.getString('settings-adaptive-tracking-saving')}
              {saveState === 'error' &&
                l10n.getString('settings-adaptive-tracking-error')}
            </Typography>
          </div>
        </div>
        <div className="flex flex-wrap items-center gap-3">
          <Button
            type="button"
            variant="secondary"
            disabled={clearState === 'clearing'}
            onClick={clearLearnedCalibration}
          >
            {l10n.getString('settings-adaptive-tracking-clear-calibration')}
          </Button>
          <div role="status" aria-live="polite">
            <Typography>
              {clearState === 'clearing' &&
                l10n.getString(
                  'settings-adaptive-tracking-clearing-calibration'
                )}
              {clearState === 'cleared' &&
                l10n.getString(
                  'settings-adaptive-tracking-calibration-cleared'
                )}
              {clearState === 'error' &&
                l10n.getString(
                  'settings-adaptive-tracking-clear-calibration-error'
                )}
            </Typography>
          </div>
        </div>
        {liveDiagnosticsEnabled && (
          <section
            className="flex flex-col gap-2"
            aria-label={l10n.getString(
              'settings-adaptive-tracking-live-diagnostics'
            )}
          >
            <Typography variant="section-title">
              {l10n.getString('settings-adaptive-tracking-live-diagnostics')}
            </Typography>
            {!isConnected && (
              <Typography>
                {l10n.getString('settings-adaptive-tracking-live-disconnected')}
              </Typography>
            )}
            {isConnected && diagnosticsError && (
              <div role="status">
                <Typography>{diagnosticsError}</Typography>
              </div>
            )}
            {diagnostics && <DiagnosticsView frame={diagnostics} />}
          </section>
        )}
      </div>
    </SettingsPagePaneLayout>
  );
}
