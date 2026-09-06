import { useCallback, useEffect, useRef, useState } from 'react';
import type { TrainerLaunchInput, TrainerStatus } from '../../../shared/trainer-types';

/**
 * Fly what is planned here, in the Trainer, in one action.
 *
 * The flow this replaces was: plan a mission, start SITL, open a terminal, start the game, try
 * to take the flight controller off this app, pick a region, start, come back here, take off,
 * switch windows, then cycle the camera by hand until the right one appeared. Every one of
 * those steps except planning and taking off is a consequence of the two programs not knowing
 * about each other.
 *
 * There is deliberately almost nothing to set. The region comes from where the flight
 * controller is standing, the airframe from what the stack is mixing for. What is left is one
 * button and an honest account of why it is disabled.
 */

const LOG_LINES = 200;

export function TrainerView(): JSX.Element {
  const [status, setStatus] = useState<TrainerStatus | null>(null);
  const [busy, setBusy] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [log, setLog] = useState<string[]>([]);
  const logRef = useRef<HTMLDivElement>(null);

  const refresh = useCallback(async () => {
    setStatus(await window.electronAPI.trainerStatus());
  }, []);

  useEffect(() => {
    void refresh();
    // The take-off point appears when the flight controller gets a fix, which is not an event
    // this view is told about, so it is polled rather than left showing "no GPS fix" forever.
    const timer = setInterval(() => void refresh(), 2000);
    return () => clearInterval(timer);
  }, [refresh]);

  useEffect(() => {
    return window.electronAPI.onTrainerLog((line) => {
      setLog((prev) => [...prev, line].slice(-LOG_LINES));
    });
  }, []);

  useEffect(() => {
    logRef.current?.scrollTo({ top: logRef.current.scrollHeight });
  }, [log]);

  const fly = async (): Promise<void> => {
    setBusy(true);
    setError(null);
    setLog([]);
    try {
      // The feed comes back into this app's camera panel, so the picture is here rather than in
      // another window: that is the "switch to the sim to show people" step, deleted.
      const input: TrainerLaunchInput = { stream: { enabled: true } };
      const result = await window.electronAPI.trainerLaunch(input);
      if (!result.ok) setError(result.error ?? 'The Trainer did not start.');
    } catch (err) {
      setError((err as Error).message);
    } finally {
      setBusy(false);
      void refresh();
    }
  };

  const home = status?.home;

  return (
    <div className="flex h-full flex-col gap-4 overflow-auto p-6">
      <header>
        <h1 className="text-xl font-semibold tracking-tight">Trainer</h1>
        <p className="mt-1 text-sm text-content-secondary">
          Fly this vehicle, from where it stands, in the simulator. This app keeps the flight
          controller.
        </p>
      </header>

      <section className="rounded-lg border p-4">
        <dl className="grid grid-cols-[10rem_1fr] gap-y-2 text-sm">
          <dt className="text-content-tertiary">Take-off point</dt>
          <dd>
            {home ? `${home.lat.toFixed(5)}, ${home.lon.toFixed(5)}` : 'waiting for a GPS fix'}
          </dd>
          <dt className="text-content-tertiary">Trainer</dt>
          <dd>{status?.installed ? `${status.path} (${status.kind})` : 'not installed'}</dd>
        </dl>

        {status && !status.installed && (
          <details className="mt-3 text-xs text-content-tertiary">
            <summary className="cursor-pointer">Where this looked</summary>
            <ul className="mt-1 list-disc pl-5">
              {status.searched.map((path) => (
                <li key={path}>{path}</li>
              ))}
            </ul>
          </details>
        )}
      </section>

      <div className="flex items-center gap-3">
        <button
          className="rounded-md bg-accent px-4 py-2 font-medium text-black disabled:opacity-40"
          disabled={busy || !status?.canLaunch}
          onClick={() => void fly()}
        >
          {busy ? 'Starting the Trainer…' : 'Fly in Trainer'}
        </button>
        {/* The reason a button is disabled belongs beside it, not in a log nobody opens. */}
        {status?.reason && <span className="text-sm text-content-tertiary">{status.reason}</span>}
      </div>

      {error && <p className="text-sm text-warn">{error}</p>}

      {log.length > 0 && (
        <div
          ref={logRef}
          className="min-h-0 flex-1 overflow-auto rounded-lg border bg-surface-inset p-3 font-mono text-xs"
        >
          {log.map((line, i) => (
            <div key={i}>{line}</div>
          ))}
        </div>
      )}
    </div>
  );
}
