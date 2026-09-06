import { app, ipcMain, type BrowserWindow } from 'electron';
import { existsSync } from 'node:fs';
import { IPC_CHANNELS } from '../../shared/ipc-channels.js';
import type { TrainerLaunchInput, TrainerStatus } from '../../shared/trainer-types.js';
import { getInstalledModules } from '../modules/module-manager.js';
import {
  locateTrainer,
  TRAINER_CARGO_SLUG,
  TRAINER_PATH_ENV,
  type TrainerTarget,
} from './trainer-locator.js';
import { buildTrainerRequest } from './trainer-request.js';
import { launchTrainer, type TrainerLaunchOutcome } from './trainer-process.js';

/**
 * Flying what is planned here, in the Trainer, without leaving this app first.
 *
 * ArduDeck owns the flight controller on this path BY CONSTRUCTION: it is the one spawning the
 * Trainer, so the Trainer serves the physics and never starts a flight stack of its own. That
 * replaces the negotiation the two used to perform over a loopback endpoint, which had to infer
 * another program's intentions from open ports and process command lines and was wrong often
 * enough to be a coin flip in front of an audience.
 *
 * What this does NOT do is decide how a flight is configured. Regions, cameras, weather and the
 * launch config belong to the Trainer; sending intent and letting it answer is what keeps one
 * implementation of that across two repositories.
 */

export interface TrainerDeps {
  /** Where this app's flight controller takes off from, or null when there is none. */
  home: () => { lat: number; lon: number; altM?: number | null; headingDeg?: number | null } | null;
  /** FRAME_CLASS / FRAME_TYPE of the running stack, and its custom frame JSON. */
  frame: () => { frameClass: number | null; frameType: number | null; framePath: string | null };
  /** Lets go of UDP 9002, because the Trainer binds it before it does anything else. */
  releasePhysics: () => Promise<boolean>;
  log?: (level: 'info' | 'warn', message: string) => void;
}

function cargoPath(): string | undefined {
  return getInstalledModules().find(
    (m) => m.slug === TRAINER_CARGO_SLUG && m.enabled !== false,
  )?.installPath;
}

function find(): { target: TrainerTarget | null; searched: string[] } {
  return locateTrainer({
    exists: existsSync,
    platform: process.platform,
    override: process.env[TRAINER_PATH_ENV],
    cargoPath: cargoPath(),
    homeDir: app.getPath('home'),
  });
}

export function trainerStatus(deps: TrainerDeps): TrainerStatus {
  const { target, searched } = find();
  const home = deps.home();
  return {
    installed: target !== null,
    kind: target?.kind ?? null,
    path: target?.path ?? null,
    searched,
    home,
    canLaunch: target !== null && buildTrainerRequest({ home }).ok,
    reason: target === null ? 'The Trainer is not installed.' : notReady(deps),
  };
}

function notReady(deps: TrainerDeps): string | null {
  const built = buildTrainerRequest({ home: deps.home() });
  return built.ok ? null : built.error;
}

export function setupTrainerHandlers(mainWindow: BrowserWindow | null, deps: TrainerDeps): void {
  const send = (line: string): void => {
    if (mainWindow && !mainWindow.isDestroyed()) {
      mainWindow.webContents.send(IPC_CHANNELS.TRAINER_LOG, line);
    }
  };

  ipcMain.handle(IPC_CHANNELS.TRAINER_STATUS, (): TrainerStatus => trainerStatus(deps));

  ipcMain.handle(
    IPC_CHANNELS.TRAINER_LAUNCH,
    async (_e, input: TrainerLaunchInput = {}): Promise<TrainerLaunchOutcome> => {
      const { target, searched } = find();
      if (!target) {
        return {
          ok: false,
          error: `The Trainer is not installed. Looked in: ${searched.join(', ')}`,
        };
      }

      const frame = deps.frame();
      const built = buildTrainerRequest({
        home: deps.home(),
        frameClass: frame.frameClass,
        frameType: frame.frameType,
        framePath: frame.framePath,
        ...input,
      });
      if (!built.ok) return { ok: false, error: built.error };

      // BEFORE the spawn, not after: the Trainer binds UDP 9002 first and drops to a silent
      // hover when it cannot, so a race here produces an aircraft that looks like it is flying
      // and answers to nothing.
      if (!(await deps.releasePhysics())) {
        return {
          ok: false,
          error: 'Could not stop this app’s own flight model, so the Trainer cannot take it.',
        };
      }
      deps.log?.('info', 'trainer: physics released, starting the Trainer');

      return launchTrainer(built.request, {
        target,
        userDataPath: app.getPath('userData'),
        onLog: send,
      });
    },
  );
}
