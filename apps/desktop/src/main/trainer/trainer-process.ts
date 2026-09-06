import { spawn, type ChildProcess } from 'node:child_process';
import { mkdir, writeFile } from 'node:fs/promises';
import { join } from 'node:path';
import type { TrainerRequest } from './trainer-request';
import type { TrainerTarget } from './trainer-locator';

/**
 * Starting the Trainer and finding out whether it worked.
 *
 * The Trainer answers on stdout with one marked line among its ordinary log, because the thing
 * that matters (did a flight actually start, and if not why) arrives minutes after the process
 * does: it may compile the game extension, prepare a region and wait for a port. An exit code
 * would say nothing, since a SUCCESSFUL launch does not exit at all - the Trainer stays alive
 * as the game's parent.
 */

/** The line the Trainer prints when it has decided. Must match its `TRAINER_RESULT_MARKER`. */
export const RESULT_MARKER = 'ARDUDECK_TRAINER_RESULT';

/** Long enough to cover a first run compiling the game's Rust extension on a cold cache. */
const RESULT_TIMEOUT_MS = 300_000;

export interface TrainerLaunchOutcome {
  ok: boolean;
  error?: string;
  pid?: number;
  configPath?: string;
}

export interface SpawnDeps {
  target: TrainerTarget;
  /** `app.getPath('userData')`, where the request file is written. */
  userDataPath: string;
  onLog?: (line: string) => void;
}

/** The command line for each shape a Trainer can take. */
export function trainerCommand(target: TrainerTarget, requestPath: string): {
  command: string;
  args: string[];
} {
  const flag = `--trainer-request=${requestPath}`;
  if (target.kind === 'checkout') {
    // The checkout's OWN Electron, and the directory as the app to run. A global `electron`
    // would be a different major version against a pinned preload.
    return { command: target.electron, args: [target.path, flag] };
  }
  return { command: target.exec, args: [flag] };
}

/**
 * Runs the Trainer and resolves once it says what happened.
 *
 * The child is deliberately NOT killed on resolve: a successful launch means the Trainer is now
 * the game's parent, and it has to outlive this call. It is detached and unreferenced so
 * ArduDeck quitting does not take a flight down with it.
 */
export async function launchTrainer(
  request: TrainerRequest,
  deps: SpawnDeps,
): Promise<TrainerLaunchOutcome> {
  const dir = join(deps.userDataPath, 'trainer');
  const requestPath = join(dir, 'request.json');
  try {
    await mkdir(dir, { recursive: true });
    await writeFile(requestPath, JSON.stringify(request, null, 2), 'utf8');
  } catch (err) {
    return { ok: false, error: `Could not write the Trainer request: ${(err as Error).message}` };
  }

  const { command, args } = trainerCommand(deps.target, requestPath);
  let child: ChildProcess;
  try {
    child = spawn(command, args, { detached: true, stdio: ['ignore', 'pipe', 'pipe'] });
  } catch (err) {
    return { ok: false, error: `Could not start the Trainer: ${(err as Error).message}` };
  }
  child.unref();

  return new Promise<TrainerLaunchOutcome>((resolve) => {
    let settled = false;
    const done = (outcome: TrainerLaunchOutcome): void => {
      if (settled) return;
      settled = true;
      clearTimeout(timer);
      resolve(outcome);
    };

    const timer = setTimeout(
      () => done({ ok: false, error: 'The Trainer did not report back in time.' }),
      RESULT_TIMEOUT_MS,
    );

    let lastError = '';
    const read = (stream: 'stdout' | 'stderr') => {
      let buffer = '';
      child[stream]?.on('data', (chunk: Buffer) => {
        buffer += chunk.toString();
        const lines = buffer.split('\n');
        buffer = lines.pop() ?? '';
        for (const line of lines) {
          deps.onLog?.(line);
          if (stream === 'stderr' && line.trim()) lastError = line.trim();
          const at = line.indexOf(RESULT_MARKER);
          if (at === -1) continue;
          try {
            done(JSON.parse(line.slice(at + RESULT_MARKER.length)) as TrainerLaunchOutcome);
          } catch {
            // A marker we cannot parse is a Trainer newer than this build. Not fatal: it has
            // already started or failed on its own, and guessing which would be worse.
            done({ ok: false, error: 'The Trainer answered in a format this build cannot read.' });
          }
        }
      });
    };
    read('stdout');
    read('stderr');

    child.on('error', (err) => done({ ok: false, error: err.message }));
    // Only a FAILED run exits before the marker, so an exit here is always bad news.
    child.on('exit', (code) =>
      done({
        ok: false,
        error: lastError || `The Trainer exited (${code}) without starting a flight.`,
      }),
    );
  });
}
