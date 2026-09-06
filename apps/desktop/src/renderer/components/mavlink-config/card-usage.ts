// What is actually taking up room on the flight controller's SD card.
//
// STORAGE_INFORMATION (261) is a CAMERA-component message. ArduPilot's
// autopilot component does not answer it for its own SD card, so the request
// times out and the panel falls back to "capacity not reported" forever.
// Mission Planner only ever sends it to a camera, and shows no SD figure at all.
//
// What we CAN see is the filesystem, over MAVLink-FTP. That is the number this
// bug needs: AP_Logger_File::start_new_log() bails silently when
// disk_space_avail() < LOG_FILE_MB_FREE, and Prep_MinSpace only deletes files
// matching NNN.BIN in /APM/LOGS — so anything else on the card can never be
// freed and quietly starves logging.

export interface FsEntry {
  kind: 'dir' | 'file';
  name: string;
  size?: number;
}

export type ListDir = (path: string) => Promise<{ entries?: FsEntry[]; error?: string }>;

export interface CardUsage {
  /** Bytes in /APM/LOGS held by NNN.BIN files, the only ones ArduPilot can reclaim. */
  logBytes: number;
  logCount: number;
  /** Bytes ArduPilot's log rotation can never free. */
  otherBytes: number;
  /** Largest non-log contributors, biggest first. */
  otherTop: { path: string; bytes: number }[];
  /** Directories that could not be listed, so the totals are a floor, not a total. */
  unreadable: string[];
  scannedAt: number;
}

const LOG_DIR = '/APM/LOGS';
/** ArduPilot's Prep_MinSpace only reclaims files named like 42.BIN. */
const RECLAIMABLE = /^\d+\.BIN$/i;

function join(dir: string, name: string): string {
  return dir.endsWith('/') ? `${dir}${name}` : `${dir}/${name}`;
}

/**
 * Walks the card and totals it. `maxDepth` keeps a deep tree from turning into
 * hundreds of FTP round trips on a slow link.
 */
export async function scanCardUsage(
  list: ListDir,
  opts: { root?: string; maxDepth?: number } = {},
): Promise<CardUsage> {
  const root = opts.root ?? '/';
  const maxDepth = opts.maxDepth ?? 3;

  let logBytes = 0;
  let logCount = 0;
  let otherBytes = 0;
  const other: { path: string; bytes: number }[] = [];
  const unreadable: string[] = [];

  const walk = async (dir: string, depth: number): Promise<void> => {
    const res = await list(dir);
    if (!res.entries) {
      unreadable.push(dir);
      return;
    }
    const inLogDir = dir.toUpperCase().startsWith(LOG_DIR);
    for (const e of res.entries) {
      if (e.name === '.' || e.name === '..') continue;
      const full = join(dir, e.name);
      if (e.kind === 'file') {
        const size = e.size ?? 0;
        if (inLogDir && RECLAIMABLE.test(e.name)) {
          logBytes += size;
          logCount++;
        } else {
          otherBytes += size;
          other.push({ path: full, bytes: size });
        }
      } else if (depth < maxDepth) {
        await walk(full, depth + 1);
      } else {
        unreadable.push(full);
      }
    }
  };

  await walk(root, 0);

  other.sort((a, b) => b.bytes - a.bytes);
  return {
    logBytes,
    logCount,
    otherBytes,
    otherTop: other.slice(0, 8),
    unreadable,
    scannedAt: Date.now(),
  };
}

/**
 * Whether ArduPilot can still start a log, given the card's real capacity.
 * `capacityBytes` has to come from the operator: the firmware never reports it.
 */
export function loggingHeadroom(
  usage: Pick<CardUsage, 'logBytes' | 'otherBytes'>,
  capacityBytes: number,
  logFileMbFree: number,
): { freeBytes: number; requiredBytes: number; willLog: boolean } | null {
  if (!Number.isFinite(capacityBytes) || capacityBytes <= 0) return null;
  const freeBytes = capacityBytes - usage.logBytes - usage.otherBytes;
  const requiredBytes = logFileMbFree * 1024 * 1024;
  return { freeBytes, requiredBytes, willLog: freeBytes >= requiredBytes };
}
