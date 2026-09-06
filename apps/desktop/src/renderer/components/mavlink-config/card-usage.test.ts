import { describe, it, expect } from 'vitest';
import { scanCardUsage, loggingHeadroom, type FsEntry } from './card-usage';

function fakeCard(tree: Record<string, FsEntry[]>, broken: string[] = []) {
  const calls: string[] = [];
  const list = async (path: string) => {
    calls.push(path);
    if (broken.includes(path)) return { error: 'FileNotFound' };
    return { entries: tree[path] ?? [] };
  };
  return { list, calls };
}

const MB = 1024 * 1024;

describe('scanCardUsage', () => {
  it('separates reclaimable logs from everything else', async () => {
    // Prep_MinSpace only deletes NNN.BIN in /APM/LOGS. LASTLOG.TXT sits in the
    // same folder and is not reclaimable, so it must not count as log space.
    const { list } = fakeCard({
      '/': [{ kind: 'dir', name: 'APM' }],
      '/APM': [{ kind: 'dir', name: 'LOGS' }, { kind: 'file', name: 'PARAM.CFG', size: 2 * MB }],
      '/APM/LOGS': [
        { kind: 'file', name: '1.BIN', size: 100 * MB },
        { kind: 'file', name: '2.BIN', size: 250 * MB },
        { kind: 'file', name: 'LASTLOG.TXT', size: 12 },
      ],
    });

    const u = await scanCardUsage(list);
    expect(u.logCount).toBe(2);
    expect(u.logBytes).toBe(350 * MB);
    expect(u.otherBytes).toBe(2 * MB + 12);
    expect(u.otherTop[0]!.path).toBe('/APM/PARAM.CFG');
  });

  it('counts terrain and script data as unreclaimable', async () => {
    // This is the shape that starves logging on a card that "isn't full":
    // gigabytes ArduPilot's rotation can never touch.
    const { list } = fakeCard({
      '/': [{ kind: 'dir', name: 'APM' }, { kind: 'dir', name: 'TERRAIN' }],
      '/APM': [{ kind: 'dir', name: 'LOGS' }],
      '/APM/LOGS': [{ kind: 'file', name: '9.BIN', size: 50 * MB }],
      '/TERRAIN': [
        { kind: 'file', name: 'N45E015.DAT', size: 900 * MB },
        { kind: 'file', name: 'N46E015.DAT', size: 800 * MB },
      ],
    });

    const u = await scanCardUsage(list);
    expect(u.logBytes).toBe(50 * MB);
    expect(u.otherBytes).toBe(1700 * MB);
    expect(u.otherTop.map((o) => o.path)).toEqual(['/TERRAIN/N45E015.DAT', '/TERRAIN/N46E015.DAT']);
  });

  it('records directories it could not read so totals are not passed off as complete', async () => {
    const { list } = fakeCard(
      { '/': [{ kind: 'dir', name: 'APM' }, { kind: 'dir', name: 'LOCKED' }], '/APM': [] },
      ['/LOCKED'],
    );
    const u = await scanCardUsage(list);
    expect(u.unreadable).toEqual(['/LOCKED']);
  });

  it('stops descending past maxDepth instead of walking forever', async () => {
    const { list, calls } = fakeCard({
      '/': [{ kind: 'dir', name: 'a' }],
      '/a': [{ kind: 'dir', name: 'b' }],
      '/a/b': [{ kind: 'dir', name: 'c' }],
      '/a/b/c': [{ kind: 'file', name: 'deep.bin', size: 5 }],
    });
    const u = await scanCardUsage(list, { maxDepth: 2 });
    expect(calls).not.toContain('/a/b/c');
    expect(u.unreadable).toContain('/a/b/c');
  });

  it('ignores . and .. so a card cannot be double counted', async () => {
    const { list } = fakeCard({
      '/': [{ kind: 'dir', name: '.' }, { kind: 'dir', name: '..' }, { kind: 'file', name: 'x', size: 7 }],
    });
    expect((await scanCardUsage(list)).otherBytes).toBe(7);
  });
});

describe('loggingHeadroom', () => {
  // 32 GB card = 32768 MB. 30000 + 2500 = 32500 used, leaving 268 MB.
  const usage = { logBytes: 30_000 * MB, otherBytes: 2_500 * MB };

  it('says logging will fail when free space is under LOG_FILE_MB_FREE', async () => {
    // A 32 GB card that "is not full" but has under 500 MB spare: start_new_log
    // bails silently and the flight produces no .BIN.
    const h = loggingHeadroom(usage, 32 * 1024 * MB, 500)!;
    expect(h.freeBytes).toBeLessThan(h.requiredBytes);
    expect(h.willLog).toBe(false);
  });

  it('says logging will run with room to spare', () => {
    const h = loggingHeadroom({ logBytes: 1000 * MB, otherBytes: 0 }, 32 * 1024 * MB, 500)!;
    expect(h.willLog).toBe(true);
  });

  it('has no answer without a capacity, which the firmware never reports', () => {
    expect(loggingHeadroom(usage, 0, 500)).toBeNull();
    expect(loggingHeadroom(usage, NaN, 500)).toBeNull();
  });
});
