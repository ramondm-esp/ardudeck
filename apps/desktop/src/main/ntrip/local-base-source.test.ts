import { describe, it, expect, vi } from 'vitest';
import { crc24q, RtcmFramer, parseBasePosition, type RtcmFrame } from './rtcm.js';
import { LocalBaseSource, type BasePortLike } from './local-base-source.js';
import { DEFAULT_NTRIP_CONFIG, type NtripConfig, type NtripStatus } from '../../shared/ntrip-types.js';

/** MSB-first bit packer for building synthetic RTCM payloads. */
class BitWriter {
  private bits: number[] = [];

  write(value: bigint | number, width: number): this {
    const v = BigInt(value);
    const mask = (1n << BigInt(width)) - 1n;
    const raw = v & mask;
    for (let i = width - 1; i >= 0; i--) {
      this.bits.push(Number((raw >> BigInt(i)) & 1n));
    }
    return this;
  }

  toBytes(): Uint8Array {
    const out = new Uint8Array(Math.ceil(this.bits.length / 8));
    this.bits.forEach((bit, i) => {
      if (bit) out[i >> 3]! |= 0x80 >> (i & 7);
    });
    return out;
  }
}

function wrapRtcmFrame(payload: Uint8Array): Uint8Array {
  const frame = new Uint8Array(payload.length + 6);
  frame[0] = 0xd3;
  frame[1] = (payload.length >> 8) & 0x03;
  frame[2] = payload.length & 0xff;
  frame.set(payload, 3);
  const crc = crc24q(frame, frame.length - 3);
  frame[frame.length - 3] = (crc >> 16) & 0xff;
  frame[frame.length - 2] = (crc >> 8) & 0xff;
  frame[frame.length - 1] = crc & 0xff;
  return frame;
}

/** RTCM 1005 with the given ECEF position in meters. */
function build1005(xM: number, yM: number, zM: number): Uint8Array {
  const w = new BitWriter();
  w.write(1005, 12); // message number
  w.write(0, 12); // station id
  w.write(0, 6); // ITRF year
  w.write(1, 1).write(0, 1).write(0, 1).write(0, 1); // GPS/GLO/GAL/ref flags
  w.write(BigInt(Math.round(xM * 1e4)), 38);
  w.write(0, 1).write(0, 1); // oscillator + reserved
  w.write(BigInt(Math.round(yM * 1e4)), 38);
  w.write(0, 2); // quarter cycle
  w.write(BigInt(Math.round(zM * 1e4)), 38);
  return wrapRtcmFrame(w.toBytes());
}

/** Forward WGS84 geodetic -> ECEF, so tests round-trip instead of trusting constants. */
function geodeticToEcef(latDeg: number, lonDeg: number, h: number): { x: number; y: number; z: number } {
  const a = 6378137;
  const f = 1 / 298.257223563;
  const e2 = f * (2 - f);
  const lat = (latDeg * Math.PI) / 180;
  const lon = (lonDeg * Math.PI) / 180;
  const n = a / Math.sqrt(1 - e2 * Math.sin(lat) ** 2);
  return {
    x: (n + h) * Math.cos(lat) * Math.cos(lon),
    y: (n + h) * Math.cos(lat) * Math.sin(lon),
    z: (n * (1 - e2) + h) * Math.sin(lat),
  };
}

const MUNICH_ECEF = geodeticToEcef(48.0, 11.0, 550);

describe('parseBasePosition', () => {
  it('recovers lat/lon/alt from a 1005 frame', () => {
    const bytes = build1005(MUNICH_ECEF.x, MUNICH_ECEF.y, MUNICH_ECEF.z);
    const frames = new RtcmFramer().push(bytes);
    expect(frames).toHaveLength(1);
    const pos = parseBasePosition(frames[0]!);
    expect(pos).not.toBeNull();
    expect(pos!.lat).toBeCloseTo(48.0, 5);
    expect(pos!.lon).toBeCloseTo(11.0, 5);
    expect(pos!.altM).toBeCloseTo(550, 1);
  });

  it('handles negative ECEF components (western/southern hemisphere)', () => {
    const ecef = geodeticToEcef(-33.9, -70.7, 520);
    const frames = new RtcmFramer().push(build1005(ecef.x, ecef.y, ecef.z));
    const pos = parseBasePosition(frames[0]!);
    expect(pos!.lat).toBeCloseTo(-33.9, 5);
    expect(pos!.lon).toBeCloseTo(-70.7, 5);
    expect(pos!.altM).toBeCloseTo(520, 1);
  });

  it('returns null for an unsurveyed all-zero position', () => {
    const frames = new RtcmFramer().push(build1005(0, 0, 0));
    expect(parseBasePosition(frames[0]!)).toBeNull();
  });

  it('returns null for non-1005/1006 message types', () => {
    const frame: RtcmFrame = { bytes: new Uint8Array(25), type: 1074 };
    expect(parseBasePosition(frame)).toBeNull();
  });
});

class FakePort implements BasePortLike {
  handlers = new Map<string, Array<(...args: never[]) => void>>();
  openError: Error | null = null;
  closed = false;

  open(cb: (err: Error | null) => void): void {
    cb(this.openError);
  }

  close(): void {
    this.closed = true;
  }

  on(event: string, cb: (...args: never[]) => void): void {
    const list = this.handlers.get(event) ?? [];
    list.push(cb);
    this.handlers.set(event, list);
  }

  removeAllListeners(): void {
    this.handlers.clear();
  }

  emit(event: string, ...args: unknown[]): void {
    for (const cb of this.handlers.get(event) ?? []) (cb as (...a: unknown[]) => void)(...args);
  }
}

function serialConfig(overrides: Partial<NtripConfig> = {}): NtripConfig {
  return { ...DEFAULT_NTRIP_CONFIG, source: 'serial', serialPath: '/dev/tty.base', ...overrides };
}

function makeSource(port: FakePort) {
  const frames: RtcmFrame[] = [];
  const statuses: NtripStatus[] = [];
  const source = new LocalBaseSource({
    onRtcmFrame: (f) => { frames.push(f); },
    onStatus: (s) => { statuses.push(s); },
    openPort: () => port,
  });
  return { source, frames, statuses };
}

describe('LocalBaseSource', () => {
  it('rejects connect without a serial path', () => {
    const { source } = makeSource(new FakePort());
    const result = source.connect(serialConfig({ serialPath: '' }));
    expect(result.success).toBe(false);
    expect(result.error).toMatch(/serial port/i);
  });

  it('streams frames split across chunks into whole injections', () => {
    const port = new FakePort();
    const { source, frames } = makeSource(port);
    expect(source.connect(serialConfig()).success).toBe(true);
    expect(source.getStatus().state).toBe('connected');

    const frame = build1005(MUNICH_ECEF.x, MUNICH_ECEF.y, MUNICH_ECEF.z);
    port.emit('data', Buffer.from(frame.subarray(0, 7)));
    expect(frames).toHaveLength(0);
    port.emit('data', Buffer.from(frame.subarray(7)));
    expect(frames).toHaveLength(1);
    expect(frames[0]!.type).toBe(1005);

    const status = source.getStatus();
    expect(status.rtcmTypeCounts[1005]).toBe(1);
    expect(status.bytesReceived).toBe(frame.length);
    expect(status.basePosition?.lat).toBeCloseTo(48.0, 4);
    source.disconnect();
  });

  it('fails permanently when the port cannot be opened', () => {
    const port = new FakePort();
    port.openError = new Error('Resource busy');
    const { source, statuses } = makeSource(port);
    source.connect(serialConfig());
    const last = statuses.at(-1)!;
    expect(last.state).toBe('error');
    expect(last.error).toContain('/dev/tty.base');
    expect(last.error).toContain('Resource busy');
  });

  it('reconnects after a stream drop and stops on disconnect', () => {
    vi.useFakeTimers();
    try {
      const port = new FakePort();
      const { source } = makeSource(port);
      source.connect(serialConfig());
      expect(source.getStatus().state).toBe('connected');

      port.emit('error', new Error('Device unplugged'));
      expect(source.getStatus().state).toBe('reconnecting');
      expect(source.getStatus().error).toContain('Device unplugged');

      source.disconnect();
      expect(source.getStatus().state).toBe('disconnected');
      vi.runOnlyPendingTimers();
      expect(source.getStatus().state).toBe('disconnected');
    } finally {
      vi.useRealTimers();
    }
  });

  it('counts injection results per source', () => {
    const port = new FakePort();
    const { source } = makeSource(port);
    source.connect(serialConfig());
    source.noteInjection(true);
    source.noteInjection(false);
    const status = source.getStatus();
    expect(status.rtcmForwarded).toBe(1);
    expect(status.rtcmDropped).toBe(1);
    source.disconnect();
  });
});
