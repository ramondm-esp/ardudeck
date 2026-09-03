import { describe, it, expect } from 'vitest';
import {
  buildFlightTrack,
  groundAmsl,
  frameBounds,
  trackHorizontalSpanM,
  trackIndexAtTime,
  trackAltitudeRange,
} from './flight-track';

type Msg = { type: string; timeUs: number; fields: Record<string, number | string> };

function log(messages: Record<string, Msg[]>) {
  return { messages };
}

function gpsRows(alts: number[], opts: { lat?: number; lon?: number } = {}): Msg[] {
  return alts.map((alt, i) => ({
    type: 'GPS',
    timeUs: i * 1_000_000,
    fields: {
      Lat: (opts.lat ?? 47.1) + i * 1e-5,
      Lng: opts.lon ?? 8.5,
      Alt: alt,
      Spd: 1.5,
    },
  }));
}

describe('flight track altitude', () => {
  it('uses the height-above-home POS already carries', () => {
    const track = buildFlightTrack(log({
      POS: [0, 2.5, 4.8, 1.2].map((rel, i) => ({
        type: 'POS',
        timeUs: i * 1_000_000,
        fields: { Lat: 47.1 + i * 1e-5, Lng: 8.5, Alt: 500 + rel, RelHomeAlt: rel },
      })),
    }));

    expect(track.source).toBe('POS');
    expect(track.altitudeBasis).toBe('relative');
    expect(track.points.map((p) => p.altRel)).toEqual([0, 2.5, 4.8, 1.2]);
    expect(track.points[1]!.altAmsl).toBe(502.5);
  });

  it('does not report a 5 m hop as hundreds of metres', () => {
    // A GPS-only log at a 500 m site. Taking Alt as height above the takeoff
    // point is what stacked the flight on top of the terrain elevation.
    const track = buildFlightTrack(log({ GPS: gpsRows([500, 501, 504.5, 502, 500]) }));

    expect(track.source).toBe('GPS');
    expect(track.altitudeBasis).toBe('derived');
    expect(trackAltitudeRange(track.points).max).toBeCloseTo(4.5, 6);
    expect(trackAltitudeRange(track.points).min).toBeCloseTo(0, 6);
  });

  it('prefers POS over GPS when both are present', () => {
    expect(buildFlightTrack(log({
      GPS: gpsRows([500, 505]),
      POS: [0, 5].map((rel, i) => ({
        type: 'POS',
        timeUs: i * 1_000_000,
        fields: { Lat: 47.1, Lng: 8.5, Alt: 500 + rel, RelHomeAlt: rel },
      })),
    })).source).toBe('POS');
  });

  it('falls back to GPS when POS has no RelHomeAlt field', () => {
    const track = buildFlightTrack(log({
      GPS: gpsRows([500, 503]),
      POS: [0, 1].map((i) => ({
        type: 'POS',
        timeUs: i * 1_000_000,
        fields: { Lat: 47.1, Lng: 8.5, Alt: 500 + i * 3 },
      })),
    }));
    expect(track.source).toBe('GPS');
  });

  it('takes the ground datum from ORGN home when the log has one', () => {
    // A log that only starts recording once airborne has no on-ground samples,
    // so the opening-median datum would read the whole flight as level.
    const track = buildFlightTrack(log({
      GPS: gpsRows([540, 542, 545]),
      ORGN: [{ type: 'ORGN', timeUs: 0, fields: { Type: 1, Lat: 47.1, Lng: 8.5, Alt: 500 } }],
    }));

    expect(track.points.map((p) => p.altRel)).toEqual([40, 42, 45]);
  });

  it('ignores the EKF origin ORGN record', () => {
    const track = buildFlightTrack(log({
      GPS: gpsRows([500, 502]),
      ORGN: [{ type: 'ORGN', timeUs: 0, fields: { Type: 0, Lat: 47.1, Lng: 8.5, Alt: 123 } }],
    }));
    expect(track.points[0]!.altRel).toBeCloseTo(0, 6);
  });

  it('shrugs off one bad fix at power-up', () => {
    const alts = [900, ...Array.from({ length: 30 }, () => 500)];
    const track = buildFlightTrack(log({ GPS: gpsRows(alts) }));
    expect(track.points[1]!.altRel).toBeCloseTo(0, 6);
  });

  it('drops null-island and out-of-range fixes', () => {
    const rows = gpsRows([500, 500, 500, 500]);
    rows[1]!.fields['Lat'] = 0;
    rows[1]!.fields['Lng'] = 0;
    rows[2]!.fields['Lat'] = 412.7;
    const track = buildFlightTrack(log({ GPS: rows }));
    expect(track.points).toHaveLength(2);
    // Times stay attached to the points that survived, so a chart hover can't
    // land on the wrong vertex.
    expect(track.points.map((p) => p.timeS)).toEqual([0, 3]);
  });

  it('reads PX4 gps topics in their raw scaling', () => {
    const track = buildFlightTrack(log({
      sensor_gps: [0, 1, 2].map((i) => ({
        type: 'sensor_gps',
        timeUs: i * 1_000_000,
        fields: { lat: 471_000_000, lon: 85_000_000, alt: (500 + i * 3) * 1000, vel_m_s: 2 },
      })),
    }));

    expect(track.source).toBe('sensor_gps');
    expect(track.points[0]!.lat).toBeCloseTo(47.1, 6);
    expect(track.points[0]!.altAmsl).toBeCloseTo(500, 6);
    expect(track.points[2]!.altRel).toBeCloseTo(6, 6);
  });

  it('reads PX4 global position as degrees and metres', () => {
    const track = buildFlightTrack(log({
      vehicle_global_position: [0, 1].map((i) => ({
        type: 'vehicle_global_position',
        timeUs: i * 1_000_000,
        fields: { lat: 47.1, lon: 8.5, alt: 500 + i * 4 },
      })),
    }));
    expect(track.source).toBe('vehicle_global_position');
    expect(track.points[1]!.altRel).toBeCloseTo(4, 6);
  });

  it('returns an empty track for a log with no position at all', () => {
    expect(buildFlightTrack(log({ ATT: [] })).points).toEqual([]);
    expect(buildFlightTrack(null).points).toEqual([]);
  });
});

describe('ground elevation from the log', () => {
  it('recovers the takeoff ground level from POS', () => {
    const track = buildFlightTrack(log({
      POS: [0, 2.2].map((rel, i) => ({
        type: 'POS',
        timeUs: i * 1_000_000,
        fields: { Lat: 47.1, Lng: 8.5, Alt: 512.4 + rel, RelHomeAlt: rel },
      })),
    }));
    expect(groundAmsl(track.points)).toBeCloseTo(512.4, 4);
  });

  it('recovers it from a GPS-only log too', () => {
    const track = buildFlightTrack(log({ GPS: gpsRows([512, 514, 517]) }));
    expect(groundAmsl(track.points)).toBeCloseTo(512, 4);
  });

  it('returns null when the log carries no sea-level altitude', () => {
    expect(groundAmsl([
      { lat: 47.1, lon: 8.5, altRel: 3, altAmsl: null, timeS: 0, speed: null },
    ])).toBeNull();
    expect(groundAmsl([])).toBeNull();
  });
});

describe('track geometry helpers', () => {
  it('measures the horizontal span in metres', () => {
    const track = buildFlightTrack(log({ GPS: gpsRows([500, 500, 500]) }));
    // Three samples 1e-5 deg apart in latitude is a couple of metres.
    expect(trackHorizontalSpanM(track.points)).toBeGreaterThan(1);
    expect(trackHorizontalSpanM(track.points)).toBeLessThan(5);
  });

  it('pads a tiny flight out to a viewable frame', () => {
    const track = buildFlightTrack(log({ GPS: gpsRows([500, 500, 500]) }));
    const b = frameBounds(track.points, 90)!;
    const spanLat = (b.north - b.south) * 111_320;
    expect(spanLat).toBeCloseTo(90, 0);
    // Still centred on the flight.
    expect((b.north + b.south) / 2).toBeCloseTo(47.10001, 4);
  });

  it('leaves a flight bigger than the floor alone', () => {
    const rows = gpsRows(new Array(200).fill(500));
    const track = buildFlightTrack(log({ GPS: rows }));
    const b = frameBounds(track.points, 90)!;
    const spanLat = (b.north - b.south) * 111_320;
    expect(spanLat).toBeGreaterThan(200);
  });

  it('has no frame without points', () => {
    expect(frameBounds([], 90)).toBeNull();
  });

  it('finds the point at or before a time', () => {
    const track = buildFlightTrack(log({ GPS: gpsRows([500, 501, 502, 503]) }));
    expect(trackIndexAtTime(track.points, -5)).toBe(0);
    expect(trackIndexAtTime(track.points, 1.4)).toBe(1);
    expect(trackIndexAtTime(track.points, 2)).toBe(2);
    expect(trackIndexAtTime(track.points, 99)).toBe(3);
    expect(trackIndexAtTime([], 1)).toBe(-1);
  });
});
