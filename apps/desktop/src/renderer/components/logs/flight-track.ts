/**
 * Flight track extraction for the log explorer's 3D path.
 *
 * The altitude a log carries is almost always AMSL. The map draws the track
 * above the terrain surface, so it needs height above the takeoff point; using
 * AMSL there stacks the vehicle's sea-level altitude on top of the terrain's
 * and puts a 5 m hop hundreds of metres in the air.
 */

export interface TrackPoint {
  lat: number;
  lon: number;
  /** Metres above the takeoff point. This is what the map draws. */
  altRel: number;
  /** Metres above mean sea level, when the source carries it. */
  altAmsl: number | null;
  /** Seconds on the log's clock, matching the chart x axis. */
  timeS: number;
  /** Ground speed in m/s, when the source carries it. */
  speed: number | null;
}

export type AltitudeBasis =
  /** The log had a real height-above-home field. */
  | 'relative'
  /** AMSL minus a datum taken from the log's own start. */
  | 'derived';

export interface FlightTrack {
  points: TrackPoint[];
  /** Log message the track came from, so the UI can say which. */
  source: string;
  altitudeBasis: AltitudeBasis;
}

export const EMPTY_TRACK: FlightTrack = { points: [], source: '', altitudeBasis: 'derived' };

type LogMessages = Record<string, { type: string; timeUs: number; fields: Record<string, number | string> }[]>;

export function isValidLatLon(lat: number, lon: number): boolean {
  if (!Number.isFinite(lat) || !Number.isFinite(lon)) return false;
  if (lat === 0 && lon === 0) return false;
  return Math.abs(lat) <= 90 && Math.abs(lon) <= 180;
}

function num(v: number | string | undefined): number | null {
  return typeof v === 'number' && Number.isFinite(v) ? v : null;
}

/**
 * Ground datum for sources with no relative-altitude field. A flight begins and
 * ends on the ground, so the bottom of the altitude distribution is the ground.
 * The 5th percentile rather than the true minimum, so one glitched low fix
 * can't lift the whole track.
 */
function groundDatum(alts: number[]): number {
  if (alts.length === 0) return 0;
  const sorted = [...alts].sort((a, b) => a - b);
  return sorted[Math.floor(0.05 * (sorted.length - 1))]!;
}

/**
 * ArduPilot logs home as an ORGN record with Type 1 (Type 0 is the EKF
 * origin). When present it is the exact ground reference, including for a log
 * that only starts recording once the vehicle is already airborne.
 */
function ardupilotHomeAlt(messages: LogMessages): number | null {
  const orgn = messages['ORGN'];
  if (!orgn) return null;
  for (const msg of orgn) {
    // Alt arrives in metres: the parser's 'e' format char already applied the
    // x0.01 that turns ArduPilot's stored centimetres into metres.
    if (num(msg.fields['Type']) === 1) {
      const alt = num(msg.fields['Alt']);
      if (alt !== null) return alt;
    }
  }
  return null;
}

/** ArduPilot POS: EKF-fused position with height above home already in it. */
function fromArduPilotPos(messages: LogMessages): FlightTrack | null {
  const pos = messages['POS'];
  if (!pos || pos.length < 2) return null;

  const points: TrackPoint[] = [];
  let sawRelHome = false;
  for (const msg of pos) {
    const lat = num(msg.fields['Lat']);
    const lon = num(msg.fields['Lng']);
    if (lat === null || lon === null || !isValidLatLon(lat, lon)) continue;
    const relHome = num(msg.fields['RelHomeAlt']);
    if (relHome !== null) sawRelHome = true;
    points.push({
      lat,
      lon,
      altRel: relHome ?? 0,
      altAmsl: num(msg.fields['Alt']),
      timeS: msg.timeUs / 1e6,
      speed: null,
    });
  }
  if (points.length < 2 || !sawRelHome) return null;
  return { points, source: 'POS', altitudeBasis: 'relative' };
}

/** ArduPilot GPS: AMSL only, so the ground reference has to be recovered. */
function fromArduPilotGps(messages: LogMessages): FlightTrack | null {
  const gps = messages['GPS'];
  if (!gps || gps.length < 2) return null;

  const raw: { lat: number; lon: number; amsl: number; timeS: number; speed: number | null }[] = [];
  for (const msg of gps) {
    const lat = num(msg.fields['Lat']);
    const lon = num(msg.fields['Lng']);
    if (lat === null || lon === null || !isValidLatLon(lat, lon)) continue;
    raw.push({
      lat,
      lon,
      amsl: num(msg.fields['Alt']) ?? 0,
      timeS: msg.timeUs / 1e6,
      speed: num(msg.fields['Spd']),
    });
  }
  if (raw.length < 2) return null;

  const datum = ardupilotHomeAlt(messages) ?? groundDatum(raw.map((r) => r.amsl));
  return {
    points: raw.map((r) => ({
      lat: r.lat,
      lon: r.lon,
      altRel: r.amsl - datum,
      altAmsl: r.amsl,
      timeS: r.timeS,
      speed: r.speed,
    })),
    source: 'GPS',
    altitudeBasis: 'derived',
  };
}

/** PX4 topics, all AMSL. `scale` normalises lat/lon and altitude to deg / m. */
function fromPx4(
  messages: LogMessages,
  topic: string,
  latField: string,
  lonField: string,
  altField: string,
  speedField: string | null,
  coordScale: number,
  altScale: number,
): FlightTrack | null {
  const rows = messages[topic];
  if (!rows || rows.length < 2) return null;

  const raw: { lat: number; lon: number; amsl: number; timeS: number; speed: number | null }[] = [];
  for (const msg of rows) {
    const rawLat = num(msg.fields[latField]);
    const rawLon = num(msg.fields[lonField]);
    if (rawLat === null || rawLon === null) continue;
    const lat = rawLat / coordScale;
    const lon = rawLon / coordScale;
    if (!isValidLatLon(lat, lon)) continue;
    raw.push({
      lat,
      lon,
      amsl: (num(msg.fields[altField]) ?? 0) / altScale,
      timeS: msg.timeUs / 1e6,
      speed: speedField ? num(msg.fields[speedField]) : null,
    });
  }
  if (raw.length < 2) return null;

  const datum = groundDatum(raw.map((r) => r.amsl));
  return {
    points: raw.map((r) => ({
      lat: r.lat,
      lon: r.lon,
      altRel: r.amsl - datum,
      altAmsl: r.amsl,
      timeS: r.timeS,
      speed: r.speed,
    })),
    source: topic,
    altitudeBasis: 'derived',
  };
}

/**
 * Best available track for a parsed log. Sources are tried in order of how
 * well they answer "how high above the takeoff point was it": POS carries that
 * directly, everything else has to derive it.
 */
export function buildFlightTrack(log: { messages: LogMessages } | null | undefined): FlightTrack {
  if (!log) return EMPTY_TRACK;
  const m = log.messages;

  return (
    fromArduPilotPos(m) ??
    fromArduPilotGps(m) ??
    fromPx4(m, 'vehicle_global_position', 'lat', 'lon', 'alt', null, 1, 1) ??
    fromPx4(m, 'vehicle_gps_position', 'lat', 'lon', 'alt', 'vel_m_s', 1e7, 1000) ??
    fromPx4(m, 'sensor_gps', 'lat', 'lon', 'alt', 'vel_m_s', 1e7, 1000) ??
    EMPTY_TRACK
  );
}

/** Vertical extent of the track, used to size the drawn path against the flight. */
export function trackAltitudeRange(points: TrackPoint[]): { min: number; max: number } {
  if (points.length === 0) return { min: 0, max: 0 };
  let min = Infinity;
  let max = -Infinity;
  for (const p of points) {
    if (p.altRel < min) min = p.altRel;
    if (p.altRel > max) max = p.altRel;
  }
  return { min, max };
}

/**
 * Horizontal span of the track in metres. The path ribbon is sized from this:
 * a fixed 6 m ribbon is wider than the whole flight when someone hovers in a
 * back garden, which is what made short flights render as a solid slab.
 */
export function trackHorizontalSpanM(points: TrackPoint[]): number {
  if (points.length < 2) return 0;
  let minLat = Infinity, maxLat = -Infinity, minLon = Infinity, maxLon = -Infinity;
  for (const p of points) {
    if (p.lat < minLat) minLat = p.lat;
    if (p.lat > maxLat) maxLat = p.lat;
    if (p.lon < minLon) minLon = p.lon;
    if (p.lon > maxLon) maxLon = p.lon;
  }
  const midLat = (minLat + maxLat) / 2;
  const dLat = (maxLat - minLat) * 111_320;
  const dLon = (maxLon - minLon) * 111_320 * Math.cos((midLat * Math.PI) / 180);
  return Math.hypot(dLat, dLon);
}

/** Index of the track point at or before `timeS`, or -1 when the track is empty. */
export function trackIndexAtTime(points: TrackPoint[], timeS: number): number {
  if (points.length === 0) return -1;
  let lo = 0;
  let hi = points.length - 1;
  if (timeS <= points[0]!.timeS) return 0;
  if (timeS >= points[hi]!.timeS) return hi;
  while (lo < hi) {
    const mid = (lo + hi + 1) >> 1;
    if (points[mid]!.timeS <= timeS) lo = mid;
    else hi = mid - 1;
  }
  return lo;
}

/**
 * Ground speed per point in m/s. Sources that log it win; POS does not, so the
 * rest is derived from consecutive fixes rather than left blank.
 */
export function trackSpeeds(points: TrackPoint[]): number[] {
  const speeds = new Array<number>(points.length).fill(0);
  for (let i = 0; i < points.length; i++) {
    const logged = points[i]!.speed;
    if (logged !== null) {
      speeds[i] = logged;
      continue;
    }
    const a = points[i === 0 ? 0 : i - 1]!;
    const b = points[i === 0 ? Math.min(1, points.length - 1) : i]!;
    const dt = b.timeS - a.timeS;
    if (dt <= 0) {
      speeds[i] = i > 0 ? speeds[i - 1]! : 0;
      continue;
    }
    const midLat = ((a.lat + b.lat) / 2) * (Math.PI / 180);
    const dx = (b.lon - a.lon) * 111_320 * Math.cos(midLat);
    const dy = (b.lat - a.lat) * 111_320;
    speeds[i] = Math.hypot(dx, dy) / dt;
  }
  return speeds;
}

/**
 * Sea-level altitude of the ground under the takeoff point, from the log's own
 * measurements. The vehicle was sitting there and recorded both its AMSL and
 * its height above that spot, so the difference is the ground.
 *
 * The map needs this to place the track on the terrain surface. Querying the
 * DEM instead loses the race with tile loading and silently yields 0, which
 * buries the whole flight at sea level under the hill it was flown on.
 */
export function groundAmsl(points: TrackPoint[]): number | null {
  for (const p of points) {
    if (p.altAmsl !== null) return p.altAmsl - p.altRel;
  }
  return null;
}

/**
 * Lon/lat bounds to frame the track in, never smaller than `minSpanM` across.
 *
 * A back-garden hover spans a few metres; fitting that literally pins the
 * camera to maximum zoom, where a terrain-enabled map puts the viewpoint at (or
 * under) the ground and a 2 m track fills the screen. Padding to a floor keeps
 * short flights legible and in context.
 */
export function frameBounds(
  points: TrackPoint[],
  minSpanM = 90,
): { west: number; south: number; east: number; north: number } | null {
  if (points.length === 0) return null;
  let minLat = Infinity, maxLat = -Infinity, minLon = Infinity, maxLon = -Infinity;
  for (const p of points) {
    if (p.lat < minLat) minLat = p.lat;
    if (p.lat > maxLat) maxLat = p.lat;
    if (p.lon < minLon) minLon = p.lon;
    if (p.lon > maxLon) maxLon = p.lon;
  }

  const midLat = (minLat + maxLat) / 2;
  const mPerDegLat = 111_320;
  const mPerDegLon = Math.max(1, 111_320 * Math.cos((midLat * Math.PI) / 180));

  const halfLat = Math.max((maxLat - minLat) / 2, minSpanM / 2 / mPerDegLat);
  const halfLon = Math.max((maxLon - minLon) / 2, minSpanM / 2 / mPerDegLon);
  const midLon = (minLon + maxLon) / 2;

  return {
    west: midLon - halfLon,
    south: midLat - halfLat,
    east: midLon + halfLon,
    north: midLat + halfLat,
  };
}
