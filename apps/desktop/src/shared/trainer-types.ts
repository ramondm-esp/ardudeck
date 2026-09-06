/** Shapes shared between the Trainer's main-process module and its renderer view. */

export interface TrainerStatus {
  installed: boolean;
  /** How it will be started, for the diagnostics line. Null when not installed. */
  kind: 'app' | 'binary' | 'checkout' | null;
  path: string | null;
  /** Every place looked. Shown when nothing was found, so this never fails silently. */
  searched: string[];
  home: { lat: number; lon: number; altM?: number | null; headingDeg?: number | null } | null;
  canLaunch: boolean;
  /** Why not, in words a pilot can act on. Null when it can. */
  reason: string | null;
}

/** What the view can override for one flight. Everything absent stays the Trainer's own. */
export interface TrainerLaunchInput {
  region?: string | null;
  camera?: { kind: string; tiltDeg?: number; lensFovDeg?: number } | null;
  stream?: { enabled: boolean; port?: number } | null;
  fullscreen?: boolean;
}
