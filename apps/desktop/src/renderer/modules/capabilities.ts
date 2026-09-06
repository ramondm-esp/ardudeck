import { useMemo } from 'react';
import type { ViewId } from '../stores/navigation-store';
import { useModuleStore } from '../stores/module-store';

/**
 * Built-in features gated behind Hangar cargo.
 *
 * Anything NOT listed here is always available. To put an existing built-in
 * feature behind a cargo, add one entry mapping the cargo slug to what it
 * gates - a whole view, individual HUD widgets, and/or text-OSD elements.
 * NavigationRail, the deep-link handler, the HUD renderers and the OSD
 * element browser all consult this map; no other code change is needed.
 *
 * A gated feature shows only while its cargo is installed AND not toggled
 * off, so the Cargo Bay enable/disable switch governs built-ins too.
 */
export interface Capability {
  /** Hangar cargo slug that enables the features below. */
  slug: string;
  /** A built-in view this cargo gates. */
  viewId?: ViewId;
  /** Built-in fighter-HUD widget ids (see hud-config HUD_WIDGETS). */
  hudWidgets?: string[];
  /** Built-in text-OSD element ids (see osd element-registry). */
  osdElements?: string[];
}

/** Cargo slug that enables the entire Fleet Vault surface. */
export const VAULT_CARGO_SLUG = 'com.ardudeck.vault';
/** Cargo slug that enables the Pre-Flight Weather Briefing surface. */
export const WEATHER_CARGO_SLUG = 'com.ardudeck.weather';

/**
 * Whether the Pre-Flight Weather Briefing view is reachable. Gated on its cargo:
 * the view and its settings-card "View in detail" entry appear only while the
 * Weather Briefing cargo is installed and not toggled off.
 */
export function isWeatherBriefingAvailable(): boolean {
  return isCargoEnabled(WEATHER_CARGO_SLUG);
}
/** Cargo slug that enables the Mission Library surface. */
export const MISSION_LIBRARY_CARGO_SLUG = 'com.ardudeck.mission-library';
// Public API-key Claude Advisor cargo. Enables both the live advisor panel and
// the AI flight-log analysis surfaces. Note the slug is `ardudeck.advisor`, NOT
// the `com.ardudeck.*` convention used by the other cargo above.
export const ADVISOR_CARGO_SLUG = 'ardudeck.advisor';

/**
 * Cargo slug that delivers the ArduDeck Trainer.
 *
 * Unlike the other entries here this cargo is INSTALLABLE, not activatable: the view ships in
 * this app, but the thing it launches is a downloaded bundle. The gate is the same either way,
 * because it only asks whether the cargo is present and enabled, so a Trainer view with nothing
 * behind it never appears in the rail.
 *
 * Must match `TRAINER_CARGO_SLUG` in `main/trainer/trainer-locator.ts`.
 */
export const TRAINER_CARGO_SLUG = 'com.ardudeck.trainer';

export const CAPABILITIES: Capability[] = [
  // Example (not active): { slug: 'com.ardudeck.area-editor', viewId: 'mission' },
  // Fleet Vault: nav view plus every vault surface embedded in other screens
  // (sync badges, auto-backup chips) - those consult useCargoEnabled directly.
  { slug: VAULT_CARGO_SLUG, viewId: 'vault' },
  // Mission Library: nav view plus the "Save to Library" entry in the mission
  // toolbar's save menu (gated via useCargoEnabled in MissionToolbar).
  { slug: MISSION_LIBRARY_CARGO_SLUG, viewId: 'library' },
  // Pre-Flight Weather Briefing: nav view plus the "View in detail" button on
  // the Vehicle & Status weather card (gated via isWeatherBriefingAvailable).
  { slug: WEATHER_CARGO_SLUG, viewId: 'weather' },
  // ArduDeck Trainer: the nav view plus its quick action on the SITL screen.
  { slug: TRAINER_CARGO_SLUG, viewId: 'trainer' },
];

const GATED_VIEWS: ReadonlyMap<ViewId, string> = new Map(
  CAPABILITIES.filter((c) => c.viewId).map((c): [ViewId, string] => [c.viewId!, c.slug]),
);

/** True if `viewId` is available given the set of enabled activatable slugs. */
export function isViewAvailable(viewId: ViewId, enabledSlugs: ReadonlySet<string>): boolean {
  const requiredSlug = GATED_VIEWS.get(viewId);
  return !requiredSlug || enabledSlugs.has(requiredSlug);
}

/** Reactive: true when the given cargo is installed and not toggled off. */
export function useCargoEnabled(slug: string): boolean {
  const modules = useModuleStore((s) => s.modules);
  return useMemo(
    () => modules.some((m) => m.slug === slug && m.enabled !== false),
    [modules, slug],
  );
}

/** Non-hook variant for imperative call sites (event handlers, stores). */
export function isCargoEnabled(slug: string): boolean {
  return useModuleStore.getState().modules.some((m) => m.slug === slug && m.enabled !== false);
}

/** Ids from the chosen Capability field whose gating cargo is missing or off. */
function useGatedOffIds(field: 'hudWidgets' | 'osdElements'): ReadonlySet<string> {
  const modules = useModuleStore((s) => s.modules);
  return useMemo(() => {
    const off = new Set<string>();
    for (const cap of CAPABILITIES) {
      const ids = cap[field];
      if (!ids?.length) continue;
      const enabled = modules.some((m) => m.slug === cap.slug && m.enabled !== false);
      if (!enabled) for (const id of ids) off.add(id);
    }
    return off;
  }, [modules, field]);
}

/** Reactive: built-in HUD widget ids to hide (their gating cargo is off/absent). */
export function useGatedOffHudWidgets(): ReadonlySet<string> {
  return useGatedOffIds('hudWidgets');
}

/** Reactive: built-in text-OSD element ids to hide (their gating cargo is off/absent). */
export function useGatedOffOsdElements(): ReadonlySet<string> {
  return useGatedOffIds('osdElements');
}

/**
 * Reactive set of module slugs currently enabled on this device. A slug only
 * has gating power when it appears in CAPABILITIES, so including installable
 * (bundle-shipping) cargo alongside activatable cargo is harmless and lets a
 * bundle cargo gate built-in surfaces too (the Fleet Vault pattern).
 */
export function useEnabledCapabilitySlugs(): ReadonlySet<string> {
  const modules = useModuleStore((s) => s.modules);
  return useMemo(
    () => new Set(modules.filter((m) => m.enabled !== false).map((m) => m.slug)),
    [modules],
  );
}
