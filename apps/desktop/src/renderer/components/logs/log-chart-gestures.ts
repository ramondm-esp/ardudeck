// Wheel handling shared by every chart gesture.
//
// A notched mouse wheel sends ~100 per event; a Mac trackpad sends a stream of
// 2-10px ones. A fixed zoom step per event therefore feels right on a mouse and
// makes a trackpad rocket past the target, so zoom is exponential in the delta
// and composes the same either way.

export interface WheelLike {
  deltaX: number;
  deltaY: number;
  deltaMode: number;
  ctrlKey: boolean;
}

/** Scroll deltas normalised to pixels, whatever deltaMode the device reports. */
export function wheelDeltas(e: WheelLike): { dominant: number; dx: number; dy: number } {
  // deltaMode: 1 = lines (~16px each), 2 = pages.
  const unit = e.deltaMode === 1 ? 16 : e.deltaMode === 2 ? 100 : 1;
  // macOS reroutes deltaY to deltaX while shift is held; take whichever moved.
  const dominant = (Math.abs(e.deltaY) >= Math.abs(e.deltaX) ? e.deltaY : e.deltaX) * unit;
  return { dominant, dx: e.deltaX * unit, dy: e.deltaY * unit };
}

/** Clamp on the delta so one flick of a high-resolution wheel can't jump scales. */
const MAX_WHEEL_PX = 240;
const ZOOM_GAIN = 0.002;
/** Pinch (ctrl+wheel) reports much smaller deltas, so it needs more gain. */
const PINCH_GAIN = 0.006;

/** Zoom multiplier for one wheel event. >1 zooms out, <1 zooms in. */
export function wheelZoomFactor(e: WheelLike): number {
  const { dominant } = wheelDeltas(e);
  const gain = e.ctrlKey ? PINCH_GAIN : ZOOM_GAIN;
  return Math.exp(Math.min(Math.max(dominant, -MAX_WHEEL_PX), MAX_WHEEL_PX) * gain);
}
