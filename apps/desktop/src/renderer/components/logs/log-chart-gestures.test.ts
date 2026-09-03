import { describe, it, expect } from 'vitest';
import { wheelDeltas, wheelZoomFactor, type WheelLike } from './log-chart-gestures';

function wheel(over: Partial<WheelLike>): WheelLike {
  return { deltaX: 0, deltaY: 0, deltaMode: 0, ctrlKey: false, ...over };
}

describe('wheelDeltas', () => {
  it('normalises line and page deltas to pixels', () => {
    expect(wheelDeltas(wheel({ deltaY: 3, deltaMode: 1 })).dominant).toBe(48);
    expect(wheelDeltas(wheel({ deltaY: 1, deltaMode: 2 })).dominant).toBe(100);
    expect(wheelDeltas(wheel({ deltaY: 12 })).dominant).toBe(12);
  });

  it('normalises each axis independently for the pan-vs-zoom decision', () => {
    const d = wheelDeltas(wheel({ deltaX: 3, deltaY: 1, deltaMode: 1 }));
    expect(d.dx).toBe(48);
    expect(d.dy).toBe(16);
  });

  it('takes the axis that actually moved', () => {
    // macOS reroutes deltaY to deltaX while shift is held.
    expect(wheelDeltas(wheel({ deltaX: 30, deltaY: 0 })).dominant).toBe(30);
    expect(wheelDeltas(wheel({ deltaX: 2, deltaY: -40 })).dominant).toBe(-40);
  });
});

describe('wheelZoomFactor', () => {
  it('gives a notched mouse a decisive step', () => {
    const f = wheelZoomFactor(wheel({ deltaY: 100 }));
    expect(f).toBeGreaterThan(1.15);
    expect(f).toBeLessThan(1.3);
  });

  it('gives a trackpad tick a small step so a stream of them lands', () => {
    // The whole point: ~1% per event, not ~12%, or a flick overshoots wildly.
    const f = wheelZoomFactor(wheel({ deltaY: 4 }));
    expect(f).toBeGreaterThan(1.005);
    expect(f).toBeLessThan(1.02);
  });

  it('composes: many small ticks equal one big one', () => {
    const many = Array.from({ length: 25 }, () => wheelZoomFactor(wheel({ deltaY: 4 })))
      .reduce((a, b) => a * b, 1);
    expect(many).toBeCloseTo(wheelZoomFactor(wheel({ deltaY: 100 })), 6);
  });

  it('inverts direction', () => {
    expect(wheelZoomFactor(wheel({ deltaY: -100 }))).toBeCloseTo(1 / wheelZoomFactor(wheel({ deltaY: 100 })), 10);
  });

  it('clamps one huge flick', () => {
    expect(wheelZoomFactor(wheel({ deltaY: 100_000 }))).toBeCloseTo(Math.exp(240 * 0.002), 10);
  });

  it('gives pinch more gain than scroll', () => {
    expect(wheelZoomFactor(wheel({ deltaY: 10, ctrlKey: true })))
      .toBeGreaterThan(wheelZoomFactor(wheel({ deltaY: 10 })));
  });
});
