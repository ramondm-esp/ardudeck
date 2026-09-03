import { describe, it, expect } from 'vitest';
import { scaleKeyFor, unitOfLabel, type YMode } from './log-y-scales';

/** The exact chart from the report: three altitudes, all in metres. */
const CTUN_ALTS = ['CTUN.DAlt (m)', 'CTUN.Alt (m)', 'CTUN.BAlt (m)'];

function keys(mode: YMode, labels: string[]): string[] {
  return labels.map((l, i) => scaleKeyFor(mode, l, i));
}

describe('unitOfLabel', () => {
  it('pulls the unit off a labelled series', () => {
    expect(unitOfLabel('CTUN.Alt (m)')).toBe('m');
    expect(unitOfLabel('ATT.Roll (deg)')).toBe('deg');
    expect(unitOfLabel('BAT.Volt (V)')).toBe('V');
  });

  it('is undefined when the log gave no unit', () => {
    expect(unitOfLabel('MODE.Mode')).toBeUndefined();
    expect(unitOfLabel('GPS.NSats')).toBeUndefined();
  });
});

describe('y scale grouping', () => {
  it('puts same-unit fields on one axis', () => {
    // Three metre readings on three independent axes is what made 0.739 and
    // 0.744 render a fifth of the panel apart.
    expect(new Set(keys('unit', CTUN_ALTS)).size).toBe(1);
  });

  it('separates fields that are not measured in the same thing', () => {
    const k = keys('unit', ['CTUN.Alt (m)', 'BAT.Volt (V)', 'ATT.Roll (deg)']);
    expect(new Set(k).size).toBe(3);
  });

  it('groups unitless fields together, apart from any unit', () => {
    const k = keys('unit', ['GPS.NSats', 'MODE.Mode', 'CTUN.Alt (m)']);
    expect(k[0]).toBe(k[1]);
    expect(k[2]).not.toBe(k[0]);
  });

  it('shared mode puts everything on one axis', () => {
    expect(new Set(keys('shared', ['CTUN.Alt (m)', 'BAT.Volt (V)'])).size).toBe(1);
  });

  it('field mode gives every field its own axis, even at the same unit', () => {
    expect(new Set(keys('field', CTUN_ALTS)).size).toBe(3);
  });
});
