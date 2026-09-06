import { describe, it, expect } from 'vitest';
import { SwitchDetector, modeSlotForPwm, rcinPassthroughFunction, switchPosition } from './switch-detect';

describe('SwitchDetector', () => {
  it('identifies a flicked switch', () => {
    const d = new SwitchDetector();
    d.add([1500, 1500, 1000, 1500, 1500, 1000]);
    d.add([1500, 1502, 1000, 1500, 1500, 2000]);
    expect(d.identified()).toBe(6);
  });

  it('stays silent below the deliberate-travel threshold', () => {
    const d = new SwitchDetector();
    d.add([1500, 1500, 1000]);
    d.add([1510, 1495, 1080]);
    expect(d.identified()).toBeNull();
  });

  it('refuses to pick when two channels move together (stick)', () => {
    const d = new SwitchDetector();
    d.add([1500, 1500]);
    d.add([1900, 1850]);
    expect(d.identified()).toBeNull();
  });

  it('ignores 0 and 65535 no-channel markers', () => {
    const d = new SwitchDetector();
    d.add([1500, 0, 65535]);
    d.add([1500, 65535, 0]);
    expect(d.identified()).toBeNull();
  });

  it('resets cleanly between attempts', () => {
    const d = new SwitchDetector();
    d.add([1000]);
    d.add([2000]);
    expect(d.identified()).toBe(1);
    d.reset();
    d.add([1500]);
    expect(d.identified()).toBeNull();
  });
});

describe('helpers', () => {
  it('mode slots follow ArduPilot band edges', () => {
    expect(modeSlotForPwm(1230)).toBe(1);
    expect(modeSlotForPwm(1231)).toBe(2);
    expect(modeSlotForPwm(1490)).toBe(3);
    expect(modeSlotForPwm(1749)).toBe(5);
    expect(modeSlotForPwm(1750)).toBe(6);
  });

  it('RCIN passthrough function is 50 + channel', () => {
    expect(rcinPassthroughFunction(7)).toBe(57);
    expect(rcinPassthroughFunction(16)).toBe(66);
  });

  it('switch position bands', () => {
    expect(switchPosition(1000)).toBe('low');
    expect(switchPosition(1500)).toBe('mid');
    expect(switchPosition(2000)).toBe('high');
  });
});
