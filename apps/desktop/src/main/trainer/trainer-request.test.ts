import { describe, expect, it } from 'vitest';
import { buildTrainerRequest, TRAINER_REQUEST_VERSION } from './trainer-request';

const BAR = { lat: 42.0936, lon: 19.0947 };

describe('what ArduDeck tells the Trainer', () => {
  it('sends the take-off point and the version', () => {
    const out = buildTrainerRequest({ home: BAR });
    expect(out.ok && out.request).toMatchObject({
      version: TRAINER_REQUEST_VERSION,
      home: BAR,
    });
  });

  it('refuses without a take-off point, and says what to do', () => {
    const out = buildTrainerRequest({ home: null });
    expect(out.ok).toBe(false);
    expect(!out.ok && out.error).toMatch(/GPS fix/);
  });

  it('recognises the no-fix coordinate rather than passing it on', () => {
    // 0, 0 is in the Gulf of Guinea. Sent onward, the Trainer refuses with a puzzling message
    // about no region covering it, when the true answer is that there is no fix yet.
    const out = buildTrainerRequest({ home: { lat: 0, lon: 0 } });
    expect(!out.ok && out.error).toMatch(/no GPS fix/i);
  });

  it('sends both halves of the frame or neither', () => {
    const half = buildTrainerRequest({ home: BAR, frameClass: 1 });
    expect(half.ok && half.request.frame).toBeUndefined();

    const both = buildTrainerRequest({ home: BAR, frameClass: 1, frameType: 1, framePath: '/f.json' });
    expect(both.ok && both.request.frame).toEqual({
      frameClass: 1,
      frameType: 1,
      specPath: '/f.json',
    });
  });

  it('sends the layout without a spec file rather than dropping it', () => {
    const out = buildTrainerRequest({ home: BAR, frameClass: 2, frameType: 3 });
    expect(out.ok && out.request.frame).toEqual({ frameClass: 2, frameType: 3, specPath: null });
  });

  it('never sends fcOwner: asking is what decides it', () => {
    const out = buildTrainerRequest({ home: BAR });
    expect(out.ok && out.request).not.toHaveProperty('fcOwner');
  });

  it('leaves out what ArduDeck has no opinion about', () => {
    const out = buildTrainerRequest({ home: BAR });
    expect(out.ok && Object.keys(out.request).sort()).toEqual(['home', 'version']);
  });

  it('drops a non-finite altitude instead of sending NaN', () => {
    const out = buildTrainerRequest({ home: { ...BAR, altM: Number.NaN } });
    expect(out.ok && out.request.home).not.toHaveProperty('altM');
  });
});
