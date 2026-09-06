import { describe, expect, it } from 'vitest';
import { locateTrainer, TRAINER_PATH_ENV, type LocateOptions } from './trainer-locator';

const HOME = '/Users/pilot';

function opts(present: string[], over: Partial<LocateOptions> = {}): LocateOptions {
  const set = new Set(present);
  return {
    exists: (p) => set.has(p),
    platform: 'darwin',
    homeDir: HOME,
    ...over,
  };
}

const APP_EXEC = '/Applications/ArduDeck Trainer.app/Contents/MacOS/ArduDeck Trainer';
const CHECKOUT = `${HOME}/work/ardudeck-game/apps/launcher`;
const CHECKOUT_ENTRY = `${CHECKOUT}/out/main/index.js`;
const CHECKOUT_ELECTRON = `${CHECKOUT}/node_modules/electron/dist/Electron.app/Contents/MacOS/Electron`;

describe('finding the Trainer', () => {
  it('finds an app the user installed', () => {
    const { target } = locateTrainer(opts([APP_EXEC]));
    expect(target).toEqual({
      kind: 'app',
      path: '/Applications/ArduDeck Trainer.app',
      exec: APP_EXEC,
    });
  });

  it('finds a development checkout, so this works before anything is packaged', () => {
    const { target } = locateTrainer(opts([CHECKOUT_ENTRY, CHECKOUT_ELECTRON]));
    expect(target).toMatchObject({ kind: 'checkout', path: CHECKOUT });
  });

  it('prefers an installed app to a checkout', () => {
    const { target } = locateTrainer(opts([APP_EXEC, CHECKOUT_ENTRY, CHECKOUT_ELECTRON]));
    expect(target?.kind).toBe('app');
  });

  it(`lets ${TRAINER_PATH_ENV} win over everything`, () => {
    const mine = '/opt/mine/ArduDeck Trainer.app';
    const { target } = locateTrainer(
      opts([`${mine}/Contents/MacOS/ArduDeck Trainer`, APP_EXEC], { override: mine }),
    );
    expect(target?.path).toBe(mine);
  });

  it('takes the cargo bundle over the default locations', () => {
    const cargo = '/Users/pilot/Library/Application Support/@ardudeck/desktop/modules/x/extracted';
    const { target } = locateTrainer(
      opts([`${cargo}/ArduDeck Trainer.app/Contents/MacOS/ArduDeck Trainer`, APP_EXEC], {
        cargoPath: cargo,
      }),
    );
    expect(target?.path).toBe(`${cargo}/ArduDeck Trainer.app`);
  });

  it('reports every place it looked when it finds nothing', () => {
    const { target, searched } = locateTrainer(opts([], { override: '/nope' }));
    expect(target).toBeNull();
    expect(searched[0]).toBe('/nope');
    expect(searched).toContain(CHECKOUT);
  });

  it('does not treat an unbuilt checkout as usable', () => {
    // A checkout that has never been built has a package.json and nothing to run, and
    // Electron's failure there is a blank window rather than an error.
    const { target } = locateTrainer(opts([`${CHECKOUT}/package.json`, CHECKOUT_ELECTRON]));
    expect(target).toBeNull();
  });

  it('does not run a checkout through an Electron it does not have', () => {
    const { target } = locateTrainer(opts([CHECKOUT_ENTRY]));
    expect(target).toBeNull();
  });

  it('finds a Windows build', () => {
    const dir = `${HOME}/AppData/Local/Programs/ArduDeck Trainer`;
    const exe = `${dir}/ArduDeck Trainer.exe`;
    const { target } = locateTrainer(opts([exe], { platform: 'win32' }));
    expect(target).toEqual({ kind: 'binary', path: exe, exec: exe });
  });

  it('finds a Linux build', () => {
    const bin = `${HOME}/.local/share/ardudeck-trainer/ardudeck-trainer`;
    const { target } = locateTrainer(opts([bin], { platform: 'linux' }));
    expect(target).toMatchObject({ kind: 'binary', exec: bin });
  });
});
