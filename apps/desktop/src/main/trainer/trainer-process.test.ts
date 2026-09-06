import { describe, expect, it } from 'vitest';
import { trainerCommand } from './trainer-process';

describe('how each shape of Trainer is started', () => {
  it('runs an installed app directly', () => {
    const exec = '/Applications/ArduDeck Trainer.app/Contents/MacOS/ArduDeck Trainer';
    expect(trainerCommand({ kind: 'app', path: '/Applications/x.app', exec }, '/tmp/r.json')).toEqual({
      command: exec,
      args: ['--trainer-request=/tmp/r.json'],
    });
  });

  it('runs a checkout through its OWN Electron, with the directory as the app', () => {
    // A global electron would be a different major version against a pinned preload, and that
    // failure is a blank window rather than an error.
    const target = { kind: 'checkout', path: '/src/launcher', electron: '/src/launcher/node_modules/electron/dist/electron' } as const;
    expect(trainerCommand(target, '/tmp/r.json')).toEqual({
      command: target.electron,
      args: ['/src/launcher', '--trainer-request=/tmp/r.json'],
    });
  });

  it('passes a path with spaces as one argument, not a quoted string', () => {
    const exec = '/Applications/ArduDeck Trainer.app/Contents/MacOS/ArduDeck Trainer';
    const { args } = trainerCommand({ kind: 'app', path: '/x.app', exec }, '/Users/a b/r.json');
    expect(args).toEqual(['--trainer-request=/Users/a b/r.json']);
  });
});
