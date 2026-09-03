import { describe, it, expect, beforeEach } from 'vitest';
import { useParameterStore } from './parameter-store';
import type { ParamValuePayload } from '../../shared/parameter-types';

const REAL32 = 9;

/**
 * The connect-time batch reads that put real parameters in the store before any
 * download runs: five from the safety monitor, one from the flight-control
 * panel. Six is exactly the count that showed up under "Unrecognized PID
 * Parameters" with no download having happened.
 */
const STRAY_READS = [
  'GCS_PID_MASK',
  'ATC_RAT_RLL_IMAX',
  'ATC_RAT_PIT_IMAX',
  'AHRS_TRIM_X',
  'AHRS_TRIM_Y',
  'Q_ENABLE',
];

function payload(paramId: string, index: number): ParamValuePayload {
  return { paramId, paramValue: 1, paramType: REAL32, paramCount: 1200, paramIndex: index };
}

describe('parameter download state', () => {
  beforeEach(() => {
    useParameterStore.getState().reset();
  });

  it('starts idle with no full set', () => {
    expect(useParameterStore.getState().downloadState).toBe('idle');
    expect(useParameterStore.getState().hasFullParameterSet()).toBe(false);
  });

  it('does not count connect-time batch reads as a loaded parameter set', () => {
    const store = useParameterStore.getState();
    STRAY_READS.forEach((id, i) => store.updateParameter(payload(id, i)));

    expect(useParameterStore.getState().paramCount).toBe(6);
    expect(useParameterStore.getState().hasFullParameterSet()).toBe(false);
  });

  it('is complete once the FTP bulk load lands', () => {
    useParameterStore.getState().bulkLoadParameters([
      payload('ATC_RAT_RLL_P', 0),
      payload('ATC_RAT_PIT_P', 1),
    ]);
    expect(useParameterStore.getState().hasFullParameterSet()).toBe(true);
  });

  it('is complete once a streamed download finishes', () => {
    const store = useParameterStore.getState();
    store.updateParameter(payload('ATC_RAT_RLL_P', 0));
    expect(useParameterStore.getState().hasFullParameterSet()).toBe(false);

    useParameterStore.getState().setComplete();
    expect(useParameterStore.getState().hasFullParameterSet()).toBe(true);
  });

  it('marks a download that errored as failed so the next attempt runs', () => {
    useParameterStore.setState({ downloadState: 'loading' });
    useParameterStore.getState().setError('Timeout: received 6/1200 parameters');

    expect(useParameterStore.getState().downloadState).toBe('failed');
    expect(useParameterStore.getState().hasFullParameterSet()).toBe(false);
  });

  it('leaves a completed download alone when an unrelated error is set', () => {
    // A failed PARAM_SET must not invalidate a parameter set already in hand.
    useParameterStore.getState().bulkLoadParameters([payload('ATC_RAT_RLL_P', 0)]);
    useParameterStore.getState().setError('Parameter "FOO" not found on this board.');

    expect(useParameterStore.getState().hasFullParameterSet()).toBe(true);
  });

  it('does not start a second download while one is running', () => {
    useParameterStore.setState({ downloadState: 'loading' });
    expect(useParameterStore.getState().needsParameterFetch()).toBe(false);
  });

  it('lets a failed download be retried but not a completed one', () => {
    useParameterStore.setState({ downloadState: 'failed' });
    expect(useParameterStore.getState().needsParameterFetch()).toBe(true);
    useParameterStore.setState({ downloadState: 'complete' });
    expect(useParameterStore.getState().needsParameterFetch()).toBe(false);
  });

  it('goes back to idle on reset', () => {
    useParameterStore.getState().bulkLoadParameters([payload('ATC_RAT_RLL_P', 0)]);
    useParameterStore.getState().reset();

    expect(useParameterStore.getState().downloadState).toBe('idle');
    expect(useParameterStore.getState().hasFullParameterSet()).toBe(false);
  });
});
