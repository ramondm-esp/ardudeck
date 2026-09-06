import { describe, it, expect } from 'vitest';
import { telemetryKeyFor, isRadioSourcedMessage, COMP_ID_TELEMETRY_RADIO } from '../telemetry-routing';

const VEHICLE = 'usb0:1.1';
const ELRS_RADIO = 'usb0:51.68';
const SIK_RADIO = 'usb0:1.68';

const RADIO_STATUS = 109;
const HEARTBEAT = 0;
const ATTITUDE = 30;

describe('radio-sourced messages', () => {
  it('recognises RADIO_STATUS and the legacy RADIO message', () => {
    expect(isRadioSourcedMessage(109, 1)).toBe(true);
    expect(isRadioSourcedMessage(166, 1)).toBe(true);
  });

  it('recognises anything from the telemetry radio component', () => {
    expect(isRadioSourcedMessage(HEARTBEAT, COMP_ID_TELEMETRY_RADIO)).toBe(true);
  });

  it('leaves autopilot messages alone', () => {
    expect(isRadioSourcedMessage(ATTITUDE, 1)).toBe(false);
    expect(isRadioSourcedMessage(HEARTBEAT, 1)).toBe(false);
  });
});

describe('telemetryKeyFor', () => {
  it('files ELRS link stats against the vehicle being watched', () => {
    // ELRS sends RADIO_STATUS from its own sysid AND compid 68. Keyed by
    // sender it lands on a phantom vehicle and the UI shows no signal while
    // the handset reads 100%.
    expect(telemetryKeyFor(RADIO_STATUS, COMP_ID_TELEMETRY_RADIO, ELRS_RADIO, VEHICLE)).toBe(VEHICLE);
  });

  it('does the same for a SiK modem on the vehicle sysid', () => {
    expect(telemetryKeyFor(RADIO_STATUS, COMP_ID_TELEMETRY_RADIO, SIK_RADIO, VEHICLE)).toBe(VEHICLE);
  });

  it('keeps autopilot telemetry on its own sender key', () => {
    // Fleet mode depends on this: two vehicles must not merge.
    expect(telemetryKeyFor(ATTITUDE, 1, VEHICLE, 'usb0:2.1')).toBe(VEHICLE);
  });

  it('falls back to the sender when no vehicle is active yet', () => {
    expect(telemetryKeyFor(RADIO_STATUS, COMP_ID_TELEMETRY_RADIO, ELRS_RADIO, null)).toBe(ELRS_RADIO);
  });
});
