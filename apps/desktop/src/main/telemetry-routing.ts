/**
 * Which vehicle a telemetry message belongs to.
 *
 * Telemetry is normally scoped by the sender's `sysid.compid`, which is right
 * for anything the autopilot emits. Some messages come from a peripheral on the
 * same link instead: RADIO_STATUS is emitted by the telemetry MODEM as
 * MAV_COMP_ID_TELEMETRY_RADIO (68), and on ELRS from its own sysid entirely.
 * Scoped by sender, those land under a vehicle key that is never the active one
 * and the link quality never reaches the UI - the radio reports 100% while the
 * app shows no signal at all.
 */

/** MAV_COMP_ID_TELEMETRY_RADIO. */
export const COMP_ID_TELEMETRY_RADIO = 68;

/** RADIO_STATUS, and the legacy 3DR RADIO message that predates it. */
const RADIO_MSGIDS = new Set([109, 166]);

export function isRadioSourcedMessage(msgid: number, compid: number): boolean {
  return RADIO_MSGIDS.has(msgid) || compid === COMP_ID_TELEMETRY_RADIO;
}

/**
 * Key to file this message's telemetry under. Radio-sourced messages describe
 * the link to the vehicle the user is watching, so they follow the active
 * vehicle rather than inventing one per modem.
 */
export function telemetryKeyFor(
  msgid: number,
  compid: number,
  senderKey: string,
  activeKey: string | null,
): string {
  if (isRadioSourcedMessage(msgid, compid)) return activeKey ?? senderKey;
  return senderKey;
}
