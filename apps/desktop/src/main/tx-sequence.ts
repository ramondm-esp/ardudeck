/**
 * MAVLink wants one sequence counter per link. A single counter shared across
 * every open transport punctures each link's stream, so a router or FC counting
 * gaps reports uplink loss ArduDeck never caused.
 */
const txSeqByLink = new WeakMap<object, number>();

/** Next sequence byte for `link`, or undefined when there is no link to key on. */
export function nextTxSeq(link: object | null | undefined): number | undefined {
  if (!link) return undefined;
  const next = ((txSeqByLink.get(link) ?? -1) + 1) & 0xff;
  txSeqByLink.set(link, next);
  return next;
}
