/** RTK status for map instruments: one lazy IPC listener behind a module store. */
import { create } from 'zustand';
import { INITIAL_NTRIP_STATUS, type NtripStatus } from '../../../../shared/ntrip-types';

const useRtkStatusStore = create<{ status: NtripStatus }>(() => ({
  status: { ...INITIAL_NTRIP_STATUS },
}));

let wired = false;

function ensureWired(): void {
  if (wired) return;
  wired = true;
  void window.electronAPI.ntripGetStatus().then((status) => useRtkStatusStore.setState({ status }));
  window.electronAPI.onNtripStatus((status) => useRtkStatusStore.setState({ status }));
}

export function useRtkStatus(): NtripStatus {
  ensureWired();
  return useRtkStatusStore((s) => s.status);
}
