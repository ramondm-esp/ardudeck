// The TX mixer is invisible to a GCS; the only way to learn which switch is
// which channel is to watch what moves. Port of mobile's switch_detect.dart.

export interface ChannelTravel {
  /** 1-based RC channel number. */
  channel: number;
  min: number;
  max: number;
  travel: number;
}

export class SwitchDetector {
  /** Below this a channel has not been moved deliberately: sticks jitter. */
  private readonly minimumTravel: number;
  private readonly min = new Map<number, number>();
  private readonly max = new Map<number, number>();

  constructor(minimumTravel = 120) {
    this.minimumTravel = minimumTravel;
  }

  reset(): void {
    this.min.clear();
    this.max.clear();
  }

  /** Feed one RC_CHANNELS frame as an array of microseconds (index 0 = CH1). */
  add(channels: number[]): void {
    for (let i = 0; i < channels.length; i++) {
      const pwm = channels[i]!;
      // 0 and 65535 are "no such channel" / "unknown" in RC_CHANNELS.
      if (pwm <= 0 || pwm >= 65535) continue;
      const ch = i + 1;
      const lo = this.min.get(ch);
      const hi = this.max.get(ch);
      if (lo === undefined || pwm < lo) this.min.set(ch, pwm);
      if (hi === undefined || pwm > hi) this.max.set(ch, pwm);
    }
  }

  /** Travel per channel, most moved first. */
  ranked(): ChannelTravel[] {
    const out: ChannelTravel[] = [];
    for (const [channel, lo] of this.min) {
      const hi = this.max.get(channel)!;
      out.push({ channel, min: lo, max: hi, travel: hi - lo });
    }
    out.sort((a, b) => b.travel - a.travel);
    return out;
  }

  /** Null until one channel clearly out-travels the rest (a stick moves two at once). */
  identified(): number | null {
    const ranked = this.ranked();
    if (ranked.length === 0) return null;
    const best = ranked[0]!;
    if (best.travel < this.minimumTravel) return null;
    if (ranked.length > 1 && ranked[1]!.travel * 2 > best.travel) return null;
    return best.channel;
  }
}

/** ArduPilot's fixed PWM bands mapping one channel to six flight-mode slots. */
export function modeSlotForPwm(pwm: number): number {
  if (pwm <= 1230) return 1;
  if (pwm <= 1360) return 2;
  if (pwm <= 1490) return 3;
  if (pwm <= 1620) return 4;
  if (pwm <= 1749) return 5;
  return 6;
}

/** Coarse switch position for display. */
export function switchPosition(pwm: number): 'low' | 'mid' | 'high' {
  if (pwm < 1300) return 'low';
  if (pwm > 1700) return 'high';
  return 'mid';
}

/** SERVOn_FUNCTION value that makes an output follow RC channel `ch` (RCIN passthrough). */
export function rcinPassthroughFunction(ch: number): number {
  return 50 + ch;
}
