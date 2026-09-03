import { describe, it, expect } from 'vitest';
import { nextTxSeq } from '../tx-sequence';

describe('per-link TX sequence numbering', () => {
  it('gives each link a contiguous stream when sends interleave', () => {
    const linkA = {};
    const linkB = {};
    const a: number[] = [];
    const b: number[] = [];

    for (let i = 0; i < 10; i++) {
      a.push(nextTxSeq(linkA)!);
      b.push(nextTxSeq(linkB)!);
      b.push(nextTxSeq(linkB)!);
    }

    expect(a).toEqual([0, 1, 2, 3, 4, 5, 6, 7, 8, 9]);
    expect(b).toEqual(Array.from({ length: 20 }, (_, i) => i));
  });

  it('wraps at 255', () => {
    const link = {};
    let last = -1;
    for (let i = 0; i < 258; i++) last = nextTxSeq(link)!;
    expect(last).toBe(1);
  });

  it('falls back to the serializer default when no link is known', () => {
    expect(nextTxSeq(null)).toBeUndefined();
    expect(nextTxSeq(undefined)).toBeUndefined();
  });
});
