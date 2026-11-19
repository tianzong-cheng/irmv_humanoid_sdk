/**
 * The shared time reducer. Every chart reads the same window and cursor; this
 * module only decides their next value. It reads no clock: the caller supplies
 * a monotonic milliseconds value with each step, so a playback test and a real
 * animation frame follow exactly the same rules.
 *
 * All absolute times are BigInt. A window moves by a BigInt delta computed from
 * real elapsed milliseconds and the playback speed; it never passes through a
 * floating absolute timestamp.
 */

import { Us } from './api.js';

export const SPEEDS = [0.25, 1, 2, 4];

function LiveWindow(coverage) {
  const beginUs = coverage.endUs - 10000000n;
  return {
    beginUs: beginUs < coverage.beginUs ? coverage.beginUs : beginUs,
    endUs: coverage.endUs,
  };
}

function WindowContaining(window, cursor, coverage) {
  if (cursor >= window.beginUs && cursor <= window.endUs) return window;
  const span = window.endUs - window.beginUs;
  let begin = cursor - span / 2n;
  if (begin < coverage.beginUs) begin = coverage.beginUs;
  if (begin + span > coverage.endUs) begin = coverage.endUs - span;
  return { beginUs: begin, endUs: begin + span };
}

function InitialState() {
  return {
    mode: 'history',
    following: false,
    playing: false,
    speed: 1,
    window: { beginUs: 0n, endUs: 1n },
    cursor: 0n,
    coverage: { beginUs: 0n, endUs: 1n },
    runId: null,
    lastAdvanceMs: 0,
    lastReceivedMs: null,
    offline: false,
  };
}

/** Scale a time window about an anchor, retaining its screen fraction. */
export function ZoomWindow(window, coverage, numerator, denominator, anchorUs = null) {
  const begin = Us(window.beginUs);
  const end = Us(window.endUs);
  const span = end - begin;
  const coverageBegin = Us(coverage.beginUs);
  const coverageEnd = Us(coverage.endUs);
  const maximumSpan = coverageEnd - coverageBegin;
  if (span <= 0n || maximumSpan <= 0n || numerator <= 0n || denominator <= 0n) {
    return window;
  }
  // Outward zoom must progress even from a one-microsecond window. Rounding
  // down would otherwise trap the real 6/5 and 5/4 controls at tiny spans.
  let nextSpan = numerator > denominator
    ? (span * numerator + denominator - 1n) / denominator
    : span * numerator / denominator;
  if (nextSpan < 1n) {
    nextSpan = 1n;
  }
  if (nextSpan >= maximumSpan) {
    return { beginUs: coverageBegin, endUs: coverageEnd };
  }
  const requestedAnchor = anchorUs === null ? begin + span / 2n : Us(anchorUs);
  const anchor = requestedAnchor < begin ? begin : requestedAnchor > end ? end : requestedAnchor;
  let nextBegin = anchor - (anchor - begin) * nextSpan / span;
  if (nextBegin < coverageBegin) {
    nextBegin = coverageBegin;
  }
  if (nextBegin + nextSpan > coverageEnd) {
    nextBegin = coverageEnd - nextSpan;
  }
  return { beginUs: nextBegin, endUs: nextBegin + nextSpan };
}

export class TimeModel {
  static Initial() {
    return InitialState();
  }

  /** Pure reducer: Step(state, action, monotonicNowMs) -> TimeState. */
  static Step(state, action, monotonicNowMs = 0) {
    const next = { ...state };
    const coverage = state.coverage;
    switch (action.type) {
      case 'reset': {
        const begin = Us(action.coverage?.beginUs ?? 0n);
        const end = Us(action.coverage?.endUs ?? 1n);
        const span = end > begin ? end - begin : 1n;
        return {
          ...InitialState(),
          runId: action.runId ?? null,
          coverage: { beginUs: begin, endUs: end },
          window: { beginUs: begin, endUs: begin + span },
          cursor: begin,
          mode: action.mode ?? 'history',
          following: action.following ?? false,
          playing: false,
          lastAdvanceMs: monotonicNowMs,
          lastReceivedMs: action.lastReceivedMs ?? null,
        };
      }
      case 'setWindow': {
        const beginUs = Us(action.window.beginUs);
        const endUs = Us(action.window.endUs);
        if (beginUs < 0n || endUs <= beginUs) {
          return state;
        }
        next.window = { beginUs, endUs };
        next.mode = 'history';
        next.following = false;
        next.playing = false;
        next.lastAdvanceMs = monotonicNowMs;
        return next;
      }
      case 'setMode': {
        next.mode = action.mode === 'live' ? 'live' : 'history';
        if (next.mode === 'history') {
          next.following = false;
        }
        return next;
      }
      case 'play': {
        if (state.mode === 'history') {
          next.playing = true;
          next.following = false;
          next.lastAdvanceMs = monotonicNowMs;
          if (next.cursor < next.window.beginUs || next.cursor > next.window.endUs || next.cursor >= coverage.endUs) {
            next.cursor = next.window.beginUs;
          }
          return next;
        }
        next.playing = false;
        return next;
      }
      case 'pause': {
        next.playing = false;
        next.following = false;
        next.lastAdvanceMs = monotonicNowMs;
        return next;
      }
      case 'seek': {
        const stamp = Us(action.timeUs);
        const clamped = stamp < coverage.beginUs
          ? coverage.beginUs
          : (stamp > coverage.endUs ? coverage.endUs : stamp);
        next.mode = 'history';
        next.following = false;
        next.playing = false;
        next.cursor = clamped;
        const span = state.window.endUs - state.window.beginUs;
        if (span > 0n) {
          next.window = WindowContaining(state.window, clamped, coverage);
        }
        return next;
      }
      case 'setSpeed': {
        const speed = Number(action.speed);
        if (!SPEEDS.includes(speed)) {
          return state;
        }
        next.speed = speed;
        next.lastAdvanceMs = monotonicNowMs;
        return next;
      }
      case 'stepSample': {
        const times = (action.times ?? []).map((value) => Us(value))
          .sort((left, right) => (left < right ? -1 : left > right ? 1 : 0));
        if (times.length === 0) {
          return state;
        }
        const cursor = state.cursor ?? coverage.beginUs;
        let chosen = cursor;
        if (action.direction === 'previous') {
          const lower = [...times].reverse().find((time) => time < cursor);
          if (lower === undefined) {
            return state;
          }
          chosen = lower;
        } else {
          const upper = times.find((time) => time > cursor);
          if (upper === undefined) {
            return state;
          }
          chosen = upper;
        }
        next.cursor = chosen;
        next.mode = 'history';
        next.following = false;
        next.playing = false;
        next.window = WindowContaining(state.window, chosen, coverage);
        return next;
      }
      case 'followLatest': {
        next.window = LiveWindow(coverage);
        next.mode = 'live';
        next.following = true;
        next.playing = false;
        next.cursor = coverage.endUs;
        next.lastAdvanceMs = monotonicNowMs;
        return next;
      }
      case 'liveAdvance': {
        const begin = Us(action.coverage?.beginUs ?? coverage.beginUs);
        const end = Us(action.coverage?.endUs ?? coverage.endUs);
        if (end <= coverage.endUs && begin >= coverage.beginUs) {
          return state;
        }
        next.coverage = {
          beginUs: begin < coverage.beginUs ? begin : coverage.beginUs,
          endUs: end > coverage.endUs ? end : coverage.endUs,
        };
        next.lastReceivedMs = action.receivedAtMs ?? state.lastReceivedMs;
        next.offline = false;
        if (state.following) {
          next.window = LiveWindow(next.coverage);
          next.cursor = next.coverage.endUs;
        }
        return next;
      }
      case 'advance': {
        if (!state.playing || state.mode !== 'history') {
          return state;
        }
        const now = monotonicNowMs;
        const elapsed = Math.max(0, now - state.lastAdvanceMs);
        const delta = BigInt(Math.round(elapsed * state.speed * 1000));
        const target = state.cursor + delta;
        next.cursor = target < coverage.endUs ? target : coverage.endUs;
        if (next.cursor > state.window.endUs || next.cursor < state.window.beginUs) {
          const span = state.window.endUs - state.window.beginUs;
          let begin = next.cursor - span;
          if (begin < coverage.beginUs) begin = coverage.beginUs;
          if (begin + span > coverage.endUs) begin = coverage.endUs - span;
          next.window = { beginUs: begin, endUs: begin + span };
        }
        if (target >= coverage.endUs) {
          next.playing = false;
        }
        next.lastAdvanceMs = now;
        return next;
      }
      case 'markOffline': {
        next.offline = true;
        return next;
      }
      default:
        return state;
    }
  }
}

export function SampleTimes(series) {
  const times = new Set();
  for (const entry of series ?? []) {
    const kind = entry.signal?.value_kind;
    for (const point of entry.points ?? []) {
      if (point.valid !== false && (kind === undefined || kind === 'scalar')) {
        times.add(Us(point.t).toString());
      }
    }
  }
  return [...times].map((value) => BigInt(value)).sort(
    (left, right) => (left < right ? -1 : left > right ? 1 : 0),
  );
}
