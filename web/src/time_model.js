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

function WindowAround(cursor, span) {
  const half = span / 2n;
  const begin = cursor > half ? cursor - half : 0n;
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
          next.window = WindowAround(clamped, span);
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
        next.window = WindowAround(chosen, state.window.endUs - state.window.beginUs);
        return next;
      }
      case 'followLatest': {
        const span = state.window.endUs - state.window.beginUs;
        next.mode = 'live';
        next.following = true;
        next.playing = false;
        next.cursor = coverage.endUs;
        next.window = span > 0n
          ? { beginUs: coverage.endUs - span < 0n ? 0n : coverage.endUs - span, endUs: coverage.endUs }
          : { beginUs: coverage.beginUs, endUs: coverage.endUs };
        next.lastAdvanceMs = monotonicNowMs;
        return next;
      }
      case 'liveAdvance': {
        const begin = Us(action.coverage?.beginUs ?? coverage.beginUs);
        const end = Us(action.coverage?.endUs ?? coverage.endUs);
        if (end <= coverage.endUs && begin >= coverage.beginUs) {
          return state;
        }
        next.coverage = { beginUs: begin, endUs: end };
        next.lastReceivedMs = action.receivedAtMs ?? state.lastReceivedMs;
        next.offline = false;
        if (state.following) {
          const span = state.window.endUs - state.window.beginUs;
          next.window = {
            beginUs: end - span < 0n ? 0n : end - span,
            endUs: end,
          };
          next.cursor = end;
        }
        return next;
      }
      case 'advance': {
        if (!state.playing || state.mode !== 'history') {
          return state;
        }
        const now = monotonicNowMs;
        const elapsed = Math.max(0, now - state.lastAdvanceMs);
        const delta = BigInt(Math.round(elapsed * state.speed)) * 1000n;
        const span = state.window.endUs - state.window.beginUs;
        const nextBegin = state.window.beginUs + delta;
        const coverageEnd = coverage.endUs;
        if (nextBegin >= coverageEnd) {
          next.window = { beginUs: coverageEnd > span ? coverageEnd - span : 0n, endUs: coverageEnd };
          next.cursor = coverageEnd;
          next.playing = false;
        } else {
          next.window = { beginUs: nextBegin, endUs: nextBegin + span };
          next.cursor = nextBegin;
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
