import test from 'node:test';
import assert from 'node:assert/strict';

import { SampleTimes, TimeModel } from '../src/time_model.js';
import { EventsView } from '../src/events_view.js';

function initial() {
  return TimeModel.Step(
    TimeModel.Initial(),
    {
      type: 'reset',
      runId: 'run-a',
      coverage: { beginUs: 1000n, endUs: 10000n },
    },
    0,
  );
}

test('a live pause freezes the window while coverage grows', () => {
  let state = initial();
  state = TimeModel.Step(state, { type: 'followLatest' }, 10);
  state = TimeModel.Step(state, { type: 'pause' }, 20);
  const before = state;
  const after = TimeModel.Step(
    state,
    {
      type: 'liveAdvance',
      coverage: { beginUs: 1000n, endUs: 20000n },
      receivedAtMs: 200,
    },
    200,
  );
  assert.equal(after.cursor, before.cursor);
  assert.deepEqual(after.window, before.window);
  assert.equal(after.coverage.endUs, 20000n);
});

test('live follow grows with startup data then keeps the latest ten seconds', () => {
  const origin = 9007199254740993n;
  let state = TimeModel.Step(TimeModel.Initial(), {
    type: 'reset',
    runId: 'live',
    coverage: { beginUs: origin, endUs: origin + 1000000n },
  });
  state = TimeModel.Step(state, { type: 'followLatest' });
  for (const seconds of [2n, 9n, 10n, 11n, 30n]) {
    const endUs = origin + seconds * 1000000n;
    state = TimeModel.Step(state, {
      type: 'liveAdvance',
      coverage: { beginUs: origin, endUs },
    });
    const span = seconds < 10n ? seconds * 1000000n : 10000000n;
    assert.deepEqual(state.window, { beginUs: endUs - span, endUs });
    assert.equal(state.cursor, endUs);
  }
});

test('follow latest restores ten seconds after inspecting a different window', () => {
  let state = TimeModel.Step(TimeModel.Initial(), {
    type: 'reset',
    runId: 'live',
    coverage: { beginUs: 1000n, endUs: 30001000n },
  });
  for (const span of [1000000n, 20000000n]) {
    state = TimeModel.Step(state, {
      type: 'setWindow',
      window: { beginUs: 1000n, endUs: 1000n + span },
    });
    state = TimeModel.Step(state, { type: 'followLatest' });
    assert.deepEqual(state.window, { beginUs: 20001000n, endUs: 30001000n });
  }
});

test('a short live buffer does not discard the recorded experiment coverage', () => {
  let state = TimeModel.Step(TimeModel.Initial(), {
    type: 'reset',
    runId: 'live',
    coverage: { beginUs: 1000n, endUs: 20001000n },
  });
  state = TimeModel.Step(state, { type: 'followLatest' });
  state = TimeModel.Step(state, {
    type: 'liveAdvance',
    coverage: { beginUs: 20001000n, endUs: 21001000n },
  });
  assert.equal(state.coverage.beginUs, 1000n);
  assert.deepEqual(state.window, { beginUs: 11001000n, endUs: 21001000n });
});

test('seek to the start then follow latest jumps back to the new tail', () => {
  let state = initial();
  state = TimeModel.Step(state, { type: 'seek', timeUs: 1000n }, 10);
  assert.equal(state.cursor, 1000n);
  state = TimeModel.Step(state, { type: 'followLatest' }, 20);
  state = TimeModel.Step(
    state,
    {
      type: 'liveAdvance',
      coverage: { beginUs: 1000n, endUs: 30000n },
    },
    30,
  );
  assert.equal(state.cursor, 30000n);
  assert.equal(state.window.endUs, 30000n);
});

test('history playback honours every speed and pauses at the end', () => {
  for (const speed of [0.25, 1, 2, 4]) {
    let state = TimeModel.Step(
      TimeModel.Initial(),
      {
        type: 'reset',
        runId: 'run-long',
        coverage: { beginUs: 0n, endUs: 100000000n },
      },
      0,
    );
    const beforeWindow = { ...state.window };
    const base = state.cursor;
    state = TimeModel.Step(state, { type: 'setSpeed', speed }, 0);
    state = TimeModel.Step(state, { type: 'play' }, 0);
    state = TimeModel.Step(state, { type: 'advance' }, 1000);
    const advanced = state.cursor - base;
    assert.deepEqual(state.window, beforeWindow);
    assert.equal(advanced, BigInt(Math.round(1000 * speed)) * 1000n);
    state = TimeModel.Step(state, { type: 'advance' }, 1000000);
    assert.equal(state.playing, false);
    assert.equal(state.window.endUs, 100000000n);
  }
});

test('a single step uses the runtime sample times and never invents a tick', () => {
  let state = initial();
  const times = [1000n, 3000n, 9000n];
  state = TimeModel.Step(
    state,
    { type: 'stepSample', direction: 'next', times },
    0,
  );
  assert.equal(state.cursor, 3000n);
  state = TimeModel.Step(
    state,
    { type: 'stepSample', direction: 'next', times },
    0,
  );
  assert.equal(state.cursor, 9000n);
  state = TimeModel.Step(
    state,
    { type: 'stepSample', direction: 'next', times },
    0,
  );
  assert.equal(state.cursor, 9000n);
  state = TimeModel.Step(
    state,
    { type: 'stepSample', direction: 'previous', times },
    0,
  );
  assert.equal(state.cursor, 3000n);
});

test('sample times preserve order and skip invalid points', () => {
  const series = [
    {
      signal: { value_kind: 'scalar' },
      points: [
        { t: '3000', valid: true },
        { t: '1000', valid: true },
        { t: '2000', valid: false },
      ],
    },
  ];
  assert.deepEqual(SampleTimes(series), [1000n, 3000n]);
});

test('switching experiment resets the clock and coverage', () => {
  let state = initial();
  state = TimeModel.Step(state, { type: 'seek', timeUs: 9000n }, 0);
  const reset = TimeModel.Step(
    state,
    {
      type: 'reset',
      runId: 'run-b',
      coverage: { beginUs: 50000n, endUs: 60000n },
    },
    0,
  );
  assert.equal(reset.runId, 'run-b');
  assert.equal(reset.window.beginUs, 50000n);
  assert.equal(reset.window.endUs, 60000n);
  assert.equal(reset.cursor, 50000n);
});

test('same-tick events stay one marker with every event kept', () => {
  const view = new EventsView();
  view.Append({
    events: [
      { event_id: '1', detected_stamp_us: '1234', reason_name: 'A' },
      { event_id: '2', detected_stamp_us: '1234', reason_name: 'B' },
      { event_id: '3', detected_stamp_us: '2000', reason_name: 'C' },
    ],
    integrity: { faults: { complete: true, losses: [] } },
  });
  assert.equal(view.events.length, 3);
  assert.equal(view.markers.length, 2);
  assert.deepEqual(view.markers[0].eventIds, ['1', '2']);
  view.Append({
    events: [
      { event_id: '2', detected_stamp_us: '1234', reason_name: 'B' },
      { event_id: '4', detected_stamp_us: '1234', reason_name: 'D' },
    ],
    continuation: '4',
  });
  assert.equal(view.events.length, 4);
  assert.deepEqual(view.markers[0].eventIds, ['1', '2', '4']);
});

test('clicking an event seeks to its detection time and keeps its detail', () => {
  let sought = null;
  let selected = null;
  const view = new EventsView({
    onSeek: (stamp) => {
      sought = stamp;
    },
    onSelect: (event) => {
      selected = event;
    },
  });
  view.Append({
    events: [
      {
        event_id: '9',
        detected_stamp_us: '9007199254740995',
        sample_stamp_us: '9007199254740990',
        reason_name: 'JointPositionExceeded',
        condition_name: 'JointPositionExceeded',
        joint_name: 'left_hip',
        error_code: 7,
        observed_value: 'NaN',
        limit: 1,
        output_action_name: 'ZeroTorque',
        message: 'limit',
        message_hex: '6c696d6974',
      },
    ],
  });
  view.Select('9');
  assert.equal(sought, 9007199254740995n);
  assert.equal(selected.event_id, '9');
  assert.match(view.DetailText(), /observed NaN/);
  assert.match(view.DetailText(), /message_hex 6c696d6974/);
});

test('selecting a time window stops following and playback without losing timestamp precision', () => {
  const state = { ...initial(), mode: 'live', following: true, playing: true };
  const next = TimeModel.Step(
    state,
    {
      type: 'setWindow',
      window: { beginUs: 9007199254740993n, endUs: 9007199254741993n },
    },
    42,
  );
  assert.deepEqual(next.window, {
    beginUs: 9007199254740993n,
    endUs: 9007199254741993n,
  });
  assert.equal(next.mode, 'history');
  assert.equal(next.following, false);
  assert.equal(next.playing, false);
  assert.equal(next.lastAdvanceMs, 42);
  assert.deepEqual(
    TimeModel.Step(next, { type: 'advance' }, 500).window,
    next.window,
  );
  assert.equal(
    TimeModel.Step(
      next,
      { type: 'setWindow', window: { beginUs: 20n, endUs: 10n } },
      45,
    ),
    next,
  );
});

test('zoom out stops at full coverage and boundary windows stay filled with recorded data', async () => {
  const { ZoomWindow } = await import('../src/time_model.js');
  const coverage = { beginUs: 9007199254740993n, endUs: 9007199254750993n };
  assert.deepEqual(
    ZoomWindow(coverage, coverage, 6n, 5n, coverage.beginUs + 8000n),
    coverage,
  );
  const nearStart = {
    beginUs: coverage.beginUs,
    endUs: coverage.beginUs + 1000n,
  };
  assert.deepEqual(
    ZoomWindow(nearStart, coverage, 2n, 1n, coverage.beginUs + 100n),
    {
      beginUs: coverage.beginUs,
      endUs: coverage.beginUs + 2000n,
    },
  );
  const nearEnd = { beginUs: coverage.endUs - 1000n, endUs: coverage.endUs };
  assert.deepEqual(
    ZoomWindow(nearEnd, coverage, 2n, 1n, coverage.endUs - 100n),
    {
      beginUs: coverage.endUs - 2000n,
      endUs: coverage.endUs,
    },
  );
});

test('zoom keeps the mouse time at its fractional position instead of recentering it', async () => {
  const { ZoomWindow } = await import('../src/time_model.js');
  const base = 9007199254740993n;
  const coverage = { beginUs: base, endUs: base + 10000n };
  assert.deepEqual(ZoomWindow(coverage, coverage, 4n, 5n, base + 7500n), {
    beginUs: base + 1500n,
    endUs: base + 9500n,
  });
  const tiny = { beginUs: base + 50n, endUs: base + 51n };
  for (const [numerator, denominator] of [
    [2n, 1n],
    [6n, 5n],
    [5n, 4n],
  ]) {
    assert.deepEqual(
      ZoomWindow(tiny, coverage, numerator, denominator, base + 50n),
      {
        beginUs: base + 50n,
        endUs: base + 52n,
      },
    );
  }
});

test('playback moves the cursor through a fixed window and pans only after its right edge', () => {
  const base = 9007199254740993n;
  let state = TimeModel.Step(TimeModel.Initial(), {
    type: 'reset',
    runId: 'large',
    coverage: { beginUs: base, endUs: base + 10000000n },
  });
  state = {
    ...state,
    window: { beginUs: base + 1000000n, endUs: base + 2000000n },
    cursor: base + 1500000n,
  };
  state = TimeModel.Step(state, { type: 'play' }, 0);
  state = TimeModel.Step(state, { type: 'advance' }, 250);
  assert.equal(state.cursor, base + 1750000n);
  assert.deepEqual(state.window, {
    beginUs: base + 1000000n,
    endUs: base + 2000000n,
  });
  state = TimeModel.Step(state, { type: 'advance' }, 750);
  assert.equal(state.cursor, base + 2250000n);
  assert.deepEqual(state.window, {
    beginUs: base + 1250000n,
    endUs: base + 2250000n,
  });
  state = TimeModel.Step(state, { type: 'advance' }, 20000);
  assert.equal(state.playing, false);
  assert.equal(state.cursor, base + 10000000n);
  assert.deepEqual(state.window, {
    beginUs: base + 9000000n,
    endUs: base + 10000000n,
  });
});

test('relative times and timestamp-valued signals preserve microseconds beyond 2^53', async () => {
  const { RelativeTime, RelativeTimestampFields, DisplaySeries } = await import(
    '../src/time_display.js'
  );
  const base = 9007199254740993n;
  assert.equal(RelativeTime(base + 2n, base), '0.000002 s');
  assert.equal(RelativeTime(base - 10n, base), '-0.000010 s');
  assert.equal(RelativeTime(base + 1234567n, base), '1.234567 s');
  assert.equal(RelativeTime(base, null), '—');
  assert.deepEqual(
    RelativeTimestampFields(
      {
        losses: [
          {
            first_stamp_us: String(base + 7n),
            last_stamp_us: String(base + 9n),
            age_us: '12',
          },
        ],
      },
      base,
    ),
    {
      losses: [{ first_stamp_us: '7', last_stamp_us: '9', age_us: '12' }],
    },
  );
  const series = [
    {
      signal: { field: 'stamp_us', quantity: 'time', unit: 'us' },
      points: [{ t: String(base + 7n), v: Number(base + 7n), valid: true }],
    },
  ];
  const display = DisplaySeries(series, base);
  assert.equal(display[0].points[0].v, 7);
  assert.equal(series[0].points[0].v, Number(base + 7n));
  const events = new EventsView();
  events.Reset(base);
  events.Append({
    events: [
      {
        event_id: 'r',
        detected_stamp_us: String(base + 7n),
        sample_stamp_us: String(base - 10n),
      },
    ],
  });
  events.Select('r');
  assert.match(events.DetailText(), /detected 0.000007 s/);
  assert.match(events.DetailText(), /sample -0.000010 s/);
});
