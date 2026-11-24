import test from 'node:test';
import assert from 'node:assert/strict';

import {
  AssignAxis,
  ChartError,
  CreateChart,
  SetAxisRange,
  ValidateChart,
} from '../src/chart_model.js';
import { SameAxis } from '../src/signals.js';

const qTarget = {
  id: 'runtime/left_hip/q',
  source: 'runtime',
  joint_name: 'left_hip',
  field: 'q',
  label: 'position target',
  unit: 'rad',
  quantity: 'joint_position',
  value_kind: 'scalar',
  validity: 'command_present',
};
const qFeedback = {
  ...qTarget,
  id: 'runtime/left_hip/measured_q',
  field: 'measured_q',
  label: 'measured position',
  validity: 'feedback_valid+joint_valid',
};
const torque = {
  id: 'runtime/left_hip/output_torque',
  source: 'runtime',
  joint_name: 'left_hip',
  field: 'output_torque',
  label: 'output torque',
  unit: 'Nm',
  quantity: 'joint_torque',
  value_kind: 'scalar',
  validity: 'output_valid',
};
const angular = {
  id: 'runtime//imu_angular_velocity_x',
  source: 'runtime',
  joint_name: '',
  field: 'imu_angular_velocity_x',
  label: 'IMU angular velocity x',
  unit: 'rad_s',
  quantity: 'imu_angular_velocity',
  value_kind: 'scalar',
  validity: 'imu_valid',
};
const temperature = {
  id: 'runtime/left_hip/temperature',
  source: 'runtime',
  joint_name: 'left_hip',
  field: 'temperature',
  label: 'temperature',
  unit: 'degC',
  quantity: 'temperature',
  value_kind: 'scalar',
  validity: 'always',
};

test('a target and its feedback share one axis', () => {
  const chart = CreateChart('c1');
  const target = AssignAxis(chart, qTarget);
  const feedback = AssignAxis(target.chart, qFeedback);
  assert.equal(target.axisId, feedback.axisId);
  assert.equal(feedback.chart.axes.length, 1);
  assert.ok(SameAxis(qTarget, qFeedback));
});

test('four quantities create four axes and do not overwrite each other', () => {
  let chart = CreateChart('c1');
  for (const signal of [qTarget, torque, angular, temperature]) {
    chart = AssignAxis(chart, signal).chart;
  }
  assert.equal(chart.axes.length, 4);
  const keys = chart.axes.map((axis) => `${axis.quantity}|${axis.unit}`);
  assert.deepEqual(keys, [
    'joint_position|rad',
    'joint_torque|Nm',
    'imu_angular_velocity|rad_s',
    'temperature|degC',
  ]);
});

test('the same unit is not enough to share an axis', () => {
  const chart = CreateChart('c1');
  const target = AssignAxis(chart, qTarget);
  const withTorque = AssignAxis(target.chart, torque);
  assert.notEqual(target.axisId, withTorque.axisId);
});

test('an axis range must be finite and increasing', () => {
  let chart = CreateChart('c1');
  const assigned = AssignAxis(chart, qTarget);
  chart = assigned.chart;
  assert.throws(() => SetAxisRange(chart, assigned.axisId, [1, 1]), ChartError);
  assert.throws(
    () => SetAxisRange(chart, assigned.axisId, [0, Infinity]),
    ChartError,
  );
  const fixed = SetAxisRange(chart, assigned.axisId, [-1, 1]);
  assert.deepEqual(fixed.axes[0].range, [-1, 1]);
});

test('chart validation rejects a curve without its axis', () => {
  const chart = CreateChart('c1');
  chart.curves.push({
    signalId: qTarget.id,
    axisId: 'missing',
    style: 'style-1',
  });
  assert.throws(() => ValidateChart(chart), ChartError);
});

test('time selection ignores vertical movement and preserves exact large timestamps', async () => {
  const { SelectionZoom } = await import('../src/chart_view.js');
  const view = {
    plot: { left: 20, right: 220, top: 10, bottom: 110 },
    ranges: [
      { axisId: 'position', low: -10, high: 10 },
      { axisId: 'torque', low: 0, high: 100 },
    ],
  };
  const window = { beginUs: 9007199254740993n, endUs: 9007199254744993n };
  const zoom = SelectionZoom(view, { x: 170, y: 85 }, { x: 70, y: 85 }, window);
  assert.deepEqual(zoom, {
    window: { beginUs: 9007199254741993n, endUs: 9007199254743993n },
    axes: [],
  });
  assert.equal(
    SelectionZoom(view, { x: 50, y: 50 }, { x: 52, y: 51 }, window),
    null,
  );
  assert.deepEqual(
    SelectionZoom(view, { x: -20, y: -20 }, { x: 500, y: 500 }, window),
    {
      window,
      axes: [],
    },
  );
});

test('value selection ignores horizontal movement and leaves the shared time window unchanged', async () => {
  const { SelectionZoom } = await import('../src/chart_view.js');
  const view = {
    plot: { left: 20, right: 220, top: 10, bottom: 110 },
    ranges: [
      { axisId: 'position', low: -10, high: 10 },
      { axisId: 'torque', low: 0, high: 100 },
    ],
  };
  const window = { beginUs: 9007199254740993n, endUs: 9007199254744993n };
  assert.deepEqual(
    SelectionZoom(view, { x: 70, y: 85 }, { x: 70, y: 35 }, window, 'value'),
    {
      window: null,
      axes: [
        { axisId: 'position', range: [-5, 5] },
        { axisId: 'torque', range: [25, 75] },
      ],
    },
  );
  assert.equal(
    SelectionZoom(view, { x: 50, y: 50 }, { x: 200, y: 52 }, window, 'value'),
    null,
  );
});

test('time coordinates preserve subpixel sample spacing at large absolute timestamps', async () => {
  const { TimeToX } = await import('../src/chart_view.js');
  const beginUs = 9007199254740993n;
  const window = { beginUs, endUs: beginUs + 12000000n };
  assert.equal(TimeToX(beginUs + 10000n, window, 10, 2410), 12);
  assert.equal(TimeToX(beginUs + 20000n, window, 10, 2410), 14);
  assert.equal(TimeToX(beginUs + 30000n, window, 10, 2410), 16);
  const tinyWindow = { beginUs, endUs: beginUs + 10n };
  assert.equal(TimeToX(beginUs + 1n, tinyWindow, 0, 100), 10);
});

test('value ticks use round multiples within fractional and negative ranges', async () => {
  const { ValueTicks } = await import('../src/chart_view.js');
  assert.equal(typeof ValueTicks, 'function');
  assert.deepEqual(ValueTicks(-0.37, 0.86), [-0.2, 0, 0.2, 0.4, 0.6, 0.8]);
  assert.deepEqual(ValueTicks(13, 97), [20, 40, 60, 80]);
  assert.deepEqual(
    ValueTicks(0.00011, 0.00039),
    [0.00015, 0.0002, 0.00025, 0.0003, 0.00035],
  );
});

test('time ticks align to whole experiment seconds at large timestamps', async () => {
  const { TimeTicks } = await import('../src/chart_view.js');
  assert.equal(typeof TimeTicks, 'function');
  const origin = 9007199254740993n;
  const offsets = (begin, end, count) =>
    TimeTicks(origin + begin, origin + end, origin, count).map(
      (stamp) => stamp - origin,
    );
  assert.deepEqual(offsets(2300000n, 9800000n, 4), [
    4000000n,
    6000000n,
    8000000n,
  ]);
  assert.deepEqual(offsets(-2300000n, 1200000n, 4), [
    -2000000n,
    -1000000n,
    0n,
    1000000n,
  ]);
  assert.deepEqual(offsets(100000n, 900000n, 4), [
    200000n,
    400000n,
    600000n,
    800000n,
  ]);
  assert.deepEqual(offsets(0n, 60000000n, 4), [
    0n,
    20000000n,
    40000000n,
    60000000n,
  ]);
});

test('zoomed time ticks retain round millisecond and microsecond intervals', async () => {
  const { TimeTicks } = await import('../src/chart_view.js');
  const origin = 9007199254740993n;
  const offsets = (begin, end) =>
    TimeTicks(origin + begin, origin + end, origin, 4).map(
      (stamp) => stamp - origin,
    );
  assert.deepEqual(offsets(12300n, 19800n), [14000n, 16000n, 18000n]);
  assert.deepEqual(offsets(-900n, -100n), [-800n, -600n, -400n, -200n]);
  assert.deepEqual(offsets(1n, 4n), [1n, 2n, 3n, 4n]);
});
