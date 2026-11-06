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
  assert.throws(() => SetAxisRange(chart, assigned.axisId, [0, Infinity]), ChartError);
  const fixed = SetAxisRange(chart, assigned.axisId, [-1, 1]);
  assert.deepEqual(fixed.axes[0].range, [-1, 1]);
});

test('chart validation rejects a curve without its axis', () => {
  const chart = CreateChart('c1');
  chart.curves.push({ signalId: qTarget.id, axisId: 'missing', style: 'style-1' });
  assert.throws(() => ValidateChart(chart), ChartError);
});
