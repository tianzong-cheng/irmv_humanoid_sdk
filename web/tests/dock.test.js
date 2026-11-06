import test from 'node:test';
import assert from 'node:assert/strict';

import {
  CloseChart,
  DropSignal,
  MoveChart,
  RemoveSignal,
  ResizeSplit,
} from '../src/dock_model.js';
import { EmptyLayout, LayoutError } from '../src/layout_model.js';

const q = {
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
const dq = {
  ...q,
  id: 'runtime/left_hip/dq',
  field: 'dq',
  label: 'velocity target',
  unit: 'rad_s',
  quantity: 'joint_velocity',
};
const torque = {
  ...q,
  id: 'runtime/left_hip/output_torque',
  field: 'output_torque',
  label: 'output torque',
  unit: 'Nm',
  quantity: 'joint_torque',
  validity: 'output_valid',
};

test('a signal dropped on an empty workspace creates one chart', () => {
  const layout = DropSignal(EmptyLayout(), q, null, 'empty');
  assert.equal(layout.dock.type, 'leaf');
  assert.equal(layout.charts.length, 1);
  assert.deepEqual(layout.charts[0].curves.map((curve) => curve.signalId), [q.id]);
});

test('an edge drop splits the target chart on that side', () => {
  let layout = DropSignal(EmptyLayout(), q, null, 'empty');
  const originalId = layout.charts[0].id;
  layout = DropSignal(layout, torque, originalId, 'right');
  assert.equal(layout.dock.type, 'split');
  assert.equal(layout.dock.direction, 'row');
  assert.equal(layout.dock.ratio, 0.5);
  assert.equal(layout.dock.first.chartId, originalId);
  assert.notEqual(layout.dock.second.chartId, originalId);
  assert.equal(layout.charts.length, 2);
});

test('a centered drop overlays the signal without duplicating it', () => {
  let layout = DropSignal(EmptyLayout(), q, null, 'empty');
  const chartId = layout.charts[0].id;
  layout = DropSignal(layout, q, chartId, 'center');
  assert.equal(layout.charts[0].curves.length, 1);
  layout = DropSignal(layout, torque, chartId, 'center');
  assert.equal(layout.charts[0].curves.length, 2);
});

test('recursive splits and closing a chart collapse empty nodes', () => {
  let layout = DropSignal(EmptyLayout(), q, null, 'empty');
  const first = layout.charts[0].id;
  layout = DropSignal(layout, dq, first, 'bottom');
  const second = layout.charts[1].id;
  layout = DropSignal(layout, torque, second, 'right');
  assert.equal(layout.dock.type, 'split');
  layout = CloseChart(layout, second);
  assert.equal(layout.charts.length, 2);
  assert.equal(layout.dock.type, 'split');
  layout = CloseChart(layout, layout.charts[0].id);
  assert.equal(layout.charts.length, 1);
  assert.equal(layout.dock.type, 'leaf');
  layout = CloseChart(layout, layout.charts[0].id);
  assert.equal(layout.charts.length, 0);
  assert.equal(layout.dock.type, 'empty');
});

test('removing the last curve closes its chart', () => {
  let layout = DropSignal(EmptyLayout(), q, null, 'empty');
  const chartId = layout.charts[0].id;
  layout = RemoveSignal(layout, chartId, q.id);
  assert.equal(layout.charts.length, 0);
  assert.equal(layout.dock.type, 'empty');
});

test('a chart moves to an edge but never onto itself or the center', () => {
  let layout = DropSignal(EmptyLayout(), q, null, 'empty');
  const first = layout.charts[0].id;
  layout = DropSignal(layout, dq, first, 'right');
  const second = layout.charts[1].id;
  assert.throws(() => MoveChart(layout, first, first, 'left'), LayoutError);
  assert.throws(() => MoveChart(layout, first, second, 'center'), LayoutError);
  const moved = MoveChart(layout, second, first, 'bottom');
  assert.equal(moved.charts.length, 2);
  assert.equal(moved.dock.type, 'split');
  assert.equal(moved.dock.direction, 'column');
  assert.equal(moved.dock.first.chartId, first);
  assert.equal(moved.dock.second.chartId, second);
});

test('a split ratio is resizeable and bounded', () => {
  let layout = DropSignal(EmptyLayout(), q, null, 'empty');
  const first = layout.charts[0].id;
  layout = DropSignal(layout, dq, first, 'right');
  const resized = ResizeSplit(layout, [], 0.3);
  assert.equal(resized.dock.ratio, 0.3);
  assert.throws(() => ResizeSplit(layout, [], 0.95), LayoutError);
  assert.throws(() => ResizeSplit(layout, [0, 0], 0.5), LayoutError);
});

test('the input layout is never mutated by a refused drop', () => {
  const layout = DropSignal(EmptyLayout(), q, null, 'empty');
  const before = JSON.stringify(layout);
  assert.throws(() => DropSignal(layout, dq, 'missing', 'left'), LayoutError);
  assert.equal(JSON.stringify(layout), before);
});
