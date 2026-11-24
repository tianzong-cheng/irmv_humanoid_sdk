import test from 'node:test';
import assert from 'node:assert/strict';

import { DropSignal } from '../src/dock_model.js';
import {
  EmptyLayout,
  LayoutError,
  ResolveLayout,
  ValidateLayout,
} from '../src/layout_model.js';

function signal(joint, field, quantity, unit = 'rad') {
  return {
    id: `runtime/${joint}/${field}`,
    source: 'runtime',
    joint_name: joint,
    field,
    label: `${joint} ${field}`,
    unit,
    quantity,
    value_kind: 'scalar',
    validity: 'joint_valid',
  };
}

test('a layout with an empty dock and no charts is valid', () => {
  ValidateLayout(EmptyLayout());
});

test('unknown versions, bad ratios and invalid ranges are refused', () => {
  const layout = DropSignal(
    EmptyLayout(),
    signal('j1', 'q', 'joint_position'),
    null,
    'empty',
  );
  layout.version = 2;
  assert.throws(() => ValidateLayout(layout), LayoutError);

  const valid = DropSignal(
    EmptyLayout(),
    signal('j1', 'q', 'joint_position'),
    null,
    'empty',
  );
  valid.charts[0].axes[0].range = [1, 1];
  assert.throws(() => ValidateLayout(valid), LayoutError);

  const ratio = DropSignal(
    EmptyLayout(),
    signal('j1', 'q', 'joint_position'),
    null,
    'empty',
  );
  const first = ratio.charts[0].id;
  const split = DropSignal(
    ratio,
    signal('j1', 'dq', 'joint_velocity'),
    first,
    'left',
  );
  split.dock.ratio = 0.95;
  assert.throws(() => ValidateLayout(split), LayoutError);
});

test('a layout deeper than 32 is refused', () => {
  const left = signal('left', 'q', 'joint_position');
  const right = signal('right', 'q', 'joint_position');
  const layout = EmptyLayout();
  layout.charts = [
    {
      id: 'c1',
      title: '',
      curves: [{ signalId: left.id, axisId: 'a1', style: 's1' }],
      axes: [{ id: 'a1', quantity: 'joint_position', unit: 'rad' }],
    },
    {
      id: 'c2',
      title: '',
      curves: [{ signalId: right.id, axisId: 'a2', style: 's2' }],
      axes: [{ id: 'a2', quantity: 'joint_position', unit: 'rad' }],
    },
  ];
  let node = { type: 'leaf', chartId: 'c1' };
  for (let index = 0; index < 40; index += 1) {
    node = {
      type: 'split',
      direction: 'column',
      ratio: 0.5,
      first: { type: 'empty' },
      second: node,
    };
  }
  layout.dock = node;
  assert.throws(() => ValidateLayout(layout), LayoutError);
});

test('more than 128 charts are refused before a new chart is created', () => {
  const layout = EmptyLayout();
  layout.charts = Array.from({ length: 129 }, (_, index) => ({
    id: `c${index}`,
    title: '',
    curves: [],
    axes: [],
  }));
  layout.dock = { type: 'leaf', chartId: 'c0' };
  assert.throws(() => ValidateLayout(layout), LayoutError);
});

test('curves bind by semantic id after the joint order changes', () => {
  const first = signal('left_hip', 'q', 'joint_position');
  const second = signal('right_hip', 'q', 'joint_position');
  let layout = DropSignal(EmptyLayout(), first, null, 'empty');
  layout = DropSignal(layout, second, layout.charts[0].id, 'center');
  const resolved = ResolveLayout(layout, [second, first]);
  assert.equal(resolved.charts[0].curves[0].signal.id, first.id);
  assert.equal(resolved.charts[0].curves[1].signal.id, second.id);
  assert.equal(resolved.missingSignals.length, 0);
});

test('a missing signal keeps a placeholder instead of rebinding by position', () => {
  const left = signal('left_hip', 'q', 'joint_position');
  const right = signal('right_hip', 'q', 'joint_position');
  let layout = DropSignal(EmptyLayout(), left, null, 'empty');
  layout = DropSignal(layout, right, layout.charts[0].id, 'center');
  const resolved = ResolveLayout(layout, [right]);
  assert.equal(resolved.charts[0].curves[0].missing, true);
  assert.equal(resolved.charts[0].curves[0].signal, null);
  assert.equal(resolved.charts[0].curves[1].signal.id, right.id);
  assert.deepEqual(resolved.missingSignals, [left.id]);
});
