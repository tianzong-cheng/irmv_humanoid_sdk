/**
 * Dock tree operations: create a chart from a dropped signal, split a target
 * chart on an edge, move an existing chart, remove a curve and collapse empty
 * splits. Every operation is a pure function of the layout value; the caller's
 * object is never mutated, so a refused operation leaves the current layout
 * exactly as it was.
 */

import {
  AddCurve,
  AssignAxis,
  Clone,
  CreateChart,
  RemoveCurve,
} from './chart_model.js';
import { LayoutError, ValidateLayout } from './layout_model.js';
import { SignalKey } from './signals.js';

const EDGES = new Set(['left', 'right', 'top', 'bottom']);

function NextChartId(layout) {
  const used = new Set(layout.charts.map((chart) => chart.id));
  let index = 1;
  while (used.has(`chart-${index}`)) {
    index += 1;
  }
  return `chart-${index}`;
}

function Collapse(node) {
  if (node.type !== 'split') {
    return node;
  }
  const first = Collapse(node.first);
  const second = Collapse(node.second);
  if (first.type === 'empty' && second.type === 'empty') {
    return { type: 'empty' };
  }
  if (first.type === 'empty') {
    return second;
  }
  if (second.type === 'empty') {
    return first;
  }
  return { ...node, first, second };
}

function ReplaceLeaf(node, chartId, replacement) {
  if (node.type === 'leaf' && node.chartId === chartId) {
    return { node: replacement, found: true };
  }
  if (node.type !== 'split') {
    return { node, found: false };
  }
  const first = ReplaceLeaf(node.first, chartId, replacement);
  if (first.found) {
    return { node: { ...node, first: first.node }, found: true };
  }
  const second = ReplaceLeaf(node.second, chartId, replacement);
  if (second.found) {
    return { node: { ...node, second: second.node }, found: true };
  }
  return { node, found: false };
}

function RemoveLeaf(node, chartId) {
  return ReplaceLeaf(node, chartId, { type: 'empty' });
}

function NodeForChart(node, chartId) {
  if (node.type === 'leaf' && node.chartId === chartId) {
    return node;
  }
  if (node.type !== 'split') {
    return null;
  }
  return (
    NodeForChart(node.first, chartId) ?? NodeForChart(node.second, chartId)
  );
}

function ContainsChart(node, chartId) {
  if (node.type === 'leaf') {
    return node.chartId === chartId;
  }
  if (node.type !== 'split') {
    return false;
  }
  return (
    ContainsChart(node.first, chartId) || ContainsChart(node.second, chartId)
  );
}

function CheckZone(zone) {
  if (!EDGES.has(zone)) {
    throw new LayoutError(`the drop zone ${zone} is not an edge`);
  }
}

function SplitTarget(layout, targetChartId, zone, chart) {
  CheckZone(zone);
  const direction = zone === 'left' || zone === 'right' ? 'row' : 'column';
  const first = zone === 'left' || zone === 'top' ? chart : null;
  const second = first === null ? chart : null;
  const replaced = ReplaceLeaf(layout.dock, targetChartId, {
    type: 'split',
    direction,
    ratio: 0.5,
    first: first ?? { type: 'leaf', chartId: targetChartId },
    second: second ?? { type: 'leaf', chartId: targetChartId },
  });
  if (!replaced.found) {
    throw new LayoutError(`no chart ${targetChartId}`);
  }
  return { ...layout, dock: Collapse(replaced.node) };
}

function ChartFor(layout, chartId) {
  const chart = layout.charts.find((entry) => entry.id === chartId);
  if (!chart) {
    throw new LayoutError(`no chart ${chartId}`);
  }
  return chart;
}

function ReplaceChart(layout, chart) {
  return {
    ...layout,
    charts: layout.charts.map((entry) =>
      entry.id === chart.id ? chart : entry,
    ),
  };
}

function Finalize(layout) {
  ValidateLayout(layout);
  return layout;
}

export function DropSignal(layout, signal, targetChartId, zone) {
  ValidateLayout(layout);
  const next = Clone(layout);
  if (!signal || typeof signal.id !== 'string') {
    throw new LayoutError('a dropped signal has no id');
  }
  if (zone === 'empty') {
    if (targetChartId) {
      throw new LayoutError('the empty zone does not name a target chart');
    }
    if (next.dock.type !== 'empty') {
      throw new LayoutError('the layout already has a dock tree');
    }
    const chart = AddCurve(
      CreateChart(NextChartId(next), signal.label ?? ''),
      signal,
    );
    next.charts.push(chart);
    next.dock = { type: 'leaf', chartId: chart.id };
    return Finalize(next);
  }
  if (!targetChartId) {
    throw new LayoutError('an edge drop needs a target chart');
  }
  const target = ChartFor(next, targetChartId);
  if (zone === 'center') {
    const updated = AddCurve(target, signal);
    return Finalize(ReplaceChart(next, updated));
  }
  CheckZone(zone);
  const chart = AddCurve(
    CreateChart(NextChartId(next), signal.label ?? ''),
    signal,
  );
  next.charts.push(chart);
  return Finalize(
    SplitTarget(next, targetChartId, zone, { type: 'leaf', chartId: chart.id }),
  );
}

export function MoveChart(layout, chartId, targetChartId, zone) {
  ValidateLayout(layout);
  const next = Clone(layout);
  if (chartId === targetChartId) {
    throw new LayoutError('a chart cannot move onto itself');
  }
  CheckZone(zone);
  const moved = ChartFor(next, chartId);
  ChartFor(next, targetChartId);
  const sourceNode = NodeForChart(next.dock, chartId);
  if (sourceNode && ContainsChart(sourceNode, targetChartId)) {
    throw new LayoutError('a chart cannot move into its own subtree');
  }
  const removed = RemoveLeaf(next.dock, chartId);
  if (!removed.found) {
    throw new LayoutError(`no chart ${chartId}`);
  }
  next.dock = Collapse(removed.node);
  if (!ContainsChart(next.dock, targetChartId)) {
    throw new LayoutError('the target chart was collapsed by the move');
  }
  next.charts = next.charts.filter((chart) => chart.id !== chartId);
  next.charts.push(moved);
  return Finalize(
    SplitTarget(next, targetChartId, zone, { type: 'leaf', chartId }),
  );
}

export function RemoveSignal(layout, chartId, signalId) {
  ValidateLayout(layout);
  const next = Clone(layout);
  const chart = ChartFor(next, chartId);
  const updated = RemoveCurve(chart, signalId);
  if (updated.curves.length === 0) {
    return CloseChart(next, chartId);
  }
  return Finalize(ReplaceChart(next, updated));
}

export function CloseChart(layout, chartId) {
  ValidateLayout(layout);
  const next = Clone(layout);
  ChartFor(next, chartId);
  const removed = RemoveLeaf(next.dock, chartId);
  next.dock = Collapse(removed.node);
  next.charts = next.charts.filter((chart) => chart.id !== chartId);
  return Finalize(next);
}

export function ResizeSplit(layout, path, ratio) {
  ValidateLayout(layout);
  if (!Array.isArray(path)) {
    throw new LayoutError('a split path is not a list');
  }
  if (!Number.isFinite(ratio) || ratio < 0.1 || ratio > 0.9) {
    throw new LayoutError('a split ratio must be between 0.1 and 0.9');
  }
  const next = Clone(layout);
  let node = next.dock;
  for (const step of path) {
    if (node.type !== 'split' || (step !== 0 && step !== 1)) {
      throw new LayoutError('the split path does not name a split');
    }
    node = step === 0 ? node.first : node.second;
  }
  if (node.type !== 'split') {
    throw new LayoutError('the split path ends at a leaf');
  }
  node.ratio = ratio;
  return Finalize(next);
}

export { AssignAxis, SignalKey };
