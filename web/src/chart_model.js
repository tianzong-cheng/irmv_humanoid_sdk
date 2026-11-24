/**
 * Chart and axis grouping. Pure JSON-in/JSON-out: no DOM, no clock, no I/O.
 * The default grouping is the signal's explicit quantity plus unit. A unit
 * alone never merges two physical quantities, and no conversion is applied.
 */

export const MAX_CURVES_PER_CHART = 128;

export class ChartError extends Error {}

export function Clone(value) {
  return JSON.parse(JSON.stringify(value));
}

function NextId(prefix, existing) {
  let index = 1;
  const used = new Set(existing);
  while (used.has(`${prefix}-${index}`)) {
    index += 1;
  }
  return `${prefix}-${index}`;
}

export function CreateChart(id, title = '') {
  if (!id || typeof id !== 'string') {
    throw new ChartError('a chart needs an id');
  }
  return { id, title, curves: [], axes: [] };
}

export function AxisIdForSignal(chart, signal) {
  const axis = chart.axes.find(
    (entry) => entry.quantity === signal.quantity && entry.unit === signal.unit,
  );
  return axis ? axis.id : null;
}

/** Ensure an axis for the signal's quantity+unit and return it with a copy. */
export function AssignAxis(chart, signal) {
  const next = Clone(chart);
  const existing = AxisIdForSignal(next, signal);
  if (existing !== null) {
    return { chart: next, axisId: existing };
  }
  const axisId = NextId(
    'axis',
    next.axes.map((axis) => axis.id),
  );
  next.axes.push({
    id: axisId,
    quantity: signal.quantity ?? '',
    unit: signal.unit ?? '',
    range: null,
  });
  return { chart: next, axisId };
}

export function AddCurve(chart, signal, style = 'auto') {
  if (!signal || typeof signal.id !== 'string') {
    throw new ChartError('a curve needs a signal id');
  }
  if (chart.curves.some((curve) => curve.signalId === signal.id)) {
    return Clone(chart);
  }
  if (chart.curves.length >= MAX_CURVES_PER_CHART) {
    throw new ChartError('a chart holds more than 128 curves');
  }
  const assigned = AssignAxis(chart, signal);
  const next = assigned.chart;
  next.curves.push({
    signalId: signal.id,
    axisId: assigned.axisId,
    style:
      style === 'auto'
        ? NextId(
            'style',
            next.curves.map((curve) => curve.style),
          )
        : style,
  });
  return next;
}

export function RemoveCurve(chart, signalId) {
  const next = Clone(chart);
  next.curves = next.curves.filter((curve) => curve.signalId !== signalId);
  const usedAxes = new Set(next.curves.map((curve) => curve.axisId));
  next.axes = next.axes.filter((axis) => usedAxes.has(axis.id));
  return next;
}

export function CurvesForAxis(chart, axisId) {
  return chart.curves.filter((curve) => curve.axisId === axisId);
}

export function ValidateAxisRange(range) {
  if (range === null || range === undefined) {
    return null;
  }
  if (!Array.isArray(range) || range.length !== 2) {
    throw new ChartError('an axis range is not a pair');
  }
  const [low, high] = range;
  if (!Number.isFinite(low) || !Number.isFinite(high) || !(low < high)) {
    throw new ChartError('an axis range must be finite and increasing');
  }
  return [low, high];
}

export function SetAxisRange(chart, axisId, range) {
  const checked = ValidateAxisRange(range);
  const next = Clone(chart);
  const axis = next.axes.find((entry) => entry.id === axisId);
  if (!axis) {
    throw new ChartError(`no axis ${axisId}`);
  }
  axis.range = checked;
  return next;
}

export function ClearAxisRange(chart, axisId) {
  return SetAxisRange(chart, axisId, null);
}

export function ValidateChart(chart) {
  if (!chart || typeof chart !== 'object') {
    throw new ChartError('a chart is not an object');
  }
  if (typeof chart.id !== 'string' || chart.id === '') {
    throw new ChartError('a chart has no id');
  }
  if (chart.title !== undefined && typeof chart.title !== 'string') {
    throw new ChartError('a chart title is not a string');
  }
  if (!Array.isArray(chart.curves) || !Array.isArray(chart.axes)) {
    throw new ChartError('a chart needs curves and axes arrays');
  }
  if (chart.curves.length > MAX_CURVES_PER_CHART) {
    throw new ChartError('a chart holds more than 128 curves');
  }
  const axisIds = new Set();
  for (const axis of chart.axes) {
    if (!axis || typeof axis.id !== 'string' || axisIds.has(axis.id)) {
      throw new ChartError('an axis id is missing or duplicated');
    }
    axisIds.add(axis.id);
    ValidateAxisRange(axis.range ?? null);
    if (axis.quantity !== undefined && typeof axis.quantity !== 'string') {
      throw new ChartError('an axis quantity is not a string');
    }
    if (axis.unit !== undefined && typeof axis.unit !== 'string') {
      throw new ChartError('an axis unit is not a string');
    }
  }
  const signals = new Set();
  for (const curve of chart.curves) {
    if (!curve || typeof curve.signalId !== 'string' || curve.signalId === '') {
      throw new ChartError('a curve has no signal id');
    }
    if (signals.has(curve.signalId)) {
      throw new ChartError(`a chart repeats ${curve.signalId}`);
    }
    signals.add(curve.signalId);
    if (!axisIds.has(curve.axisId)) {
      throw new ChartError('a curve names no axis of its chart');
    }
  }
}

export function AxisGroups(chart) {
  return chart.axes.map((axis) => ({
    axis,
    curves: CurvesForAxis(chart, axis.id),
  }));
}
