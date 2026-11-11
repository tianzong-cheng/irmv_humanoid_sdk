/**
 * Canvas 2D rendering of one chart. It takes a layout chart, the already
 * loaded series and a shared time window; it never loads data, advances time or
 * changes the layout. All value axes live on the left, in their own measured
 * columns, so labels never overlap and none is dropped when a split is narrow.
 */

import { Us } from './api.js';

const FALLBACK_COLORS = ['#4f8ef7', '#f2a33c', '#3bb273', '#d95d8c', '#8a7cf0', '#38a3a5'];

function FiniteNumbers(series, signalId) {
  const entry = series.find((item) => item?.signal?.id === signalId);
  if (!entry) {
    return [];
  }
  return entry.points
    .map((point) => (typeof point.v === 'number' && Number.isFinite(point.v) ? point.v : null))
    .filter((value) => value !== null);
}

function AxisRange(chart, series, axisId) {
  const axis = chart.axes.find((entry) => entry.id === axisId);
  if (axis?.range && axis.range.length === 2) {
    return axis.range;
  }
  const values = [];
  for (const curve of chart.curves.filter((entry) => entry.axisId === axisId)) {
    values.push(...FiniteNumbers(series, curve.signalId));
  }
  if (values.length === 0) {
    return [-1, 1];
  }
  let low = Math.min(...values);
  let high = Math.max(...values);
  if (low === high) {
    low -= 1;
    high += 1;
  }
  const padding = (high - low) * 0.08;
  return [low - padding, high + padding];
}

function NiceStep(target) {
  const power = 10 ** Math.floor(Math.log10(target));
  const fraction = target / power;
  return (fraction < 1.5 ? 1 : fraction < 3.5 ? 2 : fraction < 7.5 ? 5 : 10) * power;
}

export function ValueTicks(low, high, count = 5) {
  if (!Number.isFinite(low) || !Number.isFinite(high) || high <= low) {
    return [];
  }
  const step = NiceStep((high - low) / Math.max(1, count));
  if (!Number.isFinite(step) || step <= 0) {
    return [];
  }
  const first = Math.ceil(low / step - 1e-10);
  const last = Math.floor(high / step + 1e-10);
  const ticks = [];
  for (let index = 0; index <= last - first; index += 1) {
    // Multiplication avoids accumulated drift; rounding removes binary tails.
    const value = Number(((first + index) * step).toPrecision(12));
    ticks.push(Object.is(value, -0) ? 0 : value);
  }
  return ticks;
}

export function TimeTicks(begin, end, origin, count = 5) {
  if (origin === null || origin === undefined || end <= begin) {
    return [];
  }
  const step = BigInt(Math.max(1, Math.round(NiceStep(
    Number(end - begin) / Math.max(1, count),
  ))));
  const offset = begin - origin;
  // BigInt division truncates toward zero, which is ceil only for negatives.
  const first = (offset / step + (offset % step > 0n ? 1n : 0n)) * step;
  const ticks = [];
  for (let stamp = origin + first; stamp <= end; stamp += step) {
    ticks.push(stamp);
  }
  return ticks;
}

function FormatValue(value) {
  if (!Number.isFinite(value)) {
    return '';
  }
  const magnitude = Math.abs(value);
  if (magnitude !== 0 && (magnitude < 0.001 || magnitude >= 100000)) {
    return value.toExponential(1);
  }
  return Number(value.toPrecision(6)).toString();
}

function Palette(canvas) {
  const style = getComputedStyle(canvas);
  const colors = [];
  for (let index = 1; index <= 6; index += 1) {
    const color = style.getPropertyValue(`--c${index}`).trim();
    if (color) {
      colors.push(color);
    }
  }
  return colors.length > 0 ? colors : FALLBACK_COLORS;
}

export function CurveAppearance(canvas, curve, index) {
  const colors = Palette(canvas);
  const styleIndex = /^style-(\d+)$/.exec(curve.style ?? '');
  const slot = styleIndex ? Number(styleIndex[1]) - 1 : index;
  const patterns = [[], [6, 3], [2, 3], [8, 3, 2, 3]];
  return {
    color: curve.style?.startsWith('#') ? curve.style : colors[slot % colors.length],
    dash: patterns[slot % patterns.length],
  };
}

/** Map a one-dimensional selection to shared time or local value ranges. */
export function SelectionZoom(view, start, end, timeWindow, mode = 'time') {
  const plot = view.plot;
  const clampX = (x) => Math.max(plot.left, Math.min(plot.right, x));
  const clampY = (y) => Math.max(plot.top, Math.min(plot.bottom, y));
  const left = clampX(Math.min(start.x, end.x));
  const right = clampX(Math.max(start.x, end.x));
  const top = clampY(Math.min(start.y, end.y));
  const bottom = clampY(Math.max(start.y, end.y));
  if (mode === 'value') {
    if (bottom - top < 6) return null;
    return {
      window: null,
      axes: view.ranges.map(({ axisId, low, high }) => ({
        axisId,
        range: [
          low + (plot.bottom - bottom) / (plot.bottom - plot.top) * (high - low),
          low + (plot.bottom - top) / (plot.bottom - plot.top) * (high - low),
        ],
      })),
    };
  }
  if (right - left < 6) {
    return null;
  }
  const span = Us(timeWindow.endUs) - Us(timeWindow.beginUs);
  if (span <= 0n) {
    return null;
  }
  const stamp = (x) => Us(timeWindow.beginUs) + span *
    BigInt(Math.round((x - plot.left) / (plot.right - plot.left) * 1e9)) / 1000000000n;
  const beginUs = stamp(left);
  const endUs = stamp(right);
  if (endUs <= beginUs) {
    return null;
  }
  return {
    window: { beginUs, endUs },
    axes: [],
  };
}

export function Render(canvas, chart, series, timeWindow, cursor = null, markers = [], highlightedSignal = null, timeOriginUs = null) {
  const ctx = canvas.getContext('2d');
  if (!ctx) {
    return { drawn: false, axisWidths: [], plotted: 0 };
  }
  const axes = chart.axes.length > 0
    ? chart.axes
    : [{ id: 'axis-1', quantity: '', unit: '' }];
  ctx.font = '11px system-ui, sans-serif';
  const ranges = new Map(axes.map(axis => [axis.id, AxisRange(chart, series, axis.id)]));
  const axisWidths = axes.map((axis) => {
    const [low, high] = ranges.get(axis.id);
    const labels = [low, high, ...ValueTicks(low, high)].map(FormatValue);
    return Math.max(36, ...labels.map(label => Math.ceil(ctx.measureText(label).width) + 12));
  });
  const gutter = axisWidths.reduce((sum, value) => sum + value, 0);
  // Keep every value axis in the canvas. Narrow chart bodies scroll rather
  // than dropping labels or duplicating them in a separate DOM sidebar.
  canvas.style.minWidth = `${gutter + 180}px`;
  const width = Math.max(1, Math.round(canvas.clientWidth || 640));
  const height = Math.max(1, Math.round(canvas.clientHeight || 280));
  const ratio = globalThis.devicePixelRatio || 1;
  if (canvas.width !== Math.round(width * ratio) ||
      canvas.height !== Math.round(height * ratio)) {
    canvas.width = Math.round(width * ratio);
    canvas.height = Math.round(height * ratio);
  }
  ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
  ctx.clearRect(0, 0, width, height);
  const style = getComputedStyle(canvas);
  const fg = style.getPropertyValue('--fg').trim() || '#d8dee9';
  const muted = style.getPropertyValue('--muted').trim() || '#7f8794';
  const border = style.getPropertyValue('--border').trim() || '#39414d';
  const surface = style.getPropertyValue('--surface').trim() || 'transparent';

  const begin = Us(timeWindow.beginUs);
  const end = Us(timeWindow.endUs);
  // Resizing the backing canvas resets the context font.
  ctx.font = '11px system-ui, sans-serif';
  const plotLeft = gutter + 6;
  const plotRight = width - 8;
  const plotTop = 10;
  const plotBottom = height - 26;
  if (plotRight <= plotLeft || plotBottom <= plotTop) {
    return { drawn: false, axisWidths, plotted: 0 };
  }

  if (surface !== 'transparent') {
    ctx.fillStyle = surface;
    ctx.fillRect(0, 0, width, height);
  }
  const intervals = Math.max(1, Math.floor((plotRight - plotLeft) / 100));
  const origin = timeOriginUs === null || timeOriginUs === undefined ? null : Us(timeOriginUs);
  const timeTicks = TimeTicks(begin, end, origin, intervals);
  const valueIntervals = Math.max(1, Math.min(5, Math.floor((plotBottom - plotTop) / 40)));
  // A single horizontal grid follows the first value axis; overlaying grids
  // from unrelated units would make the background ambiguous and cluttered.
  const [gridLow, gridHigh] = ranges.get(axes[0].id);
  ctx.save();
  ctx.strokeStyle = muted;
  ctx.globalAlpha = 0.16;
  ctx.lineWidth = 1;
  ctx.beginPath();
  for (const stamp of timeTicks) {
    const x = TimeToX(stamp, timeWindow, plotLeft, plotRight);
    ctx.moveTo(x, plotTop);
    ctx.lineTo(x, plotBottom);
  }
  for (const tick of ValueTicks(gridLow, gridHigh, valueIntervals)) {
    const y = plotBottom - (tick - gridLow) / (gridHigh - gridLow) * (plotBottom - plotTop);
    ctx.moveTo(plotLeft, y);
    ctx.lineTo(plotRight, y);
  }
  ctx.stroke();
  ctx.restore();

  // Axis columns are drawn from the plot outwards; each has its own column.
  let columnLeft = 0;
  for (let index = 0; index < axisWidths.length; index += 1) {
    const axis = axes[index];
    const columnWidth = axisWidths[index];
    const [low, high] = ranges.get(axis.id);
    const ticks = ValueTicks(low, high, valueIntervals);
    ctx.fillStyle = muted;
    for (const tick of ticks) {
      const y = plotBottom - ((tick - low) / (high - low)) * (plotBottom - plotTop);
      ctx.fillStyle = muted;
      ctx.fillText(FormatValue(tick), columnLeft + 4, y + 3);
    }
    columnLeft += columnWidth;
  }

  ctx.strokeStyle = border;
  ctx.strokeRect(plotLeft, plotTop, plotRight - plotLeft, plotBottom - plotTop);

  ctx.font = '10px system-ui, sans-serif';
  ctx.fillStyle = muted;
  for (const stamp of timeTicks) {
    const x = TimeToX(stamp, timeWindow, plotLeft, plotRight);
    const delta = stamp - origin;
    const magnitude = delta < 0n ? -delta : delta;
    const fraction = String(magnitude % 1000000n).padStart(6, '0').replace(/0+$/, '');
    const label = `${delta < 0n ? '-' : ''}${magnitude / 1000000n}${fraction ? `.${fraction}` : ''} s`;
    const halfLabel = ctx.measureText(label).width / 2;
    ctx.textAlign = x - halfLabel < plotLeft ? 'left'
      : x + halfLabel > plotRight ? 'right' : 'center';
    ctx.fillText(label, x, height - 7);
  }
  ctx.textAlign = 'left';

  // Manual/box ranges can put samples outside the plot. Keep traces and cursor
  // dots inside the plot instead of drawing over labels and neighboring axes.
  ctx.save();
  ctx.beginPath();
  ctx.rect(plotLeft, plotTop, plotRight - plotLeft, plotBottom - plotTop);
  ctx.clip();
  const colorByCurve = new Map();
  let plotted = 0;
  chart.curves.forEach((curve, index) => {
    const entry = series.find((item) => item?.signal?.id === curve.signalId);
    if (!entry) {
      return;
    }
    const axis = axes.find((item) => item.id === curve.axisId) ?? axes[0];
    const [low, high] = ranges.get(axis.id);
    const { color, dash } = CurveAppearance(canvas, curve, index);
    colorByCurve.set(curve.signalId, color);
    ctx.strokeStyle = color;
    ctx.fillStyle = color;
    ctx.setLineDash(dash);
    ctx.globalAlpha = highlightedSignal && highlightedSignal !== curve.signalId ? 0.25 : 1;
    ctx.lineWidth = highlightedSignal === curve.signalId ? 2.8 : 1.7;
    const step = entry.signal?.value_kind === 'bool' || entry.signal?.value_kind === 'enum';
    let previous = null;
    let lastSegment = null;
    ctx.beginPath();
    for (const point of entry.points) {
      if (!point.valid || typeof point.v !== 'number' || !Number.isFinite(point.v)) {
        previous = null;
        lastSegment = point.segment;
        continue;
      }
      if (lastSegment !== null && point.segment !== lastSegment) {
        previous = null;
      }
      lastSegment = point.segment;
      const stamp = Us(point.t);
      const x = TimeToX(stamp, timeWindow, plotLeft, plotRight);
      const y = plotBottom - ((point.v - low) / (high - low)) * (plotBottom - plotTop);
      if (previous === null) {
        ctx.moveTo(x, y);
      } else if (step) {
        ctx.lineTo(x, previous.y);
        ctx.lineTo(x, y);
      } else {
        ctx.lineTo(x, y);
      }
      previous = { x, y };
      plotted += 1;
    }
    ctx.stroke();
  });

  ctx.setLineDash([]);
  ctx.globalAlpha = 1;
  for (const marker of markers) {
    const stamp = Us(marker.timeUs);
    if (stamp < begin || stamp > end) {
      continue;
    }
    const x = TimeToX(stamp, timeWindow, plotLeft, plotRight);
    ctx.fillStyle = style.getPropertyValue('--error').trim() || '#d95d8c';
    ctx.beginPath();
    ctx.moveTo(x - 3, plotTop);
    ctx.lineTo(x + 3, plotTop);
    ctx.lineTo(x, plotTop + 5);
    ctx.closePath();
    ctx.fill();
  }

  if (cursor?.timeUs !== undefined && cursor.timeUs !== null) {
    const stamp = Us(cursor.timeUs);
    if (stamp >= begin && stamp <= end) {
      const x = TimeToX(stamp, timeWindow, plotLeft, plotRight);
      ctx.strokeStyle = fg;
      ctx.globalAlpha = 0.6;
      ctx.beginPath();
      ctx.moveTo(x, plotTop);
      ctx.lineTo(x, plotBottom);
      ctx.stroke();
      ctx.globalAlpha = 1;
      const values = cursor.values ?? new Map();
      chart.curves.forEach((curve) => {
        const value = values.get?.(curve.signalId);
        if (!value || !value.valid || typeof value.value !== 'number' ||
            !Number.isFinite(value.value)) {
          return;
        }
        const axis = axes.find((item) => item.id === curve.axisId) ?? axes[0];
        const [low, high] = ranges.get(axis.id);
        const y = plotBottom -
          ((value.value - low) / (high - low)) * (plotBottom - plotTop);
        ctx.fillStyle = colorByCurve.get(curve.signalId) ?? fg;
        ctx.beginPath();
        ctx.arc(x, y, 3, 0, Math.PI * 2);
        ctx.fill();
      });
    }
  }

  ctx.restore();
  return {
    drawn: plotted > 0 || axes.length > 0, axisWidths, plotted,
    plot: { left: plotLeft, right: plotRight, top: plotTop, bottom: plotBottom },
    ranges: axes.map((axis) => ({ axisId: axis.id, low: ranges.get(axis.id)[0], high: ranges.get(axis.id)[1] })),
  };
}

export function TimeToX(stampUs, timeWindow, plotLeft, plotRight) {
  const begin = Us(timeWindow.beginUs);
  const end = Us(timeWindow.endUs);
  const span = end > begin ? end - begin : 1n;
  // Subtract absolute timestamps as BigInt first. Only the relative duration
  // becomes a float, preserving precision above 2^53 without quantizing the
  // screen position into a fixed number of time buckets.
  return plotLeft + Number(Us(stampUs) - begin) / Number(span) *
    (plotRight - plotLeft);
}
