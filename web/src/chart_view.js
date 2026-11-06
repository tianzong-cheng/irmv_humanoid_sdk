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

function Ticks(low, high, count = 5) {
  const ticks = [];
  for (let index = 0; index <= count; index += 1) {
    ticks.push(low + ((high - low) * index) / count);
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
  return Number(value.toFixed(3)).toString();
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

function CurveColor(canvas, curve, index) {
  const style = getComputedStyle(canvas);
  const colors = Palette(canvas);
  if (curve.style && curve.style.startsWith('#')) {
    return curve.style;
  }
  return colors[index % colors.length];
}

export function Render(canvas, chart, series, timeWindow, cursor = null, markers = []) {
  const width = Math.max(1, Math.round(canvas.clientWidth || 640));
  const height = Math.max(1, Math.round(canvas.clientHeight || 280));
  const ratio = globalThis.devicePixelRatio || 1;
  if (canvas.width !== Math.round(width * ratio) ||
      canvas.height !== Math.round(height * ratio)) {
    canvas.width = Math.round(width * ratio);
    canvas.height = Math.round(height * ratio);
  }
  const ctx = canvas.getContext('2d');
  if (!ctx) {
    return { drawn: false, axisWidths: [], plotted: 0 };
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
  const span = end > begin ? end - begin : 1n;
  const axes = chart.axes.length > 0
    ? chart.axes
    : [{ id: 'axis-1', quantity: '', unit: '' }];
  ctx.font = '11px system-ui, sans-serif';
  const axisWidths = axes.map((axis) => {
    const label = `${axis.quantity || 'value'} (${axis.unit || '-'})`;
    return Math.max(72, Math.ceil(ctx.measureText(label).width) + 14);
  });
  const gutter = axisWidths.reduce((sum, value) => sum + value, 0);
  const plotLeft = gutter + 6;
  const plotRight = width - 8;
  const plotTop = 28;
  const plotBottom = height - 16;
  if (plotRight <= plotLeft || plotBottom <= plotTop) {
    return { drawn: false, axisWidths, plotted: 0 };
  }

  if (surface !== 'transparent') {
    ctx.fillStyle = surface;
    ctx.fillRect(0, 0, width, height);
  }
  const ranges = new Map();
  for (const axis of axes) {
    ranges.set(axis.id, AxisRange(chart, series, axis.id));
  }

  // Axis columns are drawn from the plot outwards; each has its own column.
  let columnLeft = 0;
  for (let index = 0; index < axes.length; index += 1) {
    const axis = axes[index];
    const columnWidth = axisWidths[index];
    const [low, high] = ranges.get(axis.id);
    const ticks = Ticks(low, high);
    ctx.strokeStyle = border;
    ctx.fillStyle = muted;
    ctx.lineWidth = 1;
    ctx.beginPath();
    const axisX = columnLeft + columnWidth - 4;
    ctx.moveTo(axisX, plotTop);
    ctx.lineTo(axisX, plotBottom);
    ctx.stroke();
    const label = `${axis.quantity || 'value'} (${axis.unit || '-'})`;
    // The label lives in the axis's own column above the ticks. It is measured
    // into the column width, so a long name widens the column instead of
    // covering another axis or a tick value.
    ctx.save();
    ctx.beginPath();
    ctx.rect(columnLeft, 0, columnWidth, plotTop);
    ctx.clip();
    ctx.fillStyle = fg;
    ctx.fillText(label, columnLeft + 4, 13);
    ctx.restore();
    for (const tick of ticks) {
      const y = plotBottom - ((tick - low) / (high - low)) * (plotBottom - plotTop);
      ctx.strokeStyle = border;
      ctx.beginPath();
      ctx.moveTo(axisX - 4, y);
      ctx.lineTo(axisX, y);
      ctx.stroke();
      ctx.fillStyle = muted;
      ctx.fillText(FormatValue(tick), columnLeft + 4, y + 3);
    }
    columnLeft += columnWidth;
  }

  ctx.strokeStyle = border;
  ctx.strokeRect(plotLeft, plotTop, plotRight - plotLeft, plotBottom - plotTop);

  const colorByCurve = new Map();
  let plotted = 0;
  chart.curves.forEach((curve, index) => {
    const entry = series.find((item) => item?.signal?.id === curve.signalId);
    if (!entry) {
      return;
    }
    const axis = axes.find((item) => item.id === curve.axisId) ?? axes[0];
    const [low, high] = ranges.get(axis.id);
    const color = CurveColor(canvas, curve, index);
    colorByCurve.set(curve.signalId, color);
    ctx.strokeStyle = color;
    ctx.fillStyle = color;
    ctx.lineWidth = 1.5;
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
      const x = plotLeft + Number(((stamp - begin) * 1000n) / span) / 1000 *
        (plotRight - plotLeft);
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

  for (const marker of markers) {
    const stamp = Us(marker.timeUs);
    if (stamp < begin || stamp > end) {
      continue;
    }
    const x = plotLeft + Number(((stamp - begin) * 1000n) / span) / 1000 *
      (plotRight - plotLeft);
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
      const x = plotLeft + Number(((stamp - begin) * 1000n) / span) / 1000 *
        (plotRight - plotLeft);
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

  return { drawn: plotted > 0 || axes.length > 0, axisWidths, plotted };
}

export function TimeToX(stampUs, timeWindow, plotLeft, plotRight) {
  const begin = Us(timeWindow.beginUs);
  const end = Us(timeWindow.endUs);
  const span = end > begin ? end - begin : 1n;
  return plotLeft + Number(((Us(stampUs) - begin) * 1000n) / span) / 1000 *
    (plotRight - plotLeft);
}
