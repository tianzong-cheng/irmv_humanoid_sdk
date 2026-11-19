/**
 * DOM view of the dock tree. It owns drag/drop previews, split dividers, chart
 * legends and value-axis controls; layout changes are delegated to
 * the models, so the view never invents a chart or a split of its own.
 */

import { CurveAppearance, Render, SelectionZoom, TimeToX } from './chart_view.js';
import { ValidateLayout } from './layout_model.js';
import { Us } from './api.js';

// A split must preserve space for its plot and legend rather than clipping
// controls when several charts share a short workspace. The dock can scroll.
const MIN_CHART_HEIGHT = 160;
function MinimumHeight(node) {
  if (node.type !== 'split') {
    return MIN_CHART_HEIGHT;
  }
  const first = MinimumHeight(node.first);
  const second = MinimumHeight(node.second);
  return node.direction === 'column' ? first + second + 6 : Math.max(first, second);
}

export class DockView {
  constructor(root, callbacks = {}) {
    this.root = root;
    this.callbacks = callbacks;
    this.layout = null;
    this.resolved = null;
    this.series = [];
    this.timeWindow = { beginUs: 0n, endUs: 1n };
    this.cursor = null;
    this.timeOriginUs = null;
    this.markers = [];
    this.canvases = new Map();
    this.plotViews = new Map();
    this.cursorElements = new Map();
    this.cursorFrame = null;
    this.selection = null;
    this.highlightedSignal = null;
    this.highlightedChartId = null;
    // Canvas backing dimensions must follow the responsive workspace, including
    // when a hidden page becomes visible again.
    this.resizeObserver = typeof ResizeObserver === 'function'
      ? new ResizeObserver(() => {
        if (this.root.clientWidth > 0) {
          this.selection?.cancel();
          this.Redraw();
        }
      })
      : null;
    this.resizeObserver?.observe(this.root);
    this.root.ownerDocument.addEventListener('dragend', () => this.ClearDropPreviews());
  }

  SetView(layout, resolved, series, timeWindow, cursor) {
    this.layout = layout;
    this.resolved = resolved;
    this.series = series;
    this.timeWindow = timeWindow;
    this.cursor = cursor;
    if (!this.selection) {
      // Keep interactive targets alive across live data/time refreshes.
      if (layout === this.renderedLayout && resolved === this.renderedResolved) {
        this.Redraw();
      } else {
        this.Render();
      }
    }
  }

  SetCursor(cursor) {
    this.cursor = cursor;
    this.ScheduleCursorDraw();
  }

  SetMarkers(markers) {
    this.markers = markers ?? [];
    this.Redraw();
  }

  Render() {
    if (!this.layout) {
      return;
    }
    this.highlightedSignal = null;
    this.highlightedChartId = null;
    this.cursorElements.clear();
    this.plotViews.clear();
    this.canvases.clear();
    this.root.replaceChildren(this.RenderNode(this.layout.dock, []));
    this.renderedLayout = this.layout;
    this.renderedResolved = this.resolved;
    this.DrawAll();
  }

  RenderNode(node, path) {
    if (node.type === 'empty') {
      const empty = document.createElement('div');
      empty.className = 'workspace-empty';
      empty.dataset.dropZone = 'empty';
      const title = document.createElement('strong');
      title.textContent = 'Build your signal view';
      const hint = document.createElement('p');
      hint.textContent = 'Choose an experiment above, then drag signals here to compare them.';
      empty.append(title, hint);
      this.BindDrop(empty, null);
      return empty;
    }
    if (node.type === 'leaf') {
      return this.RenderChart(node.chartId);
    }
    const split = document.createElement('div');
    split.className = 'dock-split';
    split.dataset.direction = node.direction;
    split.style.flexDirection = node.direction;
    split.style.minHeight = `${MinimumHeight(node)}px`;
    split.dataset.path = JSON.stringify(path);
    const first = document.createElement('div');
    first.className = 'dock-pane';
    first.style.minHeight = `${MinimumHeight(node.first)}px`;
    first.style.flex = `${node.ratio} 1 0`;
    first.append(this.RenderNode(node.first, [...path, 0]));
    const divider = document.createElement('div');
    divider.className = 'dock-divider';
    divider.setAttribute('role', 'separator');
    divider.dataset.splitPath = JSON.stringify(path);
    divider.addEventListener('pointerdown', (event) => {
      event.preventDefault();
      const start = node.direction === 'row' ? event.clientX : event.clientY;
      const bounds = split.getBoundingClientRect();
      const move = (moveEvent) => {
        const current = node.direction === 'row' ? moveEvent.clientX : moveEvent.clientY;
        const total = node.direction === 'row' ? bounds.width : bounds.height;
        if (total <= 0) {
          return;
        }
        const delta = (current - start) / total;
        const ratio = Math.min(0.9, Math.max(0.1, node.ratio + delta));
        this.callbacks.onResize?.(path, ratio);
      };
      const up = () => {
        window.removeEventListener('pointermove', move);
        window.removeEventListener('pointerup', up);
      };
      window.addEventListener('pointermove', move);
      window.addEventListener('pointerup', up);
    });
    const second = document.createElement('div');
    second.className = 'dock-pane';
    second.style.minHeight = `${MinimumHeight(node.second)}px`;
    second.style.flex = `${1 - node.ratio} 1 0`;
    second.append(this.RenderNode(node.second, [...path, 1]));
    split.append(first, divider, second);
    return split;
  }

  RenderChart(chartId) {
    const chart = this.resolved?.charts.find((entry) => entry.id === chartId) ??
      this.layout.charts.find((entry) => entry.id === chartId);
    const shell = document.createElement('section');
    shell.className = 'chart';
    shell.style.minHeight = `${MIN_CHART_HEIGHT}px`;
    shell.dataset.chartId = chartId;
    const body = document.createElement('div');
    body.className = 'chart-body';
    body.dataset.chartId = chartId;
    const preview = document.createElement('div');
    preview.className = 'chart-drop-preview';
    preview.hidden = true;
    preview.setAttribute('aria-hidden', 'true');
    const previewLabel = document.createElement('span');
    preview.append(previewLabel);
    this.BindDrop(body, chartId, preview);
    const canvas = document.createElement('canvas');
    canvas.className = 'chart-canvas';
    canvas.dataset.chartId = chartId;
    canvas.title = 'Drag to select time across all charts. Ctrl-drag to zoom this chart’s value axes. Double-click to reset value axes. Esc cancels selection.';
    this.BindSelection(canvas, shell, chartId);
    body.append(canvas);
    body.addEventListener('scroll', () => this.ScheduleCursorDraw(), { passive: true });
    const cursorLine = document.createElement('div');
    cursorLine.className = 'chart-cursor';
    cursorLine.hidden = true;
    cursorLine.setAttribute('aria-hidden', 'true');
    this.cursorElements.set(chartId, cursorLine);
    const legend = document.createElement('div');
    legend.className = 'chart-legend';
    legend.draggable = true;
    legend.title = 'Drag the legend area to move this chart';
    legend.addEventListener('dragstart', event => {
      if (event.target.closest('button')) {
        event.preventDefault();
        return;
      }
      event.dataTransfer?.setData('application/x-irmv-chart', chartId);
      if (event.dataTransfer) event.dataTransfer.effectAllowed = 'move';
    });
    for (const [index, curve] of (chart?.curves ?? []).entries()) {
      const item = document.createElement('span');
      item.className = 'legend-item';
      item.dataset.signalId = curve.signalId;
      item.dataset.chartId = chartId;
      const label = document.createElement('span');
      label.className = 'legend-label';
      item.title = curve.signalId;
      const { color, dash } = CurveAppearance(this.root, curve, index);
      const swatch = document.createElement('span');
      swatch.className = 'legend-swatch';
      swatch.setAttribute('aria-hidden', 'true');
      swatch.style.borderTopColor = color;
      swatch.style.borderTopStyle = dash.length ? (dash[0] === 2 ? 'dotted' : 'dashed') : 'solid';
      item.append(swatch);
      item.addEventListener('mouseenter', () => {
        item.classList.add('highlight');
        this.highlightedSignal = curve.signalId;
        this.highlightedChartId = chartId;
        this.Redraw();
      });
      item.addEventListener('mouseleave', () => {
        item.classList.remove('highlight');
        this.highlightedSignal = null;
        this.highlightedChartId = null;
        this.Redraw();
      });
      if (curve.missing) {
        item.classList.add('missing');
        label.textContent = `${curve.signalId} (missing)`;
        const replace = document.createElement('button');
        replace.type = 'button';
        replace.className = 'legend-replace';
        replace.textContent = 'Replace';
        replace.addEventListener('click', () =>
          this.callbacks.onReplaceSignal?.(chartId, curve.signalId));
        item.append(label, replace);
      } else {
        const signal = curve.signal;
        label.textContent = signal
          ? `${signal.joint_name || signal.source} / ${signal.label}`
          : curve.signalId;
        const unit = document.createElement('span');
        unit.className = 'legend-unit';
        unit.textContent = signal?.unit || signal?.value_kind || 'value';
        unit.title = `Automatic axis: ${signal?.quantity || 'value'} (${unit.textContent})`;
        const remove = document.createElement('button');
        remove.type = 'button';
        remove.className = 'legend-remove';
        remove.title = 'Remove curve';
        remove.setAttribute('aria-label', `Remove ${label.textContent}`);
        remove.textContent = '×';
        remove.addEventListener('click', () =>
          this.callbacks.onRemoveSignal?.(chartId, curve.signalId));
        item.append(label, unit, remove);
      }
      legend.append(item);
    }
    const reset = document.createElement('button');
    reset.type = 'button';
    reset.className = 'chart-reset-y';
    reset.textContent = 'Reset Y';
    reset.title = 'Restore automatic value axes for this chart';
    reset.setAttribute('aria-label', 'Reset value axes');
    reset.addEventListener('click', () => this.callbacks.onResetValueAxes?.(chartId));
    const footer = document.createElement('div');
    footer.className = 'chart-footer';
    footer.append(legend, reset);
    shell.append(body, footer, preview, cursorLine);
    this.canvases.set(chartId, canvas);
    return shell;
  }

  ClearDropPreviews() {
    for (const element of this.root.querySelectorAll('.drop-empty, .drop-center, .drop-left, .drop-right, .drop-top, .drop-bottom')) {
      element.classList.remove('drop-empty', 'drop-center', 'drop-left',
        'drop-right', 'drop-top', 'drop-bottom');
    }
    for (const preview of this.root.querySelectorAll('.chart-drop-preview')) {
      preview.hidden = true;
    }
  }

  BindDrop(element, chartId, preview = null) {
    const clear = () => {
      element.classList.remove('drop-empty', 'drop-center', 'drop-left',
        'drop-right', 'drop-top', 'drop-bottom');
      if (preview) {
        preview.hidden = true;
      }
    };
    element.addEventListener('dragover', (event) => {
      event.preventDefault();
      event.dataTransfer.dropEffect = 'copy';
      const zone = chartId === null ? 'empty' : this.ZoneAt(event, element);
      this.ClearDropPreviews();
      element.classList.add(`drop-${zone}`);
      if (preview) {
        // The preview is a sibling of the scrolling plot, so its coordinates
        // follow the visible target rather than the scrolled canvas content.
        const bounds = element.getBoundingClientRect();
        const parent = preview.parentElement.getBoundingClientRect();
        const width = element.clientWidth;
        const height = element.clientHeight;
        const horizontal = zone === 'left' || zone === 'right';
        const vertical = zone === 'top' || zone === 'bottom';
        preview.style.left = `${bounds.left - parent.left + (zone === 'right' ? width / 2 : 0)}px`;
        preview.style.top = `${bounds.top - parent.top + (zone === 'bottom' ? height / 2 : 0)}px`;
        preview.style.width = `${horizontal ? width / 2 : width}px`;
        preview.style.height = `${vertical ? height / 2 : height}px`;
        preview.firstElementChild.textContent = zone === 'center'
          ? 'Overlay signal'
          : `Split ${zone}`;
        preview.hidden = false;
      }
    });
    element.addEventListener('dragleave', (event) => {
      if (event.relatedTarget && element.contains(event.relatedTarget)) {
        return;
      }
      clear();
    });
    element.addEventListener('drop', (event) => {
      event.preventDefault();
      this.ClearDropPreviews();
      const zone = chartId === null ? 'empty' : this.ZoneAt(event, element);
      const signalId = event.dataTransfer?.getData('application/x-irmv-signal');
      const movedChart = event.dataTransfer?.getData('application/x-irmv-chart');
      if (movedChart && chartId !== null && zone !== 'center') {
        this.callbacks.onMoveChart?.(movedChart, chartId, zone);
        return;
      }
      if (signalId) {
        this.callbacks.onDropSignal?.(signalId, chartId, zone);
      }
    });
  }

  ZoneAt(event, element) {
    const bounds = element.getBoundingClientRect();
    const x = (event.clientX - bounds.left) / Math.max(1, element.clientWidth);
    const y = (event.clientY - bounds.top) / Math.max(1, element.clientHeight);
    if (x > 0.25 && x < 0.75 && y > 0.25 && y < 0.75) {
      return 'center';
    }
    const distances = [
      ['left', x],
      ['right', 1 - x],
      ['top', y],
      ['bottom', 1 - y],
    ];
    distances.sort((left, right) => left[1] - right[1]);
    return distances[0][0];
  }

  BindSelection(canvas, shell, chartId) {
    const rectangle = document.createElement('div');
    rectangle.className = 'chart-selection';
    rectangle.hidden = true;
    rectangle.setAttribute('aria-hidden', 'true');
    shell.append(rectangle);
    const pointAt = (event, plot) => {
      const bounds = canvas.getBoundingClientRect();
      return {
        x: Math.max(plot.left, Math.min(plot.right, event.clientX - bounds.left)),
        y: Math.max(plot.top, Math.min(plot.bottom, event.clientY - bounds.top)),
      };
    };
    canvas.addEventListener('pointerdown', (event) => {
      if (event.button !== 0 || this.selection) {
        return;
      }
      const view = this.plotViews.get(chartId);
      if (!view?.plot) {
        return;
      }
      const bounds = canvas.getBoundingClientRect();
      const x = event.clientX - bounds.left;
      const y = event.clientY - bounds.top;
      const plot = view.plot;
      if (x < plot.left || x > plot.right || y < plot.top || y > plot.bottom) {
        return;
      }
      event.preventDefault();
      this.callbacks.onSelectionStart?.();
      this.ClearDropPreviews();
      const start = { x, y };
      const timeWindow = { ...this.timeWindow };
      const mode = event.ctrlKey ? 'value' : 'time';
      const selection = { chartId, start, end: start, mode };
      this.selection = selection;
      this.DrawCursors();
      canvas.setPointerCapture(event.pointerId);
      const cleanup = () => {
        rectangle.hidden = true;
        canvas.removeEventListener('pointermove', move);
        canvas.removeEventListener('pointerup', up);
        canvas.removeEventListener('pointercancel', cancel);
        canvas.removeEventListener('lostpointercapture', cancel);
        canvas.ownerDocument.removeEventListener('keydown', escape);
        this.selection = null;
        if (canvas.hasPointerCapture(event.pointerId)) {
          canvas.releasePointerCapture(event.pointerId);
        }
      };
      const move = (moveEvent) => {
        if (moveEvent.pointerId !== event.pointerId) {
          return;
        }
        selection.end = pointAt(moveEvent, plot);
        const canvasBounds = canvas.getBoundingClientRect();
        const parent = shell.getBoundingClientRect();
        const left = mode === 'value' ? plot.left : Math.min(start.x, selection.end.x);
        const top = mode === 'time' ? plot.top : Math.min(start.y, selection.end.y);
        rectangle.style.left = `${canvasBounds.left - parent.left + left}px`;
        rectangle.style.top = `${canvasBounds.top - parent.top + top}px`;
        rectangle.style.width = `${mode === 'value' ? plot.right - plot.left : Math.abs(start.x - selection.end.x)}px`;
        rectangle.style.height = `${mode === 'time' ? plot.bottom - plot.top : Math.abs(start.y - selection.end.y)}px`;
        rectangle.hidden = false;
      };
      const cancel = () => {
        cleanup();
        this.Render();
        this.callbacks.onSelectionEnd?.();
      };
      const escape = (keyEvent) => {
        if (keyEvent.key === 'Escape') {
          keyEvent.preventDefault();
          cancel();
        }
      };
      const up = (upEvent) => {
        if (upEvent.pointerId !== event.pointerId) {
          return;
        }
        const zoom = SelectionZoom(view, start, pointAt(upEvent, plot), timeWindow, mode);
        cleanup();
        if (zoom) {
          this.callbacks.onSelectRegion?.(chartId, zoom);
        } else {
          this.Render();
        }
        this.callbacks.onSelectionEnd?.();
      };
      selection.cancel = cancel;
      canvas.addEventListener('pointermove', move);
      canvas.addEventListener('pointerup', up);
      canvas.addEventListener('pointercancel', cancel);
      canvas.addEventListener('lostpointercapture', cancel);
      canvas.ownerDocument.addEventListener('keydown', escape);
    });
    const timeAt = (event, view) => {
      const point = pointAt(event, view.plot);
      const plot = view.plot;
      const span = Us(this.timeWindow.endUs) - Us(this.timeWindow.beginUs);
      return Us(this.timeWindow.beginUs) + span *
        BigInt(Math.round((point.x - plot.left) / (plot.right - plot.left) * 1e9)) / 1000000000n;
    };
    canvas.addEventListener('pointermove', (event) => {
      if (this.selection) {
        return;
      }
      const view = this.plotViews.get(chartId);
      if (view?.plot) {
        this.callbacks.onCursor?.(timeAt(event, view));
      }
    });
    canvas.addEventListener('wheel', (event) => {
      const view = this.plotViews.get(chartId);
      if (this.selection || !view?.plot || event.deltaY === 0) {
        return;
      }
      event.preventDefault();
      // Read coordinates from this wheel event, not a previous hover or an
      // asynchronous value query. This also works with a stationary mouse.
      this.callbacks.onWheelZoom?.(event.deltaY, timeAt(event, view));
    }, { passive: false });
    canvas.addEventListener('dblclick', () => this.callbacks.onResetValueAxes?.(chartId));
  }

  DrawAll() {
    if (this.selection) {
      return;
    }
    for (const [chartId, canvas] of this.canvases) {
      const chart = this.resolved?.charts.find((entry) => entry.id === chartId) ??
        this.layout?.charts.find((entry) => entry.id === chartId);
      if (!chart || !canvas.isConnected) {
        continue;
      }
      const view = Render(canvas, chart, this.series, this.timeWindow, null,
        this.markers, this.highlightedChartId === chartId ? this.highlightedSignal : null, this.timeOriginUs);
      this.plotViews.set(chartId, view);
    }
    this.DrawCursors();
  }

  ScheduleCursorDraw() {
    if (this.cursorFrame !== null) {
      return;
    }
    this.cursorFrame = requestAnimationFrame(() => {
      this.cursorFrame = null;
      this.DrawCursors();
    });
  }

  DrawCursors() {
    for (const [chartId, line] of this.cursorElements) {
      const canvas = this.canvases.get(chartId);
      const view = this.plotViews.get(chartId);
      const stamp = this.cursor?.timeUs;
      line.hidden = true;
      if (this.selection || stamp === undefined || stamp === null || !view?.plot ||
          !canvas?.isConnected || stamp < this.timeWindow.beginUs || stamp > this.timeWindow.endUs) {
        continue;
      }
      const plot = view.plot;
      const bounds = canvas.getBoundingClientRect();
      const body = canvas.parentElement;
      const visible = body.getBoundingClientRect();
      const parent = line.parentElement.getBoundingClientRect();
      const x = bounds.left + TimeToX(stamp, this.timeWindow, plot.left, plot.right);
      const top = Math.max(bounds.top + plot.top, visible.top);
      const bottom = Math.min(bounds.top + plot.bottom, visible.top + body.clientHeight);
      if (x < visible.left || x > visible.left + body.clientWidth || bottom <= top) {
        continue;
      }
      line.style.left = `${x - parent.left}px`;
      line.style.top = `${top - parent.top}px`;
      line.style.height = `${bottom - top}px`;
      line.replaceChildren();
      line.hidden = false;
      // Exact sampled values arrive separately. Only show dots when they
      // belong to the current pointer time, not a previous mouse position.
      if (this.cursor.valuesTimeUs !== stamp) {
        continue;
      }
      const chart = this.resolved?.charts.find((entry) => entry.id === chartId);
      for (const [index, curve] of (chart?.curves ?? []).entries()) {
        const value = this.cursor.values?.get(curve.signalId);
        const range = view.ranges.find((entry) => entry.axisId === curve.axisId);
        if (!value?.valid || !Number.isFinite(value.value) || !range ||
            value.value < range.low || value.value > range.high) {
          continue;
        }
        const y = bounds.top + plot.bottom -
          (value.value - range.low) / (range.high - range.low) * (plot.bottom - plot.top);
        if (y < top || y > bottom) {
          continue;
        }
        const dot = document.createElement('span');
        dot.className = 'chart-cursor-dot';
        dot.style.top = `${y - top}px`;
        dot.style.background = CurveAppearance(canvas, curve, index).color;
        line.append(dot);
      }
    }
  }

  Redraw() {
    this.DrawAll();
  }
}

export function FormatTime(us) {
  return Us(us).toString();
}
