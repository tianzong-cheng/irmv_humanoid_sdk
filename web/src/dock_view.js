/**
 * DOM view of the dock tree. It owns drag/drop previews, split dividers, chart
 * headers, legends and the left axis columns; layout changes are delegated to
 * the models, so the view never invents a chart or a split of its own.
 */

import { Render } from './chart_view.js';
import { ValidateLayout } from './layout_model.js';
import { Us } from './api.js';

export class DockView {
  constructor(root, callbacks = {}) {
    this.root = root;
    this.callbacks = callbacks;
    this.layout = null;
    this.resolved = null;
    this.series = [];
    this.timeWindow = { beginUs: 0n, endUs: 1n };
    this.cursor = null;
    this.canvases = new Map();
    this.axisElements = new Map();
  }

  SetView(layout, resolved, series, timeWindow, cursor) {
    this.layout = layout;
    this.resolved = resolved;
    this.series = series;
    this.timeWindow = timeWindow;
    this.cursor = cursor;
    this.Render();
  }

  SetCursor(cursor) {
    this.cursor = cursor;
    this.Redraw();
  }

  Render() {
    if (!this.layout) {
      return;
    }
    this.canvases.clear();
    this.axisElements.clear();
    this.root.replaceChildren(this.RenderNode(this.layout.dock, []));
    this.DrawAll();
  }

  RenderNode(node, path) {
    if (node.type === 'empty') {
      const empty = document.createElement('div');
      empty.className = 'workspace-empty';
      empty.dataset.dropZone = 'empty';
      empty.textContent = 'Drag a signal here to create a chart.';
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
    split.dataset.path = JSON.stringify(path);
    const first = document.createElement('div');
    first.className = 'dock-pane';
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
    shell.dataset.chartId = chartId;
    const header = document.createElement('header');
    header.className = 'chart-header';
    header.draggable = true;
    header.addEventListener('dragstart', (event) => {
      event.dataTransfer?.setData('application/x-irmv-chart', chartId);
      event.dataTransfer.effectAllowed = 'move';
    });
    const title = document.createElement('span');
    title.className = 'chart-title';
    title.textContent = chart?.title || chartId;
    const close = document.createElement('button');
    close.type = 'button';
    close.className = 'chart-close';
    close.title = 'Close chart';
    close.textContent = '×';
    close.addEventListener('click', () => this.callbacks.onCloseChart?.(chartId));
    header.append(title, close);
    const body = document.createElement('div');
    body.className = 'chart-body';
    body.dataset.chartId = chartId;
    this.BindDrop(body, chartId);
    const axes = document.createElement('div');
    axes.className = 'chart-axes';
    const canvas = document.createElement('canvas');
    canvas.className = 'chart-canvas';
    canvas.dataset.chartId = chartId;
    body.append(axes, canvas);
    const legend = document.createElement('div');
    legend.className = 'chart-legend';
    for (const curve of chart?.curves ?? []) {
      const item = document.createElement('span');
      item.className = 'legend-item';
      item.dataset.signalId = curve.signalId;
      item.dataset.chartId = chartId;
      const label = document.createElement('span');
      label.className = 'legend-label';
      item.addEventListener('mouseenter', () => {
        item.classList.add('highlight');
        const column = this.axisElements.get(chartId)?.querySelector(
          `[data-axis-id="${curve.axisId}"]`);
        column?.classList.add('highlight');
      });
      item.addEventListener('mouseleave', () => {
        item.classList.remove('highlight');
        const column = this.axisElements.get(chartId)?.querySelector(
          `[data-axis-id="${curve.axisId}"]`);
        column?.classList.remove('highlight');
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
        label.textContent = curve.signal?.label ?? curve.signalId;
        const remove = document.createElement('button');
        remove.type = 'button';
        remove.className = 'legend-remove';
        remove.title = 'Remove curve';
        remove.textContent = '×';
        remove.addEventListener('click', () =>
          this.callbacks.onRemoveSignal?.(chartId, curve.signalId));
        const axisSelect = document.createElement('select');
        axisSelect.className = 'curve-axis';
        axisSelect.title = 'Axis for this curve (quantity and unit must match)';
        for (const axis of chart?.axes ?? []) {
          const option = document.createElement('option');
          option.value = axis.id;
          option.textContent = `${axis.quantity || 'value'} (${axis.unit || '-'})`;
          axisSelect.append(option);
        }
        axisSelect.value = curve.axisId;
        axisSelect.addEventListener('change', () =>
          this.callbacks.onReassignCurve?.(chartId, curve.signalId,
            axisSelect.value));
        item.append(label, remove, axisSelect);
      }
      legend.append(item);
    }
    shell.append(header, body, legend);
    this.canvases.set(chartId, canvas);
    this.axisElements.set(chartId, axes);
    this.RenderAxes(chartId, chart, axes);
    return shell;
  }

  RenderAxes(chartId, chart, container) {
    container.replaceChildren();
    if (!chart) {
      return;
    }
    const usedAxes = new Set(chart.curves.map((curve) => curve.axisId));
    for (const axis of chart.axes) {
      if (!usedAxes.has(axis.id) && chart.axes.length > 1) {
        continue;
      }
      const column = document.createElement('div');
      column.className = 'axis-column';
      column.dataset.axisId = axis.id;
      const label = document.createElement('div');
      label.className = 'axis-label';
      label.textContent = `${axis.quantity || 'value'} (${axis.unit || '-'})`;
      column.append(label);
      const range = axis.range ?? [null, null];
      for (const [index, edge] of ['min', 'max'].entries()) {
        const input = document.createElement('input');
        input.type = 'number';
        input.className = `axis-range axis-range-${edge}`;
        input.placeholder = edge;
        input.step = 'any';
        input.value = range[index] ?? '';
        input.addEventListener('change', () => {
          const low = Number(column.querySelector('.axis-range-min')?.value);
          const high = Number(column.querySelector('.axis-range-max')?.value);
          if (input.value === '' && !column.querySelector('.axis-range-min')?.value &&
              !column.querySelector('.axis-range-max')?.value) {
            this.callbacks.onAxisRange?.(chartId, axis.id, null);
            return;
          }
          if (!Number.isFinite(low) || !Number.isFinite(high) || !(low < high)) {
            input.value = '';
            this.callbacks.onStatus?.('axis range must be finite and increasing');
            return;
          }
          this.callbacks.onAxisRange?.(chartId, axis.id, [low, high]);
        });
        column.append(input);
      }
      container.append(column);
    }
  }

  BindDrop(element, chartId) {
    const clear = () => {
      element.classList.remove('drop-center', 'drop-left', 'drop-right',
        'drop-top', 'drop-bottom');
    };
    element.addEventListener('dragover', (event) => {
      event.preventDefault();
      event.dataTransfer.dropEffect = 'copy';
      const zone = chartId === null ? 'empty' : this.ZoneAt(event, element);
      clear();
      element.classList.add(`drop-${zone}`);
    });
    element.addEventListener('dragleave', clear);
    element.addEventListener('drop', (event) => {
      event.preventDefault();
      clear();
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
    const x = (event.clientX - bounds.left) / Math.max(1, bounds.width);
    const y = (event.clientY - bounds.top) / Math.max(1, bounds.height);
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

  DrawAll() {
    for (const [chartId, canvas] of this.canvases) {
      const chart = this.resolved?.charts.find((entry) => entry.id === chartId) ??
        this.layout?.charts.find((entry) => entry.id === chartId);
      if (!chart || !canvas.isConnected) {
        continue;
      }
      Render(canvas, chart, this.series, this.timeWindow, this.cursor);
    }
  }

  Redraw() {
    this.DrawAll();
  }
}

export function FormatTime(us) {
  return Us(us).toString();
}
