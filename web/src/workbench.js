/**
 * The experiment workbench: one page for browsing a run, arranging charts and
 * reusing a layout. It owns the HTTP generation, the raw-response cache and the
 * shared time window; the dock and canvas views are pure renderers.
 */

import {
  Api,
  ApiError,
  LatestQuery,
  RequestGate,
  SeriesCache,
  Us,
} from './api.js';
import {
  AddCurve,
  RemoveCurve,
  SetAxisRange,
} from './chart_model.js';
import {
  CloseChart,
  DropSignal,
  MoveChart,
  RemoveSignal,
  ResizeSplit,
} from './dock_model.js';
import { DockView } from './dock_view.js';
import {
  EmptyLayout,
  LayoutError,
  ResolveLayout,
  ValidateLayout,
} from './layout_model.js';

function FormatUs(value) {
  if (value === null || value === undefined) {
    return '—';
  }
  try {
    return Us(value).toString();
  } catch {
    return '—';
  }
}

export class Workbench {
  constructor(options = {}) {
    this.document = options.document ?? globalThis.document;
    this.api = options.api ?? new Api({ base: '' });
    this.gate = new RequestGate();
    this.cache = new SeriesCache(32 * 1024 * 1024);
    this.layout = EmptyLayout();
    this.resolved = null;
    this.catalog = [];
    this.run = null;
    this.appRun = null;
    this.series = [];
    this.timeWindow = { beginUs: 0n, endUs: 1n };
    this.cursor = null;
    this.currentLayoutId = null;
    this.runs = [];
    this.layouts = [];
    this.replacing = null;
    this.dock = null;
  }

  $(selector) {
    return this.document.querySelector(selector);
  }

  async Init() {
    this.dock = new DockView(this.$('#dock-root'), {
      onDropSignal: (signalId, chartId, zone) => this.DropSignal(signalId, chartId, zone),
      onMoveChart: (chartId, target, zone) => this.MoveChart(chartId, target, zone),
      onResize: (path, ratio) => this.Resize(path, ratio),
      onCloseChart: (chartId) => this.CloseChart(chartId),
      onRemoveSignal: (chartId, signalId) => this.RemoveSignal(chartId, signalId),
      onAxisRange: (chartId, axisId, range) => this.SetAxisRange(chartId, axisId, range),
      onReplaceSignal: (chartId, signalId) => this.BeginReplace(chartId, signalId),
      onReassignCurve: (chartId, signalId, axisId) =>
        this.ReassignCurve(chartId, signalId, axisId),
      onStatus: (message) => this.Status(message),
    });
    this.dock.SetView(this.layout, ResolveLayout(this.layout, []), [], this.timeWindow, null);
    this.BindControls();
    await Promise.all([this.RefreshRuns(), this.RefreshLayouts()]);
    const saved = globalThis.localStorage?.getItem('irmv.workbench.layout');
    if (saved) {
      try {
        this.ApplyLayout(JSON.parse(saved));
        this.currentLayoutId = null;
      } catch {
        globalThis.localStorage?.removeItem('irmv.workbench.layout');
      }
    }
  }

  BindControls() {
    this.$('#run-list')?.addEventListener('click', (event) => {
      const button = event.target.closest('button[data-run-id]');
      if (!button) {
        return;
      }
      const appRunId = this.$('#app-run-select')?.value || null;
      this.LoadRun(button.dataset.runId, appRunId || null)
        .catch((error) => this.Status(error.message));
    });
    this.$('#refresh-runs')?.addEventListener('click', () => this.RefreshRuns());
    this.$('#signal-search')?.addEventListener('input', () => this.RenderSignals());
    this.$('#save-layout')?.addEventListener('click', () => this.SaveLayout(false));
    this.$('#save-as-layout')?.addEventListener('click', () => this.SaveLayout(true));
    this.$('#load-layout')?.addEventListener('click', () => {
      const id = this.$('#layout-select')?.value;
      if (id) {
        this.LoadLayout(id).catch((error) => this.Status(error.message));
      }
    });
    this.$('#delete-layout')?.addEventListener('click', () => {
      const id = this.$('#layout-select')?.value;
      if (id) {
        this.DeleteLayout(id).catch((error) => this.Status(error.message));
      }
    });
    this.$('#zoom-in')?.addEventListener('click', () => this.Zoom(4n, 5n));
    this.$('#zoom-out')?.addEventListener('click', () => this.Zoom(5n, 4n));
    this.$('#zoom-fit')?.addEventListener('click', () => this.FitWindow());
    this.$('#layout-name')?.addEventListener('change', () => {
      this.layout = { ...this.layout, name: this.$('#layout-name').value || this.layout.name };
    });
    this.$('#tab-workbench')?.addEventListener('click', () => this.ShowPage('workbench'));
    this.$('#tab-records')?.addEventListener('click', () => this.ShowPage('records'));
    this.$('#tab-maintenance')?.addEventListener('click', () => this.ShowPage('maintenance'));
    this.$('#replace-select')?.addEventListener('change', (event) => {
      const signalId = this.$('#replace-select')?.value;
      if (signalId) {
        this.ReplaceSignal(signalId).catch((error) => this.Status(error.message));
      }
    });
    this.$('#dock-root')?.addEventListener('wheel', (event) => {
      if (!this.run) {
        return;
      }
      event.preventDefault();
      this.Zoom(event.deltaY > 0 ? 6n : 4n, event.deltaY > 0 ? 5n : 5n);
    }, { passive: false });
    this.$('#dock-root')?.addEventListener('pointermove', (event) => {
      const root = this.$('#dock-root');
      if (!root || !this.run) {
        return;
      }
      const bounds = root.getBoundingClientRect();
      const ratio = Math.min(1, Math.max(0, (event.clientX - bounds.left) / bounds.width));
      const begin = this.timeWindow.beginUs;
      const span = this.timeWindow.endUs - begin;
      const stamp = begin + BigInt(Math.round(Number(span) * ratio));
      this.RequestCursor(stamp);
    });
  }

  Status(message) {
    const line = this.$('#status-line');
    if (line) {
      line.textContent = message ?? '';
    }
  }

  ShowPage(page) {
    for (const name of ['workbench', 'records', 'maintenance']) {
      const element = this.$(`#page-${name}`);
      if (element) {
        element.hidden = name !== page;
      }
    }
  }

  async RefreshRuns() {
    try {
      const payload = await this.api.Runs();
      this.runs = payload.runs ?? [];
      this.RenderRuns();
      this.RenderAppCandidates();
    } catch (error) {
      this.Status(error.message);
    }
  }

  RenderRuns() {
    const list = this.$('#run-list');
    if (!list) {
      return;
    }
    list.replaceChildren();
    for (const run of this.runs) {
      const item = this.document.createElement('div');
      item.className = 'run-item';
      item.dataset.kind = run.kind;
      const name = this.document.createElement('span');
      name.className = 'run-name';
      name.textContent = run.name;
      const meta = this.document.createElement('span');
      meta.className = 'run-meta';
      meta.textContent = `${run.kind}${run.complete === false ? ' • open' : ''}${run.error ? ` • ${run.error}` : ''}`;
      const button = this.document.createElement('button');
      button.type = 'button';
      button.textContent = 'Open';
      button.dataset.runId = run.run_id;
      button.disabled = !run.readable;
      item.append(name, meta, button);
      list.append(item);
    }
  }

  RenderAppCandidates() {
    const select = this.$('#app-run-select');
    if (!select) {
      return;
    }
    const previous = select.value;
    select.replaceChildren();
    const none = this.document.createElement('option');
    none.value = '';
    none.textContent = 'No application stream';
    select.append(none);
    for (const run of this.runs.filter((entry) => entry.kind === 'application' && entry.readable)) {
      const option = this.document.createElement('option');
      option.value = run.run_id;
      option.textContent = run.name;
      select.append(option);
    }
    if ([...select.options].some((option) => option.value === previous)) {
      select.value = previous;
    }
  }

  async LoadRun(runId, appRunId = null) {
    this.gate.Invalidate();
    this.cache.Clear();
    this.series = [];
    this.cursor = null;
    this.replacing = null;
    this.Status('loading run…');
    const runtime = this.runs.find((run) => run.run_id === runId);
    if (!runtime) {
      throw new Error('the run is not in the catalog');
    }
    const described = await this.api.Describe(runId);
    let catalog = [...(described.context?.signals ?? [])];
    let appDescription = null;
    if (appRunId) {
      appDescription = await this.api.Describe(appRunId);
      catalog = catalog.concat(appDescription.context?.signals ?? []);
    }
    this.run = { ...runtime, id: runId, context: described.context };
    this.appRun = appRunId ? { id: appRunId, context: appDescription?.context } : null;
    this.catalog = catalog;
    const coverage = described.context?.coverage;
    if (coverage?.begin_us && coverage?.end_us) {
      this.timeWindow = {
        beginUs: Us(coverage.begin_us),
        endUs: Us(coverage.end_us),
      };
    } else {
      this.timeWindow = { beginUs: 0n, endUs: 1000000n };
    }
    this.RenderSignals();
    this.RenderRunHeader();
    this.RenderWindowControls();
    this.ApplyLayout(this.layout, { request: true });
    this.Status(`run ${runtime.name} loaded`);
  }

  RenderRunHeader() {
    const title = this.$('#run-title');
    if (title) {
      title.textContent = this.run ? this.run.name : 'No experiment loaded';
    }
    const detail = this.$('#run-detail');
    if (detail) {
      const context = this.run?.context ?? {};
      detail.textContent = [
        context.kind,
        context.sdk_git_sha ? `SDK ${String(context.sdk_git_sha).slice(0, 12)}` : '',
        context.clock?.boot_id ? `boot ${context.clock.boot_id}` : '',
        context.clock?.clock ?? '',
      ].filter(Boolean).join(' · ');
    }
  }

  RenderSignals() {
    const tree = this.$('#signal-tree');
    if (!tree) {
      return;
    }
    const query = (this.$('#signal-search')?.value ?? '').toLowerCase();
    tree.replaceChildren();
    const groups = new Map();
    for (const signal of this.catalog) {
      if (query && !`${signal.label} ${signal.id} ${signal.unit}`.toLowerCase().includes(query)) {
        continue;
      }
      const groupName = signal.source === 'application'
        ? 'application'
        : (signal.joint_name || 'runtime · diagnostics');
      const key = `${signal.source}:${groupName}`;
      if (!groups.has(key)) {
        groups.set(key, { name: groupName, source: signal.source, signals: [] });
      }
      groups.get(key).signals.push(signal);
    }
    for (const group of groups.values()) {
      const section = this.document.createElement('section');
      section.className = 'signal-group';
      const header = this.document.createElement('h3');
      header.textContent = group.name;
      section.append(header);
      for (const signal of group.signals) {
        const item = this.document.createElement('div');
        item.className = 'signal';
        item.draggable = true;
        item.dataset.signalId = signal.id;
        item.dataset.source = signal.source;
        item.title = signal.id;
        item.addEventListener('dragstart', (event) => {
          event.dataTransfer?.setData('application/x-irmv-signal', signal.id);
          event.dataTransfer.effectAllowed = 'copy';
        });
        const label = this.document.createElement('span');
        label.textContent = signal.label;
        const unit = this.document.createElement('span');
        unit.className = 'signal-unit';
        unit.textContent = signal.unit || signal.value_kind;
        item.append(label, unit);
        section.append(item);
      }
      tree.append(section);
    }
  }

  ApplyLayout(layout, options = {}) {
    try {
      ValidateLayout(layout);
      this.layout = layout;
      this.resolved = ResolveLayout(layout, this.catalog);
      const name = this.$('#layout-name');
      if (name) {
        name.value = layout.name;
      }
      this.dock.SetView(layout, this.resolved, this.series, this.timeWindow, this.cursor);
      const missing = this.$('#missing-signals');
      if (missing) {
        missing.textContent = this.resolved.missingSignals.length
          ? `${this.resolved.missingSignals.length} missing signal placeholder(s)`
          : '';
      }
      if (options.request !== false && this.run) {
        this.RequestWindow();
      }
    } catch (error) {
      if (error instanceof LayoutError) {
        this.Status(error.message);
        return;
      }
      throw error;
    }
  }

  SignalById(signalId) {
    return this.catalog.find((signal) => signal.id === signalId) ?? null;
  }

  DropSignal(signalId, chartId, zone) {
    const signal = this.SignalById(signalId);
    if (!signal) {
      this.Status('the signal is not in this run');
      return;
    }
    try {
      this.ApplyLayout(DropSignal(this.layout, signal, chartId, zone));
    } catch (error) {
      this.Status(error.message);
    }
  }

  MoveChart(chartId, targetChartId, zone) {
    try {
      this.ApplyLayout(MoveChart(this.layout, chartId, targetChartId, zone));
    } catch (error) {
      this.Status(error.message);
    }
  }

  Resize(path, ratio) {
    try {
      this.ApplyLayout(ResizeSplit(this.layout, path, ratio), { request: false });
    } catch (error) {
      this.Status(error.message);
    }
  }

  CloseChart(chartId) {
    try {
      this.ApplyLayout(CloseChart(this.layout, chartId));
    } catch (error) {
      this.Status(error.message);
    }
  }

  RemoveSignal(chartId, signalId) {
    try {
      this.ApplyLayout(RemoveSignal(this.layout, chartId, signalId));
    } catch (error) {
      this.Status(error.message);
    }
  }

  SetAxisRange(chartId, axisId, range) {
    const chart = this.layout.charts.find((entry) => entry.id === chartId);
    if (!chart) {
      return;
    }
    try {
      const updated = SetAxisRange(chart, axisId, range);
      this.ApplyLayout({
        ...this.layout,
        charts: this.layout.charts.map((entry) => (entry.id === chartId ? updated : entry)),
      }, { request: false });
    } catch (error) {
      this.Status(error.message);
    }
  }

  ReassignCurve(chartId, signalId, axisId) {
    const chart = this.layout.charts.find((entry) => entry.id === chartId);
    const signal = this.SignalById(signalId);
    const axis = chart?.axes.find((entry) => entry.id === axisId);
    if (!chart || !signal || !axis) {
      return;
    }
    if (axis.quantity !== signal.quantity || axis.unit !== signal.unit) {
      this.Status('an axis can only group signals of the same quantity and unit');
      return;
    }
    const updated = {
      ...chart,
      curves: chart.curves.map((curve) =>
        curve.signalId === signalId ? { ...curve, axisId } : curve),
    };
    this.ApplyLayout({
      ...this.layout,
      charts: this.layout.charts.map((entry) => (entry.id === chartId ? updated : entry)),
    });
  }

  BeginReplace(chartId, signalId) {
    this.replacing = { chartId, signalId };
    const select = this.$('#replace-select');
    if (!select) {
      return;
    }
    select.replaceChildren();
    const placeholder = this.document.createElement('option');
    placeholder.value = '';
    placeholder.textContent = 'Replace missing signal…';
    select.append(placeholder);
    for (const signal of this.catalog) {
      const option = this.document.createElement('option');
      option.value = signal.id;
      option.textContent = `${signal.label} (${signal.unit || signal.value_kind})`;
      select.append(option);
    }
    select.hidden = false;
    select.focus();
  }

  async ReplaceSignal(signalId) {
    if (!this.replacing) {
      return;
    }
    const signal = this.SignalById(signalId);
    if (!signal) {
      return;
    }
    const { chartId, signalId: missingId } = this.replacing;
    const chart = this.layout.charts.find((entry) => entry.id === chartId);
    if (!chart) {
      return;
    }
    const without = RemoveCurve(chart, missingId);
    const replaced = AddCurve(without, signal);
    this.replacing = null;
    const select = this.$('#replace-select');
    if (select) {
      select.hidden = true;
    }
    this.ApplyLayout({
      ...this.layout,
      charts: this.layout.charts.map((entry) => (entry.id === chartId ? replaced : entry)),
    });
  }

  CurrentSignalIds() {
    return [...new Set(this.layout.charts.flatMap((chart) =>
      chart.curves.map((curve) => curve.signalId)))];
  }

  RequestWindow() {
    if (!this.run || this.timeWindow.endUs <= this.timeWindow.beginUs) {
      return;
    }
    const signalIds = this.CurrentSignalIds();
    if (signalIds.length === 0) {
      this.series = [];
      this.dock.SetView(this.layout, this.resolved, this.series, this.timeWindow, this.cursor);
      return;
    }
    const begin = this.timeWindow.beginUs;
    const end = this.timeWindow.endUs;
    const key = [
      this.run.id,
      this.appRun?.id ?? '',
      begin.toString(),
      end.toString(),
      signalIds.join(','),
    ].join('|');
    const cached = this.cache.Get(key);
    if (cached) {
      this.series = cached.series;
      this.dock.SetView(this.layout, this.resolved, this.series, this.timeWindow, this.cursor);
      this.ShowIntegrity(cached.integrity);
      return;
    }
    const pixelWidth = Math.min(4096, Math.max(200, Math.round(this.$('#dock-root')?.clientWidth ?? 800)));
    LatestQuery(this.gate, {
      key: 'range',
      loader: (signal) => this.api.Range({
        runId: this.run.id,
        appRunId: this.appRun?.id ?? null,
        signalIds,
        beginUs: begin,
        endUs: end,
        pixelWidth,
      }, { signal }),
      onValue: (payload) => {
        this.series = payload.series ?? [];
        this.cache.Set(key, { series: this.series, integrity: payload.integrity });
        this.dock.SetView(this.layout, this.resolved, this.series, this.timeWindow, this.cursor);
        this.ShowIntegrity(payload.integrity);
      },
      onError: (error) => this.Status(error.message),
      onAbort: () => {},
    });
  }

  ShowIntegrity(integrity) {
    const element = this.$('#integrity-line');
    if (!element) {
      return;
    }
    if (!integrity) {
      element.textContent = '';
      return;
    }
    const parts = [];
    for (const [name, stream] of Object.entries(integrity)) {
      if (!stream) {
        continue;
      }
      parts.push(`${name}:${stream.complete ? 'complete' : 'incomplete'}${stream.corrupt ? ',corrupt' : ''}${stream.truncated ? ',truncated' : ''}`);
    }
    element.textContent = parts.join(' · ');
  }

  RequestCursor(stamp) {
    if (!this.run) {
      return;
    }
    const now = globalThis.performance?.now?.() ?? Date.now();
    if (this.lastCursorAt !== undefined && now - this.lastCursorAt < 100) {
      return;
    }
    this.lastCursorAt = now;
    const signalIds = this.CurrentSignalIds();
    if (signalIds.length === 0) {
      return;
    }
    this.dock.cursor = { timeUs: stamp, values: this.cursor?.values ?? new Map() };
    LatestQuery(this.gate, {
      key: 'cursor',
      loader: (signal) => this.api.Cursor({
        runId: this.run.id,
        appRunId: this.appRun?.id ?? null,
        signalIds,
        cursorUs: stamp,
      }, { signal }),
      onValue: (payload) => {
        const values = new Map();
        for (const entry of payload.series ?? []) {
          values.set(entry.signal.id, {
            value: entry.value,
            valid: entry.valid,
            ageUs: entry.age_us,
            sampleStampUs: entry.sample_stamp_us,
          });
        }
        this.cursor = { timeUs: stamp, values, raw: payload };
        this.dock.SetCursor(this.cursor);
        const readout = this.$('#cursor-readout');
        if (readout) {
          const details = [];
          for (const [signalId, value] of values) {
            const signal = this.SignalById(signalId);
            details.push(`${signal?.label ?? signalId}=${value.valid ? value.value : 'unavailable'}${value.ageUs !== null && value.ageUs !== undefined ? ` (${value.ageUs} us ago)` : ''}`);
          }
          readout.textContent = `t=${stamp.toString()} · ${details.slice(0, 4).join(' · ')}`;
        }
      },
      onError: () => {},
      onAbort: () => {},
    });
  }

  Zoom(numerator, denominator) {
    const begin = this.timeWindow.beginUs;
    const end = this.timeWindow.endUs;
    const span = end - begin;
    if (span <= 1n) {
      return;
    }
    const center = this.cursor?.timeUs ?? begin + span / 2n;
    const next = (span * numerator) / denominator;
    if (next <= 0n) {
      return;
    }
    const half = next / 2n;
    let nextBegin = center - half;
    let nextEnd = center + half;
    if (nextBegin < 0n) {
      nextBegin = 0n;
      nextEnd = next;
    }
    this.timeWindow = { beginUs: nextBegin, endUs: nextEnd };
    this.RequestWindow();
    this.RenderWindowControls();
  }

  FitWindow() {
    const coverage = this.run?.context?.coverage;
    if (coverage?.begin_us && coverage?.end_us) {
      this.timeWindow = { beginUs: Us(coverage.begin_us), endUs: Us(coverage.end_us) };
    } else {
      this.timeWindow = { beginUs: 0n, endUs: 1000000n };
    }
    this.RequestWindow();
    this.RenderWindowControls();
  }

  RenderWindowControls() {
    const element = this.$('#window-readout');
    if (element) {
      element.textContent = `${FormatUs(this.timeWindow.beginUs)} – ${FormatUs(this.timeWindow.endUs)}`;
    }
  }

  async RefreshLayouts() {
    try {
      const payload = await this.api.Layouts();
      this.layouts = payload.layouts ?? [];
      const select = this.$('#layout-select');
      if (!select) {
        return;
      }
      select.replaceChildren();
      for (const layout of this.layouts) {
        const option = this.document.createElement('option');
        option.value = layout.id;
        option.textContent = layout.name;
        select.append(option);
      }
      if (this.currentLayoutId &&
          [...select.options].some((option) => option.value === this.currentLayoutId)) {
        select.value = this.currentLayoutId;
      }
    } catch (error) {
      this.Status(error.message);
    }
  }

  async SaveLayout(asNew) {
    const name = this.$('#layout-name')?.value || this.layout.name;
    const documentValue = { ...JSON.parse(JSON.stringify(this.layout)), name };
    try {
      ValidateLayout(documentValue);
      const result = await this.api.SaveLayout(
        documentValue,
        asNew ? null : this.currentLayoutId,
      );
      this.currentLayoutId = result.id;
      this.layout = documentValue;
      globalThis.localStorage?.setItem('irmv.workbench.layout',
        JSON.stringify(documentValue));
      await this.RefreshLayouts();
      this.Status(`layout saved as ${result.id.slice(0, 8)}`);
    } catch (error) {
      this.Status(error.message);
    }
  }

  async LoadLayout(id) {
    const payload = await this.api.Layout(id);
    this.currentLayoutId = id;
    this.ApplyLayout(payload.layout);
    this.Status(`layout ${payload.layout.name} loaded`);
  }

  async DeleteLayout(id) {
    await this.api.DeleteLayout(id);
    if (this.currentLayoutId === id) {
      this.currentLayoutId = null;
    }
    await this.RefreshLayouts();
    this.Status('layout deleted');
  }
}
