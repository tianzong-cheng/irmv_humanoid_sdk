/**
 * The experiment workbench: one page for browsing a run, arranging charts and
 * reusing a layout. It owns the HTTP generation, the raw-response cache and the
 * shared time window; the dock and canvas views are pure renderers.
 */

import { Api, LatestQuery, RequestGate, SeriesCache, Us } from './api.js';
import { AddCurve, RemoveCurve, SetAxisRange } from './chart_model.js';
import {
  CloseChart,
  DropSignal,
  MoveChart,
  RemoveSignal,
  ResizeSplit,
} from './dock_model.js';
import { DockView } from './dock_view.js';
import { EventsView } from './events_view.js';
import { SampleTimes, TimeModel, ZoomWindow } from './time_model.js';
import {
  DisplaySeries,
  DisplaySignalValue,
  RelativeTime,
  RelativeTimestampFields,
} from './time_display.js';
import {
  EmptyLayout,
  LayoutError,
  ResolveLayout,
  ValidateLayout,
} from './layout_model.js';

// Runtime recorder directories use system-clock microseconds, whereas samples
// use a monotonic clock. Only directory names can provide a calendar start time.
function RecordingStart(name) {
  const epoch = /^run_(\d{13,17})$/.exec(name);
  const utc = /^run_(\d{4})(\d{2})(\d{2})T(\d{2})(\d{2})(\d{2})Z(?:_|$)/.exec(
    name,
  );
  const date = epoch
    ? new Date(Number(BigInt(epoch[1]) / 1000n))
    : utc
      ? new Date(`${utc[1]}-${utc[2]}-${utc[3]}T${utc[4]}:${utc[5]}:${utc[6]}Z`)
      : null;
  return date && Number.isFinite(date.getTime()) ? date : null;
}

function RecordingStartLabel(run) {
  const date = RecordingStart(run.name);
  return date
    ? `${date.toISOString().slice(0, 19).replace('T', ' ')} UTC`
    : 'Start time unavailable';
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
    this.timeOriginUs = null;
    this.currentLayoutId = null;
    this.runs = [];
    this.matchedApplications = [];
    this.sourceMode = 'history';
    this.loadGeneration = 0;
    this.runLoadController = null;
    this.catalogInFlight = false;
    this.catalogTimer = null;
    this.layouts = [];
    this.replacing = null;
    this.dock = null;
    this.time = TimeModel.Initial();
    this.eventsView = new EventsView({
      onSeek: (stamp) => this.SeekTo(stamp),
      onSelect: (event) => this.ShowEventDetail(event),
      onMore: (continuation) => this.LoadEvents(continuation),
    });
    this.liveTimer = null;
    this.liveInFlight = false;
    this.liveRevision = null;
    this.liveServiceEpoch = null;
    this.liveSeen = false;
    this.playbackFrame = null;
    this.lastRangeAt = 0;
    this.lastCursorAt = undefined;
    this.cursorTimer = null;
    this.pendingCursor = undefined;
  }

  $(selector) {
    return this.document.querySelector(selector);
  }

  async Init() {
    this.dock = new DockView(this.$('#dock-root'), {
      onDropSignal: (signalId, chartId, zone) =>
        this.DropSignal(signalId, chartId, zone),
      onMoveChart: (chartId, target, zone) =>
        this.MoveChart(chartId, target, zone),
      onResize: (path, ratio) => this.Resize(path, ratio),
      onCloseChart: (chartId) => this.CloseChart(chartId),
      onRemoveSignal: (chartId, signalId) =>
        this.RemoveSignal(chartId, signalId),
      onAxisRange: (chartId, axisId, range) =>
        this.SetAxisRange(chartId, axisId, range),
      onReplaceSignal: (chartId, signalId) =>
        this.BeginReplace(chartId, signalId),
      onStatus: (message) => this.Status(message),
      onSelectRegion: (chartId, zoom) => this.SelectChartRegion(chartId, zoom),
      onResetValueAxes: (chartId) => this.ResetValueAxes(chartId),
      onCursor: (stamp) => {
        if (!this.time.playing) this.RequestCursor(stamp);
      },
      onWheelZoom: (delta, stamp) => {
        this.RequestCursor(stamp);
        this.Zoom(delta > 0 ? 6n : 4n, 5n, stamp);
      },
      onSelectionEnd: () => this.ResumePendingCursor(),
      onSelectionStart: () => {
        if (this.time.playing) this.Pause();
      },
    });
    this.dock.SetView(
      this.layout,
      ResolveLayout(this.layout, []),
      [],
      this.timeWindow,
      null,
    );
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
    this.RenderTransport();
    this.OpenExperiments();
    this.catalogTimer = setInterval(() => {
      if (this.document.visibilityState === 'visible')
        this.RefreshRuns({ quiet: true });
    }, 2000);
  }

  BindControls() {
    this.$('#run-list')?.addEventListener('click', (event) => {
      const button = event.target.closest('button[data-run-id]');
      if (!button) {
        return;
      }
      this.LoadRun(button.dataset.runId).catch((error) => {
        if (error.name !== 'AbortError')
          this.BrowserMessage(error.message, true);
      });
    });
    this.$('#refresh-runs')?.addEventListener('click', () =>
      this.RefreshRuns(),
    );
    this.$('#experiment-picker-toggle')?.addEventListener('click', () => {
      if (this.$('#experiment-browser').hidden) this.OpenExperiments();
      else this.CloseExperiments();
    });
    this.$('#live-experiments')?.addEventListener('click', () =>
      this.ViewLive(),
    );
    this.$('#app-run-select')?.addEventListener('change', (event) => {
      if (this.run)
        this.LoadRun(
          this.run.id,
          event.target.value || null,
          this.sourceMode,
        ).catch((error) => {
          if (error.name !== 'AbortError') this.Status(error.message);
          this.RenderAppCandidates();
        });
    });
    this.$('#close-experiments')?.addEventListener('click', () =>
      this.CloseExperiments(),
    );
    this.$('#run-search')?.addEventListener('input', () => this.RenderRuns());
    this.document.addEventListener('keydown', (event) => {
      if (event.key === 'Escape' && !this.$('#experiment-browser').hidden) {
        this.CloseExperiments();
      }
    });
    this.document.addEventListener('pointerdown', (event) => {
      if (
        !this.$('#experiment-browser').hidden &&
        !event.target.closest(
          '#experiment-browser, #experiment-picker-toggle, #live-experiments',
        )
      ) {
        this.CloseExperiments(false);
      }
    });
    this.$('#signal-search')?.addEventListener('input', () =>
      this.RenderSignals(),
    );
    this.$('#save-layout')?.addEventListener('click', () =>
      this.SaveLayout(false),
    );
    this.$('#save-as-layout')?.addEventListener('click', () =>
      this.SaveLayout(true),
    );
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
    this.$('#layout-name')?.addEventListener('change', () => {
      this.layout = {
        ...this.layout,
        name: this.$('#layout-name').value || this.layout.name,
      };
    });
    this.$('#tab-workbench')?.addEventListener('click', () =>
      this.ShowPage('workbench'),
    );
    this.$('#tab-maintenance')?.addEventListener('click', () =>
      this.ShowPage('maintenance'),
    );
    this.$('#replace-select')?.addEventListener('change', (event) => {
      const signalId = this.$('#replace-select')?.value;
      if (signalId) {
        this.ReplaceSignal(signalId).catch((error) =>
          this.Status(error.message),
        );
      }
    });
    this.$('#go-live')?.addEventListener('click', () => this.GoLive());
    this.$('#play')?.addEventListener('click', () => this.Play());
    this.$('#pause')?.addEventListener('click', () => this.Pause());
    this.$('#reset-time')?.addEventListener('click', () =>
      this.ResetTimeAxis(),
    );
    this.$('#step-prev')?.addEventListener('click', () =>
      this.Step('previous'),
    );
    this.$('#step-next')?.addEventListener('click', () => this.Step('next'));
    this.$('#speed-select')?.addEventListener('change', (event) =>
      this.SetSpeed(Number(event.target.value)),
    );
    this.$('#load-events')?.addEventListener('click', () =>
      this.LoadEvents().catch((error) => this.Status(error.message)),
    );
    this.document.addEventListener('visibilitychange', () => {
      if (this.document.visibilityState === 'visible' && this.liveTimer) {
        this.PollLive();
      }
    });
  }

  Status(message) {
    const line = this.$('#status-line');
    if (line) {
      line.textContent = message ?? '';
    }
  }

  ShowPage(page) {
    this.CloseExperiments(false);
    for (const name of ['workbench', 'maintenance']) {
      const element = this.$(`#page-${name}`);
      if (element) {
        element.hidden = name !== page;
      }
    }
  }

  BrowserMessage(message = '', error = false) {
    const element = this.$('#experiment-browser-message');
    element.textContent = message;
    element.classList.toggle('warning', error);
  }

  OpenExperiments() {
    this.$('#experiment-browser').hidden = false;
    this.$('#experiment-picker-toggle').setAttribute('aria-expanded', 'true');
    this.$('#run-search').value = '';
    this.BrowserMessage();
    this.RenderRuns();
    this.$('#run-search').focus();
    this.RefreshRuns({ quiet: true });
  }

  CloseExperiments(focus = true) {
    this.$('#experiment-browser').hidden = true;
    this.$('#experiment-picker-toggle').setAttribute('aria-expanded', 'false');
    if (focus) this.$('#experiment-picker-toggle').focus();
  }

  LiveRuntimes() {
    return this.runs.filter(
      (run) => run.kind === 'runtime' && run.live?.active,
    );
  }

  RenderLiveAvailability() {
    const live = this.LiveRuntimes();
    const ready = live.length === 1 && live[0].readable;
    const status = this.$('#telemetry-availability');
    status.textContent =
      live.length > 1
        ? 'Conflicting runtime streams'
        : ready
          ? 'Live telemetry available'
          : 'No live telemetry';
    status.dataset.state = ready
      ? 'available'
      : live.length > 1
        ? 'conflict'
        : 'unavailable';
    const button = this.$('#live-experiments');
    button.disabled = !ready;
    button.title =
      live.length > 1
        ? 'More than one runtime is emitting telemetry. Check the SDK processes.'
        : ready
          ? 'View the current robot experiment'
          : 'No runtime telemetry received. Check the SDK telemetry connection.';
  }

  async ViewLive() {
    const live = this.LiveRuntimes();
    if (live.length !== 1 || !live[0].readable) {
      this.RenderLiveAvailability();
      return;
    }
    this.CloseExperiments(false);
    try {
      await this.LoadRun(live[0].run_id, undefined, 'live');
      this.$('#live-experiments').focus();
    } catch (error) {
      if (error.name !== 'AbortError') this.Status(error.message);
    }
  }

  async RefreshRuns({ quiet = false } = {}) {
    if (this.catalogInFlight) return;
    this.catalogInFlight = true;
    try {
      const runs = [];
      const seen = new Set();
      let continuation;
      do {
        const payload = await this.api.Runs({ continuation });
        runs.push(...(payload.runs ?? []));
        continuation = payload.continuation;
        if (continuation && seen.has(continuation))
          throw new Error(
            'The experiment list could not be loaded. Refresh to try again.',
          );
        seen.add(continuation);
      } while (continuation);
      this.runs = runs;
      if (this.run)
        this.run.live = runs.find((run) => run.run_id === this.run.id)?.live;
      this.RenderLiveAvailability();
      this.RenderRuns();
      this.RenderExperimentState();
      if (!quiet) this.BrowserMessage();
    } catch (error) {
      this.$('#telemetry-availability').textContent =
        'Telemetry status unavailable';
      this.$('#telemetry-availability').dataset.state = 'unavailable';
      this.$('#live-experiments').disabled = true;
      if (!quiet || !this.$('#experiment-browser').hidden)
        this.BrowserMessage(error.message, true);
    } finally {
      this.catalogInFlight = false;
    }
  }

  RenderRuns() {
    const list = this.$('#run-list');
    if (!list) return;
    const query = (this.$('#run-search')?.value ?? '').trim().toLowerCase();
    const runs = this.runs
      .filter(
        (run) =>
          run.kind !== 'application' &&
          `${run.name} ${RecordingStartLabel(run)}`
            .toLowerCase()
            .includes(query),
      )
      .sort(
        (left, right) =>
          (RecordingStart(right.name)?.getTime() ?? -Infinity) -
            (RecordingStart(left.name)?.getTime() ?? -Infinity) ||
          right.name.localeCompare(left.name),
      );
    this.$('#recording-count').textContent =
      `${runs.length} recording${runs.length === 1 ? '' : 's'}`;
    const signature = JSON.stringify([
      query,
      this.run?.id,
      this.sourceMode,
      runs.map((run) => [
        run.run_id,
        run.name,
        run.kind,
        run.readable,
        run.complete,
        run.live?.active,
        run.error,
        run.begin_us,
        run.end_us,
      ]),
    ]);
    if (signature === this.runListSignature) return;
    this.runListSignature = signature;
    const focusedId = list.contains(this.document.activeElement)
      ? this.document.activeElement.dataset.runId
      : null;
    const scrollTop = list.scrollTop;
    list.replaceChildren();
    if (runs.length === 0) {
      const empty = this.document.createElement('p');
      empty.className = 'experiment-empty';
      empty.textContent = query
        ? 'No matching experiments. Try another name.'
        : 'No recordings found. Enable recording in the SDK configuration, then refresh.';
      list.append(empty);
    }
    for (const run of runs) {
      const item = this.document.createElement('div');
      item.className = 'run-item';
      item.dataset.kind = run.kind;
      item.classList.toggle('selected', this.run?.id === run.run_id);
      const name = this.document.createElement('span');
      name.className = 'run-name';
      name.textContent = run.name;
      name.title = run.name;
      const meta = this.document.createElement('span');
      meta.className = 'run-meta';
      const status = run.live?.active
        ? 'Recording'
        : run.complete === true
          ? 'Complete'
          : run.complete === false
            ? 'Unfinished'
            : 'Unavailable';
      const started = RecordingStartLabel(run);
      const durationUs =
        run.begin_us != null && run.end_us != null
          ? Us(run.end_us) - Us(run.begin_us)
          : null;
      const duration =
        durationUs == null
          ? 'Duration unavailable'
          : `${(Number(durationUs) / 1000000).toFixed(1)} s`;
      meta.textContent = `${started} · ${duration}`;
      const badge = this.document.createElement('span');
      badge.className = 'recording-status';
      badge.dataset.state = run.readable ? status.toLowerCase() : 'unavailable';
      badge.textContent = run.error ? 'Unreadable' : status;
      if (run.error) meta.textContent = run.error;
      const button = this.document.createElement('button');
      button.type = 'button';
      button.textContent =
        this.run?.id === run.run_id && this.sourceMode === 'history'
          ? 'Viewing'
          : 'Replay';
      button.dataset.runId = run.run_id;
      button.disabled = !run.readable;
      button.setAttribute('aria-label', `${button.textContent} ${run.name}`);
      item.append(name, meta, badge, button);
      list.append(item);
    }
    list.scrollTop = scrollTop;
    if (focusedId) {
      const focused = [...list.querySelectorAll('button[data-run-id]')].find(
        (button) => button.dataset.runId === focusedId,
      );
      (focused ?? this.$('#run-search')).focus({ preventScroll: true });
    }
  }

  RenderAppCandidates() {
    const select = this.$('#app-run-select');
    if (!select) {
      return;
    }
    select.replaceChildren();
    const none = this.document.createElement('option');
    none.value = '';
    none.textContent = 'Runtime only';
    select.append(none);
    for (const run of this.matchedApplications) {
      const option = this.document.createElement('option');
      option.value = run.run_id;
      option.textContent = run.name;
      select.append(option);
    }
    select.value = this.appRun?.id ?? '';
    select.disabled = this.matchedApplications.length === 0;
    this.$('#experiment-sources').hidden = !this.run;
    this.$('#runtime-source-name').textContent = this.run
      ? `Runtime: ${this.run.name}`
      : '';
    this.$('#source-summary').textContent = this.appRun
      ? 'Runtime + Application'
      : 'Runtime only';
  }

  async LoadRun(runId, appRunId = undefined, mode = 'history') {
    const generation = ++this.loadGeneration;
    this.runLoadController?.abort();
    this.runLoadController = new AbortController();
    const signal = this.runLoadController.signal;
    const runtime = this.runs.find((run) => run.run_id === runId);
    if (!runtime)
      throw new Error(
        'The experiment is no longer in the list. Refresh to try again.',
      );
    if (runtime.kind === 'application') appRunId = null;
    this.BrowserMessage('Opening experiment…');
    let described = appRunId
      ? await this.api.Context(runId, appRunId, signal)
      : await this.api.Describe(runId, signal);
    if (generation !== this.loadGeneration) return;
    if (appRunId && described.context?.association?.ok !== true) {
      throw new Error(
        'This application stream does not match the runtime session. Choose another stream or open without it.',
      );
    }
    let matched = this.run?.id === runId ? this.matchedApplications : [];
    let sourceMessage = '';
    if (appRunId === undefined && runtime.kind === 'runtime') {
      matched = [];
      let unavailable = false;
      for (const candidate of this.runs.filter(
        (run) => run.kind === 'application' && run.readable,
      )) {
        try {
          const reply = await this.api.Context(runId, candidate.run_id, signal);
          if (generation !== this.loadGeneration) return;
          if (reply.context?.association?.ok === true) {
            matched.push({ ...candidate, description: reply });
          }
        } catch (error) {
          if (error.name === 'AbortError') throw error;
          unavailable = true;
        }
      }
      if (generation !== this.loadGeneration) return;
      if (matched.length === 1) {
        appRunId = matched[0].run_id;
        described = matched[0].description;
      }
      sourceMessage = unavailable
        ? 'Some application streams could not be checked. Refresh the experiment to retry.'
        : matched.length > 1
          ? 'Multiple matching application streams. Select the stream to include.'
          : matched.length === 0
            ? 'No matching application stream. Runtime data is available.'
            : 'Application identity verified for this experiment.';
    }
    const appDescription = appRunId
      ? { context: described.context.application }
      : null;
    const catalog = [
      ...(described.context?.signals ?? []),
      ...(appDescription?.context?.signals ?? []),
    ];
    const changingExperiment = this.run?.id !== runId;
    this.dock.selection?.cancel();
    if (this.playbackFrame !== null) cancelAnimationFrame(this.playbackFrame);
    this.playbackFrame = null;
    clearInterval(this.liveTimer);
    this.liveTimer = null;
    clearTimeout(this.cursorTimer);
    this.cursorTimer = null;
    this.pendingCursor = undefined;
    this.lastCursorAt = undefined;
    this.gate.Invalidate();
    this.liveInFlight = false;
    this.cursorInFlight = false;
    this.cache.Clear();
    this.series = [];
    this.cursor = null;
    this.replacing = null;
    this.sourceMode = mode;
    this.run = { ...runtime, id: runId, context: described.context };
    this.appRun = appRunId
      ? { id: appRunId, context: appDescription?.context }
      : null;
    this.matchedApplications = matched;
    this.RenderAppCandidates();
    this.$('#source-message').textContent = sourceMessage;
    this.RenderRuns();
    this.catalog = catalog;
    const coverage = described.context?.coverage;
    const coverageBegin = coverage?.begin_us ? Us(coverage.begin_us) : 0n;
    const coverageEnd = coverage?.end_us ? Us(coverage.end_us) : 1000000n;
    this.timeOriginUs = coverage?.begin_us == null ? null : coverageBegin;
    this.dock.timeOriginUs = this.timeOriginUs;
    this.time = TimeModel.Step(
      TimeModel.Initial(),
      {
        type: 'reset',
        runId,
        coverage: { beginUs: coverageBegin, endUs: coverageEnd },
        mode: 'history',
      },
      this.Now(),
    );
    this.timeWindow = this.time.window;
    this.liveRevision = null;
    this.liveSeen = false;
    this.eventsView.Reset(this.timeOriginUs);
    this.eventsView.Render(this.$('#events-list'));
    this.dock.SetMarkers([]);
    this.$('#cursor-readout').textContent = '';
    this.$('#events-summary').textContent = '';
    this.$('#integrity-detail').hidden = true;
    this.ShowIntegrity(null);
    this.ShowEventDetail(null);
    this.RenderSignals();
    this.RenderRunHeader();
    if (changingExperiment) this.currentLayoutId = null;
    this.ApplyLayout(changingExperiment ? EmptyLayout() : this.layout, {
      request: true,
    });
    this.RenderTransport();
    if (described.context?.kind === 'runtime') {
      this.LoadEvents().catch((error) => this.Status(error.message));
    } else {
      this.eventsView.Reset(this.timeOriginUs);
      this.eventsView.Render(this.$('#events-list'));
      this.dock?.SetMarkers([]);
      this.ShowEventDetail(null);
    }
    this.Status('');
    this.BrowserMessage();
    if (!this.$('#experiment-browser').hidden) this.CloseExperiments();
    if (mode === 'live') {
      this.GoLive();
    }
  }

  RenderRunHeader() {
    const title = this.$('#run-title');
    if (title) {
      title.textContent = this.run ? this.run.name : 'Choose an experiment';
    }
    this.RenderExperimentState();
    this.RenderFaultNotice();
    const detail = this.$('#run-detail');
    if (detail) {
      const context = this.run?.context ?? {};
      detail.textContent = [
        context.kind,
        context.sdk_git_sha
          ? `SDK ${String(context.sdk_git_sha).slice(0, 12)}`
          : '',
        context.clock?.clock_domain === 'simulation'
          ? `simulation run ${context.clock.clock_run_id} · origin ${context.clock.logical_origin_us} us · physics/control ${context.clock.physics_dt_us}/${context.clock.control_dt_us} us`
          : context.clock?.boot_id
            ? `boot ${context.clock.boot_id}`
            : '',
        context.clock?.clock ?? '',
        this.appRun ? 'Application stream included' : '',
      ]
        .filter(Boolean)
        .join(' · ');
    }
  }

  RenderFaultNotice() {
    const notice = this.$('#fault-notice');
    if (!notice) return;
    const text = this.eventsView.FaultNotice(this.run?.context?.joints);
    if (notice.textContent !== text) notice.textContent = text;
    notice.hidden = !text;
  }

  RenderExperimentState() {
    const state = this.$('#experiment-state');
    if (!state) return;
    const live = this.sourceMode === 'live';
    const offline = live && (this.time.offline || !this.run?.live?.active);
    state.textContent = !this.run
      ? 'No experiment selected'
      : !live
        ? 'History'
        : offline
          ? 'Live / Offline'
          : this.time.following
            ? 'Live / Following'
            : 'Live / Reviewing';
    state.dataset.state = !live ? 'history' : offline ? 'offline' : 'live';
    const follow = this.$('#go-live');
    const resetTime = this.$('#reset-time');
    if (resetTime) resetTime.disabled = !this.run;
    if (follow) {
      follow.disabled = !this.run || (live && this.time.following && !offline);
      follow.hidden = this.run && !live && !this.run.live?.active;
    }
    for (const id of [
      'play',
      'pause',
      'step-prev',
      'step-next',
      'speed-select',
    ]) {
      const control = this.$(`#${id}`);
      if (control)
        control.disabled =
          !this.run || (id !== 'pause' && live && this.time.following);
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
      if (
        query &&
        !`${signal.label} ${signal.id} ${signal.unit}`
          .toLowerCase()
          .includes(query)
      ) {
        continue;
      }
      const groupName =
        signal.source === 'application'
          ? 'application'
          : signal.joint_name || 'runtime · diagnostics';
      const key = `${signal.source}:${groupName}`;
      if (!groups.has(key)) {
        groups.set(key, {
          name: groupName,
          source: signal.source,
          signals: [],
        });
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
      this.dock.SetView(
        layout,
        this.resolved,
        this.series,
        this.timeWindow,
        this.cursor,
      );
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
      this.ApplyLayout(ResizeSplit(this.layout, path, ratio), {
        request: false,
      });
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
      this.ApplyLayout(
        {
          ...this.layout,
          charts: this.layout.charts.map((entry) =>
            entry.id === chartId ? updated : entry,
          ),
        },
        { request: false },
      );
    } catch (error) {
      this.Status(error.message);
    }
  }

  SelectChartRegion(chartId, zoom) {
    if (!this.run) {
      return;
    }
    let chart = this.layout.charts.find((entry) => entry.id === chartId);
    if (!chart) {
      return;
    }
    if (zoom.window) {
      this.PreserveValueRanges();
      this.SetTimeWindow(zoom.window);
      return;
    }
    for (const axis of zoom.axes) {
      chart = SetAxisRange(chart, axis.axisId, axis.range);
    }
    this.ApplyLayout(
      {
        ...this.layout,
        charts: this.layout.charts.map((entry) =>
          entry.id === chartId ? chart : entry,
        ),
      },
      { request: false },
    );
  }

  PreserveValueRanges() {
    // Time-only zoom must not rescale automatic axes when new data arrives.
    // Retain their displayed extents until an explicit reset or value zoom.
    this.ApplyLayout(
      {
        ...this.layout,
        charts: this.layout.charts.map((chart) => ({
          ...chart,
          axes: chart.axes.map((axis) => {
            const displayed = this.dock.plotViews
              .get(chart.id)
              ?.ranges.find((range) => range.axisId === axis.id);
            return !axis.range && displayed
              ? { ...axis, range: [displayed.low, displayed.high] }
              : axis;
          }),
        })),
      },
      { request: false },
    );
  }

  SetTimeWindow(window) {
    this.ApplyTimeState(
      TimeModel.Step(this.time, { type: 'setWindow', window }, this.Now()),
    );
  }

  ResetValueAxes(chartId) {
    this.ApplyLayout(
      {
        ...this.layout,
        charts: this.layout.charts.map((chart) =>
          chart.id === chartId
            ? {
                ...chart,
                axes: chart.axes.map((axis) => ({ ...axis, range: null })),
              }
            : chart,
        ),
      },
      { request: false },
    );
  }

  ResetTimeAxis() {
    if (!this.run) return;
    this.PreserveValueRanges();
    this.SetTimeWindow({ ...this.time.coverage });
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
      charts: this.layout.charts.map((entry) =>
        entry.id === chartId ? replaced : entry,
      ),
    });
  }

  CurrentSignalIds() {
    return [
      ...new Set(
        this.layout.charts.flatMap((chart) =>
          chart.curves.map((curve) => curve.signalId),
        ),
      ),
    ];
  }

  RequestWindow() {
    if (!this.run || this.timeWindow.endUs <= this.timeWindow.beginUs) {
      return;
    }
    const signalIds = this.CurrentSignalIds();
    if (signalIds.length === 0) {
      this.series = [];
      this.dock.SetView(
        this.layout,
        this.resolved,
        this.series,
        this.timeWindow,
        this.cursor,
      );
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
      this.dock.SetView(
        this.layout,
        this.resolved,
        this.series,
        this.timeWindow,
        this.cursor,
      );
      this.ShowIntegrity(cached.integrity);
      return;
    }
    if (this.rangeInFlight) {
      this.rangePending = true;
      return;
    }
    this.rangeInFlight = true;
    const pixelWidth = Math.min(
      4096,
      Math.max(200, Math.round(this.$('#dock-root')?.clientWidth ?? 800)),
    );
    // Keep neighboring raw samples on both sides of the visible window. The
    // renderer clips connecting segments at the plot edge, retaining real gaps.
    const contexts = [this.run.context, this.appRun?.context].filter(Boolean);
    const period = BigInt(
      Math.max(
        1,
        ...contexts.map((context) => {
          const hz = Number(context.sample_hz);
          return Math.ceil(
            1000000 / (Number.isFinite(hz) && hz > 0 ? hz : 100),
          );
        }),
      ),
    );
    const margin =
      2n * period +
      (end - begin + BigInt(pixelWidth) - 1n) / BigInt(pixelWidth);
    const starts = contexts
      .map((context) => context.coverage?.begin_us)
      .filter((value) => value != null)
      .map(Us);
    const first = starts.length ? starts.reduce((a, b) => (a < b ? a : b)) : 0n;
    let queryBegin = begin > margin ? begin - margin : 0n;
    if (begin >= first && queryBegin < first) queryBegin = first;
    const maximum = (1n << 64n) - 1n;
    const queryEnd = end + margin + 1n > maximum ? maximum : end + margin + 1n;
    LatestQuery(this.gate, {
      key: 'range',
      loader: (signal) =>
        this.api.Range(
          {
            runId: this.run.id,
            appRunId: this.appRun?.id ?? null,
            signalIds,
            beginUs: queryBegin,
            endUs: queryEnd,
            pixelWidth,
          },
          { signal },
        ),
      onValue: (payload) => {
        this.rangeInFlight = false;
        this.series = DisplaySeries(payload.series ?? [], this.timeOriginUs);
        this.cache.Set(key, {
          series: this.series,
          integrity: payload.integrity,
        });
        this.dock.SetView(
          this.layout,
          this.resolved,
          this.series,
          this.timeWindow,
          this.cursor,
        );
        this.ShowIntegrity(payload.integrity);
        this.ResumePendingRange();
      },
      onError: (error) => {
        this.rangeInFlight = false;
        this.Status(error.message);
        this.ResumePendingRange();
      },
      onAbort: () => {
        this.rangeInFlight = false;
        this.ResumePendingRange();
      },
      onStale: () => {
        this.rangeInFlight = false;
        this.ResumePendingRange();
      },
    });
  }

  ResumePendingRange() {
    if (this.rangePending) {
      this.rangePending = false;
      this.RequestWindow();
    }
  }

  ShowIntegrity(integrity) {
    this.lastIntegrity = integrity;
    this.ShowIntegrityDetail();
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
      parts.push(
        `${name}:${stream.complete ? 'complete' : 'incomplete'}${stream.corrupt ? ',corrupt' : ''}${stream.truncated ? ',truncated' : ''}`,
      );
    }
    element.textContent = parts.join(' · ');
  }

  RequestCursor(stamp) {
    if (!this.run || this.dock.selection) {
      return;
    }
    // Pointer feedback is local and frame-paced. A slow value lookup must not
    // delay it, and an older lookup must never move the pointer backwards.
    this.cursor = {
      timeUs: stamp,
      values: this.cursor?.values ?? new Map(),
      valuesTimeUs: this.cursor?.valuesTimeUs,
    };
    this.dock.SetCursor(this.cursor);
    this.pendingCursor = stamp;
    this.ResumePendingCursor();
  }

  ResumePendingCursor() {
    if (
      !this.run ||
      this.cursorInFlight ||
      this.pendingCursor === undefined ||
      this.dock.selection
    ) {
      return;
    }
    const delay =
      this.lastCursorAt === undefined
        ? 0
        : Math.max(0, 100 - (this.Now() - this.lastCursorAt));
    if (delay > 0) {
      if (this.cursorTimer === null) {
        this.cursorTimer = setTimeout(() => {
          this.cursorTimer = null;
          this.ResumePendingCursor();
        }, delay);
      }
      return;
    }
    clearTimeout(this.cursorTimer);
    this.cursorTimer = null;
    const stamp = this.pendingCursor;
    this.pendingCursor = undefined;
    const signalIds = this.CurrentSignalIds();
    if (signalIds.length === 0) {
      return;
    }
    this.lastCursorAt = this.Now();
    this.cursorInFlight = true;
    const complete = () => {
      this.cursorInFlight = false;
      this.ResumePendingCursor();
    };
    LatestQuery(this.gate, {
      key: 'cursor',
      loader: (signal) =>
        this.api.Cursor(
          {
            runId: this.run.id,
            appRunId: this.appRun?.id ?? null,
            signalIds,
            cursorUs: stamp,
          },
          { signal },
        ),
      onValue: (payload) => {
        if (this.dock.selection) {
          complete();
          return;
        }
        const values = new Map();
        for (const entry of payload.series ?? []) {
          values.set(entry.signal.id, {
            value: DisplaySignalValue(
              entry.signal,
              entry.value,
              entry.sample_stamp_us,
              this.timeOriginUs,
            ),
            valid: entry.valid,
            ageUs: entry.age_us,
            sampleStampUs: entry.sample_stamp_us,
          });
        }
        this.cursor = {
          timeUs: this.cursor?.timeUs ?? stamp,
          values,
          valuesTimeUs: stamp,
          raw: payload,
        };
        this.dock.SetCursor(this.cursor);
        const readout = this.$('#cursor-readout');
        if (readout) {
          const details = [];
          for (const [signalId, value] of values) {
            const signal = this.SignalById(signalId);
            const shown =
              signal?.quantity === 'time' &&
              typeof value.value === 'number' &&
              Number.isFinite(value.value)
                ? RelativeTime(BigInt(Math.round(value.value)), 0n)
                : value.value;
            details.push(
              `${signal?.label ?? signalId}=${value.valid ? shown : 'unavailable'}${value.ageUs !== null && value.ageUs !== undefined ? ` (${value.ageUs} us ago)` : ''}`,
            );
          }
          readout.textContent = `t=${RelativeTime(stamp, this.timeOriginUs)} · ${details.slice(0, 4).join(' · ')}`;
        }
        complete();
      },
      onError: complete,
      onAbort: complete,
      onStale: complete,
    });
  }

  Now() {
    return globalThis.performance?.now?.() ?? Date.now();
  }

  RenderTransport() {
    this.RenderExperimentState();
    const mode = this.$('#transport-mode');
    if (mode) {
      const following = this.time.following ? ' · following' : '';
      const playing = this.time.playing ? ' · playing' : '';
      mode.textContent = `${this.time.mode}${following}${playing} · ${this.time.speed}×`;
    }
    const live = this.$('#live-status');
    if (live) {
      live.hidden = this.sourceMode !== 'live';
      if (this.time.offline) {
        live.textContent = 'Stream offline';
        live.classList.add('warning');
      } else {
        live.classList.remove('warning');
        live.textContent = this.liveSeen
          ? 'Receiving data'
          : 'Waiting for data';
      }
    }
    if (this.$('#speed-select')) {
      this.$('#speed-select').value = String(this.time.speed);
    }
    this.RenderWindowControls();
  }

  ApplyTimeState(state, options = {}) {
    const previous = this.timeWindow;
    const previousPlaybackCursor = this.time.cursor;
    this.time = state;
    this.timeWindow = state.window;
    if (this.dock) {
      this.dock.timeWindow = state.window;
      if (
        state.cursor !== previousPlaybackCursor ||
        (state.playing && !this.cursor)
      ) {
        this.cursor = {
          ...(this.cursor ?? { values: new Map() }),
          timeUs: state.cursor,
        };
      }
      this.dock.SetCursor(this.cursor);
    }
    this.RenderTransport();
    const changed =
      previous.beginUs !== state.window.beginUs ||
      previous.endUs !== state.window.endUs;
    if (changed && options.request !== false && this.run) {
      this.RequestWindow();
    } else if (changed && this.dock) {
      this.dock.SetView(
        this.layout,
        this.resolved,
        this.series,
        state.window,
        this.cursor,
      );
    }
  }

  Play() {
    if (!this.run) return;
    let state = this.time;
    if (state.mode !== 'history') {
      state = TimeModel.Step(
        state,
        { type: 'setMode', mode: 'history' },
        this.Now(),
      );
    }
    const hover = this.cursor?.timeUs;
    if (
      hover !== undefined &&
      hover >= state.window.beginUs &&
      hover <= state.window.endUs
    ) {
      state = { ...state, cursor: hover };
    }
    this.ApplyTimeState(TimeModel.Step(state, { type: 'play' }, this.Now()), {
      request: false,
    });
    if (this.playbackFrame !== null) cancelAnimationFrame(this.playbackFrame);
    const advance = () => {
      this.playbackFrame = null;
      if (!this.time.playing) return;
      const next = TimeModel.Step(this.time, { type: 'advance' }, this.Now());
      this.ApplyTimeState(next);
      this.RequestCursor(next.cursor);
      if (next.playing) this.playbackFrame = requestAnimationFrame(advance);
    };
    this.playbackFrame = requestAnimationFrame(advance);
  }

  Pause() {
    if (this.playbackFrame !== null) cancelAnimationFrame(this.playbackFrame);
    this.playbackFrame = null;
    const paused = TimeModel.Step(this.time, { type: 'pause' }, this.Now());
    this.ApplyTimeState(
      TimeModel.Step(paused, { type: 'setMode', mode: 'history' }, this.Now()),
    );
  }

  Step(direction) {
    if (this.playbackFrame !== null) cancelAnimationFrame(this.playbackFrame);
    this.playbackFrame = null;
    const runtimeSeries = this.series.filter(
      (entry) => entry.signal?.source === 'runtime',
    );
    const times = SampleTimes(
      runtimeSeries.length > 0 ? runtimeSeries : this.series,
    );
    this.ApplyTimeState(
      TimeModel.Step(
        this.time,
        { type: 'stepSample', direction, times },
        this.Now(),
      ),
    );
    this.RequestCursor(this.time.cursor);
  }

  SetSpeed(speed) {
    this.ApplyTimeState(
      TimeModel.Step(this.time, { type: 'setSpeed', speed }, this.Now()),
      { request: false },
    );
  }

  GoLive() {
    if (!this.run) {
      return;
    }
    this.sourceMode = 'live';
    this.ApplyTimeState(
      TimeModel.Step(this.time, { type: 'followLatest' }, this.Now()),
    );
    if (!this.liveTimer) {
      this.liveTimer = setInterval(() => this.PollLive(), 100);
    }
    this.liveSeen = false;
    this.liveRevision = null;
    this.PollLive();
  }

  PollLive() {
    if (!this.run || this.liveInFlight) {
      return;
    }
    let signalIds = this.CurrentSignalIds();
    if (signalIds.length === 0) {
      signalIds = this.catalog.slice(0, 2).map((signal) => signal.id);
    }
    if (signalIds.length === 0) {
      return;
    }
    this.liveInFlight = true;
    const query = {
      runId: this.run.id,
      signalIds,
      beginUs: this.time.coverage.beginUs,
      endUs: this.time.coverage.endUs,
      pixelWidth: 800,
    };
    if (this.liveSeen && this.liveRevision !== null) {
      query.afterRevision = this.liveRevision;
    }
    LatestQuery(this.gate, {
      key: 'live',
      loader: (signal) => this.api.LiveSnapshot(query, { signal }),
      onValue: (snapshot) => {
        this.liveInFlight = false;
        this.run.live = { ...this.run.live, active: snapshot.active === true };
        this.time = { ...this.time, offline: snapshot.active !== true };
        if (snapshot.changed === false) {
          if (snapshot.active)
            this.time = { ...this.time, lastReceivedMs: this.Now() };
          this.RenderTransport();
          return;
        }
        this.liveSeen = true;
        this.liveRevision = snapshot.revision;
        this.liveServiceEpoch = snapshot.service_epoch;
        if (snapshot.coverage?.begin_us && snapshot.coverage?.end_us) {
          if (this.timeOriginUs === null) {
            this.timeOriginUs = Us(snapshot.coverage.begin_us);
            this.dock.timeOriginUs = this.timeOriginUs;
            this.eventsView.timeOriginUs = this.timeOriginUs;
          }
          const state = TimeModel.Step(
            this.time,
            {
              type: 'liveAdvance',
              coverage: {
                beginUs: Us(snapshot.coverage.begin_us),
                endUs: Us(snapshot.coverage.end_us),
              },
              receivedAtMs: this.Now(),
            },
            this.Now(),
          );
          this.ApplyTimeState({ ...state, offline: snapshot.active !== true });
        }
        if (snapshot.faults?.length) {
          this.eventsView.Append({
            events: snapshot.faults.map((fault) => ({
              ...fault,
              event_id: String(fault.record_seq),
            })),
          });
          this.RenderFaultNotice();
        }
        this.eventsView.Render(this.$('#events-list'));
        this.dock?.SetMarkers(this.eventsView.markers);
        this.RenderTransport();
      },
      onError: (error) => {
        this.liveInFlight = false;
        this.run.live = { ...this.run.live, active: false };
        this.time = TimeModel.Step(
          this.time,
          { type: 'markOffline' },
          this.Now(),
        );
        this.RenderTransport();
      },
      onAbort: () => {
        this.liveInFlight = false;
      },
      onStale: () => {
        this.liveInFlight = false;
      },
    });
  }

  async LoadEvents(continuation = null) {
    if (!this.run) {
      return;
    }
    const generation = this.loadGeneration;
    const payload = await this.api.Events({
      runId: this.run.id,
      beginUs: 0n,
      endUs: (1n << 64n) - 1n,
      maxRows: 100,
      continuation,
    });
    if (generation !== this.loadGeneration) return;
    this.eventsView.Append(payload);
    this.eventsView.Render(this.$('#events-list'));
    this.RenderFaultNotice();
    const summary = this.$('#events-summary');
    if (summary) {
      summary.textContent = this.eventsView.Summary();
    }
    this.ShowEventDetail(this.eventsView.selected);
    this.ShowIntegrityDetail();
  }

  ShowEventDetail(event) {
    const detail = this.$('#event-detail');
    if (detail) {
      detail.textContent = event ? this.eventsView.DetailText() : '';
    }
  }

  SeekTo(stamp) {
    this.ApplyTimeState(
      TimeModel.Step(this.time, { type: 'seek', timeUs: stamp }, this.Now()),
    );
    this.RequestCursor(stamp);
  }

  ShowIntegrityDetail() {
    const detail = this.$('#integrity-detail');
    if (!detail) {
      return;
    }
    const context = this.run?.context ?? {};
    detail.textContent = JSON.stringify(
      RelativeTimestampFields(
        {
          time_reference: 'experiment start; timestamp fields in microseconds',
          sdk_git_sha: context.sdk_git_sha ?? null,
          model: context.model ?? this.appRun?.context?.model ?? null,
          clock: context.clock ?? null,
          integrity: this.lastIntegrity ?? context.integrity ?? null,
        },
        this.timeOriginUs,
      ),
      null,
      2,
    );
  }

  Zoom(numerator, denominator, anchorUs = null) {
    if (!this.run) {
      return;
    }
    const window = ZoomWindow(
      this.timeWindow,
      this.time.coverage,
      numerator,
      denominator,
      anchorUs,
    );
    if (
      window.beginUs === this.timeWindow.beginUs &&
      window.endUs === this.timeWindow.endUs
    ) {
      return;
    }
    this.PreserveValueRanges();
    this.SetTimeWindow(window);
  }

  RenderWindowControls() {
    const element = this.$('#window-readout');
    if (element) {
      element.textContent = `${RelativeTime(this.timeWindow.beginUs, this.timeOriginUs)} – ${RelativeTime(this.timeWindow.endUs, this.timeOriginUs)}`;
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
      if (
        this.currentLayoutId &&
        [...select.options].some(
          (option) => option.value === this.currentLayoutId,
        )
      ) {
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
      globalThis.localStorage?.setItem(
        'irmv.workbench.layout',
        JSON.stringify(documentValue),
      );
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
