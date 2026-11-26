import { test, expect } from '@playwright/test';
import { execFileSync } from 'node:child_process';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const HERE = path.dirname(fileURLToPath(import.meta.url));
const REPO = path.resolve(HERE, '..', '..', '..');
const FIXTURES = path.join(REPO, 'build', 'web-e2e-fixtures');
const LIVE_SOCKET = path.join(FIXTURES, 'data', 'live.sock');
const RUNTIME_RUN = path.join(FIXTURES, 'records', 'run_web_a');

function RelativeFixtureTime(stamp) {
  const elapsed = BigInt(stamp) - 9007199254740993n;
  return `${elapsed / 1000000n}.${String(elapsed % 1000000n).padStart(6, '0')} s`;
}

test.describe.configure({ mode: 'serial' });

async function openRun(page, name, appName = null) {
  await expect
    .poll(() =>
      page.evaluate(() => Boolean(globalThis.__workbench?.catalogTimer)),
    )
    .toBe(true);
  if (!(await page.locator('#experiment-browser').isVisible())) {
    await page.click('#experiment-picker-toggle');
  }
  await page.fill('#run-search', '');
  await expect(page.locator('#run-list .run-item').first()).toBeVisible();
  await page
    .locator('#run-list .run-item')
    .filter({ hasText: name })
    .locator('button')
    .click();
  await expect(page.locator('#experiment-browser')).toBeHidden();
  await expect(page.locator('#run-title')).toHaveText(name);
  await expect(page.locator('#signal-tree .signal').first()).toBeVisible();
  if (appName) {
    await page.locator('#experiment-sources summary').click();
    await expect(page.locator('#app-run-select option:checked')).toHaveText(
      appName,
    );
  }
}

async function chartId(page, index = 0) {
  return page.evaluate((position) => {
    const charts = [...document.querySelectorAll('.chart')];
    return charts[position]?.dataset.chartId ?? null;
  }, index);
}

async function drop(page, signalId, targetChartId, zone) {
  return page.evaluate(
    ({ signalId, targetChartId, zone }) => {
      const signal = [...document.querySelectorAll('.signal')].find(
        (item) => item.dataset.signalId === signalId,
      );
      const target = targetChartId
        ? document.querySelector(
            `.chart-body[data-chart-id="${targetChartId}"]`,
          )
        : document.querySelector('#dock-root .workspace-empty');
      if (!signal || !target) {
        return { ok: false, signal: Boolean(signal), target: Boolean(target) };
      }
      const data = new DataTransfer();
      signal.dispatchEvent(
        new DragEvent('dragstart', {
          bubbles: true,
          dataTransfer: data,
        }),
      );
      const box = target.getBoundingClientRect();
      let x = box.left + box.width / 2;
      let y = box.top + box.height / 2;
      if (zone === 'right') x = box.right - 6;
      if (zone === 'left') x = box.left + 6;
      if (zone === 'top') y = box.top + 6;
      if (zone === 'bottom') y = box.bottom - 6;
      if (zone === 'empty') {
        x = box.left + 20;
        y = box.top + 20;
      }
      const event = {
        bubbles: true,
        dataTransfer: data,
        clientX: x,
        clientY: y,
      };
      target.dispatchEvent(new DragEvent('dragover', event));
      const preview = target.className;
      target.dispatchEvent(new DragEvent('drop', event));
      return { ok: true, preview };
    },
    { signalId, targetChartId, zone },
  );
}

async function dropChart(page, chartIdToMove, targetChartId, zone) {
  return page.evaluate(
    ({ chartIdToMove, targetChartId, zone }) => {
      const source = document.querySelector(
        `.chart[data-chart-id="${chartIdToMove}"] .chart-legend`,
      );
      const target = document.querySelector(
        `.chart-body[data-chart-id="${targetChartId}"]`,
      );
      if (!source || !target) {
        return false;
      }
      const data = new DataTransfer();
      source.dispatchEvent(
        new DragEvent('dragstart', {
          bubbles: true,
          dataTransfer: data,
        }),
      );
      const box = target.getBoundingClientRect();
      const event = {
        bubbles: true,
        dataTransfer: data,
        clientX: box.left + 6,
        clientY: box.top + box.height / 2,
      };
      target.dispatchEvent(new DragEvent('dragover', event));
      target.dispatchEvent(new DragEvent('drop', event));
      return true;
    },
    { chartIdToMove, targetChartId, zone },
  );
}

async function canvasHasInk(page) {
  return page.evaluate(() => {
    const canvases = [...document.querySelectorAll('canvas.chart-canvas')];
    if (canvases.length === 0) {
      return false;
    }
    return canvases.every((canvas) => {
      const context = canvas.getContext('2d');
      const data = context.getImageData(0, 0, canvas.width, canvas.height).data;
      let ink = 0;
      for (let index = 3; index < data.length; index += 4) {
        if (data[index] !== 0) {
          ink += 1;
        }
      }
      return ink > 100;
    });
  });
}

async function axisBoxes(page) {
  return page.evaluate(() => {
    const canvas = document.querySelector('.chart-canvas');
    const ctx = canvas.getContext('2d');
    const original = ctx.fillText;
    const columns = new Map();
    const plot = __workbench.dock.plotViews.get(canvas.dataset.chartId).plot;
    try {
      ctx.fillText = function (text, x, y, ...args) {
        if (
          x < plot.left &&
          y >= plot.top &&
          y <= plot.bottom + 3 &&
          /^-?[\d.]/.test(String(text))
        ) {
          const right = x + ctx.measureText(text).width;
          columns.set(x, {
            x,
            right: Math.max(right, columns.get(x)?.right ?? x),
            y,
          });
        }
        return original.call(this, text, x, y, ...args);
      };
      __workbench.dock.Redraw();
    } finally {
      ctx.fillText = original;
    }
    return [...columns.values()];
  });
}

test('the first signal can be dropped anywhere in the empty workspace', async ({
  page,
}) => {
  for (const width of [1280, 390]) {
    await page.setViewportSize({ width, height: 820 });
    for (const [horizontal, vertical] of [
      [0, 0],
      [1, 0],
      [0, 1],
      [1, 1],
    ]) {
      await page.goto('/');
      await openRun(page, 'run_web_a');
      const result = await page.evaluate(
        ({ horizontal, vertical }) => {
          const root = document.querySelector('#dock-root');
          root.scrollIntoView({ block: 'center' });
          const box = root.getBoundingClientRect();
          const x = horizontal ? box.right - 8 : box.left + 8;
          const y = vertical ? box.bottom - 8 : box.top + 8;
          const target = document.elementFromPoint(x, y);
          const data = new DataTransfer();
          const signal = document.querySelector(
            '[data-signal-id="runtime/left_hip/q"]',
          );
          signal.dispatchEvent(
            new DragEvent('dragstart', { bubbles: true, dataTransfer: data }),
          );
          const event = {
            bubbles: true,
            cancelable: true,
            dataTransfer: data,
            clientX: x,
            clientY: y,
          };
          const accepted = !target.dispatchEvent(
            new DragEvent('dragover', event),
          );
          const highlighted = Boolean(root.querySelector('.drop-empty'));
          target.dispatchEvent(new DragEvent('drop', event));
          return { accepted, highlighted };
        },
        { horizontal, vertical },
      );
      expect(result).toEqual({ accepted: true, highlighted: true });
      await expect(page.locator('.chart')).toHaveCount(1);
      await expect(page.locator('.legend-item')).toHaveAttribute(
        'data-signal-id',
        'runtime/left_hip/q',
      );
    }
  }
});

test('builds a multi-axis chart, splits it and keeps every axis on the left', async ({
  page,
}) => {
  const requests = [];
  page.on('request', (request) => {
    if (request.url().includes('/range')) {
      requests.push(request.url());
    }
  });
  await page.goto('/');
  await openRun(page, 'run_web_a', 'app_run_web');

  const preview = await drop(page, 'runtime/left_hip/q', null, 'empty');
  expect(preview.preview).toContain('drop-empty');
  await expect(page.locator('.chart')).toHaveCount(1);
  const first = await chartId(page);

  await drop(page, 'runtime/left_hip/measured_q', first, 'center');
  await expect(
    page.locator('.chart').first().locator('.chart-legend .legend-item'),
  ).toHaveCount(2);
  await expect(page.locator('.chart-axes')).toHaveCount(0);
  expect(await axisBoxes(page)).toHaveLength(1);

  await drop(page, 'runtime/left_hip/dq', first, 'right');
  await expect(page.locator('.dock-split')).toHaveCount(1);
  await expect(page.locator('.chart')).toHaveCount(2);

  await drop(page, 'runtime/left_hip/output_torque', first, 'center');
  await drop(page, 'runtime//imu_angular_velocity_x', first, 'center');
  await drop(page, 'application//yaw', first, 'center');
  expect(await axisBoxes(page)).toHaveLength(4);

  const boxes = await axisBoxes(page);
  expect(boxes.length).toBe(4);
  boxes.sort((left, right) => left.x - right.x);
  for (let index = 1; index < boxes.length; index += 1) {
    expect(boxes[index].x).toBeGreaterThanOrEqual(boxes[index - 1].right - 1);
  }
  expect(await canvasHasInk(page)).toBe(true);
  expect(
    requests.some((url) => url.includes('begin_us=9007199254740993')),
  ).toBe(true);
});

test('history contains experiments while live opens directly without a picker', async ({
  page,
}) => {
  await page.goto('/');
  await expect(page.locator('#experiment-picker-toggle')).toContainText(
    'History',
  );
  await expect(page.locator('#live-experiments')).toContainText('View live');
  await expect(page.locator('#browse-live')).toHaveCount(0);
  await expect(
    page.locator('#run-list .run-item[data-kind="application"]'),
  ).toHaveCount(0);
  await expect(
    page.locator('#experiment-browser .application-choice'),
  ).toHaveCount(0);
  await expect(page.locator('#run-list')).toContainText('Replay');
  await openRun(page, 'run_web_a', 'app_run_web');
  await expect(page.locator('#app-run-select')).not.toContainText(
    'app_run_foreign',
  );
  await page.click('#experiment-picker-toggle');
  await expect(page.locator('#run-list .run-item.selected')).toContainText(
    'Viewing',
  );
  await page.keyboard.press('Escape');
  await expect(page.locator('#experiment-picker-toggle')).toBeFocused();
  execFileSync('python3', [
    path.join(HERE, 'send_live.py'),
    '--socket',
    LIVE_SOCKET,
    '--run',
    RUNTIME_RUN,
    '--start',
    '9007199268000000',
    '--count',
    '5',
    '--value',
    '0.9',
  ]);
  await page.evaluate(() => __workbench.RefreshRuns());
  const reportRequests = [];
  page.on('request', (request) => {
    const url = new URL(request.url());
    if (/\/api\/runs\/[^/]+\/(describe|context)$/.test(url.pathname)) {
      reportRequests.push(url);
    }
  });
  await page.click('#live-experiments');
  await expect(page.locator('#experiment-browser')).toBeHidden();
  await expect(page.locator('#experiment-state')).toContainText('Live');
  await expect(page.locator('#signal-tree')).toContainText('left_hip');
  expect(reportRequests.some((url) => url.pathname.endsWith('/context'))).toBe(
    true,
  );
  expect(
    reportRequests.every(
      (url) => url.searchParams.get('simulation_evidence') === '0',
    ),
  ).toBe(true);
});

test('live availability distinguishes absent, application-only and conflicting runtime telemetry', async ({
  page,
}) => {
  let mode = 'none';
  await page.route('**/api/runs', async (route) => {
    const response = await route.fetch();
    const payload = await response.json();
    for (const run of payload.runs) {
      run.live = {
        active:
          mode === 'application'
            ? run.kind === 'application'
            : mode === 'conflict' && run.kind === 'runtime',
        age_ms: 0,
      };
    }
    await route.fulfill({ response, json: payload });
  });
  await page.goto('/');
  await expect(page.locator('#telemetry-availability')).toHaveText(
    'No live telemetry',
  );
  await expect(page.locator('#live-experiments')).toBeDisabled();
  mode = 'application';
  await page.evaluate(() => __workbench.RefreshRuns());
  await expect(page.locator('#live-experiments')).toBeDisabled();
  await expect(
    page.locator('#run-list .run-item[data-kind="application"]'),
  ).toHaveCount(0);
  await openRun(page, 'run_web_b');
  mode = 'conflict';
  await page.evaluate(() => __workbench.RefreshRuns());
  await expect(page.locator('#telemetry-availability')).toHaveText(
    'Conflicting runtime streams',
  );
  await expect(page.locator('#live-experiments')).toBeDisabled();
  await expect(page.locator('#run-title')).toHaveText('run_web_b');
  await expect(page.locator('#experiment-state')).toHaveText('History');
});

test('history shows host-clock start times and sorts recordings by actual start', async ({
  page,
}) => {
  await page.route('**/api/runs', async (route) => {
    const response = await route.fetch();
    const payload = await response.json();
    const names = {
      run_web_a: 'run_1767225600000000',
      run_web_b: 'run_1735689600000000',
    };
    for (const run of payload.runs) run.name = names[run.name] ?? run.name;
    await route.fulfill({ response, json: payload });
  });
  await page.goto('/');
  const latest = page.locator('#run-list .run-item').first();
  await expect(latest).toContainText('run_1767225600000000');
  await expect(latest).toContainText('2026-01-01 00:00:00 UTC');
  await expect(latest).toContainText('12.0 s');
  await page.fill('#run-search', '2026-01-01');
  await expect(page.locator('#run-list .run-item')).toHaveCount(1);
});

test('browses experiments and clears the workspace only when a different experiment opens', async ({
  page,
}) => {
  await page.goto('/');
  await expect(page.locator('#tab-records')).toHaveCount(0);
  await expect(page.locator('#experiment-browser')).toBeVisible();
  await page.fill('#run-search', 'run_web_b');
  await expect(page.locator('#run-list .run-item')).toHaveCount(1);
  await page.fill('#run-search', 'no-such-experiment');
  await expect(page.locator('#run-list')).toContainText(
    'No matching experiments',
  );
  await openRun(page, 'run_web_a');
  await drop(page, 'runtime/left_hip/q', null, 'empty');
  const chart = await chartId(page);
  await page.click('#experiment-picker-toggle');
  await page.keyboard.press('Escape');
  await expect(page.locator('#experiment-browser')).toBeHidden();
  await openRun(page, 'run_web_a');
  expect(await chartId(page)).toBe(chart);
  await openRun(page, 'run_web_b');
  await expect(page.locator('.chart')).toHaveCount(0);
  await expect(page.locator('.workspace-empty')).toBeVisible();
  await expect(page.locator('#cursor-readout')).toBeEmpty();
  await expect(page.locator('#missing-signals')).toBeEmpty();
  expect(await page.evaluate(() => __workbench.series)).toEqual([]);
  await expect(page.locator('#experiment-state')).toContainText('History');
});

test('selects a recent live experiment and pauses following without changing the source', async ({
  page,
}) => {
  execFileSync('python3', [
    path.join(HERE, 'send_live.py'),
    '--socket',
    LIVE_SOCKET,
    '--run',
    RUNTIME_RUN,
    '--start',
    '9007199268000000',
    '--count',
    '5',
    '--value',
    '0.9',
  ]);
  await page.goto('/');
  await page.click('#live-experiments');
  await expect(page.locator('#experiment-browser')).toBeHidden();
  await expect(page.locator('#experiment-state')).toContainText('Live');
  await expect
    .poll(() => page.evaluate(() => __workbench.time.following))
    .toBe(true);
  await expect
    .poll(() =>
      page.evaluate(() =>
        String(__workbench.time.window.endUs - __workbench.time.window.beginUs),
      ),
    )
    .toBe('10000000');
  await page.click('#pause');
  await expect(page.locator('#experiment-state')).toContainText('Reviewing');
  await page.click('#go-live');
  await expect
    .poll(() => page.evaluate(() => __workbench.time.following))
    .toBe(true);
  await expect
    .poll(() =>
      page.evaluate(() =>
        String(__workbench.time.window.endUs - __workbench.time.window.beginUs),
      ),
    )
    .toBe('10000000');
  await page.route('**/api/live/snapshot?**', (route) => route.abort());
  await page.route('**/api/runs', (route) => route.abort());
  await expect(page.locator('#experiment-state')).toContainText('Offline');
  await expect(page.locator('#run-title')).toHaveText('run_web_a');
  await page.unroute('**/api/runs');
  await openRun(page, 'run_web_b');
  expect(await page.evaluate(() => __workbench.liveTimer)).toBeNull();
});

test('refreshing experiments preserves keyboard focus on a recording', async ({
  page,
}) => {
  await page.goto('/');
  await expect(page.locator('#experiment-browser')).toBeVisible();
  const open = page
    .locator('#run-list .run-item')
    .filter({ hasText: 'run_web_b' })
    .locator('button');
  await open.focus();
  await page.evaluate(() => __workbench.RefreshRuns({ quiet: true }));
  await expect(open).toBeFocused();
  await page.keyboard.press('Enter');
  await expect(page.locator('#run-title')).toHaveText('run_web_b');
});

test('the latest source selection wins when an earlier description is delayed', async ({
  page,
}) => {
  await page.goto('/');
  await expect(page.locator('#experiment-browser')).toBeVisible();
  const ids = await page.evaluate(() =>
    Object.fromEntries(__workbench.runs.map((run) => [run.name, run.run_id])),
  );
  await page.route(`**/api/runs/${ids.run_web_a}/describe`, async (route) => {
    await new Promise((resolve) => setTimeout(resolve, 350));
    await route.continue();
  });
  await page
    .locator('#run-list .run-item')
    .filter({ hasText: 'run_web_a' })
    .locator('button')
    .click();
  await page
    .locator('#run-list .run-item')
    .filter({ hasText: 'run_web_b' })
    .locator('button')
    .click();
  await expect(page.locator('#run-title')).toHaveText('run_web_b');
  await page.waitForTimeout(450);
  await expect(page.locator('#run-title')).toHaveText('run_web_b');
});

test('a mismatched application stream leaves the current experiment and charts intact', async ({
  page,
}) => {
  await page.goto('/');
  await openRun(page, 'run_web_b');
  await drop(page, 'runtime/waist/q', null, 'empty');
  const chart = await chartId(page);
  await page.click('#experiment-picker-toggle');
  await page.evaluate(async () => {
    const runtime = __workbench.runs.find((run) => run.name === 'run_web_a');
    const foreign = __workbench.runs.find(
      (run) => run.name === 'app_run_foreign',
    );
    await __workbench
      .LoadRun(runtime.run_id, foreign.run_id)
      .catch((error) => __workbench.BrowserMessage(error.message, true));
  });
  await expect(page.locator('#experiment-browser-message')).toContainText(
    'does not match',
  );
  await expect(page.locator('#experiment-browser')).toBeVisible();
  await expect(page.locator('#run-title')).toHaveText('run_web_b');
  expect(await chartId(page)).toBe(chart);
  await page
    .locator('#run-list .run-item')
    .filter({ hasText: 'run_web_a' })
    .locator('button')
    .click();
  await expect(page.locator('#run-title')).toHaveText('run_web_a');
});

test('a narrow window keeps all axes and scrolls instead of dropping one', async ({
  page,
}) => {
  await page.goto('/');
  await openRun(page, 'run_web_a', 'app_run_web');
  const preview = await drop(page, 'runtime/left_hip/q', null, 'empty');
  expect(preview.ok).toBe(true);
  const first = await chartId(page);
  for (const signal of [
    'runtime/left_hip/output_torque',
    'runtime//imu_angular_velocity_x',
    'application//yaw',
  ]) {
    await drop(page, signal, first, 'center');
  }
  await page.setViewportSize({ width: 720, height: 700 });
  await page.waitForTimeout(250);
  const boxes = await axisBoxes(page);
  expect(boxes.length).toBe(4);
  boxes.sort((left, right) => left.x - right.x);
  for (let index = 1; index < boxes.length; index += 1) {
    expect(boxes[index].x).toBeGreaterThanOrEqual(boxes[index - 1].right - 1);
  }
  const scroll = await page
    .locator('.chart-body')
    .first()
    .evaluate((element) => ({
      scrollWidth: element.scrollWidth,
      clientWidth: element.clientWidth,
    }));
  expect(scroll.scrollWidth).toBeGreaterThanOrEqual(scroll.clientWidth);
});

test('resizes, moves, deletes, saves and restores a dock layout', async ({
  page,
}) => {
  await page.goto('/');
  await openRun(page, 'run_web_a');
  await drop(page, 'runtime/left_hip/q', null, 'empty');
  const first = await chartId(page);
  await drop(page, 'runtime/left_hip/dq', first, 'right');
  const layoutName = `browser-${Date.now()}`;
  await page.fill('#layout-name', layoutName);
  await page.click('#save-layout');
  await expect(page.locator('#status-line')).toContainText('layout saved');

  const divider = page.locator('.dock-divider').first();
  const box = await divider.boundingBox();
  await page.mouse.move(box.x + 2, box.y + box.height / 2);
  await page.mouse.down();
  await page.mouse.move(box.x + 70, box.y + box.height / 2, { steps: 5 });
  await page.mouse.up();
  const ratio = await page.evaluate(() => window.__workbench.layout.dock.ratio);
  expect(ratio).toBeGreaterThan(0.5);

  const ids = await page.evaluate(() =>
    [...document.querySelectorAll('.chart')].map(
      (chart) => chart.dataset.chartId,
    ),
  );
  expect(ids.length).toBe(2);
  await dropChart(page, ids[1], ids[0], 'bottom');
  await expect(page.locator('.chart')).toHaveCount(2);

  await page.locator('.chart').first().locator('.legend-remove').click();
  await expect(page.locator('.chart')).toHaveCount(1);

  await page.reload();
  await expect(page.locator('#layout-name')).toHaveValue(layoutName);
  await openRun(page, 'run_web_b');
  await expect(page.locator('.chart')).toHaveCount(0);
  await page.click('#load-layout');
  await expect(page.locator('.legend-item.missing').first()).toBeVisible();
  const missingText = await page
    .locator('.legend-item.missing')
    .first()
    .textContent();
  expect(missingText).toContain('(missing)');
  await expect(page.locator('#missing-signals')).toContainText(
    'missing signal',
  );
});

test('displays fault events as text, seeks to one and keeps the live tail honest', async ({
  page,
}) => {
  await page.goto('/');
  await openRun(page, 'run_web_a');
  const notice = page.locator('#fault-notice');
  await expect(notice).toContainText('Fault observed: JointPositionExceeded');
  await expect(notice).toContainText('left_hip');
  await drop(page, 'runtime/left_hip/q', null, 'empty');
  await page.click('#load-events');
  await expect(page.locator('.event-item')).toHaveCount(3);
  const message = await page.locator('.event-message').first().textContent();
  expect(message).toContain('<script>');
  expect(await page.evaluate(() => window.__xss)).toBeUndefined();
  await page.locator('.event-item').first().click();
  await expect(page.locator('#event-detail')).toContainText('message_hex');
  await expect(page.locator('#event-detail')).toContainText(
    'JointPositionExceeded',
  );

  execFileSync('python3', [
    path.join(HERE, 'send_live.py'),
    '--socket',
    LIVE_SOCKET,
    '--run',
    RUNTIME_RUN,
    '--start',
    '9007199270000000',
    '--count',
    '8',
    '--value',
    '0.9',
    '--fault',
  ]);
  await page.click('#go-live');
  await expect(page.locator('#live-status')).toContainText('Receiving data');
  await expect(notice).toContainText('Fault observed: JointVelocityExceeded');
  await expect(notice).not.toContainText('FeedbackInvalid');
  await page.click('#load-events');
  await expect(notice).toContainText('Fault observed: JointVelocityExceeded');
  const pausedBefore = await page.evaluate(() =>
    window.__workbench.time.window.endUs.toString(),
  );
  await page.click('#pause');
  const pausedAt = await page.evaluate(() =>
    window.__workbench.time.window.endUs.toString(),
  );
  execFileSync('python3', [
    path.join(HERE, 'send_live.py'),
    '--socket',
    LIVE_SOCKET,
    '--run',
    RUNTIME_RUN,
    '--start',
    '9007199275000000',
    '--count',
    '5',
    '--value',
    '1.2',
  ]);
  await page.waitForTimeout(500);
  const stillPaused = await page.evaluate(() =>
    window.__workbench.time.window.endUs.toString(),
  );
  expect(stillPaused).toBe(pausedAt);
  await page.click('#go-live');
  await page.waitForTimeout(500);
  const followed = await page.evaluate(() =>
    window.__workbench.time.window.endUs.toString(),
  );
  expect(BigInt(followed)).toBeGreaterThan(BigInt(pausedBefore));
});

test('keeps the previous window when a query is slow or fails', async ({
  page,
}) => {
  await page.goto('/');
  await openRun(page, 'run_web_a');
  await drop(page, 'runtime/left_hip/q', null, 'empty');
  await page.waitForTimeout(400);
  const before = await page.evaluate(
    () => window.__workbench.series[0]?.points?.length ?? 0,
  );
  await page.route('**/api/runs/*/range*', async (route) => {
    await new Promise((resolve) => setTimeout(resolve, 500));
    await route.abort();
  });
  await page.locator('.chart-canvas').hover();
  await page.mouse.wheel(0, -120);
  await page.waitForTimeout(200);
  const during = await page.evaluate(
    () => window.__workbench.series[0]?.points?.length ?? 0,
  );
  expect(during).toBe(before);
  await page.unroute('**/api/runs/*/range*');
});

test('redraws charts at the new viewport size without stretching their canvas', async ({
  page,
}) => {
  await page.goto('/');
  await openRun(page, 'run_web_a');
  await drop(page, 'runtime/left_hip/q', null, 'empty');
  await page.waitForLoadState('networkidle');
  await page.mouse.move(0, 0);
  await page.setViewportSize({ width: 390, height: 844 });
  const canvas = page.locator('.chart-canvas').first();
  await expect
    .poll(() =>
      canvas.evaluate((element) =>
        Math.abs(
          element.width - Math.round(element.clientWidth * devicePixelRatio),
        ),
      ),
    )
    .toBeLessThanOrEqual(1);
  expect(
    await page.evaluate(() => document.documentElement.scrollWidth),
  ).toBeLessThanOrEqual(390);
});

test('anchors drop previews to the visible target chart even after horizontal scrolling', async ({
  page,
}) => {
  await page.goto('/');
  await openRun(page, 'run_web_a', 'app_run_web');
  await drop(page, 'runtime/left_hip/q', null, 'empty');
  await drop(page, 'runtime/left_hip/dq', await chartId(page), 'right');
  const targetId = await chartId(page, 1);
  for (const signal of [
    'runtime/left_hip/output_torque',
    'runtime//imu_angular_velocity_x',
    'application//yaw',
  ]) {
    await drop(page, signal, targetId, 'center');
  }
  await page.waitForLoadState('networkidle');
  for (const width of [1280, 720]) {
    await page.setViewportSize({ width, height: 820 });
    const body = page.locator(`.chart-body[data-chart-id="${targetId}"]`);
    await body.evaluate((element) => {
      element.scrollLeft = element.scrollWidth;
    });
    for (const zone of ['left', 'right', 'top', 'bottom', 'center']) {
      const bounds = await body.evaluate((element, zone) => {
        const box = element.getBoundingClientRect();
        const x =
          zone === 'left'
            ? box.left + 4
            : zone === 'right'
              ? box.left + element.clientWidth - 4
              : box.left + element.clientWidth / 2;
        const y =
          zone === 'top'
            ? box.top + 4
            : zone === 'bottom'
              ? box.top + element.clientHeight - 4
              : box.top + element.clientHeight / 2;
        const data = new DataTransfer();
        data.setData(
          'application/x-irmv-signal',
          'runtime/left_hip/measured_q',
        );
        element.dispatchEvent(
          new DragEvent('dragover', {
            bubbles: true,
            dataTransfer: data,
            clientX: x,
            clientY: y,
          }),
        );
        return {
          x: box.left,
          y: box.top,
          width: element.clientWidth,
          height: element.clientHeight,
        };
      }, zone);
      const preview = page.locator(
        `.chart[data-chart-id="${targetId}"] .chart-drop-preview`,
      );
      await expect(preview).toBeVisible();
      const box = await preview.boundingBox();
      const horizontal = zone === 'left' || zone === 'right';
      const vertical = zone === 'top' || zone === 'bottom';
      expect(
        Math.abs(
          box.x - (bounds.x + (zone === 'right' ? bounds.width / 2 : 0)),
        ),
      ).toBeLessThanOrEqual(1);
      expect(
        Math.abs(
          box.y - (bounds.y + (zone === 'bottom' ? bounds.height / 2 : 0)),
        ),
      ).toBeLessThanOrEqual(1);
      expect(
        Math.abs(box.width - bounds.width / (horizontal ? 2 : 1)),
      ).toBeLessThanOrEqual(1);
      expect(
        Math.abs(box.height - bounds.height / (vertical ? 2 : 1)),
      ).toBeLessThanOrEqual(1);
      // Crossing a canvas/axis child is still inside the same drop target.
      await body.evaluate((element) =>
        element.dispatchEvent(
          new DragEvent('dragleave', {
            bubbles: true,
            relatedTarget: element.querySelector('canvas'),
          }),
        ),
      );
      await expect(preview).toBeVisible();
      await body.dispatchEvent('dragleave', { relatedTarget: null });
      await expect(preview).toBeHidden();
    }
  }
  await drop(page, 'runtime/left_hip/measured_q', targetId, 'right');
  await expect(page.locator('.chart')).toHaveCount(3);
  await expect(page.locator('.chart-drop-preview:visible')).toHaveCount(0);
});

test('identifies curves through the legend without redundant chart titles', async ({
  page,
}) => {
  await page.goto('/');
  await openRun(page, 'run_web_a');
  await drop(page, 'runtime/left_hip/q', null, 'empty');
  const id = await chartId(page);
  await drop(page, 'runtime/left_hip/measured_q', id, 'center');
  await expect(page.locator('.chart-title')).toHaveCount(0);
  const legend = page.locator('.chart-legend');
  await expect(legend.locator('select')).toHaveCount(0);
  await expect(legend.locator('.legend-label').first()).toContainText(
    'left_hip',
  );
  await expect(legend.locator('.legend-unit').first()).toHaveText('rad');
  await page.waitForLoadState('networkidle');
  const colors = await legend
    .locator('.legend-swatch')
    .evaluateAll((items) =>
      items.map((item) => getComputedStyle(item).borderTopColor),
    );
  expect(colors.length).toBe(2);
  expect(new Set(colors).size, JSON.stringify(colors)).toBe(2);
  const firstTraceColor = await page
    .locator('.chart-canvas')
    .evaluate((canvas) => {
      const hex = getComputedStyle(canvas).getPropertyValue('--c1').trim();
      return `rgb(${[1, 3, 5].map((offset) => parseInt(hex.slice(offset, offset + 2), 16)).join(', ')})`;
    });
  expect(colors[0]).toBe(firstTraceColor);
  await drop(page, 'runtime/right_hip/q', id, 'center');
  await expect(legend.locator('.legend-item')).toHaveCount(3);
  await expect(legend.locator('.legend-label').last()).toContainText(
    'right_hip',
  );
  await legend.locator('.legend-remove').first().click();
  await expect(legend.locator('.legend-item')).toHaveCount(2);
  expect(
    await page.evaluate(() => __workbench.dock.highlightedSignal),
  ).not.toBe('runtime/left_hip/q');
  await page.mouse.move(0, 0);
  expect(await page.evaluate(() => __workbench.dock.highlightedSignal)).toBe(
    null,
  );
});

test('drag selects shared time while Ctrl-drag changes only local value axes', async ({
  page,
}) => {
  await page.goto('/');
  await openRun(page, 'run_web_a');
  await drop(page, 'runtime/left_hip/q', null, 'empty');
  const first = await chartId(page);
  await drop(page, 'runtime/left_hip/output_torque', first, 'center');
  await drop(page, 'runtime/left_hip/dq', first, 'bottom');
  await page.waitForLoadState('networkidle');
  await expect(page.locator('.chart-header')).toHaveCount(0);
  const before = await page.evaluate(() => ({
    begin: String(__workbench.timeWindow.beginUs),
    end: String(__workbench.timeWindow.endUs),
    ranges: [...__workbench.dock.plotViews.values()].map((view) => view.ranges),
  }));
  async function select(cancel = false, control = false) {
    const geometry = await page.evaluate(() => {
      const element = document.querySelector('.chart-canvas');
      const rect = element.getBoundingClientRect();
      const plot = globalThis.__workbench.dock.plotViews.get(
        element.dataset.chartId,
      ).plot;
      return {
        x: rect.left + plot.left,
        y: rect.top + plot.top,
        width: plot.right - plot.left,
        height: plot.bottom - plot.top,
      };
    });
    await page.mouse.move(
      geometry.x + geometry.width * 0.2,
      geometry.y + geometry.height * 0.2,
    );
    if (control) await page.keyboard.down('Control');
    await page.mouse.down();
    await page.mouse.move(
      geometry.x + geometry.width * (control ? 0.2 : 0.8),
      geometry.y + geometry.height * (control ? 0.8 : 0.2),
      { steps: 5 },
    );
    await expect(page.locator('.chart-selection:visible')).toHaveCount(1);
    const band = await page.locator('.chart-selection:visible').boundingBox();
    expect(
      Math.abs(
        (control ? band.width : band.height) -
          (control ? geometry.width : geometry.height),
      ),
    ).toBeLessThan(2);
    if (control) await page.keyboard.up('Control');
    if (cancel) await page.keyboard.press('Escape');
    await page.mouse.up();
  }
  await select(true);
  expect(
    await page.evaluate(() => String(__workbench.timeWindow.beginUs)),
  ).toBe(before.begin);
  await expect(page.locator('.chart-selection:visible')).toHaveCount(0);
  await select();
  await expect
    .poll(() => page.evaluate(() => String(__workbench.timeWindow.beginUs)))
    .not.toBe(before.begin);
  await page.waitForLoadState('networkidle');
  const after = await page.evaluate(() => ({
    begin: String(__workbench.timeWindow.beginUs),
    end: String(__workbench.timeWindow.endUs),
    stateBegin: String(__workbench.time.window.beginUs),
    ranges: __workbench.layout.charts.map((chart) =>
      chart.axes.map((axis) => axis.range),
    ),
    displayedRanges: [...__workbench.dock.plotViews.values()].map(
      (view) => view.ranges,
    ),
    dockBegin: String(__workbench.dock.timeWindow.beginUs),
    playing: __workbench.time.playing,
  }));
  expect(BigInt(after.end) - BigInt(after.begin)).toBeLessThan(
    BigInt(before.end) - BigInt(before.begin),
  );
  expect(after.stateBegin).toBe(after.begin);
  expect(after.dockBegin).toBe(after.begin);
  expect(after.ranges[0].every((range) => range && range[0] < range[1])).toBe(
    true,
  );
  expect(after.displayedRanges).toEqual(before.ranges);
  expect(after.playing).toBe(false);
  await select(false, true);
  const vertical = await page.evaluate(() => ({
    begin: String(__workbench.timeWindow.beginUs),
    end: String(__workbench.timeWindow.endUs),
    ranges: __workbench.layout.charts.map((chart) =>
      chart.axes.map((axis) => axis.range),
    ),
    displayedRanges: [...__workbench.dock.plotViews.values()].map(
      (view) => view.ranges,
    ),
  }));
  expect(vertical.begin).toBe(after.begin);
  expect(vertical.end).toBe(after.end);
  expect(vertical.ranges[1]).toEqual(after.ranges[1]);
  expect(
    vertical.ranges[0].every(
      (range, index) =>
        range[1] - range[0] <
        after.ranges[0][index][1] - after.ranges[0][index][0],
    ),
  ).toBe(true);
  expect(vertical.displayedRanges[1]).toEqual(after.displayedRanges[1]);
  await expect(page.locator('.chart-header')).toHaveCount(0);
  await page.locator('.chart-reset-y').first().click();
  expect(
    await page.evaluate(() => String(__workbench.timeWindow.beginUs)),
  ).toBe(vertical.begin);
  expect(await page.evaluate(() => String(__workbench.timeWindow.endUs))).toBe(
    vertical.end,
  );
  expect(
    await page.evaluate(() =>
      __workbench.layout.charts[0].axes.every((axis) => axis.range === null),
    ),
  ).toBe(true);
  expect(
    await page.evaluate(() =>
      __workbench.layout.charts[1].axes.map((axis) => axis.range),
    ),
  ).toEqual(vertical.ranges[1]);
  const beforeTimeReset = await page.evaluate(() =>
    [...__workbench.dock.plotViews.values()].map((view) => view.ranges),
  );
  await page.click('#reset-time');
  await expect
    .poll(() => page.evaluate(() => String(__workbench.timeWindow.beginUs)))
    .toBe(before.begin);
  expect(await page.evaluate(() => String(__workbench.timeWindow.endUs))).toBe(
    before.end,
  );
  await page.waitForLoadState('networkidle');
  expect(
    await page.evaluate(() =>
      [...__workbench.dock.plotViews.values()].map((view) => view.ranges),
    ),
  ).toEqual(beforeTimeReset);
});

test('keeps legend controls inside a chart pane when a vertical split is resized very small', async ({
  page,
}) => {
  await page.goto('/');
  await openRun(page, 'run_web_a');
  await drop(page, 'runtime/left_hip/q', null, 'empty');
  await drop(page, 'runtime/left_hip/dq', await chartId(page), 'bottom');
  await page.waitForLoadState('networkidle');
  const bounds = await page.evaluate(() => {
    __workbench.Resize([], 0.1);
    const chart = document.querySelector('.chart');
    return {
      paneHeight: chart.parentElement.getBoundingClientRect().height,
      paneBottom: chart.parentElement.getBoundingClientRect().bottom,
      legendBottom: chart.querySelector('.chart-legend').getBoundingClientRect()
        .bottom,
    };
  });
  expect(bounds.paneHeight).toBeGreaterThan(0);
  expect(bounds.legendBottom).toBeLessThanOrEqual(bounds.paneBottom + 1);
});

test('renders every sine sample at a distinct increasing subpixel time coordinate', async ({
  page,
}) => {
  await page.goto('/');
  await openRun(page, 'run_web_a');
  await drop(page, 'runtime/left_hip/q', null, 'empty');
  await page.waitForLoadState('networkidle');
  const trace = await page.locator('.chart-canvas').evaluate((canvas) => {
    const ctx = canvas.getContext('2d');
    const original = {
      beginPath: ctx.beginPath,
      lineTo: ctx.lineTo,
      moveTo: ctx.moveTo,
      stroke: ctx.stroke,
    };
    let path = [];
    let longest = [];
    try {
      ctx.beginPath = function () {
        path = [];
        return original.beginPath.call(this);
      };
      ctx.moveTo = function (x, y) {
        path.push(x);
        return original.moveTo.call(this, x, y);
      };
      ctx.lineTo = function (x, y) {
        path.push(x);
        return original.lineTo.call(this, x, y);
      };
      ctx.stroke = function () {
        if (path.length > longest.length) longest = [...path];
        return original.stroke.call(this);
      };
      __workbench.dock.Redraw();
    } finally {
      Object.assign(ctx, original);
    }
    return {
      points: longest.length,
      uniqueX: new Set(longest).size,
      increasing: longest.every(
        (x, index) => index === 0 || x > longest[index - 1],
      ),
    };
  });
  expect(trace.points).toBeGreaterThan(1000);
  expect(trace.uniqueX).toBe(trace.points);
  expect(trace.increasing).toBe(true);
});

test('tracks the pointer before a delayed cursor query and fetches the final idle position', async ({
  page,
}) => {
  await page.goto('/');
  await openRun(page, 'run_web_a');
  await drop(page, 'runtime/left_hip/q', null, 'empty');
  await drop(page, 'runtime/left_hip/dq', await chartId(page), 'bottom');
  await page.waitForLoadState('networkidle');
  let releaseFirst;
  const blocked = new Promise((resolve) => {
    releaseFirst = resolve;
  });
  let releaseLatest;
  const latestBlocked = new Promise((resolve) => {
    releaseLatest = resolve;
  });
  let queries = 0;
  const requested = [];
  await page.route('**/api/runs/*/cursor*', async (route) => {
    requested.push(
      new URL(route.request().url()).searchParams.get('cursor_us'),
    );
    const response = await route.fetch();
    const index = ++queries;
    if (index === 1) await blocked;
    if (index === 2) await latestBlocked;
    await route.fulfill({ response });
  });
  const geometry = await page
    .locator('.chart-canvas')
    .first()
    .evaluate((canvas) => {
      const box = canvas.getBoundingClientRect();
      const plot = __workbench.dock.plotViews.get(canvas.dataset.chartId).plot;
      window.__baseStrokes = 0;
      for (const item of document.querySelectorAll('.chart-canvas')) {
        const ctx = item.getContext('2d');
        const stroke = ctx.stroke;
        ctx.stroke = function (...args) {
          ++window.__baseStrokes;
          return stroke.apply(this, args);
        };
      }
      return {
        x: box.left + plot.left,
        y: box.top + plot.top,
        width: plot.right - plot.left,
        height: plot.bottom - plot.top,
      };
    });
  try {
    const firstQuery = page.waitForRequest('**/api/runs/*/cursor*');
    await page.mouse.move(
      geometry.x + geometry.width * 0.2,
      geometry.y + geometry.height * 0.5,
    );
    await firstQuery;
    await page.mouse.move(
      geometry.x + geometry.width * 0.8,
      geometry.y + geometry.height * 0.5,
      { steps: 12 },
    );
    await expect(page.locator('.chart-cursor:visible')).toHaveCount(2);
    const cursor = page.locator('.chart').first().locator('.chart-cursor');
    await expect
      .poll(async () => {
        const box = await cursor.boundingBox();
        return box
          ? Math.abs(box.x - Math.round(geometry.x + geometry.width * 0.8))
          : 10000;
      })
      .toBeLessThanOrEqual(2);
    expect(await page.evaluate(() => __baseStrokes)).toBe(0);
    const finalStamp = await page.evaluate(() =>
      String(__workbench.cursor.timeUs),
    );
    expect(requested[0]).not.toBe(finalStamp);
    releaseFirst();
    await expect(page.locator('#cursor-readout')).toContainText(
      `t=${RelativeFixtureTime(requested[0])}`,
    );
    const duringReply = await cursor.boundingBox();
    expect(
      Math.abs(duringReply.x - Math.round(geometry.x + geometry.width * 0.8)),
    ).toBeLessThanOrEqual(2);
    await expect(page.locator('.chart-cursor-dot')).toHaveCount(0);
    releaseLatest();
    await expect(page.locator('#cursor-readout')).toContainText(
      `t=${RelativeFixtureTime(finalStamp)}`,
    );
    await expect(page.locator('.chart-cursor-dot')).toHaveCount(2);
    expect(requested.at(-1)).toBe(finalStamp);
    expect(queries).toBeLessThanOrEqual(3);
    const finalBox = await cursor.boundingBox();
    expect(
      Math.abs(finalBox.x - Math.round(geometry.x + geometry.width * 0.8)),
    ).toBeLessThanOrEqual(2);
    expect(await page.evaluate(() => __baseStrokes)).toBe(0);
  } finally {
    releaseFirst();
    releaseLatest();
  }
});

test('resumes the final cursor lookup after a box selection is cancelled', async ({
  page,
}) => {
  await page.goto('/');
  await openRun(page, 'run_web_a');
  await drop(page, 'runtime/left_hip/q', null, 'empty');
  await page.waitForLoadState('networkidle');
  let release;
  const blocked = new Promise((resolve) => {
    release = resolve;
  });
  let queries = 0;
  await page.route('**/api/runs/*/cursor*', async (route) => {
    const response = await route.fetch();
    if (++queries === 1) await blocked;
    await route.fulfill({ response });
  });
  const box = await page.locator('.chart-canvas').evaluate((canvas) => {
    const rect = canvas.getBoundingClientRect();
    const plot = __workbench.dock.plotViews.get(canvas.dataset.chartId).plot;
    return {
      x: rect.left + plot.left,
      y: rect.top + plot.top,
      w: plot.right - plot.left,
      h: plot.bottom - plot.top,
    };
  });
  try {
    const first = page.waitForRequest('**/api/runs/*/cursor*');
    await page.mouse.move(box.x + box.w * 0.2, box.y + box.h * 0.2);
    await first;
    await page.mouse.move(box.x + box.w * 0.4, box.y + box.h * 0.4);
    const stamp = await page.evaluate(() => String(__workbench.cursor.timeUs));
    await page.mouse.down();
    await page.mouse.move(box.x + box.w * 0.7, box.y + box.h * 0.7, {
      steps: 3,
    });
    await expect(page.locator('.chart-selection:visible')).toHaveCount(1);
    release();
    await expect
      .poll(() => page.evaluate(() => Boolean(__workbench.cursorInFlight)))
      .toBe(false);
    await page.keyboard.press('Escape');
    await page.mouse.up();
    await expect(page.locator('#cursor-readout')).toContainText(
      `t=${RelativeFixtureTime(stamp)}`,
    );
    expect(queries).toBeGreaterThanOrEqual(2);
  } finally {
    release();
  }
});

test('limits zoom out to full coverage and keeps a stationary wheel anchor and cursor stable', async ({
  page,
}) => {
  await page.goto('/');
  await openRun(page, 'run_web_a');
  await drop(page, 'runtime/left_hip/q', null, 'empty');
  await page.waitForLoadState('networkidle');
  const full = await page.evaluate(() => {
    const w = __workbench;
    for (let i = 0; i < 15; ++i) w.Zoom(5n, 4n);
    return {
      begin: String(w.timeWindow.beginUs),
      end: String(w.timeWindow.endUs),
      coverageBegin: String(w.time.coverage.beginUs),
      coverageEnd: String(w.time.coverage.endUs),
    };
  });
  expect(full.begin).toBe(full.coverageBegin);
  expect(full.end).toBe(full.coverageEnd);
  await page.waitForLoadState('networkidle');
  const geometry = await page.locator('.chart-canvas').evaluate((canvas) => {
    const rect = canvas.getBoundingClientRect();
    const plot = __workbench.dock.plotViews.get(canvas.dataset.chartId).plot;
    return {
      x: rect.left + plot.left,
      y: rect.top + plot.top,
      w: plot.right - plot.left,
      h: plot.bottom - plot.top,
    };
  });
  const mouseX = Math.round(geometry.x + geometry.w * 0.75);
  await page.mouse.move(mouseX, Math.round(geometry.y + geometry.h * 0.5));
  const anchor = await page.evaluate(() => String(__workbench.cursor.timeUs));
  let previousSpan = BigInt(full.end) - BigInt(full.begin);
  for (let i = 0; i < 3; ++i) {
    await page.mouse.wheel(0, -100);
    await expect
      .poll(() =>
        page.evaluate(() =>
          String(__workbench.timeWindow.endUs - __workbench.timeWindow.beginUs),
        ),
      )
      .not.toBe(String(previousSpan));
    await page.waitForLoadState('networkidle');
    const after = await page.evaluate(() => ({
      begin: String(__workbench.timeWindow.beginUs),
      end: String(__workbench.timeWindow.endUs),
      cursor: String(__workbench.cursor.timeUs),
    }));
    expect(BigInt(after.cursor) - BigInt(anchor)).toBeLessThanOrEqual(3n);
    expect(BigInt(anchor) - BigInt(after.cursor)).toBeLessThanOrEqual(3n);
    previousSpan = BigInt(after.end) - BigInt(after.begin);
    const expectedX =
      geometry.x +
      (Number(BigInt(anchor) - BigInt(after.begin)) / Number(previousSpan)) *
        geometry.w;
    expect(Math.abs(expectedX - mouseX)).toBeLessThanOrEqual(1);
    await expect
      .poll(() =>
        page.evaluate((x) => {
          const line = document.querySelector('.chart-cursor');
          return line && !line.hidden
            ? Math.abs(line.getBoundingClientRect().x - x)
            : 10000;
        }, mouseX),
      )
      .toBeLessThanOrEqual(1);
  }
});

test('brackets sub-sample zoom boundaries without drawing across genuine segment gaps', async ({
  page,
}) => {
  await page.goto('/');
  await openRun(page, 'run_web_a');
  await drop(page, 'runtime/left_hip/q', null, 'empty');
  await page.waitForLoadState('networkidle');
  await page.evaluate(() => {
    const base = __workbench.time.coverage.beginUs;
    __workbench.SetTimeWindow({
      beginUs: base + 1000500n,
      endUs: base + 1004500n,
    });
  });
  await page.waitForLoadState('networkidle');
  const bounds = await page.evaluate(() => {
    const points = __workbench.series[0].points;
    return {
      begin: String(__workbench.timeWindow.beginUs),
      end: String(__workbench.timeWindow.endUs),
      first: points[0]?.t ?? null,
      last: points.at(-1)?.t ?? null,
    };
  });
  expect(bounds.first).not.toBeNull();
  expect(BigInt(bounds.first)).toBeLessThanOrEqual(BigInt(bounds.begin));
  expect(BigInt(bounds.last)).toBeGreaterThanOrEqual(BigInt(bounds.end));
  async function edgeInk() {
    return page.locator('.chart-canvas').evaluate((canvas) => {
      const ctx = canvas.getContext('2d');
      const p = __workbench.dock.plotViews.get(canvas.dataset.chartId).plot;
      const ratio = devicePixelRatio;
      return [p.left + 2, p.right - 3].map((x) => {
        const pixels = ctx.getImageData(
          Math.round(x * ratio),
          Math.ceil((p.top + 1) * ratio),
          1,
          Math.max(1, Math.floor((p.bottom - p.top - 2) * ratio)),
        ).data;
        for (let i = 0; i < pixels.length; i += 4) {
          if (pixels[i] > 60 && pixels[i + 1] > 120 && pixels[i + 2] > 130)
            return true;
        }
        return false;
      });
    });
  }
  expect(await edgeInk()).toEqual([true, true]);
  await page.evaluate(() => {
    __workbench.series = __workbench.series.map((entry) => ({
      ...entry,
      points: entry.points.map((point) => ({
        ...point,
        segment: BigInt(point.t) < __workbench.timeWindow.beginUs ? 0 : 1,
      })),
    }));
    __workbench.dock.SetView(
      __workbench.layout,
      __workbench.resolved,
      __workbench.series,
      __workbench.timeWindow,
      null,
    );
  });
  expect(await edgeInk()).toEqual([false, false]);
});

test('plays the cursor through a stationary chart before following its right edge', async ({
  page,
}) => {
  await page.goto('/');
  await openRun(page, 'run_web_a');
  await drop(page, 'runtime/left_hip/q', null, 'empty');
  await page.evaluate(() => {
    const base = __workbench.time.coverage.beginUs;
    __workbench.SetTimeWindow({ beginUs: base, endUs: base + 2000000n });
    __workbench.RequestCursor(base + 500000n);
  });
  await page.waitForLoadState('networkidle');
  const before = await page.evaluate(() => ({
    begin: String(__workbench.timeWindow.beginUs),
    end: String(__workbench.timeWindow.endUs),
    cursor: String(__workbench.cursor.timeUs),
  }));
  let rangeQueries = 0;
  page.on('request', (request) => {
    if (new URL(request.url()).pathname.endsWith('/range')) ++rangeQueries;
  });
  await page.click('#play');
  await expect
    .poll(() =>
      page.evaluate(() =>
        Number(__workbench.cursor.timeUs - __workbench.time.coverage.beginUs),
      ),
    )
    .toBeGreaterThan(650000);
  await page.click('#pause');
  const after = await page.evaluate(() => ({
    begin: String(__workbench.timeWindow.beginUs),
    end: String(__workbench.timeWindow.endUs),
    cursor: String(__workbench.cursor.timeUs),
  }));
  expect(after.begin).toBe(before.begin);
  expect(after.end).toBe(before.end);
  expect(BigInt(after.cursor)).toBeGreaterThan(BigInt(before.cursor));
  expect(rangeQueries).toBe(0);
});

test('shows experiment-relative times in readouts, events, axes and timestamp-valued signals', async ({
  page,
}) => {
  await page.goto('/');
  await openRun(page, 'run_web_a');
  await drop(page, 'runtime/left_hip/q', null, 'empty');
  await page.waitForLoadState('networkidle');
  await expect(page.locator('#window-readout')).toContainText('0.000000 s');
  const refreshed = page.waitForResponse((response) =>
    response.url().includes('/events?'),
  );
  await page.click('#load-events');
  await refreshed;
  // Old rows remain visible while the refresh has an empty event model.
  await expect
    .poll(() => page.evaluate(() => __workbench.eventsView.events.length))
    .toBeGreaterThan(0);
  await expect(page.locator('.event-time').first()).toHaveText('t=0.001000 s');
  await page.locator('.event-item').first().click();
  await expect(page.locator('#event-detail')).toContainText(
    'detected 0.001000 s',
  );
  await expect(page.locator('#event-detail')).toContainText(
    'sample 0.000990 s',
  );
  await page.evaluate(() =>
    __workbench.RequestCursor(__workbench.timeOriginUs + 250000n),
  );
  await expect(page.locator('#cursor-readout')).toContainText('t=0.250000 s');
  await drop(page, 'runtime//stamp_us', await chartId(page), 'center');
  await expect
    .poll(() =>
      page.evaluate(() =>
        __workbench.series.some((entry) => entry.signal.field === 'stamp_us'),
      ),
    )
    .toBe(true);
  const timeValues = await page.evaluate(() =>
    __workbench.series
      .find((entry) => entry.signal.field === 'stamp_us')
      .points.map((point) => point.v),
  );
  expect(Math.min(...timeValues)).toBe(0);
  expect(Math.max(...timeValues)).toBeLessThan(100000000);
  expect(await page.locator('body').innerText()).not.toContain('9007199254');
  const labels = await page.locator('.chart-canvas').evaluate((canvas) => {
    const ctx = canvas.getContext('2d');
    const original = ctx.fillText;
    const labels = [];
    try {
      ctx.fillText = function (text, ...args) {
        labels.push(String(text));
        return original.call(this, text, ...args);
      };
      __workbench.dock.Redraw();
    } finally {
      ctx.fillText = original;
    }
    return labels;
  });
  expect(labels.some((label) => label === '0 s')).toBe(true);
  expect(labels.some((label) => label.includes('9007199254'))).toBe(false);
});

test('draws round axis labels and a faint grid at the same tick positions', async ({
  page,
}) => {
  await page.goto('/');
  const result = await page.evaluate(async () => {
    const { Render, TimeToX } = await import('/src/chart_view.js');
    const canvas = document.createElement('canvas');
    canvas.style.cssText =
      'width:640px;height:300px;--surface:rgb(10,20,30);--muted:rgb(200,210,220)';
    document.body.append(canvas);
    const ctx = canvas.getContext('2d');
    const original = ctx.fillText;
    const labels = [];
    ctx.fillText = function (text, ...args) {
      labels.push(String(text));
      return original.call(this, text, ...args);
    };
    const origin = 9007199254740993n;
    const window = { beginUs: origin + 2300000n, endUs: origin + 9800000n };
    const view = Render(
      canvas,
      {
        axes: [
          { id: 'y', quantity: 'position', unit: 'rad', range: [-0.37, 0.86] },
        ],
        curves: [],
      },
      [],
      window,
      null,
      [],
      null,
      origin,
    );
    const x = TimeToX(
      origin + 4000000n,
      window,
      view.plot.left,
      view.plot.right,
    );
    const ratio = globalThis.devicePixelRatio || 1;
    const pixel = (offset) => [
      ...ctx.getImageData(
        Math.floor((x + offset) * ratio),
        Math.floor(100 * ratio),
        1,
        1,
      ).data,
    ];
    const result = {
      labels: [...labels],
      grid: pixel(0),
      background: pixel(5),
    };
    const zoomTimes = [];
    for (const [begin, end] of [
      [100000n, 900000n],
      [-900n, -100n],
    ]) {
      labels.length = 0;
      Render(
        canvas,
        {
          axes: [
            {
              id: 'y',
              quantity: 'position',
              unit: 'rad',
              range: [-0.37, 0.86],
            },
          ],
          curves: [],
        },
        [],
        { beginUs: origin + begin, endUs: origin + end },
        null,
        [],
        null,
        origin,
      );
      zoomTimes.push(labels.filter((label) => label.endsWith(' s')));
    }
    return { ...result, zoomTimes };
  });
  const times = result.labels.filter((label) => label.endsWith(' s'));
  expect(times.length).toBeGreaterThan(1);
  expect(times.every((label) => /^-?\d+ s$/.test(label))).toBe(true);
  expect(result.labels).toContain('0.2');
  expect(result.labels).not.toContain('-0.37');
  expect(result.grid[0]).toBeGreaterThan(result.background[0]);
  expect(result.grid[0] - result.background[0]).toBeLessThan(40);
  expect(result.zoomTimes[0]).toEqual(['0.2 s', '0.4 s', '0.6 s', '0.8 s']);
  expect(result.zoomTimes[1]).toEqual([
    '-0.0008 s',
    '-0.0006 s',
    '-0.0004 s',
    '-0.0002 s',
  ]);
});

test('live updates preserve drop previews and a pressed legend remove button', async ({
  page,
}) => {
  const send = (start) =>
    execFileSync('python3', [
      path.join(HERE, 'send_live.py'),
      '--socket',
      LIVE_SOCKET,
      '--run',
      RUNTIME_RUN,
      '--start',
      start,
      '--count',
      '5',
    ]);
  send('9007199280000000');
  await page.goto('/');
  await page.click('#live-experiments');
  await expect(page.locator('#experiment-state')).toContainText('Live');
  await expect(page.locator('.signal').first()).toBeVisible();
  await drop(page, 'runtime/left_hip/q', null, 'empty');
  const id = await chartId(page);
  await drop(page, 'runtime/left_hip/measured_q', id, 'center');
  const body = page.locator('.chart-body').first();
  await body.evaluate((element) => {
    const box = element.getBoundingClientRect();
    element.dispatchEvent(
      new DragEvent('dragover', {
        bubbles: true,
        dataTransfer: new DataTransfer(),
        clientX: box.left + box.width / 2,
        clientY: box.top + box.height / 2,
      }),
    );
  });
  const advance = async (start) => {
    const before = await page.evaluate(() =>
      String(__workbench.time.window.endUs),
    );
    const updated = page.waitForResponse(
      (response) =>
        new URL(response.url()).pathname.endsWith('/range') && response.ok(),
    );
    send(start);
    await page.waitForFunction(
      (old) => String(__workbench.time.window.endUs) !== old,
      before,
    );
    await updated;
  };
  await expect(page.locator('.chart-drop-preview')).toBeVisible();
  await advance('9007199281000000');
  await expect(page.locator('.chart-drop-preview')).toBeVisible();
  await body.dispatchEvent('dragleave', { relatedTarget: null });
  const button = page.locator('.legend-remove').first();
  const bounds = await button.boundingBox();
  await page.mouse.move(
    bounds.x + bounds.width / 2,
    bounds.y + bounds.height / 2,
  );
  await page.mouse.down();
  await advance('9007199282000000');
  await page.mouse.up();
  await expect(page.locator('.legend-item')).toHaveCount(1);
});
