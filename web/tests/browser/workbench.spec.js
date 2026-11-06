import { test, expect } from '@playwright/test';
import { execFileSync } from 'node:child_process';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const HERE = path.dirname(fileURLToPath(import.meta.url));
const REPO = path.resolve(HERE, '..', '..', '..');
const FIXTURES = path.join(REPO, 'build', 'web-e2e-fixtures');
const LIVE_SOCKET = path.join(FIXTURES, 'data', 'live.sock');
const RUNTIME_RUN = path.join(FIXTURES, 'records', 'run_web_a');

test.describe.configure({ mode: 'serial' });

async function openRun(page, name, appName = null) {
  await page.click('#tab-records');
  await expect(page.locator('#run-list .run-item').first()).toBeVisible();
  if (appName) {
    await page.selectOption('#app-run-select', { label: appName });
  }
  await page.locator('#run-list .run-item')
    .filter({ hasText: name })
    .locator('button')
    .click();
  await page.click('#tab-workbench');
  await expect(page.locator('#run-title')).toHaveText(name);
  await expect(page.locator('#signal-tree .signal').first()).toBeVisible();
}

async function chartId(page, index = 0) {
  return page.evaluate((position) => {
    const charts = [...document.querySelectorAll('.chart')];
    return charts[position]?.dataset.chartId ?? null;
  }, index);
}

async function drop(page, signalId, targetChartId, zone) {
  return page.evaluate(({ signalId, targetChartId, zone }) => {
    const signal = [...document.querySelectorAll('.signal')]
      .find((item) => item.dataset.signalId === signalId);
    const target = targetChartId
      ? document.querySelector(`.chart-body[data-chart-id="${targetChartId}"]`)
      : document.querySelector('#dock-root .workspace-empty');
    if (!signal || !target) {
      return { ok: false, signal: Boolean(signal), target: Boolean(target) };
    }
    const data = new DataTransfer();
    signal.dispatchEvent(new DragEvent('dragstart', {
      bubbles: true,
      dataTransfer: data,
    }));
    const box = target.getBoundingClientRect();
    let x = box.left + box.width / 2;
    let y = box.top + box.height / 2;
    if (zone === 'right') x = box.right - 6;
    if (zone === 'left') x = box.left + 6;
    if (zone === 'top') y = box.top + 6;
    if (zone === 'bottom') y = box.bottom - 6;
    if (zone === 'empty') { x = box.left + 20; y = box.top + 20; }
    const event = { bubbles: true, dataTransfer: data, clientX: x, clientY: y };
    target.dispatchEvent(new DragEvent('dragover', event));
    const preview = target.className;
    target.dispatchEvent(new DragEvent('drop', event));
    return { ok: true, preview };
  }, { signalId, targetChartId, zone });
}

async function dropChart(page, chartIdToMove, targetChartId, zone) {
  return page.evaluate(({ chartIdToMove, targetChartId, zone }) => {
    const source = document.querySelector(
      `.chart[data-chart-id="${chartIdToMove}"] .chart-header`);
    const target = document.querySelector(
      `.chart-body[data-chart-id="${targetChartId}"]`);
    if (!source || !target) {
      return false;
    }
    const data = new DataTransfer();
    source.dispatchEvent(new DragEvent('dragstart', {
      bubbles: true,
      dataTransfer: data,
    }));
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
  }, { chartIdToMove, targetChartId, zone });
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
  return page.locator('.chart').first().locator('.axis-label').evaluateAll(
    (labels) => labels.map((label) => {
      const box = label.getBoundingClientRect();
      return { text: label.textContent, x: box.left, right: box.right,
               y: box.top, bottom: box.bottom };
    }),
  );
}

test('builds a multi-axis chart, splits it and keeps every axis on the left', async ({ page }) => {
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
  await expect(page.locator('.chart').first().locator('.chart-legend .legend-item'))
    .toHaveCount(2);
  await expect(page.locator('.chart').first().locator('.axis-label')).toHaveCount(1);

  await drop(page, 'runtime/left_hip/dq', first, 'right');
  await expect(page.locator('.dock-split')).toHaveCount(1);
  await expect(page.locator('.chart')).toHaveCount(2);

  await drop(page, 'runtime/left_hip/output_torque', first, 'center');
  await drop(page, 'runtime//imu_angular_velocity_x', first, 'center');
  await drop(page, 'application//yaw', first, 'center');
  await expect(page.locator('.chart').first().locator('.axis-label')).toHaveCount(4);

  const boxes = await axisBoxes(page);
  expect(boxes.length).toBe(4);
  boxes.sort((left, right) => left.x - right.x);
  for (let index = 1; index < boxes.length; index += 1) {
    expect(boxes[index].x).toBeGreaterThanOrEqual(boxes[index - 1].right - 1);
  }
  expect(await canvasHasInk(page)).toBe(true);
  expect(requests.some((url) => url.includes('begin_us=9007199254740993'))).toBe(true);
});

test('a narrow window keeps all axes and scrolls instead of dropping one', async ({ page }) => {
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
  const scroll = await page.locator('.chart-body').first().evaluate((element) => ({
    scrollWidth: element.scrollWidth,
    clientWidth: element.clientWidth,
  }));
  expect(scroll.scrollWidth).toBeGreaterThanOrEqual(scroll.clientWidth);
});

test('resizes, moves, deletes, saves and restores a dock layout', async ({ page }) => {
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
    [...document.querySelectorAll('.chart')].map((chart) => chart.dataset.chartId));
  expect(ids.length).toBe(2);
  await dropChart(page, ids[1], ids[0], 'bottom');
  await expect(page.locator('.chart')).toHaveCount(2);

  await page.locator('.chart').first().locator('.chart-close').click();
  await expect(page.locator('.chart')).toHaveCount(1);

  await page.reload();
  await expect(page.locator('#layout-name')).toHaveValue(layoutName);
  await openRun(page, 'run_web_b');
  await expect(page.locator('.legend-item.missing').first()).toBeVisible();
  const missingText = await page.locator('.legend-item.missing').first().textContent();
  expect(missingText).toContain('(missing)');
  await expect(page.locator('#missing-signals')).toContainText('missing signal');
});

test('displays fault events as text, seeks to one and keeps the live tail honest', async ({ page }) => {
  await page.goto('/');
  await openRun(page, 'run_web_a');
  await drop(page, 'runtime/left_hip/q', null, 'empty');
  await page.click('#load-events');
  await expect(page.locator('.event-item')).toHaveCount(3);
  const message = await page.locator('.event-message').first().textContent();
  expect(message).toContain('<script>');
  expect(await page.evaluate(() => window.__xss)).toBeUndefined();
  await page.locator('.event-item').first().click();
  await expect(page.locator('#event-detail')).toContainText('message_hex');
  await expect(page.locator('#event-detail')).toContainText('JointPositionExceeded');

  execFileSync('python3', [
    path.join(HERE, 'send_live.py'),
    '--socket', LIVE_SOCKET,
    '--run', RUNTIME_RUN,
    '--start', '9007199270000000',
    '--count', '8',
    '--value', '0.9',
  ]);
  await page.click('#go-live');
  await expect(page.locator('#live-status')).toContainText('live revision');
  const pausedBefore = await page.evaluate(() =>
    window.__workbench.time.window.endUs.toString());
  await page.click('#pause');
  const pausedAt = await page.evaluate(() =>
    window.__workbench.time.window.endUs.toString());
  execFileSync('python3', [
    path.join(HERE, 'send_live.py'),
    '--socket', LIVE_SOCKET,
    '--run', RUNTIME_RUN,
    '--start', '9007199275000000',
    '--count', '5',
    '--value', '1.2',
  ]);
  await page.waitForTimeout(500);
  const stillPaused = await page.evaluate(() =>
    window.__workbench.time.window.endUs.toString());
  expect(stillPaused).toBe(pausedAt);
  await page.click('#go-live');
  await page.waitForTimeout(500);
  const followed = await page.evaluate(() =>
    window.__workbench.time.window.endUs.toString());
  expect(BigInt(followed)).toBeGreaterThan(BigInt(pausedBefore));
});

test('keeps the previous window when a query is slow or fails', async ({ page }) => {
  await page.goto('/');
  await openRun(page, 'run_web_a');
  await drop(page, 'runtime/left_hip/q', null, 'empty');
  await page.waitForTimeout(400);
  const before = await page.evaluate(() =>
    window.__workbench.series[0]?.points?.length ?? 0);
  await page.route('**/api/runs/*/range*', async (route) => {
    await new Promise((resolve) => setTimeout(resolve, 500));
    await route.abort();
  });
  await page.click('#zoom-in');
  await page.waitForTimeout(200);
  const during = await page.evaluate(() =>
    window.__workbench.series[0]?.points?.length ?? 0);
  expect(during).toBe(before);
  await page.unroute('**/api/runs/*/range*');
});
