import { defineConfig } from '@playwright/test';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const here = path.dirname(fileURLToPath(import.meta.url));

export default defineConfig({
  testDir: path.join(here, 'tests', 'browser'),
  testMatch: /.*\.spec\.js/,
  timeout: 60_000,
  expect: { timeout: 10_000 },
  workers: 1,
  fullyParallel: false,
  reporter: [
    ['list'],
    [
      'html',
      { open: 'never', outputFolder: path.join(here, 'playwright-report') },
    ],
  ],
  use: {
    baseURL: 'http://127.0.0.1:8766',
    headless: true,
    viewport: { width: 1280, height: 820 },
    trace: 'retain-on-failure',
    screenshot: 'only-on-failure',
  },
  webServer: {
    command: `python3 ${path.join(here, 'tests', 'browser', 'serve_fixture.py')}`,
    cwd: here,
    url: 'http://127.0.0.1:8766/',
    reuseExistingServer: false,
    timeout: 60_000,
    stdout: 'pipe',
    stderr: 'pipe',
  },
});
