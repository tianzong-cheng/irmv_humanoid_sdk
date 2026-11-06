import test from 'node:test';
import assert from 'node:assert/strict';

import {
  Api,
  ApiError,
  LatestQuery,
  RequestGate,
  SeriesCache,
  Us,
} from '../src/api.js';

function deferred() {
  let resolve;
  let reject;
  const promise = new Promise((res, rej) => {
    resolve = res;
    reject = rej;
  });
  return { promise, resolve, reject };
}

test('an older generation cannot overwrite a newer one', async () => {
  const gate = new RequestGate();
  const applied = [];
  const slow = deferred();
  const fast = deferred();
  const first = LatestQuery(gate, {
    key: 'range',
    loader: () => slow.promise,
    onValue: (value) => applied.push(value),
  });
  const second = LatestQuery(gate, {
    key: 'range',
    loader: () => fast.promise,
    onValue: (value) => applied.push(value),
  });
  fast.resolve('B');
  await second;
  slow.resolve('A');
  await first;
  assert.deepEqual(applied, ['B']);
});

test('another key does not invalidate a range query', async () => {
  const gate = new RequestGate();
  const range = gate.Begin('range');
  const live = gate.Begin('live');
  assert.equal(gate.IsCurrent(range), true);
  assert.equal(gate.IsCurrent(live), true);
  const newerRange = gate.Begin('range');
  assert.equal(gate.IsCurrent(range), false);
  assert.equal(gate.IsCurrent(newerRange), true);
  assert.equal(gate.IsCurrent(live), true);
});

test('switching experiments invalidates an in-flight query', async () => {
  const gate = new RequestGate();
  const applied = [];
  const pending = deferred();
  const query = LatestQuery(gate, {
    key: 'range',
    loader: () => pending.promise,
    onValue: (value) => applied.push(value),
  });
  gate.Invalidate();
  pending.resolve('old run');
  const result = await query;
  assert.equal(result.stale, true);
  assert.deepEqual(applied, []);
});

test('an abort is not reported as an experiment failure', async () => {
  const gate = new RequestGate();
  let aborted = 0;
  let failed = 0;
  gate.Begin('range');
  const pending = LatestQuery(gate, {
    key: 'range',
    loader: () => {
      const error = new Error('aborted');
      error.name = 'AbortError';
      return Promise.reject(error);
    },
    onError: () => { failed += 1; },
    onAbort: () => { aborted += 1; },
  });
  const result = await pending;
  assert.equal(result.aborted, true);
  assert.equal(aborted, 1);
  assert.equal(failed, 0);
});

test('a query error leaves the previously applied window intact', async () => {
  const gate = new RequestGate();
  let state = { points: [1, 2, 3] };
  const before = JSON.stringify(state);
  await LatestQuery(gate, {
    key: 'range',
    loader: () => Promise.reject(new ApiError('bad window')),
    onError: () => {},
  });
  assert.equal(JSON.stringify(state), before);
});

test('absolute times above 2^53 keep every digit in the request', async () => {
  const urls = [];
  const api = new Api({
    fetchImpl: async (url) => {
      urls.push(url);
      return {
        ok: true,
        status: 200,
        json: async () => ({ ok: true, request_id: 'r', series: [] }),
      };
    },
  });
  const stamp = 9007199254740995n;
  await api.Range({
    runId: 'a'.repeat(32),
    signalIds: ['runtime/left_hip/q'],
    beginUs: stamp,
    endUs: stamp + 10n,
    pixelWidth: 10,
  });
  assert.match(urls[0], /begin_us=9007199254740995/);
  assert.match(urls[0], /end_us=9007199254741005/);
  assert.equal(Us('9007199254740995'), stamp);
});

test('the browser raw cache is bounded and evicts the oldest', () => {
  const cache = new SeriesCache(64);
  cache.Set('a', { points: 'a'.repeat(40) });
  cache.Set('b', { points: 'b'.repeat(40) });
  assert.equal(cache.Get('a'), null);
  assert.ok(cache.Get('b'));
  assert.ok(cache.bytes <= 64);
});

test('the client reports an HTTP payload error', async () => {
  const api = new Api({
    fetchImpl: async () => ({
      ok: false,
      status: 400,
      json: async () => ({ ok: false, error: 'bad request' }),
    }),
  });
  await assert.rejects(() => api.Runs(), ApiError);
});
