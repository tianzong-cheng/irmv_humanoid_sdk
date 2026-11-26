/**
 * HTTP client and request generation rules of the workbench.
 *
 * Absolute times and sequences are BigInt in the UI. They are serialized with
 * `toString()` and never through Number, so a value above 2^53 keeps every
 * digit. A new query for a key aborts the older one; a response from a stale
 * generation is ignored rather than allowed to overwrite the current window.
 */

export class ApiError extends Error {
  constructor(message, options = {}) {
    super(message);
    this.name = 'ApiError';
    this.status = options.status ?? 0;
    this.payload = options.payload ?? null;
  }
}

export class TimeError extends Error {
  constructor(message) {
    super(message);
    this.name = 'TimeError';
  }
}

export function Us(value) {
  if (typeof value === 'bigint') {
    return value;
  }
  if (typeof value === 'number') {
    if (!Number.isSafeInteger(value)) {
      throw new TimeError('a time given as a Number is not a safe integer');
    }
    return BigInt(value);
  }
  if (typeof value === 'string' && /^[0-9]+$/.test(value)) {
    return BigInt(value);
  }
  throw new TimeError(`not a decimal time: ${String(value)}`);
}

export function UsText(value) {
  return Us(value).toString();
}

function AppendParams(params, values) {
  for (const [key, value] of Object.entries(values)) {
    if (value === undefined || value === null) {
      continue;
    }
    if (Array.isArray(value)) {
      if (value.length > 0) {
        params.set(key, value.join(','));
      }
      continue;
    }
    params.set(
      key,
      typeof value === 'bigint' ? value.toString() : String(value),
    );
  }
}

export class RequestGate {
  constructor() {
    this.generations = new Map();
    this.controllers = new Map();
  }

  Begin(key = 'default') {
    const generation = (this.generations.get(key) ?? 0) + 1;
    this.generations.set(key, generation);
    const previous = this.controllers.get(key);
    if (previous) {
      previous.abort();
    }
    const controller = new AbortController();
    this.controllers.set(key, controller);
    return { key, generation, signal: controller.signal };
  }

  IsCurrent(token) {
    return this.generations.get(token.key) === token.generation;
  }

  Invalidate() {
    for (const key of new Set([
      ...this.generations.keys(),
      ...this.controllers.keys(),
    ])) {
      this.generations.set(key, (this.generations.get(key) ?? 0) + 1);
      this.controllers.get(key)?.abort();
    }
    this.controllers.clear();
  }
}

/** Run a loader and apply its result only if its generation is still current. */
export async function LatestQuery(gate, options) {
  const token = gate.Begin(options.key ?? 'default');
  try {
    const value = await options.loader(token.signal);
    if (!gate.IsCurrent(token)) {
      options.onStale?.();
      return { applied: false, stale: true };
    }
    options.onValue?.(value);
    return { applied: true, value };
  } catch (error) {
    if (error?.name === 'AbortError') {
      options.onAbort?.(error);
      return { aborted: true, stale: !gate.IsCurrent(token) };
    }
    if (!gate.IsCurrent(token)) {
      options.onStale?.();
      return { applied: false, stale: true };
    }
    options.onError?.(error);
    return { applied: false, error };
  }
}

export class SeriesCache {
  constructor(limitBytes = 32 * 1024 * 1024) {
    this.limitBytes = limitBytes;
    this.bytes = 0;
    this.entries = new Map();
  }

  Get(key) {
    const entry = this.entries.get(key);
    if (!entry) {
      return null;
    }
    this.entries.delete(key);
    this.entries.set(key, entry);
    return entry.value;
  }

  Set(key, value) {
    const size = new TextEncoder().encode(JSON.stringify(value)).length;
    if (size > this.limitBytes) {
      return;
    }
    const previous = this.entries.get(key);
    if (previous) {
      this.bytes -= previous.bytes;
      this.entries.delete(key);
    }
    this.entries.set(key, { value, bytes: size });
    this.bytes += size;
    while (this.bytes > this.limitBytes && this.entries.size > 0) {
      const oldest = this.entries.keys().next().value;
      const entry = this.entries.get(oldest);
      this.entries.delete(oldest);
      this.bytes -= entry.bytes;
    }
  }

  Clear() {
    this.entries.clear();
    this.bytes = 0;
  }
}

export class Api {
  constructor(options = {}) {
    this.base = options.base ?? '';
    this.fetchImpl = options.fetchImpl ?? globalThis.fetch?.bind(globalThis);
  }

  async _request(path, { method = 'GET', params = {}, body, signal } = {}) {
    const url = new URL(
      `${this.base}${path}`,
      globalThis.location?.origin ?? 'http://127.0.0.1',
    );
    const search = new URLSearchParams();
    AppendParams(search, params);
    for (const [key, value] of search) {
      url.searchParams.set(key, value);
    }
    const headers = {};
    if (body !== undefined) {
      headers['Content-Type'] = 'application/json';
    }
    let response;
    try {
      response = await this.fetchImpl(url.toString(), {
        method,
        credentials: 'same-origin',
        headers,
        body: body === undefined ? undefined : JSON.stringify(body),
        signal,
      });
    } catch (error) {
      if (error?.name === 'AbortError') {
        throw error;
      }
      throw new ApiError(error?.message ?? 'the service is unreachable');
    }
    let payload = null;
    try {
      payload = await response.json();
    } catch (error) {
      if (error?.name === 'AbortError') {
        throw error;
      }
      throw new ApiError('the service answered without JSON', {
        status: response.status,
      });
    }
    if (!response.ok || payload?.ok === false) {
      throw new ApiError(payload?.error ?? `HTTP ${response.status}`, {
        status: response.status,
        payload,
      });
    }
    return payload;
  }

  Runs(params = {}, signal) {
    return this._request('/api/runs', { params, signal });
  }

  Describe(runId, signal, params) {
    return this._request(`/api/runs/${encodeURIComponent(runId)}/describe`, {
      params,
      signal,
    });
  }

  Context(runId, appRunId, signal, params) {
    return this._request(`/api/runs/${encodeURIComponent(runId)}/context`, {
      params: { ...params, app_run_id: appRunId },
      signal,
    });
  }

  Range(query, { signal } = {}) {
    return this._request(`/api/runs/${encodeURIComponent(query.runId)}/range`, {
      params: {
        app_run_id: query.appRunId,
        signal_ids: query.signalIds,
        begin_us:
          query.beginUs === undefined ? undefined : UsText(query.beginUs),
        end_us: query.endUs === undefined ? undefined : UsText(query.endUs),
        pixel_width: query.pixelWidth,
      },
      signal,
    });
  }

  Cursor(query, { signal } = {}) {
    return this._request(
      `/api/runs/${encodeURIComponent(query.runId)}/cursor`,
      {
        params: {
          app_run_id: query.appRunId,
          signal_ids: query.signalIds,
          cursor_us: UsText(query.cursorUs),
        },
        signal,
      },
    );
  }

  Events(query, { signal } = {}) {
    return this._request(
      `/api/runs/${encodeURIComponent(query.runId)}/events`,
      {
        params: {
          begin_us:
            query.beginUs === undefined ? undefined : UsText(query.beginUs),
          end_us: query.endUs === undefined ? undefined : UsText(query.endUs),
          max_rows: query.maxRows,
          continuation: query.continuation,
        },
        signal,
      },
    );
  }

  LiveSnapshot(query, { signal } = {}) {
    return this._request('/api/live/snapshot', {
      params: {
        run_id: query.runId,
        signal_ids: query.signalIds,
        begin_us:
          query.beginUs === undefined ? undefined : UsText(query.beginUs),
        end_us: query.endUs === undefined ? undefined : UsText(query.endUs),
        pixel_width: query.pixelWidth,
        after_revision: query.afterRevision,
      },
      signal,
    });
  }

  async Layouts(params = {}, signal) {
    return this._request('/api/layouts', { params, signal });
  }

  Layout(id, signal) {
    return this._request(`/api/layouts/${encodeURIComponent(id)}`, { signal });
  }

  SaveLayout(layout, id = null, signal) {
    if (id === null) {
      return this._request('/api/layouts', {
        method: 'POST',
        body: layout,
        signal,
      });
    }
    return this._request(`/api/layouts/${encodeURIComponent(id)}`, {
      method: 'PUT',
      body: layout,
      signal,
    });
  }

  DeleteLayout(id, signal) {
    return this._request(`/api/layouts/${encodeURIComponent(id)}`, {
      method: 'DELETE',
      signal,
    });
  }
}
