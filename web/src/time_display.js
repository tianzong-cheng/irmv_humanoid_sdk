/** Display-only time conversions. Queries and recorded timestamps stay absolute. */
import { Us } from './api.js';

export function RelativeTime(value, originUs) {
  if (
    value === null ||
    value === undefined ||
    originUs === null ||
    originUs === undefined
  ) {
    return '—';
  }
  const delta = Us(value) - Us(originUs);
  const magnitude = delta < 0n ? -delta : delta;
  return `${delta < 0n ? '-' : ''}${magnitude / 1000000n}.${String(magnitude % 1000000n).padStart(6, '0')} s`;
}

export function RelativeTimestampFields(value, originUs) {
  if (Array.isArray(value)) {
    return value.map((item) => RelativeTimestampFields(item, originUs));
  }
  if (!value || typeof value !== 'object') {
    return value;
  }
  return Object.fromEntries(
    Object.entries(value).map(([key, item]) => {
      if (
        /(?:^|_)(?:stamp|time|host|begin|end)_us$/.test(key) &&
        item !== null
      ) {
        try {
          return [
            key,
            originUs === null ? null : String(Us(item) - Us(originUs)),
          ];
        } catch {
          return [key, item];
        }
      }
      return [key, RelativeTimestampFields(item, originUs)];
    }),
  );
}

export function DisplaySignalValue(signal, value, stampUs, originUs) {
  if (signal?.quantity !== 'time' || signal.unit !== 'us') {
    return value;
  }
  if (originUs === null || originUs === undefined) {
    return null;
  }
  // Controller/application time has an exact decimal timestamp alongside its
  // scalar value. Other timestamp scalars already arrive as recorded doubles.
  const raw =
    signal.field === 'stamp_us' && stampUs !== null && stampUs !== undefined
      ? stampUs
      : value;
  try {
    const stamp =
      typeof raw === 'number' && Number.isInteger(raw) ? BigInt(raw) : Us(raw);
    return Number(stamp - Us(originUs));
  } catch {
    return value;
  }
}

export function DisplaySeries(series, originUs) {
  return series.map((entry) =>
    entry.signal?.quantity === 'time' && entry.signal.unit === 'us'
      ? {
          ...entry,
          points: entry.points.map((point) => ({
            ...point,
            v: DisplaySignalValue(entry.signal, point.v, point.t, originUs),
          })),
        }
      : entry,
  );
}
