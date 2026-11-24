/**
 * Versioned layout documents: the dock tree, its charts, curve identities and
 * axes. A layout never names a run, an absolute time or a device setting, which
 * is what lets the same document be opened against another experiment.
 */

import { ValidateChart, ChartError } from './chart_model.js';
import { CatalogMap } from './signals.js';

export const MAX_CHARTS = 128;
export const MAX_DOCK_DEPTH = 32;

export class LayoutError extends Error {}

export function EmptyLayout(name = 'Untitled layout') {
  return { version: 1, name, dock: { type: 'empty' }, charts: [] };
}

function ValidateDock(node, depth, chartIds) {
  if (depth > MAX_DOCK_DEPTH) {
    throw new LayoutError('the dock tree is deeper than 32');
  }
  if (!node || typeof node !== 'object') {
    throw new LayoutError('a dock node is not an object');
  }
  if (node.type === 'empty') {
    return;
  }
  if (node.type === 'leaf') {
    if (typeof node.chartId !== 'string' || !chartIds.has(node.chartId)) {
      throw new LayoutError('a dock leaf names no chart');
    }
    return;
  }
  if (node.type !== 'split') {
    throw new LayoutError('a dock node has an unknown type');
  }
  if (node.direction !== 'row' && node.direction !== 'column') {
    throw new LayoutError('a split has an unknown direction');
  }
  if (!Number.isFinite(node.ratio) || node.ratio < 0.1 || node.ratio > 0.9) {
    throw new LayoutError('a split ratio must be between 0.1 and 0.9');
  }
  ValidateDock(node.first, depth + 1, chartIds);
  ValidateDock(node.second, depth + 1, chartIds);
}

export function ValidateLayout(layout) {
  if (!layout || typeof layout !== 'object') {
    throw new LayoutError('the layout is not an object');
  }
  if (layout.version !== 1) {
    throw new LayoutError('the layout version is not 1');
  }
  if (
    typeof layout.name !== 'string' ||
    layout.name.length === 0 ||
    layout.name.length > 200
  ) {
    throw new LayoutError('the layout name is missing or too long');
  }
  if (!Array.isArray(layout.charts)) {
    throw new LayoutError('the layout charts field is not a list');
  }
  if (layout.charts.length > MAX_CHARTS) {
    throw new LayoutError('the layout holds more than 128 charts');
  }
  const chartIds = new Set();
  for (const chart of layout.charts) {
    try {
      ValidateChart(chart);
    } catch (error) {
      if (error instanceof ChartError) {
        throw new LayoutError(error.message);
      }
      throw error;
    }
    if (chartIds.has(chart.id)) {
      throw new LayoutError(`two charts share the id ${chart.id}`);
    }
    chartIds.add(chart.id);
  }
  ValidateDock(layout.dock, 0, chartIds);
}

/** Resolve every curve against a catalog, keeping a placeholder for a miss. */
export function ResolveLayout(layout, catalog) {
  ValidateLayout(layout);
  const signals = CatalogMap(catalog);
  const missingSignals = [];
  const charts = layout.charts.map((chart) => ({
    ...chart,
    autoTitle:
      !chart.title || catalog.some((signal) => signal.label === chart.title),
    curves: chart.curves.map((curve) => {
      const signal = signals.get(curve.signalId) ?? null;
      if (signal === null) {
        missingSignals.push(curve.signalId);
      }
      return { ...curve, signal, missing: signal === null };
    }),
  }));
  return { layout, charts, missingSignals };
}

export { ChartError };
