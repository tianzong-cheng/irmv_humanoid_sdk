"""Versioned, atomically stored workbench layouts.

A layout names a dock tree, its charts, curve signal identities and axes. It
never names a run, an absolute time or a device setting, which is what lets the
same layout be loaded in another experiment. The store validates the whole
document before it replaces the previous version, and it writes through a
same-directory temporary file so a crash leaves the old version readable.
"""

from __future__ import annotations

import json
import math
import os
import re
import uuid
from pathlib import Path

from .catalog import CatalogError, paginate


class LayoutError(ValueError):
    """A layout document the workbench refuses."""


_LAYOUT_ID = re.compile(r"^[0-9a-f]{32}$")
_MAX_CHARTS = 128
_MAX_CURVES_PER_CHART = 128
_MAX_DOCK_DEPTH = 32


def new_layout_id() -> str:
    return uuid.uuid4().hex


def _finite(value: object) -> bool:
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(value)
    )


def _validate_axis(axis: object) -> None:
    if not isinstance(axis, dict):
        raise LayoutError("an axis is not an object")
    axis_id = axis.get("id")
    if not isinstance(axis_id, str) or not axis_id:
        raise LayoutError("an axis has no id")
    for key in ("quantity", "unit"):
        if key in axis and not isinstance(axis[key], str):
            raise LayoutError(f"an axis {key} is not a string")
    if "range" in axis and axis["range"] is not None:
        value_range = axis["range"]
        if not isinstance(value_range, list) or len(value_range) != 2:
            raise LayoutError("an axis range is not a pair")
        low, high = value_range
        if not _finite(low) or not _finite(high) or not low < high:
            raise LayoutError("an axis range must be finite and increasing")


def _validate_chart(chart: object, chart_ids: set[str]) -> None:
    if not isinstance(chart, dict):
        raise LayoutError("a chart is not an object")
    chart_id = chart.get("id")
    if not isinstance(chart_id, str) or not chart_id:
        raise LayoutError("a chart has no id")
    if chart_id in chart_ids:
        raise LayoutError(f"two charts share the id {chart_id}")
    chart_ids.add(chart_id)
    if "title" in chart and not isinstance(chart["title"], str):
        raise LayoutError("a chart title is not a string")
    curves = chart.get("curves", [])
    if not isinstance(curves, list):
        raise LayoutError("a chart curves field is not a list")
    if len(curves) > _MAX_CURVES_PER_CHART:
        raise LayoutError("a chart holds more than 128 curves")
    for curve in curves:
        if not isinstance(curve, dict):
            raise LayoutError("a curve is not an object")
        signal_id = curve.get("signalId")
        if not isinstance(signal_id, str) or not signal_id:
            raise LayoutError("a curve has no signalId")
        if "axisId" in curve and not isinstance(curve["axisId"], str):
            raise LayoutError("a curve axisId is not a string")
        if "style" in curve and not isinstance(curve["style"], str):
            raise LayoutError("a curve style is not a string")
    axes = chart.get("axes", [])
    if not isinstance(axes, list):
        raise LayoutError("a chart axes field is not a list")
    axis_ids: set[str] = set()
    for axis in axes:
        _validate_axis(axis)
        axis_id = axis["id"]
        if axis_id in axis_ids:
            raise LayoutError(f"two axes share the id {axis_id}")
        axis_ids.add(axis_id)


def _validate_dock(node: object, depth: int, chart_ids: set[str]) -> None:
    if depth > _MAX_DOCK_DEPTH:
        raise LayoutError("the dock tree is deeper than 32")
    if not isinstance(node, dict):
        raise LayoutError("a dock node is not an object")
    node_type = node.get("type")
    if node_type == "empty":
        return
    if node_type == "leaf":
        chart_id = node.get("chartId")
        if not isinstance(chart_id, str) or chart_id not in chart_ids:
            raise LayoutError("a dock leaf names no chart")
        return
    if node_type != "split":
        raise LayoutError("a dock node has an unknown type")
    if node.get("direction") not in {"row", "column"}:
        raise LayoutError("a split has an unknown direction")
    ratio = node.get("ratio")
    if not _finite(ratio) or not 0.1 <= float(ratio) <= 0.9:
        raise LayoutError("a split ratio must be between 0.1 and 0.9")
    _validate_dock(node.get("first"), depth + 1, chart_ids)
    _validate_dock(node.get("second"), depth + 1, chart_ids)


def validate_layout(document: object) -> None:
    if not isinstance(document, dict):
        raise LayoutError("the layout is not an object")
    if document.get("version") != 1:
        raise LayoutError("the layout version is not 1")
    name = document.get("name")
    if not isinstance(name, str) or not name or len(name) > 200:
        raise LayoutError("the layout name is missing or too long")
    charts = document.get("charts")
    if not isinstance(charts, list):
        raise LayoutError("the layout charts field is not a list")
    if len(charts) > _MAX_CHARTS:
        raise LayoutError("the layout holds more than 128 charts")
    chart_ids: set[str] = set()
    for chart in charts:
        _validate_chart(chart, chart_ids)
    if "dock" not in document:
        raise LayoutError("the layout has no dock tree")
    _validate_dock(document["dock"], 0, chart_ids)


class LayoutStore:
    def __init__(self, data_dir: Path | str) -> None:
        self.data_dir = Path(data_dir)
        self.directory = self.data_dir / "layouts"
        self.directory.mkdir(parents=True, exist_ok=True)

    def _path(self, layout_id: str) -> Path:
        if not isinstance(layout_id, str) or not _LAYOUT_ID.fullmatch(layout_id):
            raise LayoutError("the layout id is not 32 lower-case hex digits")
        return self.directory / f"{layout_id}.json"

    def _read(self, layout_id: str) -> dict | None:
        path = self._path(layout_id)
        try:
            with path.open("r", encoding="utf-8") as handle:
                document = json.load(handle)
        except FileNotFoundError:
            return None
        except (OSError, json.JSONDecodeError) as error:
            raise LayoutError(f"the saved layout is not readable: {error}") from error
        if not isinstance(document, dict):
            raise LayoutError("the saved layout is not an object")
        return document

    def get(self, layout_id: str) -> dict | None:
        document = self._read(layout_id)
        if document is None:
            return None
        result = dict(document)
        result["id"] = layout_id
        return result

    def list_page(
        self, continuation: str | None, limit: int
    ) -> tuple[list, str | None]:
        entries = []
        for path in sorted(self.directory.glob("*.json")):
            layout_id = path.stem
            if not _LAYOUT_ID.fullmatch(layout_id):
                continue
            try:
                document = self._read(layout_id)
            except LayoutError:
                document = None
            if document is None:
                continue
            entries.append(
                {
                    "id": layout_id,
                    "name": document.get("name", ""),
                    "version": document.get("version"),
                }
            )
        return paginate(entries, continuation, limit)

    def _write_atomic(self, path: Path, document: dict) -> None:
        temporary = path.with_name(f".{path.name}.{uuid.uuid4().hex}.tmp")
        try:
            with temporary.open("w", encoding="utf-8") as handle:
                json.dump(document, handle, separators=(",", ":"), ensure_ascii=False)
                handle.flush()
                os.fsync(handle.fileno())
            os.replace(temporary, path)
        finally:
            try:
                temporary.unlink()
            except FileNotFoundError:
                pass

    def save(self, document: dict, layout_id: str | None = None) -> str:
        validate_layout(document)
        chosen = layout_id or new_layout_id()
        path = self._path(chosen)
        stored = dict(document)
        stored.pop("id", None)
        self._write_atomic(path, stored)
        return chosen

    def delete(self, layout_id: str) -> bool:
        path = self._path(layout_id)
        try:
            path.unlink()
        except FileNotFoundError:
            return False
        return True


__all__ = [
    "CatalogError",
    "LayoutError",
    "LayoutStore",
    "new_layout_id",
    "validate_layout",
]
