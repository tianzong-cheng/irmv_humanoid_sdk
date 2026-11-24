#!/usr/bin/env python3
"""Layout documents, atomic replacement and path rules of the web service.

These checks need no socket and no SDK process, so they are a core test. The
HTTP behaviour is exercised by experiment_web_server_test.py.
"""

from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

SOURCE_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(SOURCE_ROOT))

from tools.experiment_web import layout_store  # noqa: E402
from tools.experiment_web.catalog import (  # noqa: E402
    CatalogError,
    canonical_records_root,
    paginate,
    resolve_run_directory,
)
from tools.experiment_web.layout_store import (  # noqa: E402
    LayoutError,
    LayoutStore,
    validate_layout,
)


def layout(name: str = "compare", ratio: float = 0.5) -> dict:
    return {
        "version": 1,
        "name": name,
        "dock": {
            "type": "split",
            "direction": "row",
            "ratio": ratio,
            "first": {"type": "leaf", "chartId": "c1"},
            "second": {"type": "leaf", "chartId": "c2"},
        },
        "charts": [
            {
                "id": "c1",
                "title": "q",
                "curves": [
                    {"signalId": "runtime/left_hip/q", "axisId": "a1"},
                    {"signalId": "runtime/left_hip/measured_q", "axisId": "a1"},
                ],
                "axes": [{"id": "a1", "quantity": "joint_position", "unit": "rad"}],
            },
            {
                "id": "c2",
                "title": "dq",
                "curves": [{"signalId": "runtime/left_hip/dq", "axisId": "a2"}],
                "axes": [
                    {
                        "id": "a2",
                        "quantity": "joint_velocity",
                        "unit": "rad_s",
                        "range": [-1.0, 1.0],
                    }
                ],
            },
        ],
    }


class LayoutDocumentTest(unittest.TestCase):
    def test_accepts_the_shared_contract(self) -> None:
        validate_layout(layout())

    def test_refuses_unknown_version(self) -> None:
        document = layout()
        document["version"] = 2
        with self.assertRaises(LayoutError):
            validate_layout(document)

    def test_refuses_bad_ratio_and_range(self) -> None:
        document = layout(ratio=0.95)
        with self.assertRaises(LayoutError):
            validate_layout(document)
        document = layout()
        document["charts"][1]["axes"][0]["range"] = [1.0, 1.0]
        with self.assertRaises(LayoutError):
            validate_layout(document)

    def test_refuses_dangling_leaf_and_too_deep_tree(self) -> None:
        document = layout()
        document["dock"]["first"]["chartId"] = "missing"
        with self.assertRaises(LayoutError):
            validate_layout(document)
        node: dict = {"type": "leaf", "chartId": "c1"}
        for _ in range(40):
            node = {
                "type": "split",
                "direction": "column",
                "ratio": 0.5,
                "first": {"type": "empty"},
                "second": node,
            }
        document = layout()
        document["dock"] = node
        with self.assertRaises(LayoutError):
            validate_layout(document)

    def test_refuses_more_than_128_charts(self) -> None:
        document = layout()
        document["charts"] = [{"id": f"c{index}", "curves": []} for index in range(129)]
        with self.assertRaises(LayoutError):
            validate_layout(document)


class LayoutStoreTest(unittest.TestCase):
    def test_round_trip_and_delete(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            store = LayoutStore(directory)
            document = layout()
            saved_id = store.save(document)
            loaded = store.get(saved_id)
            self.assertIsNotNone(loaded)
            self.assertEqual(loaded["dock"], document["dock"])
            self.assertEqual(loaded["version"], 1)
            page, continuation = store.list_page(None, 100)
            self.assertEqual(len(page), 1)
            self.assertIsNone(continuation)
            self.assertTrue(store.delete(saved_id))
            self.assertIsNone(store.get(saved_id))
            self.assertFalse(store.delete(saved_id))

    def test_interrupted_replace_keeps_the_previous_version(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            store = LayoutStore(directory)
            document = layout("first")
            saved_id = store.save(document)
            document["name"] = "second"
            with mock.patch.object(
                layout_store.os, "replace", side_effect=OSError("interrupted")
            ):
                with self.assertRaises(OSError):
                    store.save(document, saved_id)
            previous = store.get(saved_id)
            self.assertIsNotNone(previous)
            self.assertEqual(previous["name"], "first")
            self.assertEqual(list(store.directory.glob("*.tmp")), [])

    def test_layout_id_is_not_a_path(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            store = LayoutStore(directory)
            with self.assertRaises(LayoutError):
                store.save(layout(), "../../escape")
            with self.assertRaises(LayoutError):
                store.get("../../escape")


class CatalogPathTest(unittest.TestCase):
    def test_pagination_is_bounded(self) -> None:
        items = list(range(5))
        page, continuation = paginate(items, None, 2)
        self.assertEqual(page, [0, 1])
        self.assertEqual(continuation, "2")
        page, continuation = paginate(items, continuation, 2)
        self.assertEqual(page, [2, 3])
        self.assertEqual(continuation, "4")
        page, continuation = paginate(items, continuation, 2)
        self.assertEqual(page, [4])
        self.assertIsNone(continuation)

    def test_run_selector_stays_inside_the_root(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory) / "records"
            root.mkdir()
            (root / "run_a").mkdir()
            resolved = canonical_records_root(root)
            self.assertEqual(resolve_run_directory(resolved, "run_a"), root / "run_a")
            for selector in ("..", "../etc", "/etc", "a/b", ""):
                with self.assertRaises(CatalogError):
                    resolve_run_directory(resolved, selector)

    def test_symlink_outside_the_root_is_refused(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            base = Path(directory)
            root = base / "records"
            root.mkdir()
            outside = base / "outside"
            outside.mkdir()
            (outside / "run_escape").mkdir()
            (root / "run_link").symlink_to(outside / "run_escape")
            resolved = canonical_records_root(root)
            with self.assertRaises(CatalogError):
                resolve_run_directory(resolved, "run_link")


if __name__ == "__main__":
    unittest.main(verbosity=2)
