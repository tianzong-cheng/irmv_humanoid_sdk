#!/usr/bin/env python3
"""tests/integration/policy_package_test.py <source-root>

The SDK half of the ONNX deployment contract: tools/verify_policy.py must accept
a package whose contract agrees with the model it is embedded in, and refuse
every way a package can lie about itself. The models here are synthetic - the
real exported policy is checked by running the same tool against it - so the
test needs no simulator; it asserts the tool's rules, not a model's numbers.

    policy_package_test.py <source-root>
"""

from __future__ import annotations

import json
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np
import onnx

METADATA_KEY = "irmv.deployment"


def make_model(dimension: int, out_dimension: int) -> onnx.ModelProto:
    weight = onnx.helper.make_tensor(
        "W",
        onnx.TensorProto.FLOAT,
        [dimension, out_dimension],
        np.linspace(-1.0, 1.0, dimension * out_dimension).astype(np.float32).tobytes(),
        raw=True,
    )
    node = onnx.helper.make_node("MatMul", ["obs", "W"], ["actions"])
    graph = onnx.helper.make_graph(
        [node],
        "policy",
        [onnx.helper.make_tensor_value_info("obs", onnx.TensorProto.FLOAT, [1, dimension])],
        [onnx.helper.make_tensor_value_info("actions", onnx.TensorProto.FLOAT, [1, out_dimension])],
        [weight],
    )
    model = onnx.helper.make_model(graph, opset_imports=[onnx.helper.make_opsetid("", 18)])
    model.metadata_props.add(key="unrelated", value="kept")
    return model


def write_package(path: Path, document: dict | None, *, entries: int = 1, dimension: int = 18, out_dimension: int = 3):
    model = make_model(dimension, out_dimension)
    if document is not None:
        for _ in range(entries):
            model.metadata_props.add(key=METADATA_KEY, value=json.dumps(document))
    onnx.save(model, str(path))


def write_model_with_moved_weights(path: Path, document: dict) -> None:
    """The same shapes and the same contract, but different weights."""

    model = make_model(18, 3)
    model.graph.initializer[0].CopyFrom(
        onnx.helper.make_tensor(
            "W",
            onnx.TensorProto.FLOAT,
            [18, 3],
            (np.linspace(-1.0, 1.0, 54).astype(np.float32) * 2.0).tobytes(),
            raw=True,
        )
    )
    model.metadata_props.add(key=METADATA_KEY, value=json.dumps(document))
    onnx.save(model, str(path))


def run_verify(source: Path, before: Path, after: Path) -> subprocess.CompletedProcess:
    return subprocess.run(
        [sys.executable, str(source / "tools" / "verify_policy.py"), str(before), str(after), "--quiet"],
        capture_output=True,
        text=True,
        check=False,
        timeout=180,
    )


def main() -> int:
    source_root = Path(sys.argv[1]).resolve()
    fixture = json.loads((source_root / "tests" / "support" / "deployment" / "valid_three_orders.json").read_text())
    failures: list[str] = []

    def expect(note: str, accepted: bool, result: subprocess.CompletedProcess) -> None:
        if accepted and result.returncode != 0:
            failures.append(f"{note}: expected acceptance, got rc={result.returncode}: {result.stderr.strip()}")
        if not accepted and result.returncode == 0:
            failures.append(f"{note}: expected a refusal, but the tool accepted the package")
        if not accepted and result.returncode != 0 and not result.stderr.strip():
            failures.append(f"{note}: refused without a diagnostic")

    with tempfile.TemporaryDirectory() as directory:
        work = Path(directory)
        before = work / "before.onnx"
        after = work / "after.onnx"

        # The package the exporter produces: the same graph, the contract added.
        write_package(before, None)
        write_package(after, fixture)
        expect("the exported package", True, run_verify(source_root, before, after))
        expect("a model that already carries the contract", True, run_verify(source_root, after, after))

        cases = {
            "a model without the contract": (lambda: write_package(after, None)),
            "a model that carries the contract twice": (lambda: write_package(after, fixture, entries=2)),
            "a document of the wrong policy type": (
                lambda: write_package(after, {**fixture, "policy_type": "joint_torque"})
            ),
            "a document claiming another input": (
                lambda: write_package(
                    after, {**fixture, "model": {**fixture["model"], "input": {"name": "obs", "dimension": 15}}}
                )
            ),
            "a document with an unsupported observation frame": (
                lambda: write_package(
                    after,
                    {
                        **fixture,
                        "observation": {
                            **fixture["observation"],
                            "terms": [
                                {**fixture["observation"]["terms"][0], "frame": "world"},
                                *fixture["observation"]["terms"][1:],
                            ],
                        },
                    },
                )
            ),
            "a model whose weights moved": (lambda: write_model_with_moved_weights(after, fixture)),
        }
        for note, mutate in cases.items():
            mutate()
            expect(note, False, run_verify(source_root, before, after))

    for failure in failures:
        print(f"policy_package_test: {failure}", file=sys.stderr)
    if failures:
        return 1
    print(f"policy_package_test: {len(cases) + 2} package checks passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
