#!/usr/bin/env python3
"""Generate the policy-model fixtures the onnx_model_test loads.

    make_test_model.py <output-directory> <deployment-fixture.json>

The models are built from onnx.helper instead of being checked in, so the
repository carries no binary blob and the fixtures cannot drift away from the
frozen contract document: the valid model embeds exactly the deployment fixture
the C++ contract test parses. Every other model differs from it in one way,
which is what makes each of them a single-witness rejection case.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np
import onnx

METADATA_KEY = "irmv.deployment"
INPUT = "obs"
OUTPUT = "actions"


def weights(dimension: int, out_dimension: int, scale: float = 1.0) -> onnx.TensorProto:
    return onnx.helper.make_tensor(
        "W",
        onnx.TensorProto.FLOAT,
        [dimension, out_dimension],
        (np.linspace(-1.0, 1.0, dimension * out_dimension).astype(np.float32) * scale).tobytes(),
        raw=True,
    )


def base_graph(dimension: int, out_dimension: int, input_width: object) -> onnx.GraphProto:
    node = onnx.helper.make_node("MatMul", [INPUT, "W"], [OUTPUT])
    return onnx.helper.make_graph(
        [node],
        "policy",
        [onnx.helper.make_tensor_value_info(INPUT, onnx.TensorProto.FLOAT, [1, input_width])],
        [onnx.helper.make_tensor_value_info(OUTPUT, onnx.TensorProto.FLOAT, [1, out_dimension])],
        [weights(dimension, out_dimension)],
    )


# The pinned ONNX Runtime (1.22.0) supports IR versions up to 10; the onnx
# package that builds these fixtures defaults to a newer one, so it is pinned
# explicitly. The exporter of the training side emits IR 8 for opset 18, which
# tools/verify_policy.py checks against the same limit.
IR_VERSION = 10


def make_model(graph: onnx.GraphProto, document: dict | None, entries: int = 1) -> onnx.ModelProto:
    model = onnx.helper.make_model(graph, opset_imports=[onnx.helper.make_opsetid("", 18)])
    model.ir_version = IR_VERSION
    if document is not None:
        for _ in range(entries):
            model.metadata_props.add(key=METADATA_KEY, value=json.dumps(document, sort_keys=True))
    return model


def save(directory: Path, name: str, model: onnx.ModelProto, check: bool = True) -> None:
    # A deliberately duplicated metadata key is one of the fixtures, and the
    # ONNX checker rejects exactly that - which is also how the SDK's own
    # verification tool detects it - so the checker only runs where the model is
    # meant to be acceptable.
    if check:
        onnx.checker.check_model(model)
    onnx.save(model, str(directory / name))


def main() -> int:
    directory = Path(sys.argv[1])
    fixture = json.loads(Path(sys.argv[2]).read_text(encoding="utf-8"))
    dimension = fixture["model"]["input"]["dimension"]
    out_dimension = fixture["model"]["output"]["dimension"]
    assert fixture["model"]["input"]["name"] == INPUT
    assert fixture["model"]["output"]["name"] == OUTPUT
    directory.mkdir(parents=True, exist_ok=True)

    save(directory, "valid.onnx", make_model(base_graph(dimension, out_dimension, dimension), fixture))
    save(directory, "no_metadata.onnx", make_model(base_graph(dimension, out_dimension, dimension), None))
    save(
        directory,
        "duplicated.onnx",
        make_model(base_graph(dimension, out_dimension, dimension), fixture, entries=2),
        check=False,
    )
    save(
        directory,
        "bad_version.onnx",
        make_model(
            base_graph(dimension, out_dimension, dimension),
            {**fixture, "schema_version": 2},
        ),
    )
    save(
        directory,
        "io_mismatch.onnx",
        make_model(
            base_graph(dimension, out_dimension, dimension),
            {
                **fixture,
                "model": {
                    **fixture["model"],
                    "input": {"name": INPUT, "dimension": dimension - 1},
                },
            },
        ),
    )
    save(
        directory,
        "io_name_mismatch.onnx",
        make_model(
            base_graph(dimension, out_dimension, dimension),
            {
                **fixture,
                "model": {
                    **fixture["model"],
                    "output": {"name": "torques", "dimension": out_dimension},
                },
            },
        ),
    )
    save(
        directory,
        "dynamic_width.onnx",
        make_model(base_graph(dimension, out_dimension, "N"), fixture),
    )
    save(
        directory,
        "unknown_joint.onnx",
        make_model(
            base_graph(dimension, out_dimension, dimension),
            {
                **fixture,
                "action": {**fixture["action"], "joints": ["shoulder", *fixture["action"]["joints"][1:]]},
            },
        ),
    )
    save(
        directory,
        "extra_input.onnx",
        make_model(
            onnx.helper.make_graph(
                [onnx.helper.make_node("MatMul", [INPUT, "W"], [OUTPUT])],
                "policy",
                [
                    onnx.helper.make_tensor_value_info(INPUT, onnx.TensorProto.FLOAT, [1, dimension]),
                    onnx.helper.make_tensor_value_info("bias", onnx.TensorProto.FLOAT, [1, out_dimension]),
                ],
                [onnx.helper.make_tensor_value_info(OUTPUT, onnx.TensorProto.FLOAT, [1, out_dimension])],
                [weights(dimension, out_dimension)],
            ),
            fixture,
        ),
    )

    # An output that is not finite: the graph divides the finite action by zero.
    zero = onnx.helper.make_tensor(
        "Z", onnx.TensorProto.FLOAT, [1, out_dimension], np.zeros(out_dimension, dtype=np.float32).tobytes(), raw=True
    )
    non_finite_graph = onnx.helper.make_graph(
        [
            onnx.helper.make_node("MatMul", [INPUT, "W"], ["finite"]),
            onnx.helper.make_node("Div", ["finite", "Z"], [OUTPUT]),
        ],
        "policy",
        [onnx.helper.make_tensor_value_info(INPUT, onnx.TensorProto.FLOAT, [1, dimension])],
        [onnx.helper.make_tensor_value_info(OUTPUT, onnx.TensorProto.FLOAT, [1, out_dimension])],
        [weights(dimension, out_dimension), zero],
    )
    save(directory, "non_finite.onnx", make_model(non_finite_graph, fixture))

    print(f"make_test_model: {len(list(directory.glob('*.onnx')))} fixtures in {directory}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
