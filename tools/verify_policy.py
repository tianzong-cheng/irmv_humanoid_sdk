#!/usr/bin/env python3
"""Check a policy package: the deployment contract, the ONNX, and the numerics.

    verify_policy.py <before.onnx> <after.onnx> [--schema <path>]

``after.onnx`` is the model that carries the ``irmv.deployment`` document;
``before.onnx`` is the model it was derived from, which may carry the key or
not. The tool asserts that

  * both models pass the ONNX checker,
  * ``after`` carries exactly one ``irmv.deployment`` entry, that its value is a
    JSON object conforming to the frozen schema, and that the document's declared
    model interface agrees with the graph it is embedded in,
  * ``before`` and ``after`` describe the same computation - same nodes, same
    initializers, same value info - so adding the contract changed no weight and
    no shape,
  * ONNX Runtime reads the very same document back out of the saved file, and
  * both sessions produce bit-identical outputs for the same fixed input.

What it does not do: it never writes a model, never publishes a JointCommand and
never touches DDS. It is the offline gate between the training export and the
SDK, and it is the only place a policy package is checked numerically.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
from pathlib import Path

import numpy as np
import onnx
import onnxruntime as ort

sys.path.insert(0, str(Path(__file__).resolve().parent))

from deployment_contract import SchemaError, load_policy_document, validate_file  # noqa: E402

METADATA_KEY = "irmv.deployment"
DEFAULT_SCHEMA = Path(__file__).resolve().parents[1] / "schema" / "onnx-deployment-v1.schema.json"

# The highest ONNX IR version the pinned ONNX Runtime (see
# cmake/Dependencies.cmake, IRMV_ONNXRUNTIME_VERSION) can load. A model saved by
# a newer onnx package is refused here, where the reason is legible, instead of
# failing at startup on the machine that has to run it.
MAX_IR_VERSION = 10


class VerificationError(Exception):
    """A policy package that must not be shipped."""


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def deployment_entries(model: onnx.ModelProto) -> list[onnx.StringStringEntryProto]:
    return [entry for entry in model.metadata_props if entry.key == METADATA_KEY]


def check_model_file(path: Path) -> onnx.ModelProto:
    model = onnx.load(str(path))
    onnx.checker.check_model(model)
    if model.ir_version > MAX_IR_VERSION:
        raise VerificationError(
            f"{path}: the model declares ONNX IR version {model.ir_version}, above the {MAX_IR_VERSION} the pinned"
            " ONNX Runtime loads"
        )
    if not model.graph.input:
        raise VerificationError(f"{path}: the model has no input")
    if not model.graph.output:
        raise VerificationError(f"{path}: the model has no output")
    return model


def tensor_shape(value_info: onnx.ValueInfoProto) -> tuple[str, int]:
    dims = value_info.type.tensor_type.shape.dim
    if len(dims) != 2 or dims[0].dim_value != 1:
        raise VerificationError(
            f"the tensor {value_info.name!r} must be a fixed [1, N] tensor, got {len(dims)} dimensions"
        )
    if dims[1].dim_value < 1:
        raise VerificationError(f"the tensor {value_info.name!r} has a dynamic width")
    return value_info.name, dims[1].dim_value


def read_document(model: onnx.ModelProto, schema_path: Path) -> dict:
    entries = deployment_entries(model)
    if not entries:
        raise VerificationError(f"the model carries no {METADATA_KEY} entry")
    if len(entries) > 1:
        raise VerificationError(f"the model carries {len(entries)} {METADATA_KEY} entries")
    try:
        document = load_policy_document(entries[0].value)
        validate_file(schema_path, document, METADATA_KEY)
    except SchemaError as error:
        raise VerificationError(f"the {METADATA_KEY} document is not conforming: {error}") from error

    input_name, input_dimension = tensor_shape(model.graph.input[0])
    output_name, output_dimension = tensor_shape(model.graph.output[0])
    declared_input = document["model"]["input"]
    declared_output = document["model"]["output"]
    if (declared_input["name"], declared_input["dimension"]) != (input_name, input_dimension):
        raise VerificationError(
            "the document declares the input "
            f"{declared_input['name']!r}[{declared_input['dimension']}] but the graph has {input_name!r}[{input_dimension}]"
        )
    if (declared_output["name"], declared_output["dimension"]) != (output_name, output_dimension):
        raise VerificationError(
            "the document declares the output "
            f"{declared_output['name']!r}[{declared_output['dimension']}] but the graph has"
            f" {output_name!r}[{output_dimension}]"
        )
    return document


def compare_computation(before: onnx.ModelProto, after: onnx.ModelProto) -> None:
    """Only the contract may differ; the computation itself may not move."""

    if before.graph.SerializeToString() != after.graph.SerializeToString():
        raise VerificationError("the two models do not describe the same graph")
    before_ops = [(node.op_type, tuple(node.input), tuple(node.output)) for node in before.graph.node]
    after_ops = [(node.op_type, tuple(node.input), tuple(node.output)) for node in after.graph.node]
    if before_ops != after_ops:
        raise VerificationError("the two models do not contain the same operators")
    if [initializer.name for initializer in before.graph.initializer] != [
        initializer.name for initializer in after.graph.initializer
    ]:
        raise VerificationError("the two models do not contain the same weights")


def fixed_input(input_name: str, dimension: int) -> np.ndarray:
    # A deterministic, non-constant input: a naive implementation that ignores
    # its input would still have to agree with itself, but a wrong shape or a
    # swapped axis shows up here.
    index = np.arange(dimension, dtype=np.float32)
    values = np.sin(0.37 * index) * 0.5 + np.cos(0.11 * index) * 0.25
    return values.reshape(1, dimension).astype(np.float32)


def run(model_path: Path, input_name: str, values: np.ndarray) -> np.ndarray:
    options = ort.SessionOptions()
    options.intra_op_num_threads = 1
    options.inter_op_num_threads = 1
    options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_DISABLE_ALL
    session = ort.InferenceSession(str(model_path), sess_options=options, providers=["CPUExecutionProvider"])
    outputs = session.run(None, {input_name: values})
    if len(outputs) != 1:
        raise VerificationError(f"{model_path}: expected exactly one output, got {len(outputs)}")
    return outputs[0]


def verify(before_path: Path, after_path: Path, schema_path: Path) -> dict:
    before = check_model_file(before_path)
    after = check_model_file(after_path)
    document = read_document(after, schema_path)
    compare_computation(before, after)

    entries = deployment_entries(after)
    try:
        from_file = json.loads(entries[0].value)
    except json.JSONDecodeError as error:  # already parsed above; kept for the readback comparison
        raise VerificationError(f"the {METADATA_KEY} value is not valid JSON: {error}") from error
    if from_file != document:
        raise VerificationError("the document read back from the file differs from the one validated")

    input_name, input_dimension = tensor_shape(after.graph.input[0])
    values = fixed_input(input_name, input_dimension)
    reference = run(before_path, input_name, values)
    candidate = run(after_path, input_name, values)
    if reference.shape != candidate.shape:
        raise VerificationError(f"the output shape changed: {reference.shape} -> {candidate.shape}")
    if not np.array_equal(reference, candidate):
        difference = float(np.max(np.abs(reference - candidate)))
        raise VerificationError(f"the outputs differ after the metadata write (max |delta| = {difference})")

    # The runtime's own view of the file, not the file's protobuf: the SDK reads
    # the contract through this interface.
    options = ort.SessionOptions()
    options.intra_op_num_threads = 1
    options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_DISABLE_ALL
    session = ort.InferenceSession(str(after_path), sess_options=options, providers=["CPUExecutionProvider"])
    readback = session.get_modelmeta().custom_metadata_map
    if readback.get(METADATA_KEY) != entries[0].value:
        raise VerificationError("ONNX Runtime does not return the document the file stores")

    return {
        "input": f"{input_name}[{input_dimension}]",
        "output": f"{document['model']['output']['name']}[{document['model']['output']['dimension']}]",
        "action_joints": len(document["action"]["joints"]),
        "observation_dimension": sum(term["dimension"] for term in document["observation"]["terms"]),
        "policy_period_us": document["policy_period_us"],
        "normalizer": document["normalizer"]["location"],
        "missing": document["source"]["missing"],
        "sha256": sha256_file(after_path),
        "output_sample": [float(value) for value in candidate[0][:4]],
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("before", type=Path, help="the model the contract was written into")
    parser.add_argument("after", type=Path, help="the model carrying the irmv.deployment contract")
    parser.add_argument("--schema", type=Path, default=DEFAULT_SCHEMA, help="the frozen contract schema")
    parser.add_argument("--quiet", action="store_true", help="print only the failures")
    args = parser.parse_args(argv)

    try:
        summary = verify(args.before, args.after, args.schema)
    except (VerificationError, onnx.checker.ValidationError) as error:
        print(f"verify_policy: FAILED: {error}", file=sys.stderr)
        return 1
    if not args.quiet:
        print(f"verify_policy: {args.after} carries {METADATA_KEY} and is unchanged numerically")
        print(f"  model    {summary['input']} -> {summary['output']}")
        print(f"  contract {summary['action_joints']} action joints, {summary['observation_dimension']} observation"
              f" components, {summary['policy_period_us']} us period, normalizer {summary['normalizer']}")
        print(f"  output   {summary['output_sample']}")
        if summary["missing"]:
            print(f"  missing  {', '.join(summary['missing'])}")
        print(f"  sha256   {summary['sha256']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
