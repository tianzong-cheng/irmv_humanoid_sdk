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


def base_graph(dimension: int, out_dimension: int, input_width: object, weight_scale: float = 1.0) -> onnx.GraphProto:
    node = onnx.helper.make_node("MatMul", [INPUT, "W"], [OUTPUT])
    return onnx.helper.make_graph(
        [node],
        "policy",
        [onnx.helper.make_tensor_value_info(INPUT, onnx.TensorProto.FLOAT, [1, input_width])],
        [onnx.helper.make_tensor_value_info(OUTPUT, onnx.TensorProto.FLOAT, [1, out_dimension])],
        [weights(dimension, out_dimension, weight_scale)],
    )


def typed_graph(
    dimension: int,
    out_dimension: int,
    input_type: int,
    output_type: int,
) -> onnx.GraphProto:
    """A model whose shapes and metadata are valid but whose element types differ.

    The admission check must refuse it before inference; reading a DOUBLE,
    FLOAT16 or integer tensor through GetTensorData<float>() would produce a
    successful action with the wrong numbers.
    """
    numpy_type = {
        onnx.TensorProto.DOUBLE: np.float64,
        onnx.TensorProto.FLOAT16: np.float16,
    }.get(input_type, np.float32)
    values = (np.linspace(-1.0, 1.0, dimension * out_dimension).astype(numpy_type)).tobytes()
    weight = onnx.helper.make_tensor(
        "W", input_type, [dimension, out_dimension], values, raw=True
    )
    matmul_output = "raw"
    nodes = [onnx.helper.make_node("MatMul", [INPUT, "W"], [matmul_output])]
    if output_type != input_type:
        nodes.append(onnx.helper.make_node("Cast", [matmul_output], [OUTPUT], to=output_type))
    else:
        nodes.append(onnx.helper.make_node("Identity", [matmul_output], [OUTPUT]))
    return onnx.helper.make_graph(
        nodes,
        "policy",
        [onnx.helper.make_tensor_value_info(INPUT, input_type, [1, dimension])],
        [onnx.helper.make_tensor_value_info(OUTPUT, output_type, [1, out_dimension])],
        [weight],
    )


def long_running_graph(dimension: int, out_dimension: int, iterations: int = 200_000_000) -> onnx.GraphProto:
    """A valid model whose Loop body keeps running until the executor terminates.

    The body is a real loop-carried add, not something constant folding can
    remove: a shutdown that merely waits for a join would block for the whole
    trip count.
    """
    weight = weights(dimension, out_dimension)
    trip = onnx.helper.make_tensor(
        "M", onnx.TensorProto.INT64, [], np.array([iterations], dtype=np.int64).tobytes(), raw=True
    )
    enter = onnx.helper.make_tensor(
        "COND", onnx.TensorProto.BOOL, [], np.array([True], dtype=np.bool_).tobytes(), raw=True
    )
    step = onnx.helper.make_tensor(
        "STEP", onnx.TensorProto.FLOAT, [1, out_dimension],
        np.full((1, out_dimension), 1e-9, dtype=np.float32).tobytes(), raw=True,
    )

    body = onnx.helper.make_graph(
        [
            onnx.helper.make_node("Identity", ["cond_in"], ["cond_out"]),
            onnx.helper.make_node("Add", ["acc_in", "STEP"], ["acc_out"]),
        ],
        "loop_body",
        [
            onnx.helper.make_tensor_value_info("iter", onnx.TensorProto.INT64, []),
            onnx.helper.make_tensor_value_info("cond_in", onnx.TensorProto.BOOL, []),
            onnx.helper.make_tensor_value_info("acc_in", onnx.TensorProto.FLOAT, [1, out_dimension]),
        ],
        [
            onnx.helper.make_tensor_value_info("cond_out", onnx.TensorProto.BOOL, []),
            onnx.helper.make_tensor_value_info("acc_out", onnx.TensorProto.FLOAT, [1, out_dimension]),
        ],
    )
    graph = onnx.helper.make_graph(
        [
            onnx.helper.make_node("MatMul", [INPUT, "W"], ["initial"]),
            onnx.helper.make_node("Loop", ["M", "COND", "initial"], [OUTPUT], body=body),
        ],
        "policy",
        [onnx.helper.make_tensor_value_info(INPUT, onnx.TensorProto.FLOAT, [1, dimension])],
        [onnx.helper.make_tensor_value_info(OUTPUT, onnx.TensorProto.FLOAT, [1, out_dimension])],
        [weight, trip, enter, step],
    )
    return graph


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


FAKE_JOINTS = tuple(f"fake_joint_{index:02d}" for index in range(1, 13))


def fake_document() -> dict:
    """A complete deployment document for a twelve-joint fake deployment.

    It is built here rather than taken from a fixture because the integration
    acceptance needs a model whose joint table is exactly the one the generated
    configuration configures; the values are nominal test values and say nothing
    about a machine.
    """

    joints = list(FAKE_JOINTS)
    joint_count = len(joints)

    def zeros(count: int = joint_count) -> list[float]:
        return [0.0] * count

    def terms() -> list[dict]:
        def term(name: str, dimension: int, unit: str, frame: str) -> dict:
            return {
                "name": name,
                "dimension": dimension,
                "unit": unit,
                "frame": frame,
                "scale": None,
                "clip": None,
                "history_length": 0,
            }

        return [
            term("command", 3, "m/s,m/s,rad/s", "body"),
            term("projected_gravity", 3, "1", "body"),
            term("base_ang_vel", 3, "rad/s", "body"),
            term("joint_pos_rel", joint_count, "rad", "joint"),
            term("joint_vel_rel", joint_count, "rad/s", "joint"),
            term("last_action", joint_count, "1", "none"),
        ]

    return {
        "schema_version": 1,
        "policy_type": "joint_position",
        "model": {
            "input": {"name": INPUT, "dimension": 9 + 3 * joint_count},
            "output": {"name": OUTPUT, "dimension": joint_count},
        },
        "policy_period_us": 20000,
        "action": {
            "joints": joints,
            "source_joints": joints,
            "default_q": zeros(),
            "default_dq": zeros(),
            "kp": [50.0] * joint_count,
            "kd": [2.0] * joint_count,
            "scale": [0.5] * joint_count,
            "offset": zeros(),
            "clip": {"wrapper": None, "term": None},
            "order": ["wrapper_clip", "scale", "offset", "term_clip"],
            "last_action": {"stage": "raw", "initial": zeros()},
        },
        "observation": {
            "terms": terms(),
            "joint_pos_rel": {
                "term": "joint_pos_rel",
                "joints": joints,
                "source_joints": joints,
                "reference": zeros(),
                "unit": "rad",
            },
            "joint_vel_rel": {
                "term": "joint_vel_rel",
                "joints": joints,
                "source_joints": joints,
                "reference": zeros(),
                "unit": "rad/s",
            },
        },
        "normalizer": {"enabled": False, "embedded": False, "location": "none"},
        "source": {
            "checkpoint_file": None,
            "checkpoint_sha256": None,
            "env_config_file": None,
            "env_config_sha256": None,
            "agent_config_file": None,
            "agent_config_sha256": None,
            "training_repo": None,
            "training_commit": None,
            "asset_revision": None,
            "software": {},
            "missing": [
                "checkpoint_file",
                "checkpoint_sha256",
                "env_config_file",
                "env_config_sha256",
                "agent_config_file",
                "agent_config_sha256",
                "training_repo",
                "training_commit",
                "asset_revision",
            ],
        },
    }


def write_fake_deployment(directory: Path, config_path: Path, domain: int) -> None:
    """Writes a self-contained local-policy deployment: a model and its config.

    The configuration is the shipped Fake one with every joint-keyed map
    rewritten onto the twelve joints the generated model commands, so the
    runtime, the model and the application agree. The values stay the
    demonstration values of the fake deployment.
    """

    document = fake_document()
    dimension = document["model"]["input"]["dimension"]
    out_dimension = document["model"]["output"]["dimension"]
    directory.mkdir(parents=True, exist_ok=True)
    model_path = (directory / "policy.onnx").resolve()
    # The weights are small on purpose: the first frame this model produces has
    # to be a continuous target from the pose the initialization left the joints
    # in, and the runtime refuses a jump beyond runtime.continuity.position_delta
    # - which is a safety boundary this generator must respect, never relax.
    save(
        directory,
        "policy.onnx",
        make_model(base_graph(dimension, out_dimension, dimension, weight_scale=0.01), document),
    )

    config = json.loads(config_path.read_text(encoding="utf-8"))
    joints = list(FAKE_JOINTS)
    config["robot"]["joints"] = [
        {"name": name, "position_min": -2, "position_max": 2} for name in joints
    ]

    def spread(mapping: dict) -> dict:
        value = next(iter(mapping.values()))
        return {name: value for name in joints}

    config["runtime"]["damping"]["kd"] = spread(config["runtime"]["damping"]["kd"])
    for key in ("joint_velocity_limits", "torque_limits", "target_velocity_limits"):
        config["runtime"]["safety"][key] = spread(config["runtime"]["safety"][key])
    for key in ("position_delta", "velocity_delta", "torque_delta"):
        config["runtime"]["continuity"][key] = spread(config["runtime"]["continuity"][key])
    config["application"]["initialization"]["target_position"] = {name: 0.0 for name in joints}
    gain = next(iter(config["application"]["initialization"]["gains"].values()))
    config["application"]["initialization"]["gains"] = {name: dict(gain) for name in joints}
    config["application"]["policy"]["model_path"] = str(model_path)
    config["application"]["policy"]["inference_timeout_us"] = 100000
    config["dds"]["domain"] = domain
    (directory / "sdk.json").write_text(json.dumps(config, indent=2) + "\n", encoding="utf-8")

    # The Fake deployment's feedback has to cover the same twelve joints, and
    # the IMU has to be valid: the local policy's observation contains the
    # projected gravity and the base angular velocity, so a deployment that
    # never reports an IMU snapshot cannot produce one.
    scenario = {
        "entries": [
            {
                "elapsed_us": 0,
                "feedback": {
                    "count": len(joints),
                    "joints": [
                        {
                            "q": 0.0,
                            "dq": 0.0,
                            "tau_est": 0.0,
                            "valid": True,
                            "connected": True,
                            "freq_hz": 500.0,
                            "error_code": 0,
                        }
                        for _ in joints
                    ],
                    "imu": {"valid": True, "rx_stamp_us": 0},
                },
                "write_result": {"accepted": True},
            }
        ]
    }
    (directory / "fake_scenario.json").write_text(json.dumps(scenario, indent=2) + "\n", encoding="utf-8")


def main() -> int:
    if sys.argv[1] == "--fake-deployment":
        write_fake_deployment(Path(sys.argv[2]), Path(sys.argv[3]), int(sys.argv[4]))
        print(f"make_test_model: fake deployment in {sys.argv[2]}")
        return 0
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

    # A valid model whose inference only ends when the executor is cancelled.
    save(
        directory,
        "long_running.onnx",
        make_model(long_running_graph(dimension, out_dimension), fixture),
    )

    # Element types the inference path cannot feed or read. Every other part of
    # these models - metadata, names, ranks and widths - is the valid case.
    for name, input_type, output_type in (
        ("double_input.onnx", onnx.TensorProto.DOUBLE, onnx.TensorProto.DOUBLE),
        ("double_output.onnx", onnx.TensorProto.FLOAT, onnx.TensorProto.DOUBLE),
        ("float16_output.onnx", onnx.TensorProto.FLOAT, onnx.TensorProto.FLOAT16),
        ("int32_output.onnx", onnx.TensorProto.FLOAT, onnx.TensorProto.INT32),
        ("int64_output.onnx", onnx.TensorProto.FLOAT, onnx.TensorProto.INT64),
    ):
        save(
            directory,
            name,
            make_model(typed_graph(dimension, out_dimension, input_type, output_type), fixture),
        )

    print(f"make_test_model: {len(list(directory.glob('*.onnx')))} fixtures in {directory}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
