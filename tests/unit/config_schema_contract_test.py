#!/usr/bin/env python3

import json
import sys
from decimal import Decimal
from pathlib import Path


FORMAT_NAME = "irmv-joint-name"


def accepts_joint_name(value: object) -> bool:
    return isinstance(value, str) and 1 <= len(value.encode("utf-8")) <= 64


def schema_accepts_positive_time(token: str) -> bool:
    value = json.loads(token, parse_float=Decimal, parse_int=Decimal)
    return (
        value.is_finite()
        and value == value.to_integral_value()
        and Decimal(1) <= value <= Decimal(2**64 - 1)
    )


def main() -> int:
    schema = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
    fixture = json.loads(Path(sys.argv[2]).read_text(encoding="utf-8"))

    joint_name = schema["$defs"]["jointName"]
    assert joint_name["type"] == "string"
    assert joint_name["format"] == FORMAT_NAME
    name_property = schema["properties"]["robot"]["properties"]["joints"][
        "items"
    ]["properties"]["name"]
    assert name_property == {"$ref": "#/$defs/jointName"}
    positive_time = schema["$defs"]["positiveTimeUs"]
    assert positive_time == {
        "type": "integer",
        "minimum": 1,
        "maximum": 2**64 - 1,
    }
    rotate_bytes = schema["properties"]["telemetry"]["properties"][
        "rotate_bytes"
    ]
    assert rotate_bytes == {"$ref": "#/$defs/positiveTimeUs"}
    # The schema level set must stay in step with the parser: the full report
    # range, and nothing else.
    assert schema["properties"]["telemetry"]["properties"]["level"] == {
        "enum": ["TRACE", "DEBUG", "INFO", "WARNING", "ERROR", "FATAL"]
    }
    assert fixture["telemetry"]["level"] in schema["properties"]["telemetry"][
        "properties"
    ]["level"]["enum"]

    assert "simulation" in schema["properties"]["deployment"]["enum"]
    assert "sim" in schema["properties"]["backend"]["enum"]
    sim = schema["properties"]["simulation"]
    assert sim["additionalProperties"] is False
    assert sim["properties"]["engine"] == {"const": "mujoco_warp"}
    assert sim["properties"]["physics_dt_us"] == {"$ref": "#/$defs/positiveTimeUs"}
    for key in ("engine_sha256", "asset_sha256"):
        assert sim["properties"][key]["pattern"] == "^[0-9a-f]{64}$"
    for key in ("host_startup_timeout_us", "host_phase_timeout_us"):
        assert sim["properties"][key]["maximum"] == 3600000000
        assert sim["properties"][key]["minimum"] == 1
    assert set(sim["required"]) == set(sim["properties"]) - {
        "host_startup_timeout_us", "host_phase_timeout_us"}

    for key, count in (("initial_base_position", 3), ("initial_base_quaternion", 4)):
        assert sim["properties"][key]["minItems"] == count
        assert sim["properties"][key]["maxItems"] == count
    mapping = sim["properties"]["joint_mapping"]["additionalProperties"]
    assert set(mapping["required"]) == {"joint", "actuator"}
    assert mapping["additionalProperties"] is False
    pairs = {rule["if"]["properties"]["deployment"]["const"]:
             rule["then"] for rule in schema["allOf"]
             if "backend" in rule["then"].get("properties", {})}
    assert pairs["simulation"]["required"] == ["simulation"]
    for deployment, backend in (("fake_only", "fake"), ("real", "real"), ("simulation", "sim")):
        assert pairs[deployment]["properties"]["backend"] == {"const": backend}
    for deployment in ("fake_only", "real"):
        assert pairs[deployment]["not"] == {"required": ["simulation"]}

    assert accepts_joint_name("a" * 64)
    assert not accepts_joint_name("a" * 65)
    assert accepts_joint_name("界" * 21)
    assert not accepts_joint_name("界" * 22)
    assert not accepts_joint_name("")
    assert schema_accepts_positive_time("18446744073709551615.0")
    assert schema_accepts_positive_time("1.8446744073709551615e19")
    assert not schema_accepts_positive_time("1.00000000000000001")
    assert all(
        accepts_joint_name(joint["name"])
        for joint in fixture["robot"]["joints"]
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
