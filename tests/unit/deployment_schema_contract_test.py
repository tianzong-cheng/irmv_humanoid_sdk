#!/usr/bin/env python3
"""Keep the frozen deployment schema and its fixtures in step.

The C++ deployment_test proves that irmv::policy::ParseDeployment accepts the
two fixtures and refuses the mutated documents; this check proves the same
fixtures against schema/onnx-deployment-v1.schema.json, so a parser change that
is not written down in the schema, or a schema change the parser does not
implement, fails here. The validation itself lives in
tools/deployment_contract.py, which tools/verify_policy.py uses as well.
"""

import copy
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))

from deployment_contract import SchemaError, resolve, validate  # noqa: E402


def main() -> int:
    schema_path, fixtures_dir = sys.argv[1], Path(sys.argv[2])
    schema = json.loads(Path(schema_path).read_text(encoding="utf-8"))

    # The frozen facts the parser implements. They are spelled out so a schema
    # edit cannot quietly change what the contract means.
    assert schema["additionalProperties"] is False
    assert schema["required"] == [
        "schema_version",
        "policy_type",
        "model",
        "policy_period_us",
        "action",
        "observation",
        "normalizer",
        "source",
    ]
    assert schema["properties"]["schema_version"] == {"const": 1}
    assert schema["properties"]["policy_type"]["const"] == "joint_position"
    order = schema["properties"]["action"]["properties"]["order"]
    assert order["type"] == "array"
    assert (order["minItems"], order["maxItems"]) == (4, 4)
    assert order["items"] == {
        "enum": ["wrapper_clip", "scale", "offset", "term_clip"]
    }
    terms = schema["properties"]["observation"]["properties"]["terms"]
    assert terms["minItems"] == terms["maxItems"] == 6
    assert terms["items"] is False
    frozen_terms = [
        ("command", "m/s,m/s,rad/s", "body"),
        ("projected_gravity", "1", "body"),
        ("base_ang_vel", "rad/s", "body"),
        ("joint_pos_rel", "rad", "joint"),
        ("joint_vel_rel", "rad/s", "joint"),
        ("last_action", "1", "none"),
    ]
    assert len(terms["prefixItems"]) == len(frozen_terms)
    for (name, unit, frame), reference in zip(frozen_terms, terms["prefixItems"]):
        term = resolve(schema, reference)
        assert term["properties"]["name"]["const"] == name
        assert term["properties"]["unit"]["const"] == unit
        assert term["properties"]["frame"]["const"] == frame
    shape = schema["$defs"]["observationTermShape"]
    assert shape["additionalProperties"] is False
    assert shape["properties"]["frame"]["enum"] == ["body", "joint", "none"]
    assert shape["properties"]["history_length"]["maximum"] == 1
    assert schema["$defs"]["jointName"]["format"] == "irmv-joint-name"
    assert schema["$defs"]["gainArray"]["items"]["minimum"] == 0
    positive = resolve(schema, schema["$defs"]["positiveArray"]["items"])
    assert positive["exclusiveMinimum"] == 0
    assert schema["$defs"]["nullableDigest"] == {
        "type": ["string", "null"],
        "pattern": "^[0-9a-f]{64}$",
    }
    assert (
        schema["properties"]["source"]["properties"]["missing"]["type"] == "array"
    )

    # The fixtures the C++ test parses must satisfy the schema, both as they
    # are and with one planted witness of the required set: the same document
    # that is valid with a field present must not validate without it.
    for name in ("valid_two_axis.json", "valid_three_orders.json"):
        document = json.loads((fixtures_dir / name).read_text(encoding="utf-8"))
        validate(schema, schema, document, name)
        for field in schema["required"]:
            planted = copy.deepcopy(document)
            del planted[field]
            try:
                validate(schema, schema, planted, f"{name} without {field}")
            except SchemaError:
                pass
            else:
                raise SchemaError(f"{name}: the schema accepts a document without {field}")
        # The null provenance fields of the three-order fixture must be exactly
        # the ones its missing list explains.
        nulls = sorted(
            field
            for field, value in document["source"].items()
            if value is None
        )
        assert nulls == sorted(document["source"]["missing"]), (name, nulls)

    # A field the schema does not declare is refused, which is what keeps the
    # parser's RejectUnknownKeys and the schema's additionalProperties in step.
    document = json.loads(
        (fixtures_dir / "valid_two_axis.json").read_text(encoding="utf-8")
    )
    document["undeclared"] = 1
    try:
        validate(schema, schema, document, "unknown field")
    except SchemaError:
        pass
    else:
        raise SchemaError("the schema accepts an undeclared root field")

    # ... and so is a wrong value where the schema pins one.
    for mutate in (
        lambda d: d.__setitem__("policy_type", "joint_torque"),
        lambda d: d.__setitem__("schema_version", 2),
        lambda d: d["action"]["order"].__setitem__(0, "clamp"),
        lambda d: d["observation"]["terms"][0].__setitem__("unit", "deg"),
        lambda d: d["observation"]["terms"][2].__setitem__("frame", "world"),
    ):
        document = json.loads(
            (fixtures_dir / "valid_two_axis.json").read_text(encoding="utf-8")
        )
        mutate(document)
        try:
            validate(schema, schema, document, "pinned value")
        except SchemaError:
            pass
        else:
            raise SchemaError("the schema accepts a value it pins differently")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
