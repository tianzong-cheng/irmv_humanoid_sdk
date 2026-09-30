#!/usr/bin/env python3
"""Keep the frozen deployment schema and its fixtures in step.

The C++ deployment_test proves that irmv::policy::ParseDeployment accepts the
two fixtures and refuses the mutated documents; this check proves the same
fixtures against schema/onnx-deployment-v1.schema.json, so a parser change that
is not written down in the schema, or a schema change the parser does not
implement, fails here. It is a deliberately small subset validator: it covers
exactly the keywords the schema uses (type, const, enum, required,
properties, additionalProperties, items, minItems, maxItems, oneOf, $ref,
minimum, maximum, exclusiveMinimum, pattern) and nothing else.
"""

import copy
import json
import re
import sys
from pathlib import Path


class SchemaError(AssertionError):
    pass


def resolve(root: dict, node: dict) -> dict:
    seen = 0
    while isinstance(node, dict) and "$ref" in node:
        target = node["$ref"].split("/")[-1]
        node = root["$defs"][target]
        seen += 1
        if seen > 8:
            raise SchemaError("cyclic $ref")
    return node


def validate(root: dict, node: dict, instance: object, path: str) -> None:
    node = resolve(root, node)

    if "allOf" in node:
        for branch in node["allOf"]:
            validate(root, branch, instance, path)

    if "oneOf" in node:
        failures = []
        for branch in node["oneOf"]:
            try:
                validate(root, branch, instance, path)
                return
            except SchemaError as error:
                failures.append(str(error))
        raise SchemaError(f"{path}: no oneOf branch accepted the value ({failures})")

    if "const" in node and instance != node["const"]:
        raise SchemaError(f"{path}: expected constant {node['const']!r}, got {instance!r}")
    if "enum" in node and instance not in node["enum"]:
        raise SchemaError(f"{path}: {instance!r} is not one of {node['enum']}")

    declared_type = node.get("type")
    if declared_type == "null" and instance is not None:
        raise SchemaError(f"{path}: expected null, got {instance!r}")
    if declared_type == "object":
        if not isinstance(instance, dict):
            raise SchemaError(f"{path}: expected an object")
    elif declared_type == "array":
        if not isinstance(instance, list):
            raise SchemaError(f"{path}: expected an array")
    elif declared_type == "string":
        if not isinstance(instance, str):
            raise SchemaError(f"{path}: expected a string")
    elif declared_type == "boolean":
        if not isinstance(instance, bool):
            raise SchemaError(f"{path}: expected a boolean")
    elif declared_type in ("number", "integer"):
        if isinstance(instance, bool) or not isinstance(instance, (int, float)):
            raise SchemaError(f"{path}: expected a number")
        if declared_type == "integer" and not isinstance(instance, int):
            raise SchemaError(f"{path}: expected an integer")
    elif isinstance(declared_type, list):
        if declared_type == ["string", "null"] and not (
            instance is None or isinstance(instance, str)
        ):
            raise SchemaError(f"{path}: expected a string or null")

    if isinstance(instance, dict):
        for name in node.get("required", []):
            if name not in instance:
                raise SchemaError(f"{path}: missing required field {name!r}")
        properties = node.get("properties", {})
        for name, value in instance.items():
            child = f"{path}.{name}" if path else name
            if name in properties:
                validate(root, properties[name], value, child)
            elif node.get("additionalProperties") is False:
                raise SchemaError(f"{child}: unknown field")
            elif isinstance(node.get("additionalProperties"), dict):
                validate(root, node["additionalProperties"], value, child)
        if "minProperties" in node and len(instance) < node["minProperties"]:
            raise SchemaError(f"{path}: too few properties")

    if isinstance(instance, list) and "prefixItems" in node:
        if node.get("items") is False and len(instance) > len(node["prefixItems"]):
            raise SchemaError(f"{path}: more items than the contract allows")
        for index, sub in enumerate(node["prefixItems"]):
            if index < len(instance):
                validate(root, sub, instance[index], f"{path}[{index}]")

    if isinstance(instance, list) and isinstance(node.get("items"), dict):
        if len(instance) < node.get("minItems", 0):
            raise SchemaError(f"{path}: too few items")
        if "maxItems" in node and len(instance) > node["maxItems"]:
            raise SchemaError(f"{path}: too many items")
        for index, value in enumerate(instance):
            validate(root, node["items"], value, f"{path}[{index}]")

    if isinstance(instance, (int, float)) and not isinstance(instance, bool):
        if "minimum" in node and instance < node["minimum"]:
            raise SchemaError(f"{path}: below minimum")
        if "maximum" in node and instance > node["maximum"]:
            raise SchemaError(f"{path}: above maximum")
        if "exclusiveMinimum" in node and instance <= node["exclusiveMinimum"]:
            raise SchemaError(f"{path}: not above exclusiveMinimum")

    if isinstance(instance, str):
        if "minLength" in node and len(instance) < node["minLength"]:
            raise SchemaError(f"{path}: too short")
        if "maxLength" in node and len(instance) > node["maxLength"]:
            raise SchemaError(f"{path}: too long")
        if "pattern" in node and re.search(node["pattern"], instance) is None:
            raise SchemaError(f"{path}: does not match {node['pattern']}")


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
