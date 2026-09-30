#!/usr/bin/env python3
"""The small JSON Schema subset validator behind the deployment contract checks.

schema/onnx-deployment-v1.schema.json uses a deliberately narrow set of
keywords, and both the unit test that pins the schema to the fixtures and
tools/verify_policy.py need to evaluate exactly those. One implementation keeps
the two from drifting apart. The supported keywords are: $ref into $defs, type
(including a two element ["string", "null"] form), const, enum, required,
properties, additionalProperties, items, prefixItems, allOf, oneOf, minItems,
maxItems, minProperties, minimum, maximum, exclusiveMinimum, minLength,
maxLength and pattern.
"""

from __future__ import annotations

import json
import re
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




def load_schema(path: str | Path) -> dict:
    return json.loads(Path(path).read_text(encoding="utf-8"))


def validate_file(schema_path: str | Path, document: object, label: str = "document") -> None:
    """Validates one parsed document against a schema file."""

    schema = load_schema(schema_path)
    validate(schema, schema, document, label)


def load_policy_document(value: str) -> dict:
    """Parses the metadata value, refusing anything that is not a JSON object."""

    try:
        document = json.loads(value)
    except json.JSONDecodeError as error:
        raise SchemaError(f"the metadata value is not valid JSON: {error}") from error
    if not isinstance(document, dict):
        raise SchemaError("the metadata value is not a JSON object")
    return document
