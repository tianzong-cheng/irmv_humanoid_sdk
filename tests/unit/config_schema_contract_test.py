#!/usr/bin/env python3

import json
import sys
from pathlib import Path


FORMAT_NAME = "irmv-joint-name"


def accepts_joint_name(value: object) -> bool:
    return isinstance(value, str) and 1 <= len(value.encode("utf-8")) <= 64


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

    assert accepts_joint_name("a" * 64)
    assert not accepts_joint_name("a" * 65)
    assert accepts_joint_name("界" * 21)
    assert not accepts_joint_name("界" * 22)
    assert not accepts_joint_name("")
    assert all(
        accepts_joint_name(joint["name"])
        for joint in fixture["robot"]["joints"]
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
