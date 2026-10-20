#!/usr/bin/env python3
"""Verify the real-backend dependency pins.

Checks the two submodule commits and URLs, the vendor library digest and its ELF
architecture. It reads deps/hardware-pins.json; a mismatch is an error, never a
warning.
"""

import hashlib
import json
import struct
import subprocess
import sys
from pathlib import Path


def fail(message: str) -> None:
    print("FAIL: " + message, file=sys.stderr)
    raise SystemExit(1)


def commit_of(path: Path) -> str:
    result = subprocess.run(
        ["git", "-C", str(path), "rev-parse", "HEAD"],
        text=True, capture_output=True)
    if result.returncode != 0:
        fail(f"{path} is not a git checkout")
    return result.stdout.strip()


def main() -> int:
    source = Path(sys.argv[1]).resolve()
    pins = json.loads((source / "deps/hardware-pins.json").read_text())
    modules = {}
    for line in (source / ".gitmodules").read_text().splitlines():
        if line.strip().startswith("path ="):
            modules[len(modules)] = line.split("=", 1)[1].strip()
    module_paths = {}
    current = None
    for line in (source / ".gitmodules").read_text().splitlines():
        stripped = line.strip()
        if stripped.startswith("[submodule"):
            current = {}
        elif stripped.startswith("path =") and current is not None:
            current["path"] = stripped.split("=", 1)[1].strip()
        elif stripped.startswith("url =") and current is not None:
            current["url"] = stripped.split("=", 1)[1].strip()
            module_paths[current["path"]] = current

    for name in ("irmv_control", "yis_imu_driver"):
        pin = pins[name]
        path = source / pin["path"]
        if commit_of(path) != pin["commit"]:
            fail(f"{name} is not at {pin['commit']}")
        module = module_paths.get(pin["path"])
        if module is None or module.get("url") != pin["url"]:
            fail(f"{name} submodule URL does not match the pin")

    vendor = source / pins["vendor_library"]["path"]
    digest = hashlib.sha256(vendor.read_bytes()).hexdigest()
    if digest != pins["vendor_library"]["sha256"]:
        fail("vendor library SHA-256 does not match the pin")
    header = vendor.read_bytes()[:20]
    if header[:4] != b"\x7fELF":
        fail("vendor library is not ELF")
    machine = struct.unpack("<H", header[18:20])[0]
    if machine != pins["vendor_library"]["elf_machine"]:
        fail("vendor library architecture does not match the pin")
    print("hardware dependency pins verified")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
