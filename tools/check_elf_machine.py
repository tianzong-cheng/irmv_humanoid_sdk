#!/usr/bin/env python3
"""Fail unless an ELF file's e_machine matches the expected value."""

import struct
import sys
from pathlib import Path


def main() -> int:
    path = Path(sys.argv[1])
    expected = int(sys.argv[2])
    header = path.read_bytes()[:20]
    if len(header) < 20 or header[:4] != b"\x7fELF":
        print(f"{path} is not an ELF file", file=sys.stderr)
        return 1
    machine = struct.unpack("<H", header[18:20])[0]
    if machine != expected:
        print(f"{path} has ELF machine {machine}, expected {expected}",
              file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
