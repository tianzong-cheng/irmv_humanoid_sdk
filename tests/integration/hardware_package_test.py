#!/usr/bin/env python3
"""Packaging contract: dependency pins and a runtime-only configuration.

Usage: hardware_package_test.py <source directory> <cyclonedds prefix>
"""

import json
import subprocess
import sys
import tempfile
from pathlib import Path


def main() -> int:
    source = Path(sys.argv[1]).resolve()
    prefix = Path(sys.argv[2]) if len(sys.argv) > 2 else None

    verified = subprocess.run(
        [sys.executable, str(source / "tools/verify_hardware_dependencies.py"),
         str(source)], text=True, capture_output=True)
    if verified.returncode != 0:
        print("FAIL: dependency pins\n" + verified.stdout + verified.stderr)
        return 1

    with tempfile.TemporaryDirectory(prefix="irmv-package-") as directory:
        build = Path(directory) / "runtime-only"
        configure = ["cmake", "-S", str(source), "-B", str(build),
                     "-DIRMV_BUILD_APPLICATION=OFF",
                     "-DCMAKE_BUILD_TYPE=Debug"]
        if prefix and prefix.is_dir():
            configure.append(f"-DIRMV_CYCLONEDDS_PREFIX={prefix}")
        result = subprocess.run(configure, text=True, capture_output=True)
        if result.returncode != 0:
            print("FAIL: runtime-only configure\n" + result.stderr)
            return 1
        if (build / "onnxruntime-prefix").exists():
            print("FAIL: the runtime-only build fetched ONNX Runtime")
            return 1
        targets = subprocess.run(
            ["cmake", "--build", str(build), "--target", "help"],
            text=True, capture_output=True).stdout.lower()
        for required in ("robot_runtime", "irmv_hardware", "maintenance_test"):
            if required not in targets:
                print("FAIL: runtime-only build lacks %s" % required)
                return 1
        target_lines = {line.strip() for line in targets.splitlines()}
        for forbidden in ("application", "irmv_policy", "verify_policy_sample"):
            if forbidden in target_lines:
                print("FAIL: runtime-only build still has %s" % forbidden)
                return 1

        control = source / "deps/irmv_control"
        if control.is_dir() and (control / "include").is_dir():
            bad = Path(directory) / "bad-pin"
            result = subprocess.run(
                ["cmake", "-S", str(source), "-B", str(bad),
                 "-DIRMV_ENABLE_REAL_BACKEND=ON",
                 "-DIRMV_IRMV_CONTROL_SOURCE_DIR=" + str(control),
                 "-DIRMV_IRMV_CONTROL_COMMIT=deadbeef"],
                text=True, capture_output=True)
            if result.returncode == 0:
                print("FAIL: a mismatched IRMVControl pin configured successfully")
                return 1
            if "pinned commit" not in result.stderr:
                print("FAIL: the pin mismatch is not named\n" + result.stderr)
                return 1
        else:
            print("IRMVControl is not checked out; the commit-mismatch "
                  "configuration check belongs to the real-dependency job")
    print("hardware package contract passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
