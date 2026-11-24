#!/usr/bin/env python3
"""Deterministic record fixtures for the real-browser workbench tests.

Writes runtime chunks with the schema-v4 layout, an application JSONL stream, a
second runtime run with different joint names, a corrupted chunk and an empty
fault stream. Nothing here is used by the product; it only gives the browser a
stable experiment to open.
"""

from __future__ import annotations

import argparse
import json
import math
import struct
import zlib
from pathlib import Path

BASE_US = 9007199254740993
STEP_US = 10_000
JOINTS = ["left_hip", "right_hip", "left_knee", "right_knee"]
BOOT_ID = "web-boot"
RUNTIME_ID = "11" * 16
SESSION_ID = "22" * 16


def _chunk(rows: list[dict], joint_count: int) -> bytes:
    columns = [
        "q",
        "dq",
        "tau_ff",
        "kp",
        "kd",
        "measured_q",
        "measured_dq",
        "output_torque",
        "measured_tau_est",
    ]
    payload = bytearray()
    for row in rows:
        flags = row["flags"]
        payload += struct.pack(
            "<QQIIIQ7B16s6Q9dQ",
            row["stamp_us"],
            row["state_seq"],
            row["execution_state"],
            row["output_action"],
            row["stop_reason"],
            row["fault_id"],
            flags[0],
            flags[1],
            flags[2],
            flags[3],
            flags[4],
            flags[5],
            flags[6],
            bytes.fromhex("33" * 16),
            row["epoch"],
            row["command_seq"],
            row["source_state_seq"],
            row["command_rx_us"],
            row["feedback_seq"],
            row["feedback_stamp_us"],
            row["imu"][0],
            row["imu"][1],
            row["imu"][2],
            row["imu"][3],
            row["imu"][4],
            row["imu"][5],
            row["imu"][6],
            row["imu"][7],
            row["imu"][8],
            row["imu_rx_us"],
        )
    for column in columns:
        for row in rows:
            for joint in range(joint_count):
                payload += struct.pack("<d", row[column][joint])
    for row in rows:
        for joint in range(joint_count):
            payload += bytes([1 if row["joint_valid"][joint] else 0])
    header = struct.pack(
        "<8sIIIIQQQQII",
        b"IRMVCH01",
        4,
        joint_count,
        len(rows),
        len(payload),
        rows[0]["stamp_us"],
        rows[-1]["stamp_us"],
        rows[0]["state_seq"],
        rows[-1]["state_seq"],
        zlib.crc32(payload) & 0xFFFFFFFF,
        0,
    )
    header_crc = zlib.crc32(header[:60]) & 0xFFFFFFFF
    header = header[:60] + struct.pack("<I", header_crc)
    return header + bytes(payload)


def _runtime_row(index: int, joints: int, base: int = BASE_US) -> dict:
    phase = index / 24.0
    q = [0.25 * math.sin(phase + joint) for joint in range(joints)]
    measured = [value + 0.01 * math.cos(phase) for value in q]
    dq = [math.cos(phase + joint) for joint in range(joints)]
    return {
        "stamp_us": base + index * STEP_US,
        "state_seq": 1000 + index,
        "execution_state": 3,
        "output_action": 2,
        "stop_reason": 0,
        "fault_id": index // 100,
        "flags": (1, 1, 1, 1, 1, 1, 1),
        "epoch": 1,
        "command_seq": index,
        "source_state_seq": index,
        "command_rx_us": base + index * STEP_US + 5,
        "feedback_seq": index,
        "feedback_stamp_us": base + index * STEP_US - 100,
        "imu": [0.01, 0.02, 0.03, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6],
        "imu_rx_us": base + index * STEP_US + 7,
        "joint_valid": [True] * joints,
        "q": q,
        "dq": dq,
        "tau_ff": [0.05 * math.sin(phase) for _ in range(joints)],
        "kp": [40.0] * joints,
        "kd": [2.0] * joints,
        "measured_q": measured,
        "measured_dq": dq,
        "measured_tau_est": [0.02 * math.cos(phase + joint) for joint in range(joints)],
        "output_torque": [0.15 * math.sin(phase + joint) for joint in range(joints)],
    }


def _runtime_meta(joints: list[str]) -> dict:
    return {
        "schema_version": 4,
        "config_id": "a" * 64,
        "layout_id": "b" * 64,
        "backend": "fake",
        "control_hz": 500.0,
        "sample_hz": 100,
        "joint_count": len(joints),
        "joints": joints,
        "sdk_git_sha": "web-browser-fixture",
        "clock": {
            "clock": "linux_clock_monotonic",
            "boot_id": BOOT_ID,
            "available": True,
        },
        "units": {
            "position": "rad",
            "velocity": "rad_s",
            "torque": "Nm",
            "kp": "Nm_per_rad",
            "kd": "Nm_s_per_rad",
            "time": "us",
            "imu_orientation_rpy": "rad",
            "imu_angular_velocity": "rad_s",
            "imu_linear_acceleration": "m_s2",
        },
        "imu": {
            "orientation_rpy": {
                "order": "roll,pitch,yaw",
                "unit": "rad",
                "axis_frame": "unknown",
            }
        },
    }


def _write_fault(directory: Path, seq: int, detected: int, message: str) -> None:
    raw = message.encode("utf-8")
    line = {
        "schema_version": 1,
        "record_seq": seq,
        "runtime_id": RUNTIME_ID,
        "state_seq": f"{1000 + seq:016x}",
        "fault_id": f"{seq:016x}",
        "reason": 5,
        "reason_name": "JointPositionExceeded",
        "condition": 8,
        "condition_name": "JointPositionExceeded",
        "joint_index": 0,
        "error_code": 0xDEAD0000 + seq,
        "observed_value": "NaN" if seq == 0 else 1.25,
        "limit": 1.0,
        "sample_stamp_us": detected - 10,
        "detected_stamp_us": detected,
        "output_action": 0,
        "message": message,
        "message_hex": raw.hex(),
    }
    with (directory / "faults.jsonl").open("a", encoding="utf-8") as handle:
        handle.write(json.dumps(line, separators=(",", ":")) + "\n")


def write_runtime(
    directory: Path,
    name: str,
    joints: list[str],
    rows: int,
    base: int = BASE_US,
    faults: bool = True,
) -> Path:
    path = directory / name
    path.mkdir(parents=True, exist_ok=True)
    samples = [_runtime_row(index, len(joints), base) for index in range(rows)]
    chunks = []
    for start in range(0, rows, 900):
        subset = samples[start : start + 900]
        filename = f"chunk_{start // 900:06d}.bin"
        (path / filename).write_bytes(_chunk(subset, len(joints)))
        chunks.append(
            {
                "file": filename,
                "rows": len(subset),
                "first_stamp_us": subset[0]["stamp_us"],
                "last_stamp_us": subset[-1]["stamp_us"],
                "first_state_seq": subset[0]["state_seq"],
                "last_state_seq": subset[-1]["state_seq"],
            }
        )
    (path / "meta.json").write_text(json.dumps(_runtime_meta(joints)), encoding="utf-8")
    (path / "index.json").write_text(
        json.dumps(
            {
                "schema_version": 4,
                "complete": True,
                "joint_count": len(joints),
                "rows_recorded": rows,
                "chunks": chunks,
                "gaps": [],
                "write_failures": 0,
                "faults": {
                    "complete": True,
                    "recorded": 3 if faults else 0,
                    "written": 3 if faults else 0,
                    "queue_dropped": 0,
                    "generation_dropped": 0,
                    "write_failures": 0,
                    "losses": [],
                },
            }
        ),
        encoding="utf-8",
    )
    (path / "sessions.json").write_text(
        json.dumps(
            [
                {
                    "schema_version": 4,
                    "runtime_id": RUNTIME_ID,
                    "session_id": SESSION_ID,
                    "producer_id": "33" * 16,
                    "epoch": 1,
                    "stamp_us": base,
                    "state_seq": 1000,
                    "joint_count": len(joints),
                    "kp": [40.0] * len(joints),
                    "kd": [2.0] * len(joints),
                    "model_sha256": "unknown",
                }
            ]
        ),
        encoding="utf-8",
    )
    (path / "faults.jsonl").write_text("", encoding="utf-8")
    if faults:
        _write_fault(path, 0, base + 1000, "<script>window.__xss=1</script>")
        _write_fault(path, 1, base + 1000, "<b>same tick</b>")
        _write_fault(path, 2, base + 5000, "later event")
    return path


def write_application(
    directory: Path, name: str, rows: int, base: int = BASE_US
) -> Path:
    path = directory / name
    path.mkdir(parents=True, exist_ok=True)
    meta = {
        "schema": "irmv.application_velocity",
        "version": 1,
        "sdk_git_sha": "web-browser-fixture",
        "application_config_id": "c" * 64,
        "config_id": "a" * 64,
        "layout_id": "b" * 64,
        "joint_count": 4,
        "sample_hz": 100,
        "loop_period_us": 10000,
        "sampling": {"paced_by_deadline": False},
        "source": "local",
        "clock": {
            "clock": "linux_clock_monotonic",
            "boot_id": BOOT_ID,
            "available": True,
        },
        "units": {"forward": "m_s", "lateral": "m_s", "yaw": "rad_s", "time": "us"},
        "model": {
            "status": "loaded",
            "name": "policy.onnx",
            "sha256": "d" * 64,
            "contract_sha256": "e" * 64,
        },
    }
    (path / "meta.json").write_text(json.dumps(meta), encoding="utf-8")
    (path / "index.json").write_text(
        json.dumps(
            {
                "schema": "irmv.application_velocity",
                "version": 1,
                "complete": True,
                "losses": [],
            }
        ),
        encoding="utf-8",
    )
    lines = []
    for index in range(rows):
        stamp = base + index * STEP_US
        lines.append(
            json.dumps(
                {
                    "schema": "irmv.application_velocity",
                    "version": 1,
                    "host_us": stamp,
                    "stamp_us": stamp,
                    "sample_seq": f"{index:016x}",
                    "identity_known": True,
                    "runtime_id": RUNTIME_ID,
                    "session_id": SESSION_ID,
                    "latest_state_seq": f"{1000 + index:016x}",
                    "forward": 0.4 + 0.2 * math.sin(index / 30.0),
                    "lateral": -0.15,
                    "yaw": 0.2 * math.cos(index / 45.0),
                },
                separators=(",", ":"),
            )
            + "\n"
        )
    (path / "velocity.jsonl").write_text("".join(lines), encoding="utf-8")
    return path


def write_corrupt(directory: Path, name: str) -> Path:
    path = write_runtime(directory, name, ["left_hip", "right_hip"], 20)
    chunk = next(path.glob("chunk_*.bin"))
    data = bytearray(chunk.read_bytes())
    data[-1] ^= 0x5A
    chunk.write_bytes(data)
    return path


def build(root: Path) -> None:
    if root.exists():
        for child in sorted(root.iterdir()):
            if child.is_dir():
                import shutil

                shutil.rmtree(child)
            else:
                child.unlink()
    root.mkdir(parents=True, exist_ok=True)
    write_runtime(root, "run_web_a", JOINTS, 1200)
    write_runtime(root, "run_web_b", ["waist"], 20, base=BASE_US + 50_000)
    write_application(root, "app_run_web", 1200)
    foreign = write_application(root, "app_run_foreign", 20)
    meta = json.loads((foreign / "meta.json").read_text(encoding="utf-8"))
    meta["clock"]["boot_id"] = "another-boot"
    (foreign / "meta.json").write_text(json.dumps(meta), encoding="utf-8")
    write_corrupt(root, "run_web_corrupt")
    (root / "README.txt").write_text("generated fixtures\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", required=True)
    args = parser.parse_args()
    build(Path(args.root))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
