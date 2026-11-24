#!/usr/bin/env python3
"""Benchmark the read-only bridge and the local service on a long run.

Generates a 64-joint, 100 Hz, 30-minute runtime record once (the file is large
and lives under the gitignored build tree), then measures:

* the first window of six charts and 24 curves through ``web_record_query``;
* the same window through the HTTP service;
* the resident memory of the bridge before and after the window;
* source-sample to service-snapshot latency for the live tail.

Browser paint is not measured here: it needs a browser process whose clock
origin differs. The workbench's browser acceptance measures the visible state
and the live refresh; this script reports the service-side latency only.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import socket
import struct
import subprocess
import sys
import time
import urllib.request
import zlib
from http.cookiejar import CookieJar
from pathlib import Path

SOURCE = Path(__file__).resolve().parents[1]
BASE_US = 1_000_000
STEP_US = 10_000
ROWS_PER_CHUNK = 900


def _runtime_row(index: int, joints: int) -> dict:
    phase = index / 1800.0 * math.pi
    q = [0.4 * math.sin(phase + joint * 0.1) for joint in range(joints)]
    measured = [value + 0.005 * math.cos(phase) for value in q]
    dq = [math.cos(phase + joint * 0.1) for joint in range(joints)]
    return {
        "stamp_us": BASE_US + index * STEP_US,
        "state_seq": 1 + index,
        "q": q,
        "dq": dq,
        "tau_ff": [0.02 * math.sin(phase) for _ in range(joints)],
        "kp": [40.0] * joints,
        "kd": [2.0] * joints,
        "measured_q": measured,
        "measured_dq": dq,
        "measured_tau_est": [0.01 * math.cos(phase) for _ in range(joints)],
        "output_torque": [0.15 * math.sin(phase + joint) for joint in range(joints)],
        "joint_valid": [True] * joints,
    }


def _chunk(rows: list[dict], joints: int) -> bytes:
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
        payload += struct.pack(
            "<QQIIIQ7B16s6Q9dQ",
            row["stamp_us"],
            row["state_seq"],
            3,
            2,
            0,
            0,
            1,
            1,
            1,
            1,
            1,
            1,
            1,
            bytes.fromhex("33" * 16),
            1,
            row["state_seq"],
            row["state_seq"],
            row["stamp_us"] + 5,
            row["state_seq"],
            row["stamp_us"] - 100,
            0.01,
            0.02,
            0.03,
            0.1,
            0.2,
            0.3,
            0.4,
            0.5,
            0.6,
            row["stamp_us"] + 7,
        )
    for column in columns:
        for row in rows:
            for joint in range(joints):
                payload += struct.pack("<d", row[column][joint])
    for row in rows:
        payload += bytes(1 for _ in range(joints))
    header = struct.pack(
        "<8sIIIIQQQQII",
        b"IRMVCH01",
        4,
        joints,
        len(rows),
        len(payload),
        rows[0]["stamp_us"],
        rows[-1]["stamp_us"],
        rows[0]["state_seq"],
        rows[-1]["state_seq"],
        zlib.crc32(payload) & 0xFFFFFFFF,
        0,
    )
    header = header[:60] + struct.pack("<I", zlib.crc32(header[:60]) & 0xFFFFFFFF)
    return header + bytes(payload)


def generate(root: Path, joints: int, rows: int) -> Path:
    run = root / "run_benchmark"
    if (run / "index.json").is_file():
        try:
            index = json.loads((run / "index.json").read_text())
            if (
                index.get("rows_recorded") == rows
                and index.get("joint_count") == joints
            ):
                return run
        except (OSError, ValueError):
            pass
    if run.exists():
        import shutil

        shutil.rmtree(run)
    run.mkdir(parents=True)
    meta = {
        "schema_version": 4,
        "config_id": "a" * 64,
        "layout_id": "b" * 64,
        "backend": "fake",
        "control_hz": 500.0,
        "sample_hz": 100,
        "joint_count": joints,
        "joints": [f"joint_{index:02d}" for index in range(joints)],
        "sdk_git_sha": "benchmark",
        "clock": {
            "clock": "linux_clock_monotonic",
            "boot_id": "benchmark",
            "available": True,
        },
        "units": {
            "position": "rad",
            "velocity": "rad_s",
            "torque": "Nm",
            "kp": "Nm_per_rad",
            "kd": "Nm_s_per_rad",
            "time": "us",
        },
    }
    (run / "meta.json").write_text(json.dumps(meta), encoding="utf-8")
    (run / "sessions.json").write_text("[]", encoding="utf-8")
    (run / "faults.jsonl").write_text("", encoding="utf-8")
    chunks = []
    for start in range(0, rows, ROWS_PER_CHUNK):
        stop = min(rows, start + ROWS_PER_CHUNK)
        subset = [_runtime_row(index, joints) for index in range(start, stop)]
        name = f"chunk_{start // ROWS_PER_CHUNK:06d}.bin"
        (run / name).write_bytes(_chunk(subset, joints))
        chunks.append(
            {
                "file": name,
                "rows": len(subset),
                "first_stamp_us": subset[0]["stamp_us"],
                "last_stamp_us": subset[-1]["stamp_us"],
                "first_state_seq": subset[0]["state_seq"],
                "last_state_seq": subset[-1]["state_seq"],
            }
        )
    (run / "index.json").write_text(
        json.dumps(
            {
                "schema_version": 4,
                "complete": True,
                "joint_count": joints,
                "rows_recorded": rows,
                "chunks": chunks,
                "gaps": [],
                "write_failures": 0,
                "faults": {
                    "complete": True,
                    "recorded": 0,
                    "written": 0,
                    "queue_dropped": 0,
                    "generation_dropped": 0,
                    "write_failures": 0,
                    "losses": [],
                },
            }
        ),
        encoding="utf-8",
    )
    return run


def rss_kb(pid: int) -> int:
    try:
        status = Path(f"/proc/{pid}/status").read_text()
    except OSError:
        return 0
    for line in status.splitlines():
        if line.startswith("VmRSS:"):
            return int(line.split()[1])
    return 0


def run_id(root: Path, run: Path) -> str:
    relative = run.resolve().relative_to(root.resolve()).as_posix()
    return hashlib.sha256(relative.encode()).hexdigest()[:32]


def percentile(values: list[float], fraction: float) -> float:
    ordered = sorted(values)
    if not ordered:
        return 0.0
    index = min(len(ordered) - 1, int(round((len(ordered) - 1) * fraction)))
    return ordered[index]


def bridge_query(reader: Path, root: Path, request: dict) -> tuple[float, int, int]:
    process = subprocess.Popen(
        [str(reader), "--records-root", str(root)],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        text=True,
        bufsize=1,
    )
    before = rss_kb(process.pid)
    started = time.perf_counter()
    try:
        assert process.stdin is not None and process.stdout is not None
        process.stdin.write(json.dumps(request) + "\n")
        process.stdin.flush()
        line = process.stdout.readline()
        elapsed = time.perf_counter() - started
        reply = json.loads(line)
        if not reply.get("ok"):
            raise RuntimeError(reply.get("error"))
        after = rss_kb(process.pid)
    finally:
        process.terminate()
        process.wait(timeout=10)
    return elapsed, before, after


def http_service(records: Path, data: Path, reader: Path, socket_path: Path):
    process = subprocess.Popen(
        [
            sys.executable,
            "-m",
            "tools.experiment_web",
            "--records-root",
            str(records),
            "--data-dir",
            str(data),
            "--reader",
            str(reader),
            "--assets",
            str(SOURCE / "web"),
            "--port",
            "0",
            "--telemetry-socket",
            str(socket_path),
        ],
        cwd=str(SOURCE),
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    assert process.stdout is not None
    url = process.stdout.readline().strip().rstrip("/")
    opener = urllib.request.build_opener(
        urllib.request.HTTPCookieProcessor(CookieJar())
    )
    with opener.open(url + "/", timeout=10) as response:
        response.read()
    return process, url, opener


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--joints", type=int, default=64)
    parser.add_argument("--rows", type=int, default=180000)
    parser.add_argument("--signals", type=int, default=24)
    parser.add_argument("--iterations", type=int, default=5)
    parser.add_argument("--root", default=str(SOURCE / "build" / "web-benchmark"))
    parser.add_argument(
        "--reader", default=str(SOURCE / "build-bench" / "web_record_query")
    )
    options = parser.parse_args()
    root = Path(options.root)
    records = root / "records"
    data = root / "data"
    records.mkdir(parents=True, exist_ok=True)
    data.mkdir(parents=True, exist_ok=True)
    run = generate(records, options.joints, options.rows)
    reader = Path(options.reader)
    if not reader.is_file():
        print(f"benchmark: build the reader first: {reader}", file=sys.stderr)
        return 1
    fields = ["q", "measured_q", "tau_ff", "output_torque"]
    signal_ids = [
        f"runtime/joint_{joint:02d}/{field}" for joint in range(6) for field in fields
    ][: options.signals]
    index = json.loads((run / "index.json").read_text())
    begin = index["chunks"][0]["first_stamp_us"]
    end = index["chunks"][-1]["last_stamp_us"] + 1
    request = {
        "request_id": "bench",
        "op": "range",
        "run_id": run_id(records, run),
        "signal_ids": signal_ids,
        "begin_us": str(begin),
        "end_us": str(end),
        "pixel_width": 800,
    }
    bridge_times = []
    memory = (0, 0)
    for _ in range(options.iterations):
        elapsed, before, after = bridge_query(reader, records, request)
        bridge_times.append(elapsed)
        memory = (before, after)
    process, url, opener = http_service(records, data, reader, data / "live.sock")
    http_times = []
    try:
        from urllib.parse import quote

        path = (
            f"/api/runs/{request['run_id']}/range?signal_ids="
            f"{quote(','.join(signal_ids), safe='')}"
            f"&begin_us={begin}&end_us={end}&pixel_width=800"
        )
        for _ in range(options.iterations):
            started = time.perf_counter()
            with opener.open(url + path, timeout=60) as response:
                payload = json.loads(response.read())
            http_times.append(time.perf_counter() - started)
            if not payload.get("ok"):
                raise RuntimeError(payload.get("error"))
        live_times = []
        client = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        emitter = "b" * 32
        base = 1_000_000_000
        for seq in range(40):
            stamp = base + seq * 10_000
            envelope = {
                "version": 1,
                "emitter_id": emitter,
                "run_path": str(run),
                "source": "runtime",
                "transport_seq": str(seq + 1),
                "kind": "sample",
                "payload": {
                    "kind": "sample",
                    "stamp_us": str(stamp),
                    "state_seq": str(seq + 1),
                    "command_present": True,
                    "feedback_valid": True,
                    "joint_count": options.joints,
                    "q": [0.0] * options.joints,
                    "joint_valid": [True] * options.joints,
                    "measured_tau_est": [0.0] * options.joints,
                },
            }
            sent_at = time.monotonic()
            client.sendto(json.dumps(envelope).encode(), str(data / "live.sock"))
            deadline = time.monotonic() + 1.0
            while time.monotonic() < deadline:
                with opener.open(
                    url + f"/api/live/snapshot?run_id={request['run_id']}"
                    f"&signal_ids={quote('runtime/joint_00/q', safe='')}"
                    f"&begin_us={base}&end_us={base + 5_000_000}"
                    f"&pixel_width=4096",
                    timeout=5,
                ) as response:
                    snapshot = json.loads(response.read())
                if any(
                    int(point["seq"]) == seq + 1
                    for point in snapshot.get("series", [{}])[0].get("points", [])
                ):
                    live_times.append((time.monotonic() - sent_at) * 1000.0)
                    break
                time.sleep(0.002)
        client.close()
    finally:
        process.terminate()
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            process.kill()
    report = {
        "machine": {
            "cpu": os.cpu_count(),
            "platform": sys.platform,
        },
        "record": {
            "joints": options.joints,
            "rows": options.rows,
            "sample_hz": 100,
            "seconds": options.rows / 100.0,
            "bytes": sum(path.stat().st_size for path in run.glob("chunk_*.bin")),
        },
        "signals": signal_ids,
        "bridge_first_window_s": {
            "p50": percentile(bridge_times, 0.5),
            "p95": percentile(bridge_times, 0.95),
            "max": max(bridge_times),
        },
        "http_first_window_s": {
            "p50": percentile(http_times, 0.5),
            "p95": percentile(http_times, 0.95),
            "max": max(http_times),
        },
        "bridge_rss_kb": {
            "before": memory[0],
            "after": memory[1],
            "delta": memory[1] - memory[0],
        },
        "live_source_to_snapshot_ms": {
            "samples": len(live_times),
            "p50": percentile(live_times, 0.5),
            "p95": percentile(live_times, 0.95),
            "max": max(live_times) if live_times else 0.0,
        },
        "notes": [
            "source to service snapshot only; browser paint not measured",
            "the bridge decodes one chunk at a time and holds one window",
        ],
    }
    (root / "report.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
