#!/usr/bin/env python3
"""Installed-prefix acceptance of the experiment web workbench.

The test installs the current build into a staging prefix, starts the installed
entry point from a working directory that is not the source tree, reads a
minimal application run and a static asset, saves a layout, and stops the
service. It also proves that the installed entry point names a missing reader
instead of falling back to the source tree.

Usage: experiment_web_package_test.py <source> <build> <stage>
"""

from __future__ import annotations

import json
import os
import shutil
import subprocess
import sys
import tempfile
import time
import urllib.error
import urllib.request
from http.cookiejar import CookieJar
from pathlib import Path


def wait_url(process: subprocess.Popen, timeout: float = 20.0) -> str:
    assert process.stdout is not None
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        line = process.stdout.readline()
        if not line:
            time.sleep(0.05)
            continue
        line = line.strip()
        if line.startswith("http://127.0.0.1:"):
            return line.rstrip("/")
    raise AssertionError("the installed service printed no URL")


def stop(process: subprocess.Popen) -> None:
    if process.poll() is None:
        process.terminate()
    try:
        process.wait(timeout=8.0)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=5.0)
    for stream in (process.stdout, process.stderr):
        if stream is not None:
            stream.close()


def make_application_run(root: Path) -> None:
    root.mkdir(parents=True, exist_ok=True)
    run = root / "app_run_package"
    run.mkdir()
    (run / "meta.json").write_text(
        json.dumps(
            {
                "schema": "irmv.application_velocity",
                "version": 1,
                "sdk_git_sha": "package-test",
                "application_config_id": "a" * 64,
                "config_id": "b" * 64,
                "layout_id": "c" * 64,
                "joint_count": 1,
                "sample_hz": 100,
                "source": "local",
                "clock": {
                    "clock": "linux_clock_monotonic",
                    "boot_id": "package",
                    "available": True,
                },
                "units": {"forward": "m_s", "lateral": "m_s", "yaw": "rad_s"},
            }
        ),
        encoding="utf-8",
    )
    (run / "index.json").write_text(
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
    for index in range(20):
        lines.append(
            json.dumps(
                {
                    "schema": "irmv.application_velocity",
                    "version": 1,
                    "host_us": 1000 + index * 10000,
                    "stamp_us": 1000 + index * 10000,
                    "sample_seq": f"{index:016x}",
                    "identity_known": True,
                    "runtime_id": "1" * 32,
                    "session_id": "2" * 32,
                    "latest_state_seq": f"{index:016x}",
                    "forward": 0.1 * index,
                    "lateral": 0.0,
                    "yaw": 0.0,
                }
            )
        )
    (run / "velocity.jsonl").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    if len(sys.argv) < 4:
        print(__doc__, file=sys.stderr)
        return 2
    source = Path(sys.argv[1]).resolve()
    build = Path(sys.argv[2]).resolve()
    stage = Path(sys.argv[3]).resolve()
    if stage.exists():
        shutil.rmtree(stage)
    install = subprocess.run(
        ["cmake", "--install", str(build), "--prefix", str(stage)],
        cwd=str(source),
        capture_output=True,
        text=True,
    )
    if install.returncode != 0:
        print(install.stdout + install.stderr, file=sys.stderr)
        return 1
    entry = stage / "bin" / "irmv_experiment_web"
    reader = stage / "bin" / "web_record_query"
    assets = stage / "share" / "irmv" / "experiment-web" / "web"
    if not entry.is_file() or not reader.is_file():
        print("the install has no workbench entry point or reader", file=sys.stderr)
        return 1
    if not (assets / "index.html").is_file():
        print("the install has no static workbench", file=sys.stderr)
        return 1
    linked = subprocess.run(["ldd", str(reader)], capture_output=True, text=True)
    if "onnxruntime" in linked.stdout.lower():
        print("the installed reader links ONNX Runtime", file=sys.stderr)
        return 1

    with tempfile.TemporaryDirectory() as directory:
        work = Path(directory)
        records = work / "records"
        make_application_run(records)
        data = work / "data"
        process = subprocess.Popen(
            [
                str(entry),
                "--records-root",
                str(records),
                "--data-dir",
                str(data),
                "--port",
                "0",
            ],
            cwd=str(work),
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        service_survivor = subprocess.Popen(
            [sys.executable, "-c", "import time; time.sleep(30)"],
            cwd=str(work),
        )
        try:
            url = wait_url(process)
            opener = urllib.request.build_opener(
                urllib.request.HTTPCookieProcessor(CookieJar())
            )
            with opener.open(url + "/", timeout=10) as response:
                html = response.read().decode("utf-8")
            if (
                "<title>IRMV Studio</title>" not in html
                or 'aria-label="IRMV Studio home"' not in html
            ):
                print("the installed page is not the workbench", file=sys.stderr)
                return 1
            with opener.open(url + "/api/runs", timeout=10) as response:
                runs = json.loads(response.read().decode("utf-8"))
            if not runs.get("ok") or not runs.get("runs"):
                print("the installed service read no run", file=sys.stderr)
                return 1
            layout = {
                "version": 1,
                "name": "installed-layout",
                "dock": {"type": "empty"},
                "charts": [],
            }
            request = urllib.request.Request(
                url + "/api/layouts",
                data=json.dumps(layout).encode("utf-8"),
                method="POST",
                headers={"Content-Type": "application/json", "Origin": url},
            )
            with opener.open(request, timeout=10) as response:
                saved = json.loads(response.read().decode("utf-8"))
            with opener.open(
                url + f"/api/layouts/{saved['id']}", timeout=10
            ) as response:
                loaded = json.loads(response.read().decode("utf-8"))
            if loaded["layout"]["dock"] != layout["dock"]:
                print("the installed layout did not round-trip", file=sys.stderr)
                return 1
        finally:
            stop(process)
            survivor_alive = service_survivor.poll() is None
            if survivor_alive:
                service_survivor.kill()
            service_survivor.wait(timeout=5.0)
        if not survivor_alive:
            print("stopping the service changed an unrelated process", file=sys.stderr)
            return 1

    # A stage without the reader must name the missing dependency, not silently
    # use the source tree.
    broken = Path(tempfile.mkdtemp()) / "stage"
    shutil.copytree(stage, broken)
    (broken / "bin" / "web_record_query").unlink()
    with tempfile.TemporaryDirectory() as directory:
        result = subprocess.run(
            [
                str(broken / "bin" / "irmv_experiment_web"),
                "--records-root",
                directory,
                "--data-dir",
                directory,
                "--port",
                "0",
            ],
            cwd=directory,
            capture_output=True,
            text=True,
            timeout=20,
        )
    shutil.rmtree(broken.parent, ignore_errors=True)
    if result.returncode == 0 or "reader is missing" not in result.stderr:
        print("a missing installed reader was not named", file=sys.stderr)
        print(result.stderr, file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
