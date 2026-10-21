#!/usr/bin/env python3
"""RealBackend + scripted port + real DDS contract.

Usage: real_backend_contract_test.py <helper> <source directory>
"""

import json
import os
import signal
import subprocess
import sys
import tempfile
import threading
import time
from pathlib import Path

DOMAIN = 80 + (os.getpid() % 80)
FAILURES = []


def check(condition, message):
    if not condition:
        FAILURES.append(message)
        print("FAIL: " + message)
    return condition


class Runtime:
    def __init__(self, helper, config, workspace, script):
        environment = dict(os.environ)
        environment["IRMV_TEST_SCRIPT"] = script
        environment["IRMV_LOCK_DIR"] = str(workspace / "locks")
        environment["IRMV_TEST_SEND_LOG"] = str(workspace / f"{script}.jsonl")
        self.log_path = Path(environment["IRMV_TEST_SEND_LOG"])
        self.process = subprocess.Popen(
            [str(helper), "--config", str(config), "--domain", str(DOMAIN)],
            cwd=str(workspace), stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT, text=True, bufsize=1,
            env=environment)
        self.lines = []
        self.lock = threading.Lock()
        self.reader = threading.Thread(target=self._read, daemon=True)
        self.reader.start()

    def _read(self):
        for line in self.process.stdout:
            with self.lock:
                self.lines.append(line.rstrip("\n"))

    def output(self):
        with self.lock:
            return "\n".join(self.lines)

    def wait_for(self, needle, timeout=10.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if needle in self.output():
                return True
            if self.process.poll() is not None:
                return needle in self.output()
            time.sleep(0.02)
        return False

    def stop(self):
        if self.process.poll() is None:
            self.process.send_signal(signal.SIGTERM)
        try:
            return self.process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait(timeout=5)
            return None


def make_config(source, workspace, imu_required=False):
    document = json.loads(
        (source / "tests/support/hardware/real_two_joint.json").read_text())
    document.pop("_comment", None)
    document["telemetry"]["directory"] = str(workspace / "logs")
    document["dds"]["domain"] = DOMAIN
    if imu_required:
        document["runtime"]["imu_required"] = True
        document["robot"]["imu"] = {"hardware_id": "test-imu",
                                    "baudrate": 921600}
    path = workspace / ("config-imu.json" if imu_required else "config.json")
    path.write_text(json.dumps(document))
    return path


def main() -> int:
    helper = Path(sys.argv[1]).resolve()
    source = Path(sys.argv[2]).resolve()
    client = Path(sys.argv[3]).resolve()
    with tempfile.TemporaryDirectory(prefix="irmv-real-contract-") as directory:
        workspace = Path(directory)
        (workspace / "locks").mkdir()
        config = make_config(source, workspace)
        imu_config = make_config(source, workspace, imu_required=True)

        cases = [
            ("normal", config, None, None),
            ("near-future", config, None, None),
            ("far-future", config, "FeedbackInvalid", None),
            ("write-refused", config, "WriteFailed", None),
            ("driver-error", config, "DriverError", None),
            ("overspeed", config, "JointVelocityExceeded", None),
            ("imu-missing", imu_config, "ImuInvalid", None),
        ]
        for script, case_config, fault, _ in cases:
            runtime = Runtime(helper, case_config, workspace, script)
            if fault is None:
                time.sleep(2.5)
                output = runtime.output()
                check("Fault" not in output,
                      f"{script}: unexpected fault\n{output}")
                check("runtime started" in output,
                      f"{script}: the runtime did not start")
                status = runtime.stop()
                check(status == 0,
                      f"{script}: clean stop returned {status}\n{runtime.output()}")
            else:
                check(runtime.wait_for('state="Fault"', timeout=12),
                      f"{script}: no Fault state\n{runtime.output()}")
                check(runtime.wait_for(fault, timeout=5) or fault in runtime.output(),
                      f"{script}: missing fault reason {fault}\n{runtime.output()}")
                runtime.stop()

        # The real DDS handshake enters Tracking and a nonzero target reaches
        # the port. The closing stop sends a zero frame after it.
        contract = Runtime(helper, config, workspace, "contract")
        client_result = subprocess.run(
            [str(client), "--mode", "supervisor", "--scenario", "contract",
             "--domain", str(DOMAIN), "--config", str(config)],
            cwd=str(workspace), text=True, capture_output=True, timeout=30)
        check(client_result.returncode == 0,
              "contract client failed\n" + client_result.stdout +
              client_result.stderr)
        contract.stop()
        contract_log = workspace / "contract.jsonl"
        if check(contract_log.exists(), "contract run wrote no send log"):
            records = [json.loads(line)
                       for line in contract_log.read_text().splitlines()]
            non_zero = [index for index, record in enumerate(records)
                        if abs(record["torque0"]) > 0.5]
            check(bool(non_zero),
                  "the contract run never sent the nonzero target")
            if non_zero:
                check(len(non_zero) >= 10,
                      "too few nonzero tracking frames: %d" % len(non_zero))
                # q_target 0.05, measured q 0, kp 40: the PD target is 2 Nm.
                first = records[non_zero[0]]
                check(1.5 <= abs(first["torque0"]) <= 2.5,
                      "the first tracking torque is not the PD result: %s" %
                      first)
                check(abs(first["torque0"] - first["torque1"]) < 1e-9,
                      "the two identical joints were not commanded alike")
                check(all(abs(record["torque0"]) <= 20.0
                          for record in records),
                      "a recorded torque exceeds the configured limit")
                check(any(record["torque0"] == 0.0 and record["accepted"]
                          for record in records[non_zero[-1] + 1:]),
                      "no zero frame followed the last nonzero target")
                check(records[-1]["torque0"] == 0.0 and
                      records[-1]["accepted"],
                      "the last accepted frame was not zero")
            check(all(record["accepted"] for record in records),
                  "the contract run had a rejected write")

        # The normal run recorded only zero torque frames and no non-zero target.
        log = workspace / "normal.jsonl"
        if check(log.exists(), "normal run wrote no send log"):
            for line in log.read_text().splitlines():
                record = json.loads(line)
                check(record["torque0"] == 0.0 and record["torque1"] == 0.0,
                      "normal run sent a non-zero target: " + line)
            check(len(log.read_text().splitlines()) > 0,
                  "normal run sent no frame")
        # The write-refused run really attempted a write after the accepted
        # prefix instead of failing on a fabricated result.
        refused = workspace / "write-refused.jsonl"
        if check(refused.exists(), "write-refused wrote no send log"):
            records = [json.loads(line)
                       for line in refused.read_text().splitlines()]
            check(any(not record["accepted"] for record in records),
                  "write-refused never logged a rejected attempt")

    if FAILURES:
        print(f"{len(FAILURES)} check(s) failed")
        return 1
    print("real backend contract passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
