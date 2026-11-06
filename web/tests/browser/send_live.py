#!/usr/bin/env python3
"""Send one hello and a few live runtime samples to the service's datagram."""

from __future__ import annotations

import argparse
import json
import math
import socket


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--socket", required=True)
    parser.add_argument("--run", required=True)
    parser.add_argument("--start", type=int, default=9007199270000000)
    parser.add_argument("--count", type=int, default=8)
    parser.add_argument("--value", type=float, default=0.9)
    args = parser.parse_args()
    client = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    emitter = "e" * 32

    def send(seq: int, kind: str, payload: dict) -> None:
        envelope = {
            "version": 1,
            "emitter_id": emitter,
            "run_path": args.run,
            "source": "runtime",
            "transport_seq": str(seq),
            "kind": kind,
            "payload": payload,
        }
        client.sendto(json.dumps(envelope).encode("utf-8"), args.socket)

    send(1, "hello", {})
    for index in range(args.count):
        stamp = args.start + index * 10_000
        seq = 5000 + index
        value = args.value + 0.01 * math.sin(index)
        send(2 + index, "sample", {
            "kind": "sample",
            "stamp_us": str(stamp),
            "state_seq": str(seq),
            "command_present": True,
            "feedback_valid": True,
            "output_valid": True,
            "imu_valid": True,
            "joint_count": 4,
            "q": [value, 0.1, 0.2, 0.3],
            "dq": [0.0, 0.0, 0.0, 0.0],
            "tau_ff": [0.0, 0.0, 0.0, 0.0],
            "kp": [40.0, 40.0, 40.0, 40.0],
            "kd": [2.0, 2.0, 2.0, 2.0],
            "measured_q": [value, 0.1, 0.2, 0.3],
            "measured_dq": [0.0, 0.0, 0.0, 0.0],
            "measured_tau_est": [0.0, 0.0, 0.0, 0.0],
            "output_torque": [0.0, 0.0, 0.0, 0.0],
            "joint_valid": [True, True, True, True],
            "imu_orientation_rpy": [0.0, 0.0, 0.0],
            "imu_angular_velocity": [0.0, 0.0, 0.0],
            "imu_linear_acceleration": [0.0, 0.0, 0.0],
        })
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
