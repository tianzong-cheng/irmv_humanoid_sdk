"""Reconciliation of the recorder workers' live tails with committed history.

The tail is a bounded, temporary view. Every sample carries its source run path,
transport sequence and the run identity the recorder declared; the store keeps
it only long enough to bridge the commit lag and deduplicates by the run's own
sample key. A committed record always replaces the same temporary sample, and a
tail that is evicted before commit is named as an explicit coverage fact rather
than silently joining the curves on either side.

The store is not a second recorder. It never writes a run, never starts a
process and never turns a transport loss into a recorded gap.
"""

from __future__ import annotations

import hashlib
import json
import math
import os
import socket
import sys
import threading
import time
from collections import OrderedDict
from pathlib import Path
from urllib.parse import unquote

from .catalog import CatalogError, canonical_records_root, paginate


class LiveError(ValueError):
    """A message or subscription the live store refuses."""


def run_id_for_directory(root: Path, directory: Path) -> str:
    relative = directory.resolve().relative_to(root)
    return hashlib.sha256(relative.as_posix().encode("utf-8")).hexdigest()[:32]


def _number(value: object) -> float | str:
    if isinstance(value, (int, float)) and not isinstance(value, bool):
        return value
    if isinstance(value, str) and value in {"NaN", "+Inf", "-Inf"}:
        return value
    raise LiveError("a sample value is not a number or an explicit non-finite value")


def _finite(value: float) -> bool:
    return math.isfinite(value)


def _escape_joint(name: str) -> str:
    safe = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_.-"
    result = []
    for char in name:
        if char in safe:
            result.append(char)
        else:
            for byte in char.encode("utf-8"):
                result.append(f"%{byte:02X}")
    return "".join(result)


def _decode_joint(name: str) -> str:
    return unquote(name)


def _bucket_reduce(points: list[dict], begin_us: int, end_us: int,
                   pixel_width: int) -> list[dict]:
    if not points:
        return []
    width = max(1, (end_us - begin_us + max(1, pixel_width) - 1) // max(1, pixel_width))
    buckets: "OrderedDict[int, dict]" = OrderedDict()
    for point in points:
        stamp = int(point["t"])
        index = max(0, (stamp - begin_us) // width)
        bucket = buckets.get(index)
        if bucket is None:
            buckets[index] = {
                "first": point,
                "last": point,
                "min": point,
                "max": point,
            }
            continue
        bucket["last"] = point
        value = point["v"]
        if isinstance(value, (int, float)) and not isinstance(value, bool):
            current_min = bucket["min"]["v"]
            current_max = bucket["max"]["v"]
            if not isinstance(current_min, (int, float)) or value < current_min:
                bucket["min"] = point
            if not isinstance(current_max, (int, float)) or value > current_max:
                bucket["max"] = point
    output = []
    for bucket in buckets.values():
        selected = []
        for label, point in (
            ("first", bucket["first"]),
            ("min", bucket["min"]),
            ("max", bucket["max"]),
            ("last", bucket["last"]),
        ):
            if any(
                entry["t"] == point["t"] and entry["seq"] == point["seq"]
                for entry in selected
            ):
                continue
            entry = dict(point)
            entry["agg"] = label
            selected.append(entry)
        output.extend(selected)
    return output[: 4 * max(1, pixel_width)]


class _RunTail:
    def __init__(self, run_id: str, run_path: Path, source: str,
                 metadata: dict) -> None:
        self.run_id = run_id
        self.run_path = run_path
        self.source = source
        self.metadata = metadata
        self.joints = metadata.get("joints", [])
        self.sample_hz = metadata.get("sample_hz", 0)
        self.clock = metadata.get("clock", {})
        self.config_id = metadata.get("config_id", "")
        self.layout_id = metadata.get("layout_id", "")
        self.emitter_id = ""
        self.samples: dict[tuple, dict] = {}
        self.faults: dict[int, dict] = {}
        self.revision = 0
        self.coverage_begin_us: int | None = None
        self.coverage_end_us: int | None = None
        self.evicted_until_us = 0
        self.transport_losses = 0
        self.last_transport_seq = 0
        self.pending = True
        self.last_arrival_ms = 0.0
        self.bytes = 0


class LiveStore:
    """One process's view of every active live tail."""

    MAX_RUNS = 8
    MAX_TAIL_SECONDS = 15
    MAX_RUN_BYTES = 32 * 1024 * 1024
    MAX_GLOBAL_BYTES = 128 * 1024 * 1024

    def __init__(self, records_root: Path | str,
                 clock=None) -> None:
        self.records_root = canonical_records_root(records_root)
        self._clock = clock or time.monotonic
        self._lock = threading.Lock()
        self._runs: "OrderedDict[str, _RunTail]" = OrderedDict()
        self._bytes = 0
        self._rejected: list[str] = []
        self.epoch = hashlib.sha256(str(time.time_ns()).encode()).hexdigest()[:16]

    # -- identity -----------------------------------------------------------

    def _metadata_for(self, run_path: Path) -> dict:
        try:
            with (run_path / "meta.json").open("r", encoding="utf-8") as handle:
                metadata = json.load(handle)
        except (OSError, json.JSONDecodeError) as error:
            raise LiveError(f"the live run has no readable metadata: {error}") from error
        if not isinstance(metadata, dict):
            raise LiveError("the live run metadata is not an object")
        return metadata

    def _resolve_run(self, raw_path: object, source: object) -> tuple[Path, dict]:
        if not isinstance(raw_path, str) or not raw_path:
            raise LiveError("the live message has no run_path")
        if source not in {"runtime", "application"}:
            raise LiveError("the live message has no runtime/application source")
        candidate = Path(raw_path)
        try:
            resolved = candidate.resolve(strict=True)
        except OSError as error:
            raise LiveError(f"the live run path is not readable: {error}") from error
        try:
            resolved.relative_to(self.records_root)
        except ValueError as error:
            raise LiveError("the live run path leaves the records root") from error
        if not resolved.is_dir():
            raise LiveError("the live run path is not a directory")
        metadata = self._metadata_for(resolved)
        kind = "runtime" if "schema_version" in metadata else "application"
        if source != kind:
            raise LiveError(
                f"the live source {source} does not match the run metadata {kind}"
            )
        return resolved, metadata

    # -- ingest -------------------------------------------------------------

    def Ingest(self, message: dict) -> None:
        self.ingest(message)

    def ingest(self, message: dict, now_ms: float | None = None) -> None:
        if not isinstance(message, dict):
            raise LiveError("the live message is not an object")
        if message.get("version") != 1:
            raise LiveError("the live envelope version is not 1")
        source = message.get("source")
        run_path, metadata = self._resolve_run(message.get("run_path"), source)
        run_id = run_id_for_directory(self.records_root, run_path)
        now = self._clock() * 1000.0 if now_ms is None else now_ms
        with self._lock:
            run = self._runs.get(run_id)
            if run is None:
                if len(self._runs) >= self.MAX_RUNS:
                    self._rejected.append(run_id)
                    raise LiveError(
                        "the live tail limit of 8 active runs is reached"
                    )
                run = _RunTail(run_id, run_path, source, metadata)
                self._runs[run_id] = run
            elif run.run_path != run_path:
                raise LiveError("the live run identity changed for one run id")
            emitter = message.get("emitter_id")
            if not isinstance(emitter, str) or not emitter:
                raise LiveError("the live message has no emitter_id")
            if run.emitter_id and run.emitter_id != emitter:
                raise LiveError("the live emitter identity changed mid-run")
            run.emitter_id = emitter
            kind = message.get("kind")
            payload = message.get("payload")
            if not isinstance(payload, dict):
                raise LiveError("the live message has no payload object")
            try:
                transport_seq = int(message.get("transport_seq", "0"))
            except (TypeError, ValueError) as error:
                raise LiveError("the live transport sequence is not decimal") from error
            if transport_seq <= 0:
                raise LiveError("the live transport sequence is not positive")
            if run.last_transport_seq and transport_seq > run.last_transport_seq + 1:
                run.transport_losses += transport_seq - run.last_transport_seq - 1
            if kind in {"hello", "status"}:
                run.last_transport_seq = max(run.last_transport_seq, transport_seq)
                run.last_arrival_ms = now
                if kind == "status" and payload.get("complete") is True:
                    run.pending = False
                run.revision += 1
                return
            if kind == "fault":
                record_seq = int(payload.get("record_seq", -1))
                if record_seq < 0:
                    raise LiveError("a live fault has no record_seq")
                run.faults[record_seq] = payload
                run.last_transport_seq = max(run.last_transport_seq, transport_seq)
                run.revision += 1
                run.last_arrival_ms = now
                return
            if kind != "sample":
                raise LiveError("the live message kind is unknown")
            sample = self._runtime_sample_key(payload) if source == "runtime" \
                else self._application_sample_key(payload)
            encoded = json.dumps(payload, separators=(",", ":"), ensure_ascii=False)
            size = len(encoded.encode("utf-8"))
            existing = run.samples.get(sample["key"])
            if existing is not None:
                if existing["payload"] == payload:
                    # A retransmission is not a change: it must not advance the
                    # revision the browser polls.
                    run.last_transport_seq = max(run.last_transport_seq, transport_seq)
                    return
                self._bytes -= existing["bytes"]
                run.bytes -= existing["bytes"]
            run.samples[sample["key"]] = {
                "stamp_us": sample["stamp_us"],
                "payload": payload,
                "bytes": size,
            }
            self._bytes += size
            run.bytes += size
            stamp = sample["stamp_us"]
            run.coverage_begin_us = (
                stamp if run.coverage_begin_us is None
                else min(run.coverage_begin_us, stamp)
            )
            run.coverage_end_us = (
                stamp if run.coverage_end_us is None
                else max(run.coverage_end_us, stamp)
            )
            run.last_transport_seq = max(run.last_transport_seq, transport_seq)
            run.revision += 1
            run.last_arrival_ms = now
            self._trim_run(run)
            self._trim_global()

    @staticmethod
    def _runtime_sample_key(payload: dict) -> dict:
        try:
            stamp = int(payload["stamp_us"])
            sequence = int(payload["state_seq"])
        except (KeyError, TypeError, ValueError) as error:
            raise LiveError("a runtime sample has no stamp/state sequence") from error
        payload.setdefault("stamp_us", str(stamp))
        payload.setdefault("state_seq", str(sequence))
        return {"key": ("runtime", stamp, sequence), "stamp_us": stamp}

    @staticmethod
    def _application_sample_key(payload: dict) -> dict:
        try:
            stamp = int(payload["stamp_us"])
            sequence = int(payload["sample_seq"])
        except (KeyError, TypeError, ValueError) as error:
            raise LiveError("an application sample has no stamp/sample sequence") from error
        payload.setdefault("stamp_us", str(stamp))
        payload.setdefault("sample_seq", str(sequence))
        return {"key": ("application", stamp, sequence), "stamp_us": stamp}

    def _trim_run(self, run: _RunTail) -> None:
        if run.coverage_end_us is None:
            return
        horizon = run.coverage_end_us - self.MAX_TAIL_SECONDS * 1_000_000
        while run.samples:
            oldest_key = min(run.samples, key=lambda key: run.samples[key]["stamp_us"])
            oldest = run.samples[oldest_key]
            too_old = oldest["stamp_us"] < horizon
            too_large = run.bytes > self.MAX_RUN_BYTES
            if not too_old and not too_large:
                break
            run.samples.pop(oldest_key)
            self._bytes -= oldest["bytes"]
            run.bytes -= oldest["bytes"]
            run.evicted_until_us = max(run.evicted_until_us,
                                       oldest["stamp_us"] + 1)

    def _trim_global(self) -> None:
        while self._bytes > self.MAX_GLOBAL_BYTES and self._runs:
            oldest_run = min(
                self._runs.values(),
                key=lambda item: item.last_arrival_ms,
            )
            if not oldest_run.samples:
                break
            oldest_key = min(
                oldest_run.samples,
                key=lambda key: oldest_run.samples[key]["stamp_us"],
            )
            removed = oldest_run.samples.pop(oldest_key)
            self._bytes -= removed["bytes"]
            oldest_run.bytes -= removed["bytes"]
            oldest_run.evicted_until_us = max(
                oldest_run.evicted_until_us, removed["stamp_us"] + 1
            )

    # -- signals ------------------------------------------------------------

    @staticmethod
    def _signal_parts(signal_id: str) -> tuple[str, str, str]:
        parts = signal_id.split("/", 2)
        if len(parts) != 3:
            raise LiveError(f"the signal id is not canonical: {signal_id}")
        return parts[0], _decode_joint(parts[1]), parts[2]

    def _points_for_signal(self, run: _RunTail, signal_id: str) -> list[dict]:
        source, joint, field = self._signal_parts(signal_id)
        if source != run.source:
            return []
        points = []
        if source == "application":
            for entry in run.samples.values():
                payload = entry["payload"]
                if field not in payload and field not in {
                    "stamp_us", "sample_seq", "latest_state_seq", "identity_known"
                }:
                    continue
                value = payload.get(field)
                if value is None:
                    continue
                points.append(
                    {
                        "t": str(entry["stamp_us"]),
                        "seq": str(payload.get("sample_seq", "0")),
                        "v": value,
                        "valid": True,
                        "segment": 0,
                        "agg": "live",
                    }
                )
            return points
        if joint and joint not in run.joints:
            return []
        index = run.joints.index(joint) if joint else -1
        for entry in run.samples.values():
            payload = entry["payload"]
            valid = True
            if field in {"q", "dq", "tau_ff", "kp", "kd"}:
                valid = bool(payload.get("command_present", False))
            elif field.startswith("measured_"):
                valid = bool(payload.get("feedback_valid", False)) and bool(
                    (payload.get("joint_valid") or [])[index]
                    if index >= 0 and index < len(payload.get("joint_valid") or [])
                    else False
                )
            elif field == "output_torque":
                valid = bool(payload.get("output_valid", False))
            elif field.startswith("imu_"):
                valid = bool(payload.get("imu_valid", False))
            if index >= 0:
                column = payload.get(field)
                if not isinstance(column, list) or index >= len(column):
                    continue
                value = column[index]
            else:
                value = payload.get(field)
                if value is None:
                    continue
                if field in {
                    "healthy", "damping_eligible", "command_present",
                    "feedback_valid", "imu_valid", "send_accepted", "output_valid",
                }:
                    value = 1 if value else 0
            points.append(
                {
                    "t": str(entry["stamp_us"]),
                    "seq": str(payload.get("state_seq", "0")),
                    "v": value,
                    "valid": valid,
                    "segment": 0,
                    "agg": "live",
                }
            )
        return points

    # -- queries ------------------------------------------------------------

    def Snapshot(self, run_id: str, signal_ids: list[str], begin_us: int,
                 end_us: int, pixel_width: int = 800,
                 after_revision: int | None = None) -> dict:
        with self._lock:
            run = self._runs.get(run_id)
            if run is None:
                raise LiveError("the run has no active live tail")
            revision = run.revision
            if after_revision is not None and after_revision == revision:
                return {
                    "ok": True,
                    "run_id": run_id,
                    "revision": str(revision),
                    "changed": False,
                    "service_epoch": self.epoch,
                }
            points = []
            for signal_id in signal_ids:
                raw = self._points_for_signal(run, signal_id)
                raw.sort(key=lambda item: (int(item["t"]), int(item["seq"])))
                reduced = _bucket_reduce(raw, begin_us, end_us, pixel_width)
                points.append({"signal_id": signal_id, "points": reduced})
            identity = {
                "source": run.source,
                "run_path": str(run.run_path),
                "emitter_id": run.emitter_id,
                "config_id": run.config_id,
                "layout_id": run.layout_id,
                "clock": run.clock,
                "sample_hz": run.sample_hz,
            }
            coverage = {
                "begin_us": None if run.coverage_begin_us is None
                else str(run.coverage_begin_us),
                "end_us": None if run.coverage_end_us is None
                else str(run.coverage_end_us),
                "evicted_before_us": str(run.evicted_until_us) if run.evicted_until_us else None,
            }
            payload = {
                "ok": True,
                "run_id": run_id,
                "revision": str(revision),
                "changed": True,
                "service_epoch": self.epoch,
                "series": points,
                "faults": list(run.faults.values()),
                "identity": identity,
                "coverage": coverage,
                "pending": run.pending,
                "transport_losses": run.transport_losses,
                "active_runs": len(self._runs),
            }
            return payload

    def SnapshotTail(self, run_id: str, signal_ids: list[str], begin_us: int,
                     end_us: int, pixel_width: int = 800) -> dict:
        return self.Snapshot(run_id, signal_ids, begin_us, end_us, pixel_width)

    def merge_range(self, reply: dict, run_id: str, signal_ids: list[str],
                    begin_us: int, end_us: int, pixel_width: int) -> dict:
        """Append uncommitted tail points to a committed range reply.

        A live point at or after the committed coverage end is appended unless
        the reply already holds that exact (time, sequence). Committed values
        therefore win even when a tail copy of the same sample is still cached.
        """
        try:
            snapshot = self.Snapshot(run_id, signal_ids, begin_us, end_us,
                                     pixel_width)
        except LiveError:
            return reply
        committed_end = None
        committed = reply.get("committed")
        if isinstance(committed, dict) and committed.get("end_us") is not None:
            try:
                committed_end = int(committed["end_us"])
            except (TypeError, ValueError):
                committed_end = None
        live_series = {item["signal_id"]: item["points"] for item in snapshot["series"]}
        for series in reply.get("series", []):
            signal_id = series.get("signal", {}).get("id")
            points = live_series.get(signal_id, [])
            existing = {
                (point.get("t"), point.get("seq")) for point in series.get("points", [])
            }
            for point in points:
                if committed_end is not None and int(point["t"]) < committed_end:
                    continue
                key = (point["t"], point["seq"])
                if key in existing:
                    continue
                series["points"].append(point)
                existing.add(key)
            series["points"].sort(key=lambda item: (int(item["t"]), int(item["seq"])))
        reply["live"] = {
            "active": True,
            "revision": snapshot["revision"],
            "service_epoch": snapshot["service_epoch"],
            "coverage": snapshot["coverage"],
            "pending": snapshot["pending"],
            "transport_losses": snapshot["transport_losses"],
            "evicted": snapshot["coverage"]["evicted_before_us"],
        }
        return reply

    def merge_events(self, reply: dict, run_id: str) -> dict:
        try:
            with self._lock:
                run = self._runs.get(run_id)
                if run is None:
                    return reply
                live = sorted(run.faults.values(),
                              key=lambda fault: int(fault.get("detected_stamp_us", 0)))
                existing = {
                    event.get("event_id") for event in reply.get("events", [])
                }
                for fault in live:
                    event_id = str(fault.get("record_seq"))
                    if event_id not in existing:
                        reply.setdefault("events", []).append(fault)
                        existing.add(event_id)
                reply["live"] = {
                    "active": True,
                    "revision": str(run.revision),
                    "service_epoch": self.epoch,
                }
        except (TypeError, ValueError):
            pass
        return reply

    def active_runs(self) -> list[str]:
        with self._lock:
            return list(self._runs)


class LiveReceiver:
    """The service's single datagram consumer."""

    def __init__(self, store: LiveStore, path: Path) -> None:
        self.store = store
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        try:
            self.path.unlink()
        except FileNotFoundError:
            pass
        self._socket = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        self._socket.bind(str(self.path))
        os.chmod(self.path, 0o600)
        self._closed = False
        self._thread = threading.Thread(target=self._run, name="live-receiver",
                                        daemon=True)

    def start(self) -> None:
        self._thread.start()

    def _run(self) -> None:
        while not self._closed:
            try:
                data, _ = self._socket.recvfrom(65536)
            except OSError:
                if self._closed:
                    return
                continue
            try:
                message = json.loads(data.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError):
                continue
            try:
                self.store.ingest(message)
            except LiveError:
                # A refused message is not a service failure. The sender counts
                # its own transport outcome and the history remains authoritative.
                continue

    def close(self) -> None:
        self._closed = True
        try:
            self._socket.close()
        except OSError:
            pass
        try:
            self.path.unlink()
        except FileNotFoundError:
            pass


__all__ = [
    "LiveError",
    "LiveReceiver",
    "LiveStore",
    "run_id_for_directory",
]
