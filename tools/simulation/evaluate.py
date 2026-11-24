"""Fail-closed offline acceptance. No physical safety inference or GPU fallback.

The caller supplies a byte-bound run manifest. Evidence is streamed, not loaded
with record.read's deliberately capped convenience API. Protocol events are
replayed through the clock-free coordinator to validate first-command/release,
DDS phase seals, step ordering and actual applied-write identities.
"""

import argparse
import hashlib
import json
import math
import subprocess
from collections import deque
from pathlib import Path

from . import protocol as p
from .coordinator import Bootstrap, Event, SimulationCoordinator
from .engine import Feedback
from .record import MAX_LINE, read


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def bound_file(manifest, key):
    item = manifest["files"][key]
    path = Path(item["path"])
    if sha256(path) != item["sha256"]:
        raise ValueError("file digest mismatch: " + key)
    return path


def load(path):
    return json.loads(Path(path).read_text())


def finite(value):
    if isinstance(value, dict):
        for v in value.values():
            finite(v)
    elif isinstance(value, (list, tuple)):
        for v in value:
            finite(v)
    elif isinstance(value, float) and not math.isfinite(value):
        raise ValueError("nonfinite metric/evidence")


def scenario_intent(scenario, bounds):
    if len(bounds) != 3 or any(
        len(pair) != 2
        or any(type(v) not in (int, float) or not math.isfinite(v) for v in pair)
        or pair[0] > pair[1]
        for pair in bounds
    ):
        raise ValueError("three finite ordered source intent bounds required")
    intent = {
        "standing": [0, 0, 0],
        "forward": [1, 0, 0],
        "lateral": [0, 1, 0],
        "yaw": [0, 0, 1],
    }[scenario]
    return (
        intent
        if all(low <= v <= high for v, (low, high) in zip(intent, bounds))
        else None
    )


def yaw_frame_velocity(quaternion, velocity):
    w, x, y, z = quaternion
    if abs(sum(v * v for v in quaternion) - 1) > 1e-6:
        raise ValueError("nonunit base orientation")
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    vx, vy, _, wx, wy, wz = velocity
    return [
        math.cos(yaw) * vx + math.sin(yaw) * vy,
        -math.sin(yaw) * vx + math.cos(yaw) * vy,
        2 * (x * z - w * y) * wx
        + 2 * (y * z + w * x) * wy
        + (1 - 2 * (x * x + y * y)) * wz,
    ]


def payload(kind, value):
    if kind in ("Id", "Digest", "Text256"):
        return bytes.fromhex(value)
    if kind in p.SCHEMA:
        return {n: payload(t, value[n]) for t, n in p.SCHEMA[kind]}
    return value


def wire_frame(row, b):
    frame = row["frame"]
    name = frame["name"]
    return p.decode(
        p.encode(
            p.Frame(
                p.KINDS.index(name) + 1,
                payload(name, frame["payload"]),
                b.run_id,
                row["control_step"],
                row["phase"],
                row["logical_us"],
                b.physics_dt_us,
                b.control_dt_us,
            )
        )
    )


def stream_json(path, *, validate_values=True):
    with Path(path).open("rb") as stream:
        while True:
            raw = stream.readline(MAX_LINE + 2)
            if not raw:
                break
            if len(raw) > MAX_LINE or not raw.endswith(b"\n"):
                raise ValueError("invalid SDK export line")
            row = json.loads(raw)
            if validate_values:
                finite(row)
            yield row


def require_complete_sdk_summary(summary, role):
    if (
        not summary
        or not summary.get("complete")
        or summary.get("truncated")
        or summary.get("corrupt")
        or summary.get("gaps")
        or summary.get("losses")
        or summary.get("policy_session_truncated")
        or summary.get("missing_samples", 0)
    ):
        raise ValueError("incomplete " + role + " SDK record")
    if role == "runtime":
        faults = summary.get("faults")
        if (
            not faults
            or not faults.get("complete")
            or faults.get("truncated")
            or faults.get("losses")
            or faults.get("missing_occurrences", 0)
        ):
            raise ValueError("incomplete fault accounting")


def empty_metrics():
    """None is unavailable; observed counts carry explicit coverage flags."""
    return dict(
        released_control_step=None,
        release_logical_us=None,
        completed_applied_intervals=None,
        measured_rollout_steps=None,
        observed_feedback_steps=None,
        observed_logical_duration_us=None,
        observed_rollout_duration_us=None,
        fall_observed=None,
        unexpected_runtime_faults=None,
        runtime_fault_observed=None,
        recorded_runtime_fault_count=None,
        recorded_protection_count=None,
        unexpected_protections=None,
        protection_observed=None,
        runner_failure_observed=None,
        min_base_height_m=None,
        max_abs_measured_torque_Nm=None,
        host_engine_operation_max_us=None,
        host_engine_operation_total_us=None,
        evidence_complete=None,
        measured_feedback_complete=None,
        protocol_replay_complete=False,
        rollout_measurement_coverage_complete=False,
        intent_coverage_complete=False,
        intent_samples=None,
        intent_expected_samples=None,
        intent_first_stamp_us=None,
        intent_last_stamp_us=None,
        intent_max_gap_us=None,
        joint_metric_values=None,
        joint_metric_coverage_complete=False,
        joint_rmse_rad=None,
        torque_saturation_fraction=None,
        application_fault_observed=None,
        initialization_max_target_error_rad=None,
        configured_torque_limits_Nm=None,
        saturation_limit_provenance=None,
        velocity_rmse_yaw_xy_world_yaw_rad_s=None,
    )


def evaluate(manifest):
    report = dict(
        status="fail",
        scope=manifest.get("scope", "unknown"),
        reasons=[],
        metrics=empty_metrics(),
        physical_safety="not evaluated",
        trained_policy_accepted=False,
    )
    try:
        _evaluate(manifest, report)
        if not report["reasons"]:
            report["status"] = "pass"
            report["trained_policy_accepted"] = manifest["scope"] == "trained_policy"
    except (
        OSError,
        ValueError,
        KeyError,
        TypeError,
        IndexError,
        AttributeError,
        ZeroDivisionError,
        OverflowError,
        subprocess.SubprocessError,
    ) as exc:
        report["reasons"].append(str(exc))
    for key, value in report["metrics"].items():
        try:
            finite(value)
        except ValueError:
            report["metrics"][key] = None
            report["status"] = "fail"
            report["trained_policy_accepted"] = False
            report["reasons"].append("unavailable nonfinite metric: " + key)
    return report


def audit_evidence(
    path,
    b,
    duration_us,
    fall,
    intent,
    *,
    metrics=None,
    context=None,
    phase_visitor=None,
):
    """Replay a stream; retain verified-prefix observations even when it fails.

    A complete file is distinct from a successful outcome. Fall/protection facts
    are retained while replay continues over already recorded events. Protocol
    corruption stops replay; counts then describe only the observed prefix.
    """
    metrics = empty_metrics() if metrics is None else metrics
    context = {} if context is None else context
    c = SimulationCoordinator(b)
    pending = deque()
    release = None
    measured = 0
    applied = 0
    feedback_count = 0
    last_feedback_step = None
    sum_velocity = [0.0, 0.0, 0.0]
    max_tau = None
    minimum_height = None
    host_max = None
    host_total = 0
    previous_host = -1
    arm = None
    fallen = False
    runner_failed = False
    faults = set()
    protections = set()
    issues = []
    summary = None
    protocol_complete = False
    rollout_complete = False
    started = False

    def visitor(row):
        nonlocal \
            release, \
            measured, \
            applied, \
            max_tau, \
            minimum_height, \
            host_max, \
            host_total, \
            previous_host, \
            arm
        nonlocal feedback_count, last_feedback_step, fallen, started, runner_failed
        finite(row)
        started = True
        if row.get("failure"):
            runner_failed = True
            if row["failure"] not in issues:
                issues.append(row["failure"])
        host = row["host_observed_us"]
        if type(host) is not int or host < previous_host:
            raise ValueError("host clock order")
        previous_host = host
        if (
            row["kind"] in ("event_feedback", "event_stopped")
            and "host_operation_duration_us" in row
        ):
            elapsed = row["host_operation_duration_us"]
            if type(elapsed) is not int or elapsed < 0:
                raise ValueError("invalid host operation duration")
            host_max = max(host_max or 0, elapsed)
            host_total += elapsed
        kind = row["kind"]
        if kind.startswith("event_"):
            if pending:
                raise ValueError("missing accepted/applied action evidence")
            event_kind = kind[6:]
            f = wire_frame(row, b) if row.get("frame") else None
            fb = row.get("feedback")
            feedback = (
                Feedback(
                    **{k: tuple(v) if isinstance(v, list) else v for k, v in fb.items()}
                )
                if fb
                else None
            )
            if f and p.KINDS[f.kind - 1] == "Manifest":
                message = f.payload["message"]
                if message["kind"] == 3 and (
                    message["execution_state"] == 4 or message["fault_id"]
                ):
                    faults.add(message["fault_id"] or message["sequence"])
                    if "unexpected runtime fault" not in issues:
                        issues.append("unexpected runtime fault")
                if message["kind"] == 2 and message["status"] != 0:
                    protections.add(message["request_id"])
                    if "unexpected request rejection/protection" not in issues:
                        issues.append("unexpected request rejection/protection")
            old = c.torque
            actions = c.advance(
                Event(
                    event_kind,
                    peer=row.get("peer", 0),
                    frame=f,
                    feedback=feedback,
                    reason=row.get("reason", ""),
                    released=row.get("released", False),
                ),
                host,
            )
            if c.state == "Failed":
                raise ValueError("protocol replay: " + c.failure)
            if (
                row["control_step"] != (f.control_step if f else c.step)
                or row["phase"] != (f.phase if f else c.phase)
                or row["coordinator_state"] != c.state
            ):
                raise ValueError("step/phase/state mismatch")
            if f and p.KINDS[f.kind - 1] == "Torque" and c.torque is not old:
                pending.append(("write_accepted", f))
            for action in actions:
                if action.frame and p.KINDS[action.frame.kind - 1] in (
                    "AppliedToStep",
                    "ShutdownCommitted",
                    "LeaveCommitted",
                ):
                    pending.append((p.KINDS[action.frame.kind - 1], action.frame))
            if f and p.KINDS[f.kind - 1] == "ReleaseArm":
                arm = f.payload
            if row.get("released"):
                if release is not None or arm is None:
                    raise ValueError("release lacks actual application confirmation")
                release = c.step - 1
            if feedback:
                if row.get("feedback_validated") is not True:
                    raise ValueError("unvalidated measurement")
                feedback_count += 1
                last_feedback_step = c.step
                max_tau = max(max_tau or 0, *map(abs, feedback.tau))
                if (
                    release is not None
                    and 0 < c.step - release <= duration_us // b.control_dt_us
                ):
                    measured += 1
                    height = feedback.base_position[2]
                    minimum_height = (
                        height
                        if minimum_height is None
                        else min(minimum_height, height)
                    )
                    if height < fall["minimum_height_m"]:
                        fallen = True
                        if "fall observed" not in issues:
                            issues.append("fall observed")
                    # Pose/torque observations survive an unavailable orientation;
                    # only frame-dependent tracking needs this transform.
                    values = yaw_frame_velocity(
                        feedback.base_quaternion, feedback.base_velocity
                    )
                    for i in range(3):
                        sum_velocity[i] += (values[i] - intent[i]) ** 2
            # The application samples after consuming this exact runtime state,
            # before its phase2 publication. Sequence is not inferred from step.
            if (
                phase_visitor
                and f
                and p.KINDS[f.kind - 1] == "PhaseAck"
                and f.phase == 1
                and f.payload["consumer"] == 3
            ):
                state = next(m for m in c.consumed if m["kind"] == 3)
                phase_visitor(f.control_step, state)
        elif kind in (
            "write_accepted",
            "AppliedToStep",
            "ShutdownCommitted",
            "LeaveCommitted",
        ):
            if not pending:
                raise ValueError("extra command/application cycle")
            expected, frame = pending.popleft()
            if kind != expected or wire_frame(row, b) != frame:
                raise ValueError("write/application identity mismatch")
            if kind == "AppliedToStep":
                applied += 1
        elif kind == "runner_final":
            if pending or c.state != "Closed" or not c.stop_proven:
                raise ValueError("incomplete final lifecycle")
        else:
            raise ValueError("unknown evidence kind: " + kind)

    try:
        # Bind observations to the requested run before retaining any prefix.
        with Path(path).open("rb") as stream:
            raw = stream.readline(MAX_LINE + 2)
        if len(raw) > MAX_LINE + 1 or not raw.endswith(b"\n"):
            raise ValueError("invalid evidence header")
        header = json.loads(raw)
        if header.get("version") not in (1, 2):
            raise ValueError("unknown lifecycle evidence version")
        c.legacy_shutdown = header["version"] == 1
        if header["clock"]["clock_run_id"] != b.run_id.hex():
            raise ValueError("record run mismatch")
        for key in (
            "runtime_id",
            "config_digest",
            "application_digest",
            "engine_digest",
        ):
            if header["provenance"][key] != getattr(b, key).hex():
                raise ValueError("record identity/digest mismatch: " + key)
        if (
            header["clock"]["control_dt_us"] != b.control_dt_us
            or header["clock"]["physics_dt_us"] != b.physics_dt_us
            or header["provenance"]["engine_config"] != b.engine_config
        ):
            raise ValueError("record engine clock/config mismatch")
        _, summary = read(path, visitor=visitor)
        context["summary"] = summary
        provenance = summary["provenance"]
        for key in (
            "runtime_id",
            "config_digest",
            "application_digest",
            "engine_digest",
        ):
            if provenance[key] != getattr(b, key).hex():
                raise ValueError("record identity/digest mismatch: " + key)
        if provenance["engine_config"] != b.engine_config:
            raise ValueError("record engine config mismatch")
        if summary["clock"]["clock_run_id"] != b.run_id.hex():
            raise ValueError("record run mismatch")
        if (
            not summary["evidence_complete"]
            or not summary["measured_feedback_complete"]
        ):
            raise ValueError("incomplete file/measured evidence")
        if pending or summary["final"] != dict(
            coordinator_state="Closed", stop_proven=True, failure=""
        ):
            raise ValueError("incomplete final evidence")
        protocol_complete = True
        if release is None:
            raise ValueError("base never released")
        rollout_complete = measured * b.control_dt_us == duration_us
        if not rollout_complete:
            raise ValueError("missing rollout steps")
        if issues:
            raise ValueError("; ".join(issues))
        return metrics, summary
    finally:
        metrics.update(
            released_control_step=release,
            release_logical_us=None
            if release is None
            else 1 + release * b.control_dt_us,
            completed_applied_intervals=applied if started else None,
            measured_rollout_steps=measured if feedback_count else None,
            observed_feedback_steps=feedback_count if started else None,
            observed_logical_duration_us=None
            if last_feedback_step is None
            else last_feedback_step * b.control_dt_us,
            observed_rollout_duration_us=None
            if release is None
            else measured * b.control_dt_us,
            fall_observed=True if fallen else False if rollout_complete else None,
            runtime_fault_observed=True
            if faults
            else False
            if protocol_complete and rollout_complete
            else None,
            unexpected_runtime_faults=len(faults)
            if faults or protocol_complete and rollout_complete
            else None,
            recorded_runtime_fault_count=len(faults)
            if faults or protocol_complete
            else None,
            recorded_protection_count=len(protections)
            if protections or protocol_complete
            else None,
            unexpected_protections=len(protections)
            if protections or protocol_complete and rollout_complete
            else None,
            protection_observed=True
            if protections
            else False
            if protocol_complete and rollout_complete
            else None,
            runner_failure_observed=True
            if runner_failed
            else False
            if protocol_complete
            else None,
            min_base_height_m=minimum_height,
            max_abs_measured_torque_Nm=max_tau,
            torque_measurement_scope="all validated recorded feedback, including startup",
            host_engine_operation_max_us=host_max,
            host_engine_operation_total_us=host_total if host_max is not None else None,
            evidence_complete=None if summary is None else summary["evidence_complete"],
            measured_feedback_complete=None
            if summary is None
            else summary["measured_feedback_complete"],
            protocol_replay_complete=protocol_complete,
            rollout_measurement_coverage_complete=rollout_complete,
        )
        context["velocity_squared_error_sum"] = sum_velocity
        context["audit_issues"] = issues
        context["protocol_replay_complete"] = protocol_complete


def _evaluate(m, report):
    if m.get("schema") != "irmv.policy_acceptance" or m.get("version") != 1:
        raise ValueError("explicit versioned run manifest required")
    finite(m)
    if m["scope"] not in ("trained_policy", "backend_integration_fixture"):
        raise ValueError("explicit policy/fixture scope required")
    # Every file is attested, including failed-attempt logs and original evidence.
    paths = {}
    for key in m["files"]:
        try:
            paths[key] = bound_file(m, key)
        except (OSError, ValueError) as exc:
            # Missing outcome/SDK exports cannot erase an independently bound
            # engine stream. Never read a file whose digest was refused.
            if key not in ("evidence", "runtime_export", "application_export"):
                raise
            paths[key] = None
            report["reasons"].append(str(exc))
    bdoc = load(paths["bootstrap"])
    cfg = load(paths["config"])
    evidence = load(paths["evidence"]) if paths.get("evidence") else {}
    b = Bootstrap(
        **{
            k: (
                bytes.fromhex(v)
                if k
                in (
                    "run_id",
                    "runtime_id",
                    "config_digest",
                    "application_digest",
                    "engine_digest",
                )
                else v
            )
            for k, v in bdoc.items()
        }
    )
    if m["gpu"].get("executed") is not True or m["gpu"].get("skipped") is not False:
        raise ValueError("GPU missing/skipped; cannot accept")
    if m["gpu"]["engine_digest"] != b.engine_digest.hex():
        raise ValueError("GPU binding mismatch")
    if sha256(paths["adapter"]) != m["gpu"]["adapter_sha256"]:
        raise ValueError("adapter binding mismatch")
    engine_binding = dict(
        versions=m["gpu"]["versions"], adapter_sha256=m["gpu"]["adapter_sha256"]
    )
    current = hashlib.sha256(
        json.dumps(engine_binding, sort_keys=True).encode()
    ).hexdigest()
    if (
        current != b.engine_digest.hex()
        or cfg["simulation"]["engine_sha256"] != current
    ):
        raise ValueError("stale engine binding")
    normalized = json.loads(
        subprocess.check_output(
            [str(paths["normalizer"]), str(paths["config"])], text=True, timeout=30
        )
    )
    for key in ("config_digest", "application_digest", "engine_digest"):
        if normalized[key] != getattr(b, key).hex():
            raise ValueError("actual config normalization mismatch: " + key)
    for key in (
        "engine_config",
        "physics_dt_us",
        "control_dt_us",
        "startup_timeout_us",
        "phase_timeout_us",
    ):
        if normalized[key] != getattr(b, key):
            raise ValueError("actual config normalization mismatch: " + key)
    if m["gpu"]["versions"] != {
        "mujoco": "3.9.0",
        "mujoco-warp": "3.9.0",
        "warp-lang": "1.15.0",
    }:
        raise ValueError("unvalidated GPU dependencies")
    asset = load(paths["asset_manifest"])
    if asset.get("policy_scope", "trained_policy") != m["scope"]:
        raise ValueError("fixture cannot upgrade trained-policy acceptance")
    from .assets import admit_initial_geometry

    admit_initial_geometry(b.engine_config, asset)
    from .assets import closure

    if closure(b.engine_config["model_path"]) != asset["prepared_closure"]:
        raise ValueError("asset closure mismatch")
    if cfg["simulation"]["asset_sha256"] != asset["prepared_closure"]["digest"]:
        raise ValueError("config asset mismatch")
    if sha256(paths["policy"]) != asset["policy_sha256"]:
        raise ValueError("policy binding mismatch")
    if any(
        evidence.get(k) != v
        for k, v in dict(
            run_id=b.run_id.hex(),
            runtime_id=b.runtime_id.hex(),
            state="Closed",
            stop_proven=True,
            coordinator_result=0,
            final_child_codes=[0, 0],
            release_state="Released",
            release_event_drops=0,
        ).items()
    ):
        report["reasons"].append("incomplete/failed actual experiment")
    if evidence.get("failure"):
        report["reasons"].append("experiment failure: " + evidence["failure"])
    owner_matches = (
        evidence.get("run_id") == b.run_id.hex()
        and evidence.get("runtime_id") == b.runtime_id.hex()
    )
    report["failure_observation"] = (
        evidence.get("failure_observation", {}) if owner_matches else {}
    )
    observation = report["failure_observation"]
    if any(
        "behavior=Fault" in line
        for line in observation.get("application_failure_lines", [])
    ):
        report["metrics"]["application_fault_observed"] = True
    error = observation.get("max_initialization_target_error_rad")
    if type(error) in (int, float) and math.isfinite(error):
        report["metrics"]["initialization_max_target_error_rad"] = error
    if evidence.get("acceptance_failure"):
        report["reasons"].append(evidence["acceptance_failure"])
    if evidence.get("engine_ready_before_sdk") is not True:
        report["reasons"].append("actual SDK startup absent")
    bounds = m["intent_bounds"]
    intent = scenario_intent(m["scenario"], bounds)
    if intent is None:
        raise ValueError("scenario unsupported within source bounds")
    if m["intent"] != intent:
        raise ValueError("intent does not match configured InputRouter unit keys")
    if m["duration_us"] != 60000000:
        raise ValueError("required 60 logical seconds after release")
    fall = m["fall"]
    if (
        fall["frame"] != "world"
        or fall["body"] != b.engine_config["base_body"]
        or fall["origin_offset_m"] != [0, 0, 0]
    ):
        raise ValueError("fall origin/frame not bound to measured base")
    if fall["minimum_height_m"] <= 0:
        raise ValueError("invalid source fall threshold")
    source = paths["training_env"].read_text()
    # Saved tagged YAML is inspected as text, never executed/deserialized.
    import re

    for name, limits in zip(("lin_vel_x", "lin_vel_y", "ang_vel_z"), bounds):
        match = re.search(
            r"\b" + name + r":\s*!!python/tuple\s*\n\s*- ([^\n]+)\n\s*- ([^\n]+)",
            source,
        )
        if not match or list(map(float, match.groups())) != limits:
            raise ValueError("source intent bounds mismatch")
    match = re.search(
        r"root_height_below_minimum:.*?minimum_height:\s*([0-9.]+)", source, re.S
    )
    if not match or float(match[1]) != fall["minimum_height_m"]:
        raise ValueError("source fall threshold mismatch")
    contract = load(paths["deployment"])
    if contract["source"]["asset_revision"] != asset["source_urdf_sha256"]:
        raise ValueError("deployment source asset mismatch")
    import xml.etree.ElementTree as ET

    urdf = ET.parse(paths["source_urdf"])
    roots = {n.get("name") for n in urdf.findall("link")} - {
        n.get("link") for n in urdf.findall("joint/child")
    }
    if (
        roots != {fall["body"]}
        or sha256(paths["source_urdf"]) != asset["source_urdf_sha256"]
    ):
        raise ValueError("fall actor origin not bound to URDF root")
    if contract["source"]["env_config_sha256"] != sha256(paths["training_env"]):
        raise ValueError("model deployment training source mismatch")
    if m["source_env_sha256"] != sha256(paths["training_env"]):
        raise ValueError("training source digest mismatch")
    metrics = report["metrics"]
    context = {}
    metrics["configured_torque_limits_Nm"] = [
        cfg["runtime"]["safety"]["torque_limits"][j["name"]]
        for j in cfg["robot"]["joints"]
    ]
    metrics["saturation_limit_provenance"] = m["configured_limit_provenance"]
    try:
        audit_evidence(
            paths["sidecar"],
            b,
            m["duration_us"],
            fall,
            intent,
            metrics=metrics,
            context=context,
        )
    except (
        OSError,
        ValueError,
        KeyError,
        TypeError,
        IndexError,
        AttributeError,
        OverflowError,
    ) as exc:
        report["reasons"].append(str(exc))
    for issue in context.get("audit_issues", []):
        if issue not in report["reasons"]:
            report["reasons"].append(issue)
    summary = context.get("summary")
    if (
        summary
        and summary["provenance"].get("engine_factory") != "simulation.main.warp_engine"
    ):
        report["reasons"].append("actual GPU engine factory evidence absent")
    audit_sdk_exports(paths, cfg, b, m, report, context)
    finite(report)


def audit_sdk_exports(paths, cfg, b, m, report, context):
    """Numeric prefixes survive failure; full tracking needs a joined intent grid."""
    metrics = report["metrics"]
    release = metrics["released_control_step"]
    if release is None:
        return  # No released interval exists to measure.
    start = 1 + release * b.control_dt_us
    end = start + m["duration_us"]
    names = [j["name"] for j in cfg["robot"]["joints"]]
    limits = [cfg["runtime"]["safety"]["torque_limits"][n] for n in names]
    rate = cfg["telemetry"]["recorder"]["sample_hz"]
    sample_dt = (1000000 + rate // 2) // rate
    if sample_dt < b.control_dt_us or sample_dt % b.control_dt_us:
        raise ValueError("application sampling must use configured whole control ticks")
    first_index = (start - 1 + sample_dt - 1) // sample_dt
    next_sample = 1 + ((start - 1 + sample_dt - 1) // sample_dt) * sample_dt
    joint_sum = 0.0
    joint_n = 0
    saturated = 0
    torque_n = 0
    runtime_summary = None
    numeric_seen = False
    try:
        require_export_identity(paths.get("runtime_export"), "runtime", b, rate)
        for row in stream_json(paths.get("runtime_export")):
            numeric_seen = True
            kind = row.get("record_type")
            if kind == "sample" and start <= row["stamp_us"] < end:
                if row["stamp_us"] != next_sample:
                    raise ValueError("missing/duplicate numeric joint metric sample")
                next_sample += sample_dt
                if (
                    row["execution_state"] != 3
                    or row["fault_id"]
                    or row["stop_reason"]
                    or not row["send_accepted"]
                    or not row["feedback_valid"]
                    or not row["output_valid"]
                    or not all(row["joint_valid"])
                ):
                    raise ValueError(
                        "unexpected runtime fault/protection/missing output"
                    )
                if any(
                    len(row[k]) != len(names)
                    for k in ("q", "measured_q", "output_torque", "joint_valid")
                ):
                    raise ValueError("joint metric shape")
                for q, actual, tau, limit in zip(
                    row["q"], row["measured_q"], row["output_torque"], limits
                ):
                    joint_sum += (q - actual) ** 2
                    joint_n += 1
                    saturated += abs(tau) >= limit
                    torque_n += 1
            elif kind == "fault":
                metrics["runtime_fault_observed"] = True
                metrics["unexpected_runtime_faults"] = max(
                    metrics["unexpected_runtime_faults"] or 0, 1
                )
                raise ValueError("unexpected fault event")
            elif row.get("record_kind") == "runtime":
                runtime_summary = row
        check_export_summary(runtime_summary, "runtime", b, rate)
        if next_sample < end:
            raise ValueError("incomplete numeric joint metric horizon")
        metrics["joint_metric_coverage_complete"] = bool(
            metrics["rollout_measurement_coverage_complete"]
            and metrics["protocol_replay_complete"]
        )
    except (OSError, ValueError, KeyError, TypeError, IndexError, OverflowError) as exc:
        report["reasons"].append(str(exc))
    finally:
        metrics.update(
            saturation_limit_provenance=m["configured_limit_provenance"],
            configured_torque_limits_Nm=limits,
            joint_rmse_rad=math.sqrt(joint_sum / joint_n) if joint_n else None,
            torque_saturation_fraction=saturated / torque_n if torque_n else None,
            joint_metric_values=joint_n if numeric_seen else None,
        )
    # Merge another streaming replay with the velocity stream; retain only the
    # next row and current consumed state. No unbounded step->state dictionary.
    app_summary = None
    policy_seen = False
    velocities = 0
    app_seen = False
    last_stamp = None
    index = first_index

    def expected(i):
        return (
            1
            + ((i * sample_dt + b.control_dt_us - 1) // b.control_dt_us)
            * b.control_dt_us
        )

    metrics["intent_expected_samples"] = max(
        0, (end - 2) // sample_dt - first_index + 1
    )

    def velocity_rows():
        nonlocal app_summary, policy_seen, app_seen
        for row in stream_json(paths.get("application_export")):
            app_seen = True
            if row.get("record_type") == "velocity" and start <= row["stamp_us"] < end:
                yield row
            elif row.get("record_type") == "policy_session":
                if (
                    row.get("model", {}).get("sha256") == sha256(paths["policy"])
                    and row["model"].get("contract_sha256")
                    == sha256(paths["deployment"])
                    and row.get("runtime_id") == b.runtime_id.hex()
                    and row.get("source") == "local"
                    and type(row.get("logical_us")) is int
                ):
                    policy_seen = True
            elif row.get("record_kind") == "application":
                app_summary = row

    stream = velocity_rows()

    def phase(step, state):
        nonlocal index, velocities, last_stamp
        stamp = 1 + step * b.control_dt_us
        if stamp < start or stamp >= end or stamp < expected(index):
            return
        if stamp != expected(index):
            raise ValueError("missing application sampling phase")
        row = next(stream, None)
        if row is None or row["stamp_us"] != stamp:
            raise ValueError(
                "incomplete application intent horizon: missing scheduled row"
            )
        if (
            type(row.get("sample_seq")) is not int
            or row["sample_seq"] != index
            or row.get("latest_state_seq") != state["sequence"]
            or state["stamp_us"] != stamp
            or row.get("runtime_id") != b.runtime_id.hex()
            or row.get("session_id") != state["session_id"].hex()
            or row.get("identity_known") is not True
        ):
            raise ValueError(
                "application intent sequence/state/session phase association mismatch"
            )
        if [row[k] for k in ("forward", "lateral", "yaw")] != m["intent"]:
            raise ValueError("recorded intent mismatch")
        if last_stamp is not None:
            metrics["intent_max_gap_us"] = max(
                metrics["intent_max_gap_us"] or 0, stamp - last_stamp
            )
        if velocities == 0:
            metrics["intent_first_stamp_us"] = stamp
        velocities += 1
        index += 1
        last_stamp = stamp
        metrics["intent_last_stamp_us"] = stamp

    try:
        require_export_identity(paths.get("application_export"), "application", b, rate)
        replay_context = {}
        try:
            audit_evidence(
                paths["sidecar"],
                b,
                m["duration_us"],
                m["fall"],
                m["intent"],
                context=replay_context,
                phase_visitor=phase,
            )
        except ValueError:
            # Outcome failure may follow a complete, valid protocol replay;
            # corrupted or short replay cannot establish intent coverage.
            if not replay_context.get("summary") or not replay_context.get(
                "protocol_replay_complete"
            ):
                raise
        if next(stream, None) is not None:
            raise ValueError("extra/duplicate application intent row")
        check_export_summary(app_summary, "application", b, rate)
        if expected(index) < end or velocities != metrics["intent_expected_samples"]:
            raise ValueError("incomplete application intent horizon")
        if not policy_seen:
            raise ValueError("missing actual-policy record")
        metrics["intent_coverage_complete"] = True
        if metrics["rollout_measurement_coverage_complete"]:
            metrics["velocity_rmse_yaw_xy_world_yaw_rad_s"] = [
                math.sqrt(v / metrics["measured_rollout_steps"])
                for v in context["velocity_squared_error_sum"]
            ]
    except (
        OSError,
        ValueError,
        KeyError,
        TypeError,
        IndexError,
        AttributeError,
        OverflowError,
    ) as exc:
        report["reasons"].append(str(exc))
    finally:
        stream.close()
        metrics["intent_samples"] = velocities if app_seen else None
        metrics["baseline_command_application_control_cycles"] = 1


def require_export_identity(path, role, b, rate):
    """Bind an export before attributing any metric or fault to this run.

    The SDK export puts its identity summary last. Scan it in bounded memory
    first, without requiring complete data or finite numeric rows. The normal
    data pass still validates every row and can retain a same-run valid prefix.
    No readable identity summary means unavailable SDK observations, regardless
    of the file's manifest SHA or separately bound engine evidence.
    """
    summary = None
    for row in stream_json(path, validate_values=False):
        if row.get("record_kind") in ("runtime", "application"):
            if row["record_kind"] != role or summary is not None:
                raise ValueError("ambiguous SDK export identity summary")
            summary = row
    if summary is None:
        raise ValueError("missing SDK " + role + " identity summary")
    finite(summary)
    check_export_identity(summary, role, b, rate)


def check_export_summary(summary, role, b, rate):
    require_complete_sdk_summary(summary, role)
    check_export_identity(summary, role, b, rate)


def check_export_identity(summary, role, b, rate):
    clock = summary["clock"]
    if (
        clock.get("clock_run_id") != b.run_id.hex()
        or clock.get("clock_domain") != "simulation"
        or clock.get("available") is not True
        or clock.get("logical_origin_us") != 1
        or clock.get("control_dt_us") != b.control_dt_us
        or clock.get("physics_dt_us") != b.physics_dt_us
    ):
        raise ValueError("SDK clock/run mismatch")
    if summary.get("config_id") != b.config_digest.hex():
        raise ValueError("SDK config digest mismatch")
    if summary.get("sample_hz") != rate:
        raise ValueError("SDK configured sampling rate mismatch")
    if (
        role == "application"
        and summary.get("application_config_id") != b.application_digest.hex()
    ):
        raise ValueError("SDK application digest mismatch")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("manifest", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    report = evaluate(load(args.manifest))
    result = json.dumps(report, indent=2, allow_nan=False)
    if args.output:
        args.output.write_text(result + "\n")
    print(result)
    return 0 if report["status"] == "pass" else 1


if __name__ == "__main__":
    raise SystemExit(main())
