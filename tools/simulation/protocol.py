"""Local bridge v1. Independent of DDS and of any simulator dependency."""

import math
import struct
from dataclasses import dataclass
from dataclasses import field as dataclass_field

VERSION = 2
MAX_PACKET_BYTES = 65536
MAX_JOINTS = 64
FRAME_QUEUE_CAPACITY = 32
MAX_PHASE_MANIFEST_ENTRIES = 64
# Ordered wire fields; this schema is independent of the C++ implementation.
SCHEMA = {
    "MessageIdentity": [
        ("MessageKind", "kind"),
        ("Id", "runtime_id"),
        ("Id", "supervisor_id"),
        ("Id", "session_id"),
        ("Id", "producer_id"),
        ("u64", "request_id"),
        ("u64", "epoch"),
        ("u64", "proposed_epoch"),
        ("u64", "sequence"),
        ("u64", "source_state_seq"),
        ("u64", "fault_id"),
        ("u64", "stamp_us"),
        ("u32", "operation"),
        ("u32", "status"),
        ("u32", "reason"),
        ("u32", "execution_state"),
        ("u32", "output_action"),
        ("Digest", "content_digest"),
    ],
    "Hello": [
        ("Role", "role"),
        ("Digest", "config_digest"),
        ("Digest", "application_digest"),
        ("Digest", "engine_digest"),
        ("Id", "runtime_id"),
        ("u64", "physics_dt_us"),
        ("u64", "control_dt_us"),
    ],
    "Feedback": [
        ("u64", "physics_step"),
        ("f64", "engine_time_s"),
        ("Joints", "q"),
        ("Joints", "dq"),
        ("Joints", "tau"),
        ("A4", "imu_quaternion"),
        ("A3", "imu_angular_velocity"),
        ("A3", "imu_specific_force"),
        ("A3", "base_position"),
        ("A6", "base_velocity"),
        ("bool", "valid"),
        ("bool", "imu_valid"),
    ],
    "Torque": [
        ("u64", "write_id"),
        ("bool", "accepted"),
        ("u64", "state_seq"),
        ("MessageIdentity", "command"),
        ("Joints", "tau"),
    ],
    "Manifest": [("Role", "sender"), ("MessageIdentity", "message")],
    "Ack": [
        ("Role", "consumer"),
        ("Disposition", "disposition"),
        ("Manifest", "manifest"),
    ],
    "ReleaseArm": [
        ("MessageIdentity", "tracking_request"),
        ("MessageIdentity", "first_command"),
        ("MessageIdentity", "applied_result"),
        ("MessageIdentity", "tracking_state"),
    ],
    "LeaveReady": [
        ("MessageIdentity", "stop_request"),
        ("MessageIdentity", "stop_result"),
        ("MessageIdentity", "stopped_state"),
    ],
    "LeaveCommitted": [
        ("LeaveReady", "proof"),
        ("bool", "authorization_revoked"),
        ("bool", "session_closed"),
        ("bool", "heartbeat_resolved"),
    ],
    "Abort": [("u32", "code"), ("Text256", "reason")],
    "AppliedToStep": [
        ("u64", "write_id"),
        ("u64", "physics_step_begin"),
        ("u64", "physics_step_end"),
    ],
    "PhaseSeal": [
        ("Role", "sender"),
        ("u32", "manifest_count"),
        ("Digest", "manifest_digest"),
    ],
    "PhaseAck": [
        ("Role", "consumer"),
        ("Disposition", "disposition"),
        ("PhaseSeal", "seal"),
    ],
    "InventoryReady": [("Role", "consumer"), ("PhaseSeal", "seal")],
    "PublishPermit": [("PhaseSeal", "seal")],
    "TickPermit": [("Role", "role"), ("u32", "stage")],
    "FirstCommandProof": [
        ("MessageIdentity", "request"),
        ("MessageIdentity", "command"),
    ],
}
SCHEMA.update(
    ShutdownPermit=[("Role", "role")],
    RuntimeOnlyPermit=[("Role", "role")],
    ShutdownCommitted=[("u64", "write_id"), ("u64", "state_seq")],
    ReleaseCancel=[("Role", "role")],
)
KINDS = [
    "Hello",
    "Feedback",
    "Torque",
    "Manifest",
    "Ack",
    "ReleaseArm",
    "LeaveReady",
    "LeaveCommitted",
    "Abort",
    "AppliedToStep",
    "PhaseSeal",
    "PhaseAck",
    "InventoryReady",
    "PublishPermit",
    "TickPermit",
    "FirstCommandProof",
    "ShutdownPermit",
    "RuntimeOnlyPermit",
    "ShutdownCommitted",
    "ReleaseCancel",
]

# Internal, precompiled views of SCHEMA. They only remove repeated layout work:
# the packed bytes, field order, validation and rejection behaviour are those
# the schema-derived implementation already produced.
_LAYOUT = {name: tuple(fields) for name, fields in SCHEMA.items()}
_FIELDS = {
    name: frozenset(field for _, field in fields) for name, fields in SCHEMA.items()
}
_SCALARS = {
    "u64": struct.Struct("<Q"),
    "u32": struct.Struct("<I"),
    "f64": struct.Struct("<d"),
    "bool": struct.Struct("<B"),
}
_ARRAYS = {}
_JOINTS = {}
_U32 = struct.Struct("<I")
_HEADER = struct.Struct("<IHH")
_FRAME_HEAD = struct.Struct("<QIQQQ")
_ENUM_LIMITS = {"Role": 3, "MessageKind": 5, "Disposition": 2}


def _enum_paths(name, prefix=()):
    paths = []
    for field, key in _LAYOUT[name]:
        path = prefix + (key,)
        if field in _LAYOUT:
            paths.extend(_enum_paths(field, path))
        elif field in _ENUM_LIMITS:
            paths.append((path, field))
    return paths


_ENUM_PATHS = {name: _enum_paths(name) for name in _LAYOUT}


def _float_paths(name, prefix=()):
    paths = []
    for field, key in _LAYOUT[name]:
        path = prefix + (key,)
        if field in _LAYOUT:
            paths.extend(_float_paths(field, path))
        elif field == "f64":
            paths.append((path, True))
        elif field == "Joints" or field.startswith("A"):
            paths.append((path, False))
    return paths


_FLOAT_PATHS = {name: _float_paths(name) for name in _LAYOUT}


@dataclass(frozen=True)
class Frame:
    kind: int
    payload: dict = dataclass_field(default_factory=dict)
    run_id: bytes = bytes(16)
    control_step: int = 0
    phase: int = 0
    logical_us: int = 1
    physics_dt_us: int = 1000
    control_dt_us: int = 2000


def _default(t):
    if t in ("Id", "Digest"):
        return bytes(range(1, (16 if t == "Id" else 32) + 1))
    if t in ("A3", "A4", "A6"):
        return tuple(((i + 1) * 0.125 for i in range(int(t[1:]))))
    if t == "Joints":
        return (1.25, -2.5)
    if t == "Text256":
        return b"stop\x00proof"
    if t in SCHEMA:
        value = {n: _default(t2) for t2, n in SCHEMA[t]}
        if t in ("MessageIdentity", "Hello"):
            for index, (kind, name) in enumerate(SCHEMA[t]):
                if kind in ("Id", "Digest"):
                    value[name] = bytes(
                        (i + 1 + index * 17) % 256
                        for i in range(16 if kind == "Id" else 32)
                    )
                elif t == "MessageIdentity" and kind in ("u64", "u32"):
                    value[name] += index
        return value
    if t in ("Role", "MessageKind", "Disposition"):
        return 1
    return {"u64": 72623859790382856, "u32": 16909060, "f64": 1.25, "bool": 1}[t]


def _pack(t, v):
    spec = _LAYOUT.get(t)
    if spec is not None:
        if set(v) != _FIELDS[t]:
            raise ValueError("payload fields")
        return b"".join([_pack(t2, v[n]) for t2, n in spec])
    if t in ("Id", "Digest"):
        if not isinstance(v, bytes) or len(v) != (16 if t == "Id" else 32):
            raise ValueError("identity size")
        return v
    if t in ("Joints", "Text256"):
        if len(v) > (64 if t == "Joints" else 256):
            raise ValueError("capacity")
        if t == "Text256":
            return _U32.pack(len(v)) + bytes(v)
        n = len(v)
        if not all(map(math.isfinite, v)):
            raise ValueError("nonfinite")
        packed = _JOINTS.get(n)
        if packed is None:
            packed = _JOINTS[n] = struct.Struct("<%dd" % n)
        return _U32.pack(n) + packed.pack(*v)
    if t.startswith("A"):
        n = int(t[1:])
        if len(v) != n:
            raise ValueError("shape")
        if not all(map(math.isfinite, v)):
            raise ValueError("nonfinite")
        packed = _ARRAYS.get(t)
        if packed is None:
            packed = _ARRAYS[t] = struct.Struct("<%dd" % n)
        return packed.pack(*v)
    if t == "f64" and (not math.isfinite(v)):
        raise ValueError("nonfinite")
    if t == "bool" and v not in (0, 1):
        raise ValueError("boolean")
    return _SCALARS.get(t, _U32).pack(v)


def _valid(f):
    if not 1 <= f.kind <= len(KINDS):
        raise ValueError("type")
    if (
        not 0 < f.physics_dt_us <= f.control_dt_us < 2**64
        or f.control_dt_us % f.physics_dt_us
    ):
        raise ValueError("dt")
    if (
        not 0 <= f.control_step <= (2**64 - 2) // f.control_dt_us
        or f.logical_us != 1 + f.control_step * f.control_dt_us
    ):
        raise ValueError("step/time overflow")
    v = f.payload
    if f.kind in (11, 12, 13, 14):
        seal = v if f.kind == 11 else v["seal"]
        if not 0 <= seal["manifest_count"] <= MAX_PHASE_MANIFEST_ENTRIES:
            raise ValueError("phase inventory capacity")
    if f.kind == 16 and (v["request"]["kind"] != 1 or v["command"]["kind"] != 5):
        raise ValueError("first-command proof kinds")
    if f.kind == 15 and v["stage"] not in (1, 2):
        raise ValueError("tick stage")
    if f.kind == 1 and (
        v["physics_dt_us"] != f.physics_dt_us or v["control_dt_us"] != f.control_dt_us
    ):
        raise ValueError("hello dt")
    if f.kind == 2 and (not len(v["q"]) == len(v["dq"]) == len(v["tau"])):
        raise ValueError("feedback shape")
    if f.kind == 10 and v["physics_step_end"] < v["physics_step_begin"]:
        raise ValueError("physics order")

    for path, field in _ENUM_PATHS[KINDS[f.kind - 1]]:
        value = v
        for key in path:
            value = value[key]
        if not 1 <= value <= _ENUM_LIMITS[field]:
            raise ValueError("enum")
    # Value validation owns finiteness for every f64 position, including whole
    # joint and IMU/base vectors. The codec also rejects these while packing or
    # unpacking; this keeps any Frame with non-finite measured data invalid no
    # matter how it was built.
    for path, scalar in _FLOAT_PATHS[KINDS[f.kind - 1]]:
        value = v
        for key in path:
            value = value[key]
        if not all(map(math.isfinite, (value,) if scalar else value)):
            raise ValueError("nonfinite")


def encode(frame: Frame) -> bytes:
    try:
        _valid(frame)
        body = _pack("Id", frame.run_id) + _FRAME_HEAD.pack(
            frame.control_step,
            frame.phase,
            frame.logical_us,
            frame.physics_dt_us,
            frame.control_dt_us,
        )
        body += _pack(KINDS[frame.kind - 1], frame.payload)
        if len(body) + 8 > MAX_PACKET_BYTES:
            raise ValueError("packet capacity")
        return _HEADER.pack(len(body) + 8, VERSION, frame.kind) + body
    except (struct.error, OverflowError, TypeError, KeyError) as e:
        raise ValueError("invalid frame") from e


def _unpack(t, payload, pos):
    spec = _LAYOUT.get(t)
    if spec is not None:
        value = {}
        for field, key in spec:
            value[key], pos = _unpack(field, payload, pos)
        return value, pos
    if t in ("Id", "Digest"):
        size = 16 if t == "Id" else 32
        if size > len(payload) - pos:
            raise ValueError("truncated")
        return payload[pos : pos + size], pos + size
    if t in ("Joints", "Text256"):
        if 4 > len(payload) - pos:
            raise ValueError("truncated")
        n = _U32.unpack_from(payload, pos)[0]
        pos += 4
        if n > (64 if t == "Joints" else 256):
            raise ValueError("capacity")
        size = 8 * n if t == "Joints" else n
        if size > len(payload) - pos:
            raise ValueError("truncated")
        end = pos + size
        if t == "Text256":
            return payload[pos:end], end
        if not n:
            return (), pos
        packed = _JOINTS.get(n)
        if packed is None:
            packed = _JOINTS[n] = struct.Struct("<%dd" % n)
        values = packed.unpack_from(payload, pos)
        # Bulk unpacking must not weaken the per-element f64 rule: a joint
        # value is rejected exactly like a scalar f64 if it is not finite.
        if not all(map(math.isfinite, values)):
            raise ValueError("nonfinite")
        return values, end
    if t.startswith("A"):
        n = int(t[1:])
        size = 8 * n
        if size > len(payload) - pos:
            raise ValueError("truncated")
        packed = _ARRAYS.get(t)
        if packed is None:
            packed = _ARRAYS[t] = struct.Struct("<%dd" % n)
        values = packed.unpack_from(payload, pos)
        if not all(map(math.isfinite, values)):
            raise ValueError("nonfinite")
        return values, pos + size
    packed = _SCALARS.get(t, _U32)
    if packed.size > len(payload) - pos:
        raise ValueError("truncated")
    value = packed.unpack_from(payload, pos)[0]
    if t == "f64" and (not math.isfinite(value)):
        raise ValueError("nonfinite")
    if t == "bool" and value > 1:
        raise ValueError("boolean")
    return value, pos + packed.size


def decode(payload: bytes) -> Frame:
    if len(payload) > MAX_PACKET_BYTES:
        raise ValueError("packet capacity")
    if 8 > len(payload):
        raise ValueError("truncated")
    length, version, kind = _HEADER.unpack_from(payload, 0)
    if length != len(payload) or version != VERSION or (not 1 <= kind <= len(KINDS)):
        raise ValueError("header")
    pos = 8
    if 16 + _FRAME_HEAD.size > len(payload) - pos:
        raise ValueError("truncated")
    run = payload[pos : pos + 16]
    pos += 16
    step, phase, logical, physics, control = _FRAME_HEAD.unpack_from(payload, pos)
    pos += _FRAME_HEAD.size
    body, pos = _unpack(KINDS[kind - 1], payload, pos)
    f = Frame(kind, body, run, step, phase, logical, physics, control)
    if pos != len(payload):
        raise ValueError("trailing bytes")
    _valid(f)
    return f


def matches_hello(a, b):
    return all(
        (
            a[n] == b[n]
            for n in (
                "config_digest",
                "application_digest",
                "engine_digest",
                "runtime_id",
                "physics_dt_us",
                "control_dt_us",
            )
        )
    )


def golden_frames():
    result = []
    for i, k in enumerate(KINDS, 1):
        v = _default(k)
        if k == "PhaseSeal":
            v.update(sender=2, manifest_count=33)
        if k == "PhaseAck":
            v.update(consumer=3, disposition=2)
            v["seal"].update(sender=2, manifest_count=33)
        if k in ("InventoryReady", "PublishPermit"):
            v["seal"].update(sender=2, manifest_count=33)
            if k == "InventoryReady":
                v["consumer"] = 3
        if k == "FirstCommandProof":
            v["command"]["kind"] = 5
        if k == "TickPermit":
            v.update(role=2, stage=1)
        if k == "Hello":
            v.update(physics_dt_us=1000, control_dt_us=2000)
        result.append(
            Frame(i, v, bytes(range(1, 17)), 66051, 16909060, 1 + 66051 * 2000)
        )
    return result


# Literal v2 fixtures with deliberately distinct identity fields.
GOLDEN_HEX = (
    "c0000000020001000102030405060708090a0b0c0d0e0f1003020100000000000403020171b7df0700000000e803000000000000d0070000000000000100000012131415161718191a1b1c1d1e1f202122232425262728292a2b2c2d2e2f3031232425262728292a2b2c2d2e2f303132333435363738393a3b3c3d3e3f4041423435363738393a3b3c3d3e3f404142434445464748494a4b4c4d4e4f5051525345464748494a4b4c4d4e4f5051525354e803000000000000d007000000000000",
    "22010000020002000102030405060708090a0b0c0d0e0f1003020100000000000403020171b7df0700000000e803000000000000d0070000000000000807060504030201000000000000f43f02000000000000000000f43f00000000000004c002000000000000000000f43f00000000000004c002000000000000000000f43f00000000000004c0000000000000c03f000000000000d03f000000000000d83f000000000000e03f000000000000c03f000000000000d03f000000000000d83f000000000000c03f000000000000d03f000000000000d83f000000000000c03f000000000000d03f000000000000d83f000000000000c03f000000000000d03f000000000000d83f000000000000e03f000000000000e43f000000000000e83f0101",
    "11010000020003000102030405060708090a0b0c0d0e0f1003020100000000000403020171b7df0700000000e803000000000000d00700000000000008070605040302010108070605040302010100000012131415161718191a1b1c1d1e1f2021232425262728292a2b2c2d2e2f3031323435363738393a3b3c3d3e3f4041424345464748494a4b4c4d4e4f50515253540d070605040302010e070605040302010f070605040302011007060504030201110706050403020112070605040302011307060504030201100302011103020112030201130302011403020122232425262728292a2b2c2d2e2f303132333435363738393a3b3c3d3e3f404102000000000000000000f43f00000000000004c0",
    "f0000000020004000102030405060708090a0b0c0d0e0f1003020100000000000403020171b7df0700000000e803000000000000d007000000000000010000000100000012131415161718191a1b1c1d1e1f2021232425262728292a2b2c2d2e2f3031323435363738393a3b3c3d3e3f4041424345464748494a4b4c4d4e4f50515253540d070605040302010e070605040302010f070605040302011007060504030201110706050403020112070605040302011307060504030201100302011103020112030201130302011403020122232425262728292a2b2c2d2e2f303132333435363738393a3b3c3d3e3f4041",
    "f8000000020005000102030405060708090a0b0c0d0e0f1003020100000000000403020171b7df0700000000e803000000000000d0070000000000000100000001000000010000000100000012131415161718191a1b1c1d1e1f2021232425262728292a2b2c2d2e2f3031323435363738393a3b3c3d3e3f4041424345464748494a4b4c4d4e4f50515253540d070605040302010e070605040302010f070605040302011007060504030201110706050403020112070605040302011307060504030201100302011103020112030201130302011403020122232425262728292a2b2c2d2e2f303132333435363738393a3b3c3d3e3f4041",
    "fc020000020006000102030405060708090a0b0c0d0e0f1003020100000000000403020171b7df0700000000e803000000000000d0070000000000000100000012131415161718191a1b1c1d1e1f2021232425262728292a2b2c2d2e2f3031323435363738393a3b3c3d3e3f4041424345464748494a4b4c4d4e4f50515253540d070605040302010e070605040302010f070605040302011007060504030201110706050403020112070605040302011307060504030201100302011103020112030201130302011403020122232425262728292a2b2c2d2e2f303132333435363738393a3b3c3d3e3f40410100000012131415161718191a1b1c1d1e1f2021232425262728292a2b2c2d2e2f3031323435363738393a3b3c3d3e3f4041424345464748494a4b4c4d4e4f50515253540d070605040302010e070605040302010f070605040302011007060504030201110706050403020112070605040302011307060504030201100302011103020112030201130302011403020122232425262728292a2b2c2d2e2f303132333435363738393a3b3c3d3e3f40410100000012131415161718191a1b1c1d1e1f2021232425262728292a2b2c2d2e2f3031323435363738393a3b3c3d3e3f4041424345464748494a4b4c4d4e4f50515253540d070605040302010e070605040302010f070605040302011007060504030201110706050403020112070605040302011307060504030201100302011103020112030201130302011403020122232425262728292a2b2c2d2e2f303132333435363738393a3b3c3d3e3f40410100000012131415161718191a1b1c1d1e1f2021232425262728292a2b2c2d2e2f3031323435363738393a3b3c3d3e3f4041424345464748494a4b4c4d4e4f50515253540d070605040302010e070605040302010f070605040302011007060504030201110706050403020112070605040302011307060504030201100302011103020112030201130302011403020122232425262728292a2b2c2d2e2f303132333435363738393a3b3c3d3e3f4041",
    "4c020000020007000102030405060708090a0b0c0d0e0f1003020100000000000403020171b7df0700000000e803000000000000d0070000000000000100000012131415161718191a1b1c1d1e1f2021232425262728292a2b2c2d2e2f3031323435363738393a3b3c3d3e3f4041424345464748494a4b4c4d4e4f50515253540d070605040302010e070605040302010f070605040302011007060504030201110706050403020112070605040302011307060504030201100302011103020112030201130302011403020122232425262728292a2b2c2d2e2f303132333435363738393a3b3c3d3e3f40410100000012131415161718191a1b1c1d1e1f2021232425262728292a2b2c2d2e2f3031323435363738393a3b3c3d3e3f4041424345464748494a4b4c4d4e4f50515253540d070605040302010e070605040302010f070605040302011007060504030201110706050403020112070605040302011307060504030201100302011103020112030201130302011403020122232425262728292a2b2c2d2e2f303132333435363738393a3b3c3d3e3f40410100000012131415161718191a1b1c1d1e1f2021232425262728292a2b2c2d2e2f3031323435363738393a3b3c3d3e3f4041424345464748494a4b4c4d4e4f50515253540d070605040302010e070605040302010f070605040302011007060504030201110706050403020112070605040302011307060504030201100302011103020112030201130302011403020122232425262728292a2b2c2d2e2f303132333435363738393a3b3c3d3e3f4041",
    "4f020000020008000102030405060708090a0b0c0d0e0f1003020100000000000403020171b7df0700000000e803000000000000d0070000000000000100000012131415161718191a1b1c1d1e1f2021232425262728292a2b2c2d2e2f3031323435363738393a3b3c3d3e3f4041424345464748494a4b4c4d4e4f50515253540d070605040302010e070605040302010f070605040302011007060504030201110706050403020112070605040302011307060504030201100302011103020112030201130302011403020122232425262728292a2b2c2d2e2f303132333435363738393a3b3c3d3e3f40410100000012131415161718191a1b1c1d1e1f2021232425262728292a2b2c2d2e2f3031323435363738393a3b3c3d3e3f4041424345464748494a4b4c4d4e4f50515253540d070605040302010e070605040302010f070605040302011007060504030201110706050403020112070605040302011307060504030201100302011103020112030201130302011403020122232425262728292a2b2c2d2e2f303132333435363738393a3b3c3d3e3f40410100000012131415161718191a1b1c1d1e1f2021232425262728292a2b2c2d2e2f3031323435363738393a3b3c3d3e3f4041424345464748494a4b4c4d4e4f50515253540d070605040302010e070605040302010f070605040302011007060504030201110706050403020112070605040302011307060504030201100302011103020112030201130302011403020122232425262728292a2b2c2d2e2f303132333435363738393a3b3c3d3e3f4041010101",
    "4e000000020009000102030405060708090a0b0c0d0e0f1003020100000000000403020171b7df0700000000e803000000000000d007000000000000040302010a00000073746f700070726f6f66",
    "5400000002000a000102030405060708090a0b0c0d0e0f1003020100000000000403020171b7df0700000000e803000000000000d007000000000000080706050403020108070605040302010807060504030201",
    "6400000002000b000102030405060708090a0b0c0d0e0f1003020100000000000403020171b7df0700000000e803000000000000d00700000000000002000000210000000102030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f20",
    "6c00000002000c000102030405060708090a0b0c0d0e0f1003020100000000000403020171b7df0700000000e803000000000000d007000000000000030000000200000002000000210000000102030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f20",
    "6800000002000d000102030405060708090a0b0c0d0e0f1003020100000000000403020171b7df0700000000e803000000000000d0070000000000000300000002000000210000000102030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f20",
    "6400000002000e000102030405060708090a0b0c0d0e0f1003020100000000000403020171b7df0700000000e803000000000000d00700000000000002000000210000000102030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f20",
    "4400000002000f000102030405060708090a0b0c0d0e0f1003020100000000000403020171b7df0700000000e803000000000000d0070000000000000200000001000000",
    "9c010000020010000102030405060708090a0b0c0d0e0f1003020100000000000403020171b7df0700000000e803000000000000d0070000000000000100000012131415161718191a1b1c1d1e1f2021232425262728292a2b2c2d2e2f3031323435363738393a3b3c3d3e3f4041424345464748494a4b4c4d4e4f50515253540d070605040302010e070605040302010f070605040302011007060504030201110706050403020112070605040302011307060504030201100302011103020112030201130302011403020122232425262728292a2b2c2d2e2f303132333435363738393a3b3c3d3e3f40410500000012131415161718191a1b1c1d1e1f2021232425262728292a2b2c2d2e2f3031323435363738393a3b3c3d3e3f4041424345464748494a4b4c4d4e4f50515253540d070605040302010e070605040302010f070605040302011007060504030201110706050403020112070605040302011307060504030201100302011103020112030201130302011403020122232425262728292a2b2c2d2e2f303132333435363738393a3b3c3d3e3f4041",
    "40000000020011000102030405060708090a0b0c0d0e0f1003020100000000000403020171b7df0700000000e803000000000000d00700000000000001000000",
    "40000000020012000102030405060708090a0b0c0d0e0f1003020100000000000403020171b7df0700000000e803000000000000d00700000000000001000000",
    "4c000000020013000102030405060708090a0b0c0d0e0f1003020100000000000403020171b7df0700000000e803000000000000d00700000000000008070605040302010807060504030201",
    "40000000020014000102030405060708090a0b0c0d0e0f1003020100000000000403020171b7df0700000000e803000000000000d00700000000000001000000",
)
