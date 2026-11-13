"""Local bridge v1. Independent of DDS and of any simulator dependency."""
from dataclasses import dataclass, field
import math
import struct
VERSION = 1
MAX_PACKET_BYTES = 65536
MAX_JOINTS = 64
FRAME_QUEUE_CAPACITY = 32
MAX_PHASE_MANIFEST_ENTRIES = 64
# Ordered wire fields; this schema is independent of the C++ implementation.
SCHEMA = {'MessageIdentity': [('MessageKind', 'kind'),
                     ('Id', 'runtime_id'),
                     ('Id', 'supervisor_id'),
                     ('Id', 'session_id'),
                     ('Id', 'producer_id'),
                     ('u64', 'request_id'),
                     ('u64', 'epoch'),
                     ('u64', 'proposed_epoch'),
                     ('u64', 'sequence'),
                     ('u64', 'source_state_seq'),
                     ('u64', 'fault_id'),
                     ('u64', 'stamp_us'),
                     ('u32', 'operation'),
                     ('u32', 'status'),
                     ('u32', 'reason'),
                     ('u32', 'execution_state'),
                     ('u32', 'output_action'),
                     ('Digest', 'content_digest')],
 'Hello': [('Role', 'role'),
           ('Digest', 'config_digest'),
           ('Digest', 'application_digest'),
           ('Digest', 'engine_digest'),
           ('Id', 'runtime_id'),
           ('u64', 'physics_dt_us'),
           ('u64', 'control_dt_us')],
 'Feedback': [('u64', 'physics_step'),
              ('f64', 'engine_time_s'),
              ('Joints', 'q'),
              ('Joints', 'dq'),
              ('Joints', 'tau'),
              ('A4', 'imu_quaternion'),
              ('A3', 'imu_angular_velocity'),
              ('A3', 'imu_specific_force'),
              ('A3', 'base_position'),
              ('A6', 'base_velocity'),
              ('bool', 'valid'),
              ('bool', 'imu_valid')],
 'Torque': [('u64', 'write_id'),
            ('bool', 'accepted'),
            ('u64', 'state_seq'),
            ('MessageIdentity', 'command'),
            ('Joints', 'tau')],
 'Manifest': [('Role', 'sender'), ('MessageIdentity', 'message')],
 'Ack': [('Role', 'consumer'),
         ('Disposition', 'disposition'),
         ('Manifest', 'manifest')],
 'ReleaseArm': [('MessageIdentity', 'tracking_request'),
                ('MessageIdentity', 'first_command'),
                ('MessageIdentity', 'applied_result'),
                ('MessageIdentity', 'tracking_state')],
 'LeaveReady': [('MessageIdentity', 'stop_request'),
                ('MessageIdentity', 'stop_result'),
                ('MessageIdentity', 'stopped_state')],
 'LeaveCommitted': [('LeaveReady', 'proof'),
                    ('bool', 'authorization_revoked'),
                    ('bool', 'session_closed'),
                    ('bool', 'heartbeat_resolved')],
 'Abort': [('u32', 'code'), ('Text256', 'reason')],
 'AppliedToStep': [('u64', 'write_id'),
                   ('u64', 'physics_step_begin'),
                   ('u64', 'physics_step_end')],
 'PhaseSeal': [('Role', 'sender'), ('u32', 'manifest_count'),
               ('Digest', 'manifest_digest')],
 'PhaseAck': [('Role', 'consumer'), ('Disposition', 'disposition'),
              ('PhaseSeal', 'seal')],
 'InventoryReady': [('Role', 'consumer'), ('PhaseSeal', 'seal')],
 'PublishPermit': [('PhaseSeal', 'seal')],
 'TickPermit': [('Role', 'role'), ('u32', 'stage')]}
KINDS = ['Hello', 'Feedback', 'Torque', 'Manifest', 'Ack', 'ReleaseArm', 'LeaveReady', 'LeaveCommitted', 'Abort', 'AppliedToStep', 'PhaseSeal', 'PhaseAck', 'InventoryReady', 'PublishPermit', 'TickPermit']

@dataclass(frozen=True)
class Frame:
    kind: int
    payload: dict = field(default_factory=dict)
    run_id: bytes = bytes(16)
    control_step: int = 0
    phase: int = 0
    logical_us: int = 1
    physics_dt_us: int = 1000
    control_dt_us: int = 2000

def _default(t):
    if t in ('Id', 'Digest'):
        return bytes(range(1, (16 if t == 'Id' else 32) + 1))
    if t in ('A3', 'A4', 'A6'):
        return tuple(((i + 1) * 0.125 for i in range(int(t[1:]))))
    if t == 'Joints':
        return (1.25, -2.5)
    if t == 'Text256':
        return b'stop\x00proof'
    if t in SCHEMA:
        return {n: _default(t2) for t2, n in SCHEMA[t]}
    if t in ('Role', 'MessageKind', 'Disposition'):
        return 1
    return {'u64': 72623859790382856, 'u32': 16909060, 'f64': 1.25, 'bool': 1}[t]

def _pack(t, v):
    if t in SCHEMA:
        if set(v) != {n for _, n in SCHEMA[t]}:
            raise ValueError('payload fields')
        return b''.join((_pack(t2, v[n]) for t2, n in SCHEMA[t]))
    if t in ('Id', 'Digest'):
        if not isinstance(v, bytes) or len(v) != (16 if t == 'Id' else 32):
            raise ValueError('identity size')
        return v
    if t in ('Joints', 'Text256'):
        if len(v) > (64 if t == 'Joints' else 256):
            raise ValueError('capacity')
        return struct.pack('<I', len(v)) + (b''.join((_pack('f64', x) for x in v)) if t == 'Joints' else bytes(v))
    if t.startswith('A'):
        if len(v) != int(t[1:]):
            raise ValueError('shape')
        return b''.join((_pack('f64', x) for x in v))
    if t == 'f64' and (not math.isfinite(v)):
        raise ValueError('nonfinite')
    if t == 'bool' and v not in (0, 1):
        raise ValueError('boolean')
    return struct.pack('<' + {'u64': 'Q', 'u32': 'I', 'f64': 'd', 'bool': 'B'}.get(t, 'I'), v)

def _valid(f):
    if not 1 <= f.kind <= len(KINDS):
        raise ValueError('type')
    if not 0 < f.physics_dt_us <= f.control_dt_us < 2 ** 64 or f.control_dt_us % f.physics_dt_us:
        raise ValueError('dt')
    if not 0 <= f.control_step <= (2 ** 64 - 2) // f.control_dt_us or f.logical_us != 1 + f.control_step * f.control_dt_us:
        raise ValueError('step/time overflow')
    v = f.payload
    if f.kind in (11, 12, 13, 14):
        seal = v if f.kind == 11 else v['seal']
        if not 0 <= seal['manifest_count'] <= MAX_PHASE_MANIFEST_ENTRIES:
            raise ValueError('phase inventory capacity')
    if f.kind == 15 and v['stage'] not in (1, 2):
        raise ValueError('tick stage')
    if f.kind == 1 and (v['physics_dt_us'] != f.physics_dt_us or v['control_dt_us'] != f.control_dt_us):
        raise ValueError('hello dt')
    if f.kind == 2 and (not len(v['q']) == len(v['dq']) == len(v['tau'])):
        raise ValueError('feedback shape')
    if f.kind == 10 and v['physics_step_end'] < v['physics_step_begin']:
        raise ValueError('physics order')

    def enums(t, v):
        if t in SCHEMA:
            for t2, n in SCHEMA[t]:
                enums(t2, v[n])
        elif t in ('Role', 'MessageKind', 'Disposition') and (not 1 <= v <= {'Role': 3, 'MessageKind': 5, 'Disposition': 2}[t]):
            raise ValueError('enum')
    enums(KINDS[f.kind - 1], v)

def encode(frame: Frame) -> bytes:
    try:
        _valid(frame)
        body = _pack('Id', frame.run_id) + struct.pack('<QIQQQ', frame.control_step, frame.phase, frame.logical_us, frame.physics_dt_us, frame.control_dt_us)
        body += _pack(KINDS[frame.kind - 1], frame.payload)
        if len(body) + 8 > MAX_PACKET_BYTES:
            raise ValueError('packet capacity')
        return struct.pack('<IHH', len(body) + 8, VERSION, frame.kind) + body
    except (struct.error, OverflowError, TypeError, KeyError) as e:
        raise ValueError('invalid frame') from e

def decode(payload: bytes) -> Frame:
    pos = 0

    def take(n):
        nonlocal pos
        if n > len(payload) - pos:
            raise ValueError('truncated')
        v = payload[pos:pos + n]
        pos += n
        return v

    def unpack(t):
        if t in SCHEMA:
            return {n: unpack(t2) for t2, n in SCHEMA[t]}
        if t in ('Id', 'Digest'):
            return take(16 if t == 'Id' else 32)
        if t in ('Joints', 'Text256'):
            n = struct.unpack('<I', take(4))[0]
            if n > (64 if t == 'Joints' else 256):
                raise ValueError('capacity')
            return tuple((unpack('f64') for _ in range(n))) if t == 'Joints' else take(n)
        if t.startswith('A'):
            return tuple((unpack('f64') for _ in range(int(t[1:]))))
        fmt = {'u64': 'Q', 'u32': 'I', 'f64': 'd', 'bool': 'B'}.get(t, 'I')
        v = struct.unpack('<' + fmt, take(struct.calcsize(fmt)))[0]
        if t == 'f64' and (not math.isfinite(v)):
            raise ValueError('nonfinite')
        if t == 'bool' and v > 1:
            raise ValueError('boolean')
        return v
    if len(payload) > MAX_PACKET_BYTES:
        raise ValueError('packet capacity')
    length, version, kind = struct.unpack('<IHH', take(8))
    if length != len(payload) or version != VERSION or (not 1 <= kind <= len(KINDS)):
        raise ValueError('header')
    run = take(16)
    step, phase, logical, physics, control = struct.unpack('<QIQQQ', take(36))
    f = Frame(kind, unpack(KINDS[kind - 1]), run, step, phase, logical, physics, control)
    if pos != len(payload):
        raise ValueError('trailing bytes')
    _valid(f)
    return f

def matches_hello(a, b):
    return all((a[n] == b[n] for n in ('config_digest', 'application_digest', 'engine_digest', 'runtime_id', 'physics_dt_us', 'control_dt_us')))

def golden_frames():
    result = []
    for i, k in enumerate(KINDS, 1):
        v = _default(k)
        if k == 'PhaseSeal':
            v.update(sender=2, manifest_count=33)
        if k == 'PhaseAck':
            v.update(consumer=3, disposition=2)
            v['seal'].update(sender=2, manifest_count=33)
        if k in ('InventoryReady', 'PublishPermit'):
            v['seal'].update(sender=2, manifest_count=33)
            if k == 'InventoryReady': v['consumer']=3
        if k == 'TickPermit':
            v.update(role=2, stage=1)
        if k == 'Hello':
            v.update(physics_dt_us=1000, control_dt_us=2000)
        result.append(Frame(i, v, bytes(range(1, 17)), 66051, 16909060, 1 + 66051 * 2000))
    return result
# Literal v1 golden packets, including nonzero IDs, indices, and sensors.
GOLDEN_HEX = (
    (
        'c0000000010001000102030405060708090a0b0c0d0e0f1003020100000000000403020171b7df07'
        '00000000e803000000000000d007000000000000010000000102030405060708090a0b0c0d0e0f10'
        '1112131415161718191a1b1c1d1e1f200102030405060708090a0b0c0d0e0f101112131415161718'
        '191a1b1c1d1e1f200102030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f20'
        '0102030405060708090a0b0c0d0e0f10e803000000000000d007000000000000'
    ),
    (
        '22010000010002000102030405060708090a0b0c0d0e0f1003020100000000000403020171b7df07'
        '00000000e803000000000000d0070000000000000807060504030201000000000000f43f02000000'
        '000000000000f43f00000000000004c002000000000000000000f43f00000000000004c002000000'
        '000000000000f43f00000000000004c0000000000000c03f000000000000d03f000000000000d83f'
        '000000000000e03f000000000000c03f000000000000d03f000000000000d83f000000000000c03f'
        '000000000000d03f000000000000d83f000000000000c03f000000000000d03f000000000000d83f'
        '000000000000c03f000000000000d03f000000000000d83f000000000000e03f000000000000e43f'
        '000000000000e83f0101'
    ),
    (
        '11010000010003000102030405060708090a0b0c0d0e0f1003020100000000000403020171b7df07'
        '00000000e803000000000000d0070000000000000807060504030201010807060504030201010000'
        '000102030405060708090a0b0c0d0e0f100102030405060708090a0b0c0d0e0f1001020304050607'
        '08090a0b0c0d0e0f100102030405060708090a0b0c0d0e0f10080706050403020108070605040302'
        '01080706050403020108070605040302010807060504030201080706050403020108070605040302'
        '0104030201040302010403020104030201040302010102030405060708090a0b0c0d0e0f10111213'
        '1415161718191a1b1c1d1e1f2002000000000000000000f43f00000000000004c0'
    ),
    (
        'f0000000010004000102030405060708090a0b0c0d0e0f1003020100000000000403020171b7df07'
        '00000000e803000000000000d00700000000000001000000010000000102030405060708090a0b0c'
        '0d0e0f100102030405060708090a0b0c0d0e0f100102030405060708090a0b0c0d0e0f1001020304'
        '05060708090a0b0c0d0e0f1008070605040302010807060504030201080706050403020108070605'
        '04030201080706050403020108070605040302010807060504030201040302010403020104030201'
        '04030201040302010102030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f20'
    ),
    (
        'f8000000010005000102030405060708090a0b0c0d0e0f1003020100000000000403020171b7df07'
        '00000000e803000000000000d0070000000000000100000001000000010000000100000001020304'
        '05060708090a0b0c0d0e0f100102030405060708090a0b0c0d0e0f100102030405060708090a0b0c'
        '0d0e0f100102030405060708090a0b0c0d0e0f100807060504030201080706050403020108070605'
        '04030201080706050403020108070605040302010807060504030201080706050403020104030201'
        '040302010403020104030201040302010102030405060708090a0b0c0d0e0f101112131415161718'
        '191a1b1c1d1e1f20'
    ),
    (
        'fc020000010006000102030405060708090a0b0c0d0e0f1003020100000000000403020171b7df07'
        '00000000e803000000000000d007000000000000010000000102030405060708090a0b0c0d0e0f10'
        '0102030405060708090a0b0c0d0e0f100102030405060708090a0b0c0d0e0f100102030405060708'
        '090a0b0c0d0e0f100807060504030201080706050403020108070605040302010807060504030201'
        '08070605040302010807060504030201080706050403020104030201040302010403020104030201'
        '040302010102030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f2001000000'
        '0102030405060708090a0b0c0d0e0f100102030405060708090a0b0c0d0e0f100102030405060708'
        '090a0b0c0d0e0f100102030405060708090a0b0c0d0e0f1008070605040302010807060504030201'
        '08070605040302010807060504030201080706050403020108070605040302010807060504030201'
        '04030201040302010403020104030201040302010102030405060708090a0b0c0d0e0f1011121314'
        '15161718191a1b1c1d1e1f20010000000102030405060708090a0b0c0d0e0f100102030405060708'
        '090a0b0c0d0e0f100102030405060708090a0b0c0d0e0f100102030405060708090a0b0c0d0e0f10'
        '08070605040302010807060504030201080706050403020108070605040302010807060504030201'
        '08070605040302010807060504030201040302010403020104030201040302010403020101020304'
        '05060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f20010000000102030405060708'
        '090a0b0c0d0e0f100102030405060708090a0b0c0d0e0f100102030405060708090a0b0c0d0e0f10'
        '0102030405060708090a0b0c0d0e0f10080706050403020108070605040302010807060504030201'
        '08070605040302010807060504030201080706050403020108070605040302010403020104030201'
        '0403020104030201040302010102030405060708090a0b0c0d0e0f101112131415161718191a1b1c'
        '1d1e1f20'
    ),
    (
        '4c020000010007000102030405060708090a0b0c0d0e0f1003020100000000000403020171b7df07'
        '00000000e803000000000000d007000000000000010000000102030405060708090a0b0c0d0e0f10'
        '0102030405060708090a0b0c0d0e0f100102030405060708090a0b0c0d0e0f100102030405060708'
        '090a0b0c0d0e0f100807060504030201080706050403020108070605040302010807060504030201'
        '08070605040302010807060504030201080706050403020104030201040302010403020104030201'
        '040302010102030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f2001000000'
        '0102030405060708090a0b0c0d0e0f100102030405060708090a0b0c0d0e0f100102030405060708'
        '090a0b0c0d0e0f100102030405060708090a0b0c0d0e0f1008070605040302010807060504030201'
        '08070605040302010807060504030201080706050403020108070605040302010807060504030201'
        '04030201040302010403020104030201040302010102030405060708090a0b0c0d0e0f1011121314'
        '15161718191a1b1c1d1e1f20010000000102030405060708090a0b0c0d0e0f100102030405060708'
        '090a0b0c0d0e0f100102030405060708090a0b0c0d0e0f100102030405060708090a0b0c0d0e0f10'
        '08070605040302010807060504030201080706050403020108070605040302010807060504030201'
        '08070605040302010807060504030201040302010403020104030201040302010403020101020304'
        '05060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f20'
    ),
    (
        '4f020000010008000102030405060708090a0b0c0d0e0f1003020100000000000403020171b7df07'
        '00000000e803000000000000d007000000000000010000000102030405060708090a0b0c0d0e0f10'
        '0102030405060708090a0b0c0d0e0f100102030405060708090a0b0c0d0e0f100102030405060708'
        '090a0b0c0d0e0f100807060504030201080706050403020108070605040302010807060504030201'
        '08070605040302010807060504030201080706050403020104030201040302010403020104030201'
        '040302010102030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f2001000000'
        '0102030405060708090a0b0c0d0e0f100102030405060708090a0b0c0d0e0f100102030405060708'
        '090a0b0c0d0e0f100102030405060708090a0b0c0d0e0f1008070605040302010807060504030201'
        '08070605040302010807060504030201080706050403020108070605040302010807060504030201'
        '04030201040302010403020104030201040302010102030405060708090a0b0c0d0e0f1011121314'
        '15161718191a1b1c1d1e1f20010000000102030405060708090a0b0c0d0e0f100102030405060708'
        '090a0b0c0d0e0f100102030405060708090a0b0c0d0e0f100102030405060708090a0b0c0d0e0f10'
        '08070605040302010807060504030201080706050403020108070605040302010807060504030201'
        '08070605040302010807060504030201040302010403020104030201040302010403020101020304'
        '05060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f20010101'
    ),
    (
        '4e000000010009000102030405060708090a0b0c0d0e0f1003020100000000000403020171b7df07'
        '00000000e803000000000000d007000000000000040302010a00000073746f700070726f6f66'
    ),
    (
        '5400000001000a000102030405060708090a0b0c0d0e0f1003020100000000000403020171b7df07'
        '00000000e803000000000000d0070000000000000807060504030201080706050403020108070605'
        '04030201'
    ),
)

GOLDEN_HEX += (
    (
        '6400000001000b000102030405060708090a0b0c0d0e0f1003020100000000000403020171b7df07'
        '00000000e803000000000000d00700000000000002000000210000000102030405060708090a0b0c'
        '0d0e0f101112131415161718191a1b1c1d1e1f20'
    ),
    (
        '6c00000001000c000102030405060708090a0b0c0d0e0f1003020100000000000403020171b7df07'
        '00000000e803000000000000d0070000000000000300000002000000020000002100000001020304'
        '05060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f20'
    ),
)

# Task 6 typed local permission frames (literal independent wire fixtures).
GOLDEN_HEX += ('6800000001000d000102030405060708090a0b0c0d0e0f1003020100000000000403020171b7df0700000000e803000000000000d0070000000000000300000002000000210000000102030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f20', '6400000001000e000102030405060708090a0b0c0d0e0f1003020100000000000403020171b7df0700000000e803000000000000d00700000000000002000000210000000102030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f20', '4400000001000f000102030405060708090a0b0c0d0e0f1003020100000000000403020171b7df0700000000e803000000000000d0070000000000000200000001000000')
