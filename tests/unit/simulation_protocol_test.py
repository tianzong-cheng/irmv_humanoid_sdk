"""Cross-language contract; intentionally runs without engine dependencies."""
import pathlib
import subprocess
import sys
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / 'tools'))
try:
    from simulation import protocol as p
except ImportError:
    raise AssertionError('Task 3 codec module is missing')
for f in p.golden_frames():
    wire = p.encode(f)
    assert p.encode(p.decode(wire)) == wire
    for n in range(len(wire)):
        try:
            p.decode(wire[:n])
        except ValueError:
            pass
        else:
            raise AssertionError((f.kind, n))
    if len(sys.argv) > 1:
        result = subprocess.run([sys.argv[1], wire.hex()], capture_output=True, text=True, check=True)
        assert result.stdout.strip() == wire.hex(), f.kind
        result = subprocess.run([sys.argv[1], 'fixture', str(f.kind)], capture_output=True, text=True, check=True)
        assert bytes.fromhex(result.stdout) == wire, f.kind
for offset, value in [(0, 0), (4, 2), (6, 255)]:
    bad = bytearray(p.encode(p.golden_frames()[0]))
    bad[offset] = value
    try:
        p.decode(bytes(bad))
    except ValueError:
        pass
    else:
        raise AssertionError(offset)
from dataclasses import replace
import struct

def rejected(wire):
    try:
        p.decode(wire)
    except ValueError:
        pass
    else:
        raise AssertionError('accepted malformed wire')
    if len(sys.argv) > 1 and len(wire) <= 65536:
        result = subprocess.run([sys.argv[1], wire.hex()], capture_output=True)
        assert result.returncode == 1, result.stderr

def changed(wire, offset, fmt, value):
    b = bytearray(wire)
    struct.pack_into('<' + fmt, b, offset, value)
    return bytes(b)
hello = p.golden_frames()[0]
wire = p.encode(hello)
for offset, fmt, value in [(4, 'H', 2), (6, 'H', 0), (24, 'Q', 2 ** 64 - 1), (40, 'Q', 0), (48, 'Q', 3), (60, 'I', 0)]:
    rejected(changed(wire, offset, fmt, value))
rejected(wire + b'x')
rejected(bytes(65537))
feedback = p.encode(p.golden_frames()[1])
rejected(changed(feedback, 68, 'd', float('nan')))
rejected(changed(feedback, 76, 'I', 65))
rejected(changed(feedback, 76, 'I', 2 ** 32 - 1))
manifest = p.encode(p.golden_frames()[3])
rejected(changed(manifest, 64, 'I', 0))
ack = p.encode(p.golden_frames()[4])
rejected(changed(ack, 64, 'I', 3))
a = hello.payload
b = dict(a, engine_digest=bytes([255]) * 32)
assert not p.matches_hello(a, b)
assert p.matches_hello(a, dict(a, role=2))
assert any(hello.run_id)
assert p.GOLDEN_HEX == tuple((p.encode(f).hex() for f in p.golden_frames()))
for count in (0, 64):
    v = dict(p.golden_frames()[1].payload)
    v.update(q=(0.0,) * count, dq=(0.0,) * count, tau=(0.0,) * count)
    frame = replace(p.golden_frames()[1], payload=v, run_id=bytes(16))
    wire = p.encode(frame)
    assert p.encode(p.decode(wire)) == wire
    if len(sys.argv) > 1:
        result = subprocess.run([sys.argv[1], wire.hex()], capture_output=True, text=True, check=True)
        assert result.stdout.strip() == wire.hex()
    v['q'] = (0.0,) * 65
    try:
        p.encode(replace(frame, payload=v))
    except ValueError:
        pass
    else:
        raise AssertionError('oversized encode')
for count in (0, 256):
    frame = replace(p.golden_frames()[8], payload={'code': 0, 'reason': bytes(count)})
    wire = p.encode(frame)
    assert p.encode(p.decode(wire)) == wire
    if len(sys.argv) > 1:
        result = subprocess.run([sys.argv[1], wire.hex()], capture_output=True, text=True, check=True)
        assert result.stdout.strip() == wire.hex()
    rejected(changed(wire, 64, 'I', 257))
for f in p.golden_frames():
    payload = dict(f.payload)
    if f.kind == 1:
        payload['physics_dt_us'] = 999
        try:
            p.encode(replace(f, payload=payload))
        except ValueError:
            pass
        else:
            raise AssertionError('conflicting dt')
print('simulation protocol golden exchange and rejection checks passed')

# A phase is explicitly sealed, including empty phases; queue capacity is separate.
assert p.MAX_PHASE_MANIFEST_ENTRIES == 64
assert p.KINDS[10:] == ['PhaseSeal', 'PhaseAck', 'InventoryReady', 'PublishPermit', 'TickPermit', 'FirstCommandProof']
assert len(p.GOLDEN_HEX) == 16
for kind in (11, 12):
    for count in (0, 33, 64):
        seal = {'sender': 2, 'manifest_count': count,
                'manifest_digest': bytes(range(32))}
        payload = seal if kind == 11 else {
            'consumer': 3, 'disposition': 2, 'seal': seal}
        frame = replace(hello, kind=kind, payload=payload)
        wire = p.encode(frame)
        assert p.decode(wire) == frame
        if len(sys.argv) > 1:
            result = subprocess.run([sys.argv[1], wire.hex()], capture_output=True,
                                    text=True, check=True)
            assert result.stdout.strip() == wire.hex()
        rejected(changed(wire, 6, 'H', 17))
        count_offset = 64 if kind == 11 else 72
        sender_offset = 60 if kind == 11 else 68
        for invalid in (65, 2**32-1):
            rejected(changed(wire, count_offset, 'I', invalid))
        for invalid in (0, 4):
            rejected(changed(wire, sender_offset, 'I', invalid))
        if kind == 12:
            for invalid in (0, 4):
                rejected(changed(wire, 60, 'I', invalid))
            for invalid in (0, 3):
                rejected(changed(wire, 64, 'I', invalid))
        seal['manifest_count'] = 65
        try:
            p.encode(frame)
        except ValueError:
            pass
        else:
            raise AssertionError('oversized phase inventory encode')
print('explicit bounded phase seal/ack checks passed')

# Task 6 control frames share the sealed inventory bounds and enum checks.
for kind in (13,14):
    for count in (0,64):
        seal=dict(sender=2,manifest_count=count,manifest_digest=bytes(range(32)))
        payload=dict(consumer=3,seal=seal) if kind==13 else dict(seal=seal)
        frame=replace(hello,kind=kind,payload=payload)
        wire=p.encode(frame)
        assert p.decode(wire)==frame
        if len(sys.argv)>1:
            result=subprocess.run([sys.argv[1],wire.hex()],capture_output=True,text=True,check=True)
            assert result.stdout.strip()==wire.hex()
        rejected(changed(wire,68 if kind==13 else 64,'I',65))
        rejected(changed(wire,64 if kind==13 else 60,'I',0))
        if kind==13: rejected(changed(wire,60,'I',4))
wire=p.encode(p.golden_frames()[14])
for stage in (0,3,2**32-1): rejected(changed(wire,64,'I',stage))
print('installation/publication/tick permission golden and rejection checks passed')

# First-command proof is typed control evidence, never another DDS manifest.
proof=p.encode(p.golden_frames()[15])
for offset,value in ((60,5),(236,1)):
    rejected(changed(proof,offset,'I',value))
print('auxiliary first-command proof kind validation passed')
