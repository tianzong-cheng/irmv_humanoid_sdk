"""Real local sockets; no DDS, engine, network service, or optional packages."""

import dataclasses
import os
import pathlib
import socket
import subprocess
import sys
import tempfile
import time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / "tools"))
from simulation import protocol as p

peer = sys.argv[1]
run = b"\x01" + bytes(15)
hello = p.Frame(
    1,
    dict(
        role=1,
        config_digest=bytes(32),
        application_digest=bytes(32),
        engine_digest=bytes(32),
        runtime_id=bytes(16),
        physics_dt_us=1000,
        control_dt_us=2000,
    ),
    run_id=run,
)
abort = p.Frame(9, dict(code=1, reason=b"test"), run_id=run)


def finish(proc):
    out, err = proc.communicate(timeout=2)
    assert proc.returncode == 0, (out, err)
    return out.strip()


def case(name, action, mode="wait"):
    with tempfile.TemporaryDirectory(prefix="irmv-ipc-") as directory:
        path = pathlib.Path(directory) / "coordinator.sock"
        with socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET) as listener:
            listener.bind(str(path))
            listener.listen(1)
            listener.settimeout(2)
            proc = subprocess.Popen(
                [peer, str(path), mode],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
            )
            try:
                with listener.accept()[0] as conn:
                    conn.settimeout(1)
                    request = p.decode(conn.recv(p.MAX_PACKET_BYTES))
                    assert request.payload["role"] == 2
                    assert request.run_id == run
                    result = action(conn)
                out = finish(proc)
                if result is not None:
                    assert out.startswith(result), (name, out)
                print(name, out)
            finally:
                if proc.poll() is None:
                    proc.kill()
                    proc.wait()


def bad(conn, frame):
    conn.send(p.encode(frame))
    time.sleep(0.025)
    return "0 1"


case(
    "wrong role",
    lambda c: bad(c, dataclasses.replace(hello, payload={**hello.payload, "role": 3})),
)
case(
    "wrong run",
    lambda c: bad(c, dataclasses.replace(hello, run_id=b"\x02" + bytes(15))),
)
case(
    "wrong dt",
    lambda c: bad(
        c,
        dataclasses.replace(
            hello, physics_dt_us=500, payload={**hello.payload, "physics_dt_us": 500}
        ),
    ),
)
case(
    "wrong digest",
    lambda c: bad(
        c,
        dataclasses.replace(
            hello, payload={**hello.payload, "engine_digest": b"\x01" + bytes(31)}
        ),
    ),
)
case("not Hello", lambda c: bad(c, abort))


def echo(conn):
    conn.send(p.encode(hello))
    conn.send(p.encode(abort))
    assert conn.recv(p.MAX_PACKET_BYTES) == p.encode(abort)
    return "1 exchanged"


case("successful bidirectional handoff", echo, "echo")


def duplicate(conn):
    conn.send(p.encode(hello))
    time.sleep(0.025)
    conn.send(p.encode(hello))
    time.sleep(0.025)
    return "1 1"


case("duplicate Hello", duplicate)


def timeout(conn):
    time.sleep(0.22)
    return "0 1"


case("Hello timeout", timeout)


def cancel(conn):
    time.sleep(0.05)
    return "stopped"


case("Stop while waiting for Hello", cancel, "stop")


def idle(conn):
    conn.send(p.encode(hello))
    time.sleep(0.15)
    return "1 1"


case("host idle deadline", idle)


def death(conn):
    conn.send(p.encode(hello))
    time.sleep(0.025)
    return "1 1"


case("peer death", death)


def oversized(conn):
    conn.send(p.encode(hello))
    time.sleep(0.025)
    conn.send(bytes(p.MAX_PACKET_BYTES + 1))
    time.sleep(0.025)
    return "1 1"


case("maximum packet violation", oversized)


def inbound(conn):
    conn.send(p.encode(hello))
    time.sleep(0.025)
    for _ in range(33):
        conn.send(p.encode(abort))
    time.sleep(0.025)
    return "1 1 1"


case("inbound backpressure", inbound)


def outbound(conn):
    conn.send(p.encode(hello))
    time.sleep(0.05)
    return "1 1"


case("outbound backpressure", outbound, "send")

with tempfile.TemporaryDirectory(prefix="irmv-ipc-") as directory:
    path = pathlib.Path(directory) / "coordinator.sock"

    def check(mode="wait"):
        start = time.monotonic()
        r = subprocess.run(
            [peer, str(path), mode], capture_output=True, text=True, timeout=2
        )
        assert r.returncode == 0, r.stderr
        assert time.monotonic() - start < 1, r.stdout
        return r.stdout.strip()

    assert check() == "0 1 0"  # absent listener
    assert check("stop") == "stopped"
    path.write_text("do not unlink")
    assert check() == "rejected"
    assert path.read_text() == "do not unlink"
    path.unlink()
    path.symlink_to("missing")
    assert check() == "rejected"
    assert path.is_symlink()
    path.unlink()
    with socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET) as stale:
        stale.bind(str(path))
    before = path.stat().st_ino
    assert check() == "0 1 0"
    assert path.stat().st_ino == before
    os.chmod(directory, 0o755)
    assert check() == "rejected"
print("path safety, stale socket, absent peer and bounded Stop passed")

# Keep inbound traffic alive while deliberately never draining outbound packets.
# This distinguishes a pending-send deadline from idle timeout/queue overflow.
with tempfile.TemporaryDirectory(prefix="irmv-blocked-") as directory:
    path = pathlib.Path(directory) / "bridge.sock"
    with socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET) as listener:
        listener.bind(str(path))
        listener.listen(1)
        listener.settimeout(2)
        proc = subprocess.Popen(
            [peer, str(path), "blocked"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        try:
            with listener.accept()[0] as conn:
                conn.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4096)
                assert p.decode(conn.recv(p.MAX_PACKET_BYTES)).kind == 1
                conn.send(p.encode(hello))
                conn.settimeout(0.1)
                while proc.poll() is None:
                    try:
                        conn.send(p.encode(abort))
                    except (BrokenPipeError, ConnectionResetError):
                        break
                    time.sleep(0.005)
            out, err = proc.communicate(timeout=2)
            assert proc.returncode == 0, (out, err)
            ready, failed, rejected, received, elapsed = map(int, out.split())
            assert ready == failed == 1 and rejected == 0, (out, err)
            assert received >= 10 and 100 <= elapsed < 1800, (out, err)
            print("blocked send under active inbound", out.strip())
        finally:
            if proc.poll() is None:
                proc.kill()
                proc.wait()
