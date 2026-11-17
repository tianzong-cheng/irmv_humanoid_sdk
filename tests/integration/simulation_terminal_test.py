"""Real controlling PTY, real SDK/DDS, fake physics; no injected key pipe."""
import errno
import fcntl
import json
import os
from pathlib import Path
import pty
import select
import signal
import sys
import tempfile
import termios
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tools'))
sys.path.insert(0, str(ROOT / 'tests/support/simulation'))


def peer(build, config):
    from sim_launch import Experiment
    from fake_engine import FakeEngine

    class Engine(FakeEngine):
        def initialize(self, cfg):
            super().initialize(dict(q=[0, 0]))

    owner = Experiment(config, build / 'robot_runtime', build / 'application',
                       engine_factory=Engine, binding=lambda n: {'fixture_only': True})
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, owner.notify_shutdown)
    try:
        owner.start()
        print('TERMINAL_CHILDREN ' + json.dumps([p.pid for p in owner.children]), flush=True)
        return owner.wait()
    finally:
        owner.close()


def run_case(build, quit_application):
    from launch_fixture import prepare
    with tempfile.TemporaryDirectory(prefix='irmv-tty-') as tmp:
        directory = Path(tmp)
        config = prepare(ROOT, build, directory)
        # Capture restoration baseline before the child can acquire key mode.
        # Give the peer its own controlling terminal and foreground group,
        # matching an interactive shell rather than substituting a key pipe.
        master,slave=pty.openpty()
        before=termios.tcgetattr(slave)
        pid=os.fork()
        if pid==0:
            try:
                os.close(master)
                os.setsid()
                fcntl.ioctl(slave,termios.TIOCSCTTY,0)
                for fd in (0,1,2):os.dup2(slave,fd)
                if slave>2:os.close(slave)
                os.execv(sys.executable,[sys.executable,'-u',str(Path(__file__).resolve()),
                                        str(build),'--peer',str(config)])
            finally:os._exit(127)
        os.close(slave)
        text = ''
        reaped = False
        children = []

        def drain(timeout):
            nonlocal text, children
            if not select.select([master], [], [], timeout)[0]:
                return
            try:
                data = os.read(master, 65536)
            except OSError as exc:
                if exc.errno == errno.EIO:
                    return
                raise
            text += data.decode(errors='replace')
            for line in text.splitlines():
                if line.startswith('TERMINAL_CHILDREN '):
                    children = json.loads(line[len('TERMINAL_CHILDREN '):])

        def expect(needle, timeout=12):
            deadline = time.monotonic() + timeout
            while needle not in text and time.monotonic() < deadline:
                drain(min(.1, max(0, deadline-time.monotonic())))
            app = directory / 'run/application.log'
            assert needle in text, ('missing interactive feedback', needle, text,
                                   app.read_text() if app.exists() else 'no application log')

        try:
            expect('[application] behavior=Standby')
            # Bytes have no newline: application must own immediate key mode.
            os.write(master, b'd')
            expect('[application] behavior=Damping')
            os.write(master, b'i')
            expect('[application] behavior=Ready')
            os.write(master, b'l')
            expect('[application] behavior=Locomotion')
            if quit_application:
                os.write(master, b'x')
                expect('exit stop=confirmed state=Damping')
                assert os.waitpid(pid, os.WNOHANG) == (0, 0), 'app x killed launcher/runtime'
                assert len(children) == 2
                os.kill(children[0], 0)
                # The final log precedes main's TerminalGuard destructor.
                # Observe restoration, not an assumed print/exit ordering.
                restore_deadline=time.monotonic()+2
                while termios.tcgetattr(master)!=before and time.monotonic()<restore_deadline:
                    drain(.01)
                assert termios.tcgetattr(master)==before, ('terminal not restored after x',before,termios.tcgetattr(master))
            # Real terminal Ctrl-C must reach only the launcher foreground
            # process, which obtains runtime/engine stop proof through IPC.
            os.write(master, b'\x03')
            deadline = time.monotonic()+8
            while time.monotonic() < deadline:
                drain(.05)
                result, status = os.waitpid(pid, os.WNOHANG)
                if result:
                    reaped = True
                    assert os.waitstatus_to_exitcode(status) == 0, text
                    break
            assert reaped, ('launcher ignored terminal Ctrl-C', text)
            assert termios.tcgetattr(master) == before, 'launcher left terminal attributes changed'
            evidence = json.loads((directory/'run/evidence.json').read_text())
            assert evidence['stop_proven'] and evidence['state'] == 'Closed', evidence
            assert evidence['final_child_codes'] == [0, 0], evidence
            recorded = (directory/'run/application.log').read_text()
            assert all(state in recorded for state in ('behavior=Damping', 'behavior=Ready', 'behavior=Locomotion'))
            print('PASS controlling PTY keys/status/restore/stop', 'app-x' if quit_application else 'active-Ctrl-C')
        finally:
            if not reaped:
                try:
                    os.kill(pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
                deadline = time.monotonic()+8
                while time.monotonic() < deadline:
                    result, _ = os.waitpid(pid, os.WNOHANG)
                    if result:
                        reaped = True
                        break
                    drain(.05)
                if not reaped:
                    os.kill(pid, signal.SIGKILL)
                    os.waitpid(pid, 0)
                for child in children:
                    try:
                        os.kill(child, signal.SIGTERM)
                    except ProcessLookupError:
                        pass
            os.close(master)


if __name__ == '__main__':
    build = Path(sys.argv[1]).resolve()
    if len(sys.argv) > 2 and sys.argv[2] == '--peer':
        raise SystemExit(peer(build, Path(sys.argv[3])))
    run_case(build, True)
    run_case(build, False)
