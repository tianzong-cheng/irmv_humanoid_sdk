#!/usr/bin/env python3
"""Contract of tools/sdk_launch.py with fake children.

Usage: launcher_test.py <source directory>
"""

import os
import pty
import re
import signal
import subprocess
import sys
import tempfile
import textwrap
import time
from pathlib import Path


def write(path: Path, body: str) -> None:
    path.write_text(textwrap.dedent(body), "utf-8")
    path.chmod(0o755)


def run_launcher(source: Path, workspace: Path, runtime: Path,
                 application: Path, timeout: str = "2"):
    environment = dict(os.environ)
    environment["IRMV_LAUNCH_TIMEOUT_S"] = timeout
    return subprocess.run(
        [sys.executable, str(source / "tools/sdk_launch.py"),
         "--config", str(source / "config/sdk.json"),
         "--runtime", str(runtime), "--application", str(application)],
        text=True, capture_output=True, env=environment, timeout=30)


def main() -> int:
    source = Path(sys.argv[1]).resolve()
    real_runtime = Path(sys.argv[2]) if len(sys.argv) > 2 else None
    real_application = Path(sys.argv[3]) if len(sys.argv) > 3 else None
    with tempfile.TemporaryDirectory(prefix="irmv-launcher-") as directory:
        workspace = Path(directory)
        runtime = workspace / "fake_runtime.py"
        write(runtime, """\
            #!/usr/bin/env python3
            import sys, time
            print("INFO runtime started", flush=True)
            print('INFO BootCompleted {event=1}', flush=True)
            for seq in range(600):
                print('INFO State {state_seq=%d}' % seq, flush=True)
                time.sleep(0.05)
        """)
        app = workspace / "fake_app.py"
        write(app, """\
            #!/usr/bin/env python3
            import sys
            sys.exit(7)
        """)
        result = run_launcher(source, workspace, runtime, app)
        if result.returncode != 7:
            print("FAIL: launcher returned %d, expected the application code 7\n%s\n%s"
                  % (result.returncode, result.stdout, result.stderr))
            return 1
        if "runtime is still running" not in result.stdout:
            print("FAIL: the launcher did not report the surviving runtime\n" +
                  result.stdout)
            return 1
        match = re.search(r"pid (\d+)", result.stdout)
        pid = int(match.group(1)) if match else None
        log_match = re.search(r"runtime diagnostics: (.*)", result.stdout)
        if not pid or not log_match:
            print("FAIL: the launcher did not report the runtime pid and log\n" +
                  result.stdout)
            return 1
        runtime_log = Path(log_match.group(1).strip())
        before = runtime_log.stat().st_size if runtime_log.exists() else 0
        # The application is gone and the launcher has exited. The runtime's
        # diagnostics destination must still exist and receive its state lines:
        # a pipe would have raised SIGPIPE on the first write after the reader
        # disappeared, which would now show as a dead pid or a frozen log.
        time.sleep(0.5)
        if not runtime_log.exists() or runtime_log.stat().st_size <= before:
            print("FAIL: runtime diagnostics stopped after the launcher exited\n" +
                  result.stdout)
            return 1
        try:
            os.kill(pid, 0)
        except ProcessLookupError:
            print("FAIL: the runtime exited after the launcher exited, signal %s\n"
                  % result.returncode)
            return 1
        try:
            os.kill(pid, signal.SIGKILL)
        except ProcessLookupError:
            pass

        slow = workspace / "slow_runtime.py"
        write(slow, """\
            #!/usr/bin/env python3
            import time
            time.sleep(30)
        """)
        failed = run_launcher(source, workspace, slow, app, timeout="0.3")
        if failed.returncode == 0:
            print("FAIL: a runtime that never became ready was reported ready\n" +
                  failed.stdout)
            return 1
        if "did not become ready" not in failed.stderr:
            print("FAIL: the failure message is missing\n" + failed.stderr)
            return 1

        # A signal to the launcher reaches the application only. The runtime is
        # in its own session and must survive.
        marker = workspace / "app-started"
        signal_app = workspace / "signal_app.py"
        write(signal_app, f"""\
            #!/usr/bin/env python3
            import signal, sys, time
            from pathlib import Path
            Path({str(marker)!r}).write_text("started")
            def stop(signum, frame):
                sys.exit(0)
            signal.signal(signal.SIGTERM, stop)
            signal.signal(signal.SIGINT, stop)
            time.sleep(30)
        """)
        environment = dict(os.environ)
        environment["IRMV_LAUNCH_TIMEOUT_S"] = "3"
        launcher = subprocess.Popen(
            [sys.executable, str(source / "tools/sdk_launch.py"),
             "--config", str(source / "config/sdk.json"),
             "--runtime", str(runtime), "--application", str(signal_app)],
            text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            env=environment)
        deadline = time.monotonic() + 10
        while not marker.exists() and time.monotonic() < deadline:
            time.sleep(0.02)
        if not marker.exists():
            launcher.kill()
            print("FAIL: the application never started")
            return 1
        launcher.send_signal(signal.SIGTERM)
        output, errors = launcher.communicate(timeout=15)
        if "runtime is still running" not in output:
            print("FAIL: the launcher did not report the surviving runtime\n" +
                  output + errors)
            return 1
        match = re.search(r"pid (\d+)", output)
        if match:
            try:
                os.kill(int(match.group(1)), signal.SIGKILL)
            except ProcessLookupError:
                pass

        # An application that cannot be started is reported while the runtime
        # stays alive.
        missing = workspace / "no-such-application"
        broken = run_launcher(source, workspace, runtime, missing)
        if broken.returncode != 1 or "runtime is still running" not in broken.stderr:
            print("FAIL: an application start failure was not reported\n" +
                  broken.stdout + broken.stderr)
            return 1
        match = re.search(r"pid (\d+)", broken.stderr)
        if match:
            try:
                os.kill(int(match.group(1)), signal.SIGKILL)
            except ProcessLookupError:
                pass
        if (real_runtime and real_application and real_runtime.is_file()
                and real_application.is_file()):
            import json
            config = json.loads((source / "config/sdk.json").read_text())
            config["telemetry"]["directory"] = str(workspace / "real-logs")
            config["dds"]["domain"] = 20 + (os.getpid() % 60)
            real_config = workspace / "real.json"
            real_config.write_text(json.dumps(config))
            environment = dict(os.environ)
            environment["IRMV_LAUNCH_TIMEOUT_S"] = "20"
            master, slave = pty.openpty()
            real = subprocess.Popen(
                [sys.executable, str(source / "tools/sdk_launch.py"),
                 "--config", str(real_config),
                 "--runtime", str(real_runtime),
                 "--application", str(real_application)],
                text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                stdin=slave, env=environment)
            os.close(slave)
            time.sleep(5)
            if real.poll() is not None:
                output, errors = real.communicate()
                print("FAIL: the real dual-process launcher exited early\n" +
                      output + errors)
                return 1
            real.send_signal(signal.SIGTERM)
            output, errors = real.communicate(timeout=20)
            os.close(master)
            if "runtime is still running" not in output:
                print("FAIL: the real runtime did not survive the launcher "
                      "signal\n" + output + errors)
                return 1
            match = re.search(r"pid (\d+)", output)
            if match:
                try:
                    os.kill(int(match.group(1)), signal.SIGKILL)
                except ProcessLookupError:
                    pass

    print("launcher contract passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
