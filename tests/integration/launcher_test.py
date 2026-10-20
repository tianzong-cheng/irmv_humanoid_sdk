#!/usr/bin/env python3
"""Contract of tools/sdk_launch.py with fake children.

Usage: launcher_test.py <source directory>
"""

import os
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
    with tempfile.TemporaryDirectory(prefix="irmv-launcher-") as directory:
        workspace = Path(directory)
        runtime = workspace / "fake_runtime.py"
        write(runtime, """\
            #!/usr/bin/env python3
            import sys, time
            print("INFO runtime started", flush=True)
            print('INFO BootCompleted {event=1}', flush=True)
            time.sleep(30)
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
        if pid:
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
            print("FAIL: a runtime that never becomes ready was reported ready\n" +
                  failed.stdout)
            return 1
        if "did not become ready" not in failed.stderr:
            print("FAIL: the failure message is missing\n" + failed.stderr)
            return 1
    print("launcher contract passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
