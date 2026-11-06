#!/usr/bin/env python3
"""Build the deterministic fixture runs and serve the workbench for Playwright."""

from __future__ import annotations

import os
import sys
from pathlib import Path

HERE = Path(__file__).resolve()
REPO = HERE.parents[3]
sys.path.insert(0, str(HERE.parent))
sys.path.insert(0, str(REPO))

from make_fixtures import build  # noqa: E402


def main() -> int:
    root = REPO / "build" / "web-e2e-fixtures"
    records = root / "records"
    data = root / "data"
    build(records)
    data.mkdir(parents=True, exist_ok=True)
    reader = REPO / "build" / "web_record_query"
    if not reader.is_file():
        print(f"serve_fixture: the reader is missing: {reader}", file=sys.stderr)
        return 1
    os.chdir(REPO)
    os.execv(
        sys.executable,
        [
            sys.executable,
            "-m",
            "tools.experiment_web",
            "--records-root",
            str(records),
            "--data-dir",
            str(data),
            "--reader",
            str(reader),
            "--assets",
            str(REPO / "web"),
            "--port",
            "8766",
            "--telemetry-socket",
            str(data / "live.sock"),
        ],
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
