#!/usr/bin/env python3
"""Installed entry point of the experiment workbench.

Resolves everything from its own prefix: the read-only bridge, the Python
service package and the static workbench assets. It never looks for the source
tree, and it never starts, stops or signals an SDK process.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path


def main(argv: list[str] | None = None) -> int:
    prefix = Path(__file__).resolve().parent.parent
    package_root = prefix / "share" / "irmv" / "experiment-web"
    sys.path.insert(0, str(package_root))
    from experiment_web.server import ExperimentWebServer  # noqa: PLC0415

    parser = argparse.ArgumentParser(prog="irmv_experiment_web")
    parser.add_argument("--records-root", required=True)
    parser.add_argument("--data-dir", required=True)
    parser.add_argument("--port", type=int, default=0)
    parser.add_argument("--reader", default=None,
                        help="defaults to the reader installed in this prefix")
    parser.add_argument("--assets", default=None,
                        help="defaults to the static workbench of this prefix")
    parser.add_argument("--telemetry-socket", default=None,
                        help="defaults to <data-dir>/live.sock")
    options = parser.parse_args(argv)

    reader = Path(options.reader) if options.reader else prefix / "bin" / "web_record_query"
    if not reader.is_file():
        print(f"irmv_experiment_web: the read-only reader is missing: {reader}",
              file=sys.stderr)
        return 1
    assets = Path(options.assets) if options.assets else package_root / "web"
    if not (assets / "index.html").is_file():
        print(f"irmv_experiment_web: the static workbench is missing: {assets}",
              file=sys.stderr)
        return 1
    socket_path = (Path(options.telemetry_socket)
                   if options.telemetry_socket
                   else Path(options.data_dir) / "live.sock")
    try:
        server = ExperimentWebServer(
            records_root=Path(options.records_root),
            data_dir=Path(options.data_dir),
            reader=reader,
            assets=assets,
            port=options.port,
            telemetry_socket=socket_path,
        )
    except (OSError, ValueError) as error:
        print(f"irmv_experiment_web: {error}", file=sys.stderr)
        return 1
    print(server.url, flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
