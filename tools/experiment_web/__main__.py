"""Command-line entry point of the local experiment workbench service."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from .catalog import CatalogError, canonical_records_root
from .server import ExperimentWebServer


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="experiment_web")
    parser.add_argument("--records-root", required=True)
    parser.add_argument("--data-dir", required=True)
    parser.add_argument("--reader", required=True)
    parser.add_argument(
        "--assets", default=str(Path(__file__).resolve().parents[2] / "web")
    )
    parser.add_argument("--port", type=int, default=0)
    parser.add_argument("--telemetry-socket", default=None)
    args = parser.parse_args(argv)

    try:
        records_root = canonical_records_root(args.records_root)
    except CatalogError as error:
        print(f"experiment_web: {error}", file=sys.stderr)
        return 1
    reader = Path(args.reader)
    if not reader.is_file():
        print(f"experiment_web: the reader does not exist: {reader}", file=sys.stderr)
        return 1

    server = ExperimentWebServer(
        records_root=records_root,
        data_dir=Path(args.data_dir),
        reader=reader,
        assets=Path(args.assets),
        port=args.port,
        telemetry_socket=Path(args.telemetry_socket) if args.telemetry_socket else None,
    )
    print(server.url, flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
