#!/usr/bin/env python3
"""Formatting contract of the first-party C/C++ sources of this repository.

Usage:
  tools/format.py --check [--clang-format PATH] [--jobs N]   # read-only
  tools/format.py --write [--clang-format PATH] [--jobs N]   # in place

`--check` (the default) never writes: it runs the pinned clang-format on every
first-party C/C++ header and source under include/, src/ and tests/, prints a
unified diff for every file that is not formatted and exits non-zero. `--write`
formats those files in place. Both modes use the single `.clang-format` of the
repository root, so no file can be checked against one style and written with
another.

The tool refuses to run with any clang-format version other than 18.1.8 and
reports the version and the path it found. Third-party dependencies and
generated sources are never touched: directories such as `build/` (which holds
the IDL-generated `execution_idl` sources), `third_party/` or `_deps/` are not
scanned.

Only the standard library is used.
"""

import argparse
import concurrent.futures
import difflib
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path

REQUIRED_VERSION = "18.1.8"

SOURCE_ROOT = Path(__file__).resolve().parent.parent
CONFIG_PATH = SOURCE_ROOT / ".clang-format"
REQUIREMENTS_PATH = SOURCE_ROOT / "tools" / "requirements-format.txt"

# Every first-party directory that may hold C/C++ code.
SCAN_DIRECTORIES = ("include", "src", "tests", "examples", "tools")
SOURCE_SUFFIXES = frozenset(
    {".h", ".hh", ".hpp", ".hxx", ".c", ".cc", ".cpp", ".cxx"}
)

# Directories that are never first-party sources: third-party dependencies,
# generated code and tool environments. `build/` is the out-of-source build
# directory and holds the generated IDL sources of the `execution_idl` target.
EXCLUDED_DIRECTORY_NAMES = frozenset(
    {
        ".cache",
        ".git",
        ".tools",
        "_deps",
        "build",
        "external",
        "node_modules",
        "third-party",
        "third_party",
        "thirdparty",
        "vendor",
    }
)
EXCLUDED_DIRECTORY_PREFIXES = ("cmake-build-",)

VERSION_PATTERN = re.compile(r"clang-format version (\d+\.\d+\.\d+)")
DIFF_CONTEXT_LINES = 3
DIFF_LINE_LIMIT = 200


class ToolError(Exception):
    """The formatter could not be resolved or has the wrong version."""


def clang_format_candidates(explicit):
    """Every candidate binary in precedence order.

    Each candidate is `(path, provenance, required)`. A required candidate was
    asked for by name - `--clang-format` or an environment variable - and its
    absence is an error instead of a silent fallback to another binary.
    """
    if explicit:
        yield Path(explicit), "--clang-format", True
    for variable in ("IRMV_CLANG_FORMAT", "CLANG_FORMAT"):
        value = os.environ.get(variable)
        if value:
            yield Path(value), "$%s" % variable, True
    yield SOURCE_ROOT / ".tools" / "format" / "bin" / "clang-format", (
        "the tool environment of tools/requirements-format.txt"
    ), False
    yield Path(sys.executable).parent / "clang-format", (
        "the environment of the running interpreter"
    ), False
    for name in ("clang-format-18", "clang-format"):
        found = shutil.which(name)
        if found:
            yield Path(found), "PATH (%s)" % name, False


def resolve_clang_format(explicit):
    """The first usable candidate, as (path, provenance)."""
    searched = []
    for candidate, provenance, required in clang_format_candidates(explicit):
        if candidate.is_file() and os.access(candidate, os.X_OK):
            return candidate.resolve(), provenance
        if required:
            raise ToolError(
                "%s names %s, which is not an executable file; install the "
                "pinned tool with\n"
                "  python3 -m venv .tools/format && "
                ".tools/format/bin/pip install -r tools/requirements-format.txt"
                % (provenance, candidate)
            )
        searched.append("%s (%s)" % (candidate, provenance))
    raise ToolError(
        "no clang-format found; pin %s with\n"
        "  python3 -m venv .tools/format && "
        ".tools/format/bin/pip install -r tools/requirements-format.txt\n"
        "searched: %s" % (REQUIREMENTS_PATH, ", ".join(searched))
    )


def check_version(tool):
    """Fails unless `tool` reports exactly REQUIRED_VERSION."""
    try:
        completed = subprocess.run(
            [str(tool), "--version"], capture_output=True, text=True, check=False
        )
    except OSError as error:
        raise ToolError("cannot run %s: %s" % (tool, error)) from error
    output = (completed.stdout + completed.stderr).strip()
    match = VERSION_PATTERN.search(output) if completed.returncode == 0 else None
    if match is None:
        raise ToolError(
            "cannot read the version of %s (exit %d): %r"
            % (tool, completed.returncode, output)
        )
    found = match.group(1)
    if found != REQUIRED_VERSION:
        raise ToolError(
            "refusing to run clang-format %s at %s: this repository is pinned to "
            "%s (%s)" % (found, tool, REQUIRED_VERSION, REQUIREMENTS_PATH)
        )
    return found


def is_excluded(directory):
    name = directory.name
    return name in EXCLUDED_DIRECTORY_NAMES or name.startswith(
        EXCLUDED_DIRECTORY_PREFIXES
    )


def source_files():
    """Every first-party C/C++ file, sorted, with no generated or vendored one."""
    files = []
    for root in SCAN_DIRECTORIES:
        base = SOURCE_ROOT / root
        if not base.is_dir():
            continue
        for path in base.rglob("*"):
            if path.suffix not in SOURCE_SUFFIXES or not path.is_file():
                continue
            # A file below an excluded directory is never first-party, however
            # deep in the scan tree it is. Only path components inside the
            # repository count: a directory above the source root is none of
            # this scan's business.
            relative_parents = path.relative_to(SOURCE_ROOT).parents
            if any(is_excluded(part) for part in relative_parents):
                continue
            files.append(path)
    return sorted(set(files))


def formatted_bytes(tool, path):
    """clang-format output for `path`, without touching the file."""
    completed = subprocess.run(
        [str(tool), "--style=file:%s" % CONFIG_PATH, str(path)],
        capture_output=True,
        check=False,
    )
    if completed.returncode != 0:
        raise ToolError(
            "clang-format failed on %s (exit %d):\n%s"
            % (path, completed.returncode, completed.stderr.decode("utf-8", "replace"))
        )
    return completed.stdout


def unified_diff(path, current, formatted):
    lines = difflib.unified_diff(
        current.decode("utf-8", "replace").splitlines(keepends=True),
        formatted.decode("utf-8", "replace").splitlines(keepends=True),
        fromfile=str(path.relative_to(SOURCE_ROOT)),
        tofile="%s (formatted)" % path.relative_to(SOURCE_ROOT),
        n=DIFF_CONTEXT_LINES,
    )
    return "".join(lines)


def check_file(tool, path):
    """(path, diff) of one file; the diff is empty when the file is clean."""
    current = path.read_bytes()
    formatted = formatted_bytes(tool, path)
    if current == formatted:
        return path, ""
    lines = unified_diff(path, current, formatted)
    head = lines.splitlines(keepends=True)
    if len(head) > DIFF_LINE_LIMIT:
        lines = "".join(head[:DIFF_LINE_LIMIT]) + "... diff truncated\n"
    return path, lines


def write_file(tool, path):
    """(path, changed) after formatting `path` in place."""
    current = path.read_bytes()
    formatted = formatted_bytes(tool, path)
    if current == formatted:
        return path, False
    path.write_bytes(formatted)
    return path, True


def run(function, tool, files, jobs):
    with concurrent.futures.ThreadPoolExecutor(max_workers=jobs) as pool:
        return list(pool.map(lambda path: function(tool, path), files))


def main():
    parser = argparse.ArgumentParser(
        description="Check or apply the formatting of the first-party C/C++ tree."
    )
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument(
        "--check", action="store_true", help="read-only check (the default)"
    )
    mode.add_argument(
        "--write", action="store_true", help="format the files in place"
    )
    parser.add_argument(
        "--clang-format",
        metavar="PATH",
        help="clang-format 18.1.8 to use instead of the resolved one",
    )
    parser.add_argument(
        "--jobs",
        type=int,
        default=min(32, (os.cpu_count() or 1) + 4),
        metavar="N",
        help="files to format in parallel (default: %(default)s)",
    )
    arguments = parser.parse_args()

    try:
        tool, provenance = resolve_clang_format(arguments.clang_format)
        version = check_version(tool)
    except ToolError as error:
        print("format: %s" % error, file=sys.stderr)
        return 2

    if not CONFIG_PATH.is_file():
        print("format: %s is missing" % CONFIG_PATH, file=sys.stderr)
        return 2

    files = source_files()
    if not files:
        # A check that scans nothing would pass for the wrong reason.
        print(
            "format: no C/C++ file found under %s"
            % ", ".join(str(SOURCE_ROOT / name) for name in SCAN_DIRECTORIES),
            file=sys.stderr,
        )
        return 2

    write = arguments.write
    function = write_file if write else check_file
    results = run(function, tool, files, max(1, arguments.jobs))

    print(
        "format: clang-format %s from %s (%s); %d file(s) under %s"
        % (
            version,
            tool,
            provenance,
            len(files),
            ", ".join(SCAN_DIRECTORIES),
        )
    )

    if write:
        changed = [path for path, did_change in results if did_change]
        for path in changed:
            print("  formatted %s" % path.relative_to(SOURCE_ROOT))
        print("format: %d file(s) formatted" % len(changed))
        return 0

    diffs = [(path, diff) for path, diff in results if diff]
    for path, diff in diffs:
        print("  %s is not formatted:" % path.relative_to(SOURCE_ROOT))
        print(diff, end="" if diff.endswith("\n") else "\n")
    if diffs:
        print(
            "format-check: %d of %d file(s) differ; "
            "run `cmake --build build --target format`" % (len(diffs), len(files))
        )
        return 1
    print("format-check: %d file(s) clean" % len(files))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
