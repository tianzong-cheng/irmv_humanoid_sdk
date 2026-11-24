"""Experiment catalog and path rules for the local web service.

The service never accepts a filesystem path from a browser. Run identities are
opaque values the read-only bridge resolves, and layout identities are chosen by
this service. This module owns the two checks that keep that true: a canonical
records root, and a run/layout selector that cannot escape it.
"""

from __future__ import annotations

import re
from pathlib import Path


class CatalogError(ValueError):
    """A selector or root that the service refuses."""


_IdPattern = re.compile(r"^[0-9a-f]{32}$")


def canonical_records_root(path: Path | str) -> Path:
    root = Path(path).expanduser()
    try:
        resolved = root.resolve(strict=True)
    except (OSError, RuntimeError) as error:
        raise CatalogError(f"the records root is not readable: {error}") from error
    if not resolved.is_dir():
        raise CatalogError(f"the records root is not a directory: {resolved}")
    return resolved


def valid_run_id(value: str) -> bool:
    return bool(_IdPattern.fullmatch(value))


def resolve_run_directory(root: Path, selector: str) -> Path:
    """Resolve one run selector under a canonical root.

    Only a plain directory name is accepted; a path separator, an absolute
    path, ``..`` or a symlink that leaves the root is refused. The HTTP layer
    does not use this function in normal operation - run ids are opaque - but
    the tooling and the tests use it to prove the same rule the bridge applies.
    """

    if not selector or selector in {".", ".."}:
        raise CatalogError("the run selector is empty or relative")
    if "/" in selector or "\\" in selector or Path(selector).is_absolute():
        raise CatalogError("the run selector is not a single directory name")
    candidate = (root / selector).resolve(strict=False)
    try:
        candidate.relative_to(root)
    except ValueError as error:
        raise CatalogError("the run selector leaves the records root") from error
    return candidate


def paginate(
    items: list, continuation: str | None, limit: int = 100
) -> tuple[list, str | None]:
    """Return one bounded page and the continuation of the next one."""

    if limit < 1:
        raise CatalogError("the page size must be positive")
    limit = min(limit, 100)
    offset = 0
    if continuation:
        if not continuation.isdigit():
            raise CatalogError("the continuation is not a decimal offset")
        offset = int(continuation)
    page = items[offset : offset + limit]
    next_offset = offset + len(page)
    return page, (str(next_offset) if next_offset < len(items) else None)
