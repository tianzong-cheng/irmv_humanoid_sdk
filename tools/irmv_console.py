"""Presentation helpers for the SDK launchers; evidence logs stay plain."""

import os
import sys

_COLORS = {"info": "36", "ready": "32", "warning": "33", "fault": "31;1"}
_STATUS_PREFIX = "[application] "
_COLUMNS = (("behavior", 12), ("runtime", 10), ("output", 10), ("session", 5))


def interactive(stream):
    return stream.isatty()


def _paint(text, level, stream):
    if (
        level is None
        or not interactive(stream)
        or os.environ.get("NO_COLOR")
        or os.environ.get("TERM") == "dumb"
    ):
        return text
    return f"\x1b[{_COLORS[level]}m{text}\x1b[0m"


def notice(label, text, *, level="info", stream=None):
    if stream is None:
        stream = sys.stdout
    print(
        f"{_paint(f'{label + ':':<15}', level, stream)}{text}", file=stream, flush=True
    )


def application_line(line, stream):
    if not interactive(stream):
        return line
    if line.startswith(
        (
            "[application] policy=",
            "[application] velocity record ",
            "[application] loop ",
            "[application] link ",
        )
    ):
        return ""  # Provenance and end-of-run counters remain in application.log.
    if line.startswith(("application: loop ", "application: link ")):
        return ""  # End-of-run counters remain in application.log.
    if line == "application: simulation inference pending\n":
        return "[application] Waiting for policy inference\n"
    if line.startswith("application: "):
        return _STATUS_PREFIX + line.removeprefix("application: ")
    if not line.startswith(_STATUS_PREFIX + "behavior="):
        return line
    fields = dict(token.split("=", 1) for token in line.split()[1:])
    primary = [f"{name}={fields.pop(name):<{width}}" for name, width in _COLUMNS]
    behavior = primary[0].split("=", 1)[1].rstrip()
    runtime = primary[1].split("=", 1)[1].rstrip()
    level = (
        "fault"
        if "Fault" in (behavior, runtime)
        else "warning"
        if fields.get("reject", "-") != "-" or fields.get("fresh") == "no"
        else "ready"
        if behavior in {"Ready", "Locomotion"}
        else None
    )
    details = [
        f"{name}={value}"
        for name, value in fields.items()
        if value != "-" and (name != "fresh" or value != "yes")
    ]
    return (
        _paint(_STATUS_PREFIX + " ".join(primary + details).rstrip(), level, stream)
        + "\n"
    )


def shutdown_result(failure, code):
    """(text, level) for the final launcher line; never claims an unproven stop."""
    if "fresh ZeroTorque required" in failure:
        return (
            "FAILED: ZeroTorque was not confirmed at shutdown ("
            + failure.removeprefix("shutdown stopped-unproven: ")
            + ")",
            "fault",
        )
    if code:
        return (
            "FAILED: shutdown was not proven" + (f" ({failure})" if failure else ""),
            "fault",
        )
    return "OK: ZeroTorque confirmed; all processes exited", "ready"
