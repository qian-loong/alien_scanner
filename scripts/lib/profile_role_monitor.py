#!/usr/bin/env python3
"""Monitor profiling roles by PID/starttime during a formal window."""

from __future__ import annotations

import argparse
import os
import re
import subprocess
import time
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path


class RoleMonitorError(RuntimeError):
    def __init__(self, role: str, detail: str) -> None:
        super().__init__(f"{role}: {detail}")
        self.role = role


@dataclass(frozen=True)
class RoleSpec:
    role: str
    pid: int
    starttime: str


def parse_process_stat(text: str) -> tuple[str, str]:
    suffix = text.rpartition(")")[2].split()
    if len(suffix) < 20:
        raise ValueError("process stat is incomplete")
    return suffix[0], suffix[19]


def read_process_state(pid: int, proc_root: Path = Path("/proc")) -> tuple[str, str]:
    return parse_process_stat(
        (proc_root / str(pid) / "stat").read_text(encoding="ascii")
    )


def parse_role_spec(specification: str) -> RoleSpec:
    role, raw_pid, expected_starttime = specification.split("=", 2)
    if not role or not expected_starttime:
        raise ValueError(f"invalid role specification: {specification}")
    return RoleSpec(role, int(raw_pid), expected_starttime)


def validate_role(
    role: RoleSpec,
    proc_root: Path = Path("/proc"),
    signal_probe: Callable[[int, int], None] | None = os.kill,
) -> None:
    try:
        if signal_probe is not None:
            signal_probe(role.pid, 0)
        state, starttime = read_process_state(role.pid, proc_root)
    except (FileNotFoundError, ProcessLookupError, PermissionError, ValueError) as error:
        raise RoleMonitorError(role.role, "process identity is unavailable") from error
    if state == "Z":
        raise RoleMonitorError(role.role, "process is a zombie")
    if starttime != role.starttime:
        raise RoleMonitorError(role.role, "PID starttime changed")


def trace_session_is_active(trace_session: str) -> tuple[bool, int]:
    result = subprocess.run(
        ["lttng", "list", trace_session],
        check=False,
        capture_output=True,
        text=True,
    )
    pattern = rf"(?:Recording|Tracing) session {re.escape(trace_session)}: \[active\]"
    return result.returncode == 0 and re.search(pattern, result.stdout) is not None, result.returncode


def monitor_roles(
    duration_s: int,
    trace_session: str,
    trace_monitor_path: Path,
    role_monitor_path: Path,
    roles: list[RoleSpec],
) -> None:
    for role in roles:
        validate_role(role)

    with role_monitor_path.open("w", encoding="utf-8") as stream:
        for role in roles:
            stream.write(
                f"role={role.role} pid={role.pid} starttime={role.starttime}\n"
            )

    deadline = time.monotonic() + duration_s
    next_trace_check = 0.0
    while True:
        for role in roles:
            validate_role(role)
        now = time.monotonic()
        if trace_session and now >= next_trace_check:
            active, returncode = trace_session_is_active(trace_session)
            with trace_monitor_path.open("a", encoding="utf-8") as stream:
                stream.write(
                    f"monotonic={now:.6f} returncode={returncode} "
                    f"active={str(active).lower()}\n"
                )
            if not active:
                raise RoleMonitorError("ros_trace_session", "session is not active")
            next_trace_check = now + 1.0
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            break
        time.sleep(min(0.25, remaining))

    with role_monitor_path.open("a", encoding="utf-8") as stream:
        stream.write(f"completed_monotonic={time.monotonic():.9f}\n")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("duration_s", type=int)
    parser.add_argument("trace_session")
    parser.add_argument("trace_monitor_path", type=Path)
    parser.add_argument("role_monitor_path", type=Path)
    parser.add_argument("roles", nargs="+")
    args = parser.parse_args()
    try:
        roles = [parse_role_spec(value) for value in args.roles]
        monitor_roles(
            args.duration_s,
            args.trace_session,
            args.trace_monitor_path,
            args.role_monitor_path,
            roles,
        )
    except (OSError, ValueError, RoleMonitorError) as error:
        role = error.role if isinstance(error, RoleMonitorError) else "role_monitor"
        print(role)
        raise SystemExit(1) from error


if __name__ == "__main__":
    main()
