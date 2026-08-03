#!/usr/bin/env python3
"""Sample target memory and align exact 100-revision C2 checkpoints."""

from __future__ import annotations

import argparse
import csv
import os
import signal
import time
from dataclasses import dataclass
from pathlib import Path

try:
    from .profile_role_monitor import read_process_state
except ImportError:  # Direct execution remains useful for diagnostics.
    from profile_role_monitor import read_process_state


RESOURCE_COLUMNS = (
    "receipt_monotonic_ns",
    "state_receipt_monotonic_ns",
    "map_epoch",
    "revision",
    "stamp_ns",
    "rss_kib",
    "pss_kib",
    "uss_kib",
    "cgroup_current_bytes",
    "cgroup_max_bytes",
    "mem_available_kib",
    "oom",
    "oom_kill",
)


@dataclass(frozen=True)
class ProcessMemory:
    rss_kib: int
    pss_kib: int
    uss_kib: int


@dataclass(frozen=True)
class Headroom:
    cgroup_current_bytes: int | None
    cgroup_max_bytes: int | None
    mem_available_kib: int
    oom: int
    oom_kill: int


def parse_smaps_rollup(text: str) -> ProcessMemory:
    values: dict[str, int] = {}
    for line in text.splitlines():
        fields = line.split()
        if len(fields) >= 2 and fields[0].rstrip(":") in {
            "Rss",
            "Pss",
            "Private_Clean",
            "Private_Dirty",
            "Private_Hugetlb",
        }:
            values[fields[0].rstrip(":")] = int(fields[1])
    if "Rss" not in values or "Pss" not in values:
        raise ValueError("smaps_rollup lacks RSS/PSS")
    uss = sum(
        values.get(key, 0)
        for key in ("Private_Clean", "Private_Dirty", "Private_Hugetlb")
    )
    return ProcessMemory(values["Rss"], values["Pss"], uss)


def parse_memory_events(text: str) -> tuple[int, int]:
    values = {}
    for line in text.splitlines():
        fields = line.split()
        if len(fields) == 2:
            values[fields[0]] = int(fields[1])
    return values.get("oom", 0), values.get("oom_kill", 0)


def parse_mem_available_kib(text: str) -> int:
    for line in text.splitlines():
        if line.startswith("MemAvailable:"):
            return int(line.split()[1])
    raise ValueError("/proc/meminfo lacks MemAvailable")


def read_cgroup_v2_path(pid: int, proc_root: Path = Path("/proc")) -> str:
    for line in (proc_root / str(pid) / "cgroup").read_text(encoding="ascii").splitlines():
        hierarchy, controllers, path = line.split(":", 2)
        if hierarchy == "0" and controllers == "":
            return path.lstrip("/")
    raise ValueError("target is not in a cgroup v2 hierarchy")


def _optional_integer(path: Path) -> int | None:
    try:
        value = path.read_text(encoding="ascii").strip()
    except OSError:
        return None
    return None if value == "max" else int(value)


def read_headroom(
    cgroup_directory: Path,
    meminfo_path: Path = Path("/proc/meminfo"),
) -> Headroom:
    events_path = cgroup_directory / "memory.events"
    oom, oom_kill = (0, 0)
    if events_path.is_file():
        oom, oom_kill = parse_memory_events(events_path.read_text(encoding="ascii"))
    return Headroom(
        _optional_integer(cgroup_directory / "memory.current"),
        _optional_integer(cgroup_directory / "memory.max"),
        parse_mem_available_kib(meminfo_path.read_text(encoding="ascii")),
        oom,
        oom_kill,
    )


def safety_reason(current: Headroom, baseline: Headroom) -> str | None:
    if (
        current.cgroup_current_bytes is not None
        and current.cgroup_max_bytes is not None
        and current.cgroup_max_bytes > 0
        and current.cgroup_current_bytes * 100 >= current.cgroup_max_bytes * 80
    ):
        return "cgroup_memory_at_or_above_80_percent"
    if current.mem_available_kib <= 2 * 1024 * 1024:
        return "host_memavailable_at_or_below_2_gib"
    if current.oom > baseline.oom:
        return "cgroup_oom_incremented"
    if current.oom_kill > baseline.oom_kill:
        return "cgroup_oom_kill_incremented"
    return None


def read_new_states(
    stream, fieldnames: list[str], latest_position: int
) -> tuple[list[dict[str, str]], int]:
    stream.seek(latest_position)
    rows = []
    while True:
        position = stream.tell()
        line = stream.readline()
        if not line:
            return rows, stream.tell()
        if not line.endswith("\n"):
            stream.seek(position)
            return rows, position
        values = next(csv.reader([line]))
        if len(values) != len(fieldnames):
            raise ValueError("states.csv row does not match its header")
        rows.append(dict(zip(fieldnames, values)))


def states_received_at_or_after(
    rows: list[dict[str, str]], receipt_floor_ns: int
) -> list[dict[str, str]]:
    selected = []
    for row in rows:
        try:
            receipt_ns = int(row["receipt_monotonic_ns"])
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError("states.csv has an invalid receipt_monotonic_ns") from error
        if receipt_ns >= receipt_floor_ns:
            selected.append(row)
    return selected


def _resource_row(
    state: dict[str, str], sample_ns: int, memory: ProcessMemory, headroom: Headroom
) -> dict[str, int | str]:
    return {
        "receipt_monotonic_ns": sample_ns,
        "state_receipt_monotonic_ns": state["receipt_monotonic_ns"],
        "map_epoch": state["map_epoch"],
        "revision": state["revision"],
        "stamp_ns": state["stamp_ns"],
        "rss_kib": memory.rss_kib,
        "pss_kib": memory.pss_kib,
        "uss_kib": memory.uss_kib,
        "cgroup_current_bytes": headroom.cgroup_current_bytes or "",
        "cgroup_max_bytes": headroom.cgroup_max_bytes or "",
        "mem_available_kib": headroom.mem_available_kib,
        "oom": headroom.oom,
        "oom_kill": headroom.oom_kill,
    }


def sample_loop(args: argparse.Namespace) -> None:
    proc_root: Path = args.proc_root
    state, actual_starttime = read_process_state(args.pid, proc_root)
    if state == "Z" or actual_starttime != args.starttime:
        raise RuntimeError("target identity does not match at sampler start")
    cgroup_path = read_cgroup_v2_path(args.pid, proc_root)
    cgroup_directory = args.cgroup_root / cgroup_path
    baseline = read_headroom(cgroup_directory, args.meminfo)
    sampler_start_ns = time.monotonic_ns()

    deadline = time.monotonic() + args.wait_states_timeout_s
    while not args.states.exists():
        if time.monotonic() >= deadline:
            raise RuntimeError("states.csv did not appear before sampler timeout")
        time.sleep(0.05)

    with args.states.open(newline="", encoding="utf-8") as states_stream, args.output.open(
        "x", newline="", encoding="utf-8"
    ) as resource_stream, args.checkpoints.open("x", newline="", encoding="utf-8") as checkpoint_stream:
        header_line = states_stream.readline()
        fieldnames = next(csv.reader([header_line]))
        required = {"receipt_monotonic_ns", "map_epoch", "revision", "stamp_ns"}
        if not required.issubset(fieldnames):
            raise RuntimeError("states.csv header is incomplete")
        position = states_stream.tell()
        resource_writer = csv.DictWriter(resource_stream, fieldnames=RESOURCE_COLUMNS)
        checkpoint_writer = csv.DictWriter(checkpoint_stream, fieldnames=RESOURCE_COLUMNS)
        resource_writer.writeheader()
        checkpoint_writer.writeheader()
        resource_stream.flush()
        checkpoint_stream.flush()

        latest_state: dict[str, str] | None = None
        recorded_checkpoints: set[int] = set()
        next_sample = time.monotonic()
        while not args.stop_requested():
            state, actual_starttime = read_process_state(args.pid, proc_root)
            if state == "Z" or actual_starttime != args.starttime:
                raise RuntimeError("target identity changed during sampling")
            new_states, position = read_new_states(states_stream, fieldnames, position)
            for current_state in states_received_at_or_after(new_states, sampler_start_ns):
                revision = int(current_state["revision"])
                if revision > 0:
                    latest_state = current_state
                if revision > 0 and revision % 100 == 0 and revision not in recorded_checkpoints:
                    sample_ns = time.monotonic_ns()
                    memory = parse_smaps_rollup(
                        (proc_root / str(args.pid) / "smaps_rollup").read_text(encoding="ascii")
                    )
                    headroom = read_headroom(cgroup_directory, args.meminfo)
                    checkpoint_writer.writerow(
                        _resource_row(current_state, sample_ns, memory, headroom)
                    )
                    checkpoint_stream.flush()
                    recorded_checkpoints.add(revision)

            now = time.monotonic()
            if latest_state is not None and now >= next_sample:
                sample_ns = time.monotonic_ns()
                memory = parse_smaps_rollup(
                    (proc_root / str(args.pid) / "smaps_rollup").read_text(encoding="ascii")
                )
                headroom = read_headroom(cgroup_directory, args.meminfo)
                resource_writer.writerow(_resource_row(latest_state, sample_ns, memory, headroom))
                resource_stream.flush()
                if args.capacity_safety:
                    reason = safety_reason(headroom, baseline)
                    if reason is not None and not args.safety_stop.exists():
                        args.safety_stop.write_text(
                            f"reason={reason}\nreceipt_monotonic_ns={sample_ns}\n",
                            encoding="utf-8",
                        )
                next_sample = now + args.interval_s
            time.sleep(min(0.05, args.interval_s))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("pid", type=int)
    parser.add_argument("starttime")
    parser.add_argument("states", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("checkpoints", type=Path)
    parser.add_argument("safety_stop", type=Path)
    parser.add_argument("--capacity-safety", action="store_true")
    parser.add_argument("--interval-s", type=float, default=1.0)
    parser.add_argument("--wait-states-timeout-s", type=float, default=30.0)
    parser.add_argument("--proc-root", type=Path, default=Path("/proc"))
    parser.add_argument("--cgroup-root", type=Path, default=Path("/sys/fs/cgroup"))
    parser.add_argument("--meminfo", type=Path, default=Path("/proc/meminfo"))
    args = parser.parse_args()
    if args.interval_s <= 0 or args.wait_states_timeout_s <= 0:
        parser.error("sampling intervals must be positive")

    stopping = False

    def request_stop(_signum, _frame) -> None:
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    args.stop_requested = lambda: stopping
    try:
        sample_loop(args)
    except (OSError, RuntimeError, ValueError) as error:
        print(error, file=os.sys.stderr)
        raise SystemExit(1) from error


if __name__ == "__main__":
    main()
