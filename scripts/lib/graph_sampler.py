#!/usr/bin/env python3
"""Sample RSS/PSS/USS of every node in a running ROS graph, one round at a time.

This is the multi-PID counterpart of ``profile_local_map_sampler``. It reuses
that module's zero-perturbation ``/proc`` readers instead of reimplementing
them, and adds the three properties a graph-wide measurement needs:

* a frozen ``node name -> PID -> starttime -> exe`` mapping,
* an identity check before every single read, so a recycled PID can never be
  reported as the original node,
* round atomicity: a ``sample_index`` is written only when *every* frozen node
  was read in that same round. Summing PSS across an incomplete round would
  understate the system total, so an incomplete round is never emitted.
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import signal
import time
from dataclasses import dataclass
from pathlib import Path

try:
    from .profile_local_map_sampler import (
        parse_smaps_rollup,
        read_cgroup_v2_path,
        read_headroom,
    )
    from .profile_role_monitor import read_process_state
except ImportError:  # Direct execution remains useful for diagnostics.
    from profile_local_map_sampler import (
        parse_smaps_rollup,
        read_cgroup_v2_path,
        read_headroom,
    )
    from profile_role_monitor import read_process_state


SAMPLE_COLUMNS = (
    "sample_index",
    "receipt_monotonic_ns",
    "node_name",
    "role",
    "tier",
    "pid",
    "rss_kib",
    "pss_kib",
    "uss_kib",
)

HEADROOM_COLUMNS = (
    "sample_index",
    "receipt_monotonic_ns",
    "node_count",
    "cgroup_current_bytes",
    "cgroup_max_bytes",
    "mem_available_kib",
    "oom",
    "oom_kill",
)

MEASURED_ROLE = "measured"
SUPPORT_ROLE = "support"
ROLES = (MEASURED_ROLE, SUPPORT_ROLE)

PRODUCT_TIER = "product"
FIXTURE_TIER = "fixture"
SUPPORT_TIER = "support"
TIERS = (PRODUCT_TIER, FIXTURE_TIER, SUPPORT_TIER)

IDENTITIES_FILENAME = "graph-identities.json"
SAMPLES_FILENAME = "graph-samples.csv"
HEADROOM_FILENAME = "graph-headroom.csv"


class GraphIdentityError(RuntimeError):
    """A frozen node identity is missing, ambiguous, or no longer valid."""


@dataclass(frozen=True)
class NodeIdentity:
    node_name: str
    role: str
    tier: str
    pid: int
    starttime: str
    exe: str

    def as_dict(self) -> dict[str, object]:
        return {
            "node_name": self.node_name,
            "role": self.role,
            "tier": self.tier,
            "pid": self.pid,
            "starttime": self.starttime,
            "exe": self.exe,
        }


def identity_from_dict(payload: dict[str, object]) -> NodeIdentity:
    try:
        identity = NodeIdentity(
            str(payload["node_name"]),
            str(payload["role"]),
            str(payload["tier"]),
            int(payload["pid"]),  # type: ignore[arg-type]
            str(payload["starttime"]),
            str(payload["exe"]),
        )
    except (KeyError, TypeError, ValueError) as error:
        raise GraphIdentityError("frozen identity record is incomplete") from error
    if identity.role not in ROLES:
        raise GraphIdentityError(f"{identity.node_name}: unknown role {identity.role}")
    if identity.tier not in TIERS:
        raise GraphIdentityError(f"{identity.node_name}: unknown tier {identity.tier}")
    return identity


def load_frozen_identities(path: Path) -> list[NodeIdentity]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
        records = payload["nodes"]
    except (KeyError, OSError, ValueError) as error:
        raise GraphIdentityError(f"{path}: frozen identities are unreadable") from error
    identities = [identity_from_dict(record) for record in records]
    if not identities:
        raise GraphIdentityError(f"{path}: frozen identities are empty")
    names = [identity.node_name for identity in identities]
    if len(set(names)) != len(names):
        raise GraphIdentityError(f"{path}: frozen identities repeat a node name")
    pids = [identity.pid for identity in identities]
    if len(set(pids)) != len(pids):
        raise GraphIdentityError(f"{path}: frozen identities repeat a PID")
    if not any(identity.role == MEASURED_ROLE for identity in identities):
        raise GraphIdentityError(f"{path}: frozen identities contain no measured node")
    return identities


def dump_frozen_identities(identities: list[NodeIdentity], proc_root: Path) -> str:
    payload = {
        "frozen_monotonic_ns": time.monotonic_ns(),
        "proc_root": str(proc_root),
        "nodes": [identity.as_dict() for identity in identities],
    }
    return json.dumps(payload, indent=2, sort_keys=True) + "\n"


def read_process_exe(pid: int, proc_root: Path = Path("/proc")) -> str:
    return os.readlink(proc_root / str(pid) / "exe")


def read_process_cmdline(pid: int, proc_root: Path = Path("/proc")) -> list[str]:
    raw = (proc_root / str(pid) / "cmdline").read_bytes()
    return [token for token in raw.decode("utf-8", "replace").split("\0") if token]


def executable_token(argv: list[str]) -> str | None:
    if not argv:
        return None
    if Path(argv[0]).name.startswith("python"):
        for token in argv[1:]:
            if not token.startswith("-"):
                return token
        return None
    return argv[0]


def node_names_in_cmdline(argv: list[str]) -> set[str]:
    """Return the ROS node names a command line can legitimately claim.

    An explicit ``__node:=`` / ``__name:=`` remap is authoritative. Only when no
    remap is present does the executable base name stand in for the node name,
    so a remapped process is never also matched under its executable name.
    """
    remapped = {
        token.split(":=", 1)[1]
        for token in argv
        if token.startswith(("__node:=", "__name:="))
    }
    if remapped:
        return {name for name in remapped if name}
    executable = executable_token(argv)
    return {Path(executable).name} if executable else set()


def freeze_identity(
    node_name: str,
    role: str,
    tier: str,
    pid: int,
    proc_root: Path = Path("/proc"),
) -> NodeIdentity:
    try:
        state, starttime = read_process_state(pid, proc_root)
        exe = read_process_exe(pid, proc_root)
    except (OSError, ValueError) as error:
        raise GraphIdentityError(
            f"{node_name}: process {pid} identity is unavailable"
        ) from error
    if state == "Z":
        raise GraphIdentityError(f"{node_name}: process {pid} is a zombie")
    return NodeIdentity(node_name, role, tier, pid, starttime, exe)


def verify_identity(identity: NodeIdentity, proc_root: Path = Path("/proc")) -> None:
    try:
        state, starttime = read_process_state(identity.pid, proc_root)
        exe = read_process_exe(identity.pid, proc_root)
    except (OSError, ValueError) as error:
        raise GraphIdentityError(
            f"{identity.node_name}: process {identity.pid} identity is unavailable"
        ) from error
    if state == "Z":
        raise GraphIdentityError(
            f"{identity.node_name}: process {identity.pid} is a zombie"
        )
    if starttime != identity.starttime:
        raise GraphIdentityError(
            f"{identity.node_name}: PID {identity.pid} starttime changed "
            f"({identity.starttime} -> {starttime}); the PID was reused"
        )
    if exe != identity.exe:
        raise GraphIdentityError(
            f"{identity.node_name}: PID {identity.pid} executable changed "
            f"({identity.exe} -> {exe})"
        )


def resolve_node_identities(
    specifications: list[tuple[str, str, str]],
    proc_root: Path = Path("/proc"),
) -> list[NodeIdentity]:
    """Resolve ``(node_name, role, tier)`` requests to frozen identities.

    Matching reads ``/proc/*/cmdline`` directly instead of ``ros2 node list``,
    which reports names but not PIDs.
    """
    wanted = {node_name for node_name, _role, _tier in specifications}
    if len(wanted) != len(specifications):
        raise GraphIdentityError("a node name was requested more than once")
    matches: dict[str, list[int]] = {node_name: [] for node_name in wanted}
    own_pid = os.getpid()
    for entry in sorted(proc_root.iterdir(), key=lambda path: path.name):
        if not entry.name.isdigit():
            continue
        pid = int(entry.name)
        if pid == own_pid:
            continue
        try:
            argv = read_process_cmdline(pid, proc_root)
        except OSError:
            continue
        for node_name in node_names_in_cmdline(argv) & wanted:
            matches[node_name].append(pid)

    missing = sorted(name for name, pids in matches.items() if not pids)
    if missing:
        raise GraphIdentityError(
            "expected nodes are not running: " + ", ".join(missing)
        )
    ambiguous = sorted(name for name, pids in matches.items() if len(pids) > 1)
    if ambiguous:
        detail = "; ".join(
            f"{name} -> {', '.join(str(pid) for pid in matches[name])}"
            for name in ambiguous
        )
        raise GraphIdentityError(f"expected nodes matched several processes: {detail}")

    identities = [
        freeze_identity(node_name, role, tier, matches[node_name][0], proc_root)
        for node_name, role, tier in specifications
    ]
    pids = [identity.pid for identity in identities]
    if len(set(pids)) != len(pids):
        raise GraphIdentityError("two expected node names resolved to the same PID")
    return identities


@dataclass(frozen=True)
class SampleRound:
    rows: list[dict[str, object]]
    headroom_row: dict[str, object]


def sample_round(
    identities: list[NodeIdentity],
    sample_index: int,
    cgroup_directory: Path,
    proc_root: Path = Path("/proc"),
    meminfo_path: Path = Path("/proc/meminfo"),
) -> SampleRound:
    """Read every frozen node once. Any failure discards the whole round."""
    rows: list[dict[str, object]] = []
    for identity in identities:
        verify_identity(identity, proc_root)
        receipt_ns = time.monotonic_ns()
        memory = parse_smaps_rollup(
            (proc_root / str(identity.pid) / "smaps_rollup").read_text(encoding="ascii")
        )
        rows.append(
            {
                "sample_index": sample_index,
                "receipt_monotonic_ns": receipt_ns,
                "node_name": identity.node_name,
                "role": identity.role,
                "tier": identity.tier,
                "pid": identity.pid,
                "rss_kib": memory.rss_kib,
                "pss_kib": memory.pss_kib,
                "uss_kib": memory.uss_kib,
            }
        )
    headroom = read_headroom(cgroup_directory, meminfo_path)
    headroom_row = {
        "sample_index": sample_index,
        "receipt_monotonic_ns": time.monotonic_ns(),
        "node_count": len(rows),
        "cgroup_current_bytes": headroom.cgroup_current_bytes
        if headroom.cgroup_current_bytes is not None
        else "",
        "cgroup_max_bytes": headroom.cgroup_max_bytes
        if headroom.cgroup_max_bytes is not None
        else "",
        "mem_available_kib": headroom.mem_available_kib,
        "oom": headroom.oom,
        "oom_kill": headroom.oom_kill,
    }
    return SampleRound(rows, headroom_row)


def sample_loop(
    identities: list[NodeIdentity],
    samples_path: Path,
    headroom_path: Path,
    interval_s: float,
    stop_requested,
    duration_s: float = 0.0,
    proc_root: Path = Path("/proc"),
    cgroup_root: Path = Path("/sys/fs/cgroup"),
    meminfo_path: Path = Path("/proc/meminfo"),
) -> int:
    if not identities:
        raise GraphIdentityError("no frozen node identities to sample")
    for identity in identities:
        verify_identity(identity, proc_root)
    cgroup_directory = cgroup_root / read_cgroup_v2_path(identities[0].pid, proc_root)

    sample_index = 0
    with samples_path.open("x", newline="", encoding="utf-8") as samples_stream, (
        headroom_path.open("x", newline="", encoding="utf-8")
    ) as headroom_stream:
        samples_writer = csv.DictWriter(samples_stream, fieldnames=SAMPLE_COLUMNS)
        headroom_writer = csv.DictWriter(headroom_stream, fieldnames=HEADROOM_COLUMNS)
        samples_writer.writeheader()
        headroom_writer.writeheader()
        samples_stream.flush()
        headroom_stream.flush()

        started = time.monotonic()
        next_sample = started
        while not stop_requested():
            now = time.monotonic()
            if duration_s > 0 and now - started >= duration_s:
                break
            if now >= next_sample:
                current = sample_round(
                    identities, sample_index, cgroup_directory, proc_root, meminfo_path
                )
                samples_writer.writerows(current.rows)
                headroom_writer.writerow(current.headroom_row)
                samples_stream.flush()
                headroom_stream.flush()
                sample_index += 1
                next_sample = now + interval_s
            time.sleep(min(0.05, interval_s))
    return sample_index


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("identities", type=Path)
    parser.add_argument("samples", type=Path)
    parser.add_argument("headroom", type=Path)
    parser.add_argument("violation", type=Path)
    parser.add_argument("--interval-s", type=float, default=1.0)
    parser.add_argument("--duration-s", type=float, default=0.0)
    parser.add_argument("--proc-root", type=Path, default=Path("/proc"))
    parser.add_argument("--cgroup-root", type=Path, default=Path("/sys/fs/cgroup"))
    parser.add_argument("--meminfo", type=Path, default=Path("/proc/meminfo"))
    args = parser.parse_args()
    if args.interval_s <= 0:
        parser.error("sampling interval must be positive")
    if args.duration_s < 0:
        parser.error("sampling duration must not be negative")

    stopping = False

    def request_stop(_signum, _frame) -> None:
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    try:
        identities = load_frozen_identities(args.identities)
        sample_loop(
            identities,
            args.samples,
            args.headroom,
            args.interval_s,
            lambda: stopping,
            args.duration_s,
            args.proc_root,
            args.cgroup_root,
            args.meminfo,
        )
    except (GraphIdentityError, OSError, ValueError) as error:
        args.violation.write_text(
            f"reason={error}\nreceipt_monotonic_ns={time.monotonic_ns()}\n",
            encoding="utf-8",
        )
        print(error, file=os.sys.stderr)
        raise SystemExit(1) from error


if __name__ == "__main__":
    main()
