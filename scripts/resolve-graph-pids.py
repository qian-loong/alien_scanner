#!/usr/bin/env python3
"""Resolve expected ROS node names to PIDs and freeze their process identity.

Resolution reads ``/proc/*/cmdline`` rather than ``ros2 node list``: the graph
listing reports node names but never the PIDs the samplers need. Any expected
node that is missing, or that matches more than one process, is an error - a
graph-wide measurement cannot be attributed if even one node is ambiguous.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from lib.graph_sampler import (  # noqa: E402
    FIXTURE_TIER,
    MEASURED_ROLE,
    PRODUCT_TIER,
    SUPPORT_ROLE,
    SUPPORT_TIER,
    GraphIdentityError,
    dump_frozen_identities,
    freeze_identity,
    resolve_node_identities,
)


def parse_support_specification(value: str) -> tuple[str, int]:
    role_name, _, raw_pid = value.partition("=")
    if not role_name or not raw_pid.isdigit():
        raise GraphIdentityError(f"invalid support specification: {value}")
    return role_name, int(raw_pid)


def build_identities(args: argparse.Namespace) -> list:
    specifications = [
        (node_name, MEASURED_ROLE, PRODUCT_TIER) for node_name in args.product
    ] + [(node_name, MEASURED_ROLE, FIXTURE_TIER) for node_name in args.fixture]
    identities = resolve_node_identities(specifications, args.proc_root)
    measured_pids = {identity.pid for identity in identities}
    for specification in args.support:
        role_name, pid = parse_support_specification(specification)
        if pid in measured_pids:
            raise GraphIdentityError(
                f"support role {role_name} reuses a measured node PID {pid}"
            )
        identities.append(
            freeze_identity(role_name, SUPPORT_ROLE, SUPPORT_TIER, pid, args.proc_root)
        )
    return identities


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--product",
        action="append",
        default=[],
        metavar="NODE",
        help="measured product node name (repeatable)",
    )
    parser.add_argument(
        "--fixture",
        action="append",
        default=[],
        metavar="NODE",
        help="measured simulation/fixture node name (repeatable)",
    )
    parser.add_argument(
        "--support",
        action="append",
        default=[],
        metavar="ROLE=PID",
        help="support role accounted separately, never mixed into a measured node",
    )
    parser.add_argument("--output", type=Path)
    parser.add_argument("--proc-root", type=Path, default=Path("/proc"))
    args = parser.parse_args()
    if not args.product and not args.fixture:
        parser.error("provide at least one --product or --fixture node name")

    try:
        identities = build_identities(args)
        rendered = dump_frozen_identities(identities, args.proc_root)
    except (GraphIdentityError, OSError) as error:
        print(error, file=sys.stderr)
        raise SystemExit(1) from error

    if args.output is not None:
        args.output.write_text(rendered, encoding="utf-8")
    print(rendered, end="")


if __name__ == "__main__":
    main()
