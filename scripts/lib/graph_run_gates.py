#!/usr/bin/env python3
"""Startup, teardown, and window-structure decisions for a graph profiling run.

These live here rather than inline in the runner so that both the runner and the
test suite exercise the same predicates. Every gate is expressed as "return the
reasons this failed" so a caller can record all of them at once instead of
stopping at the first.

Three gates encode lessons that were paid for in wasted runs:

* ``missing_subscriptions`` - ``ros2 bag record`` stops topic discovery once it
  has subscribed to what it found. A recorder that started before a topic
  existed keeps running happily and silently records nothing for it.
* ``bag_completion_reasons`` - a recorder that is cleaned up before it exits on
  its own never writes ``metadata.yaml``, and a bag without that file is not
  readable at all. The absence is only detectable after the fact.
* ``residual_pids`` - ``ros2 launch`` does not forward signals to the nodes it
  spawned. Without a process-group sweep the scene keeps running for tens of
  minutes and pollutes every later measurement.

The ``segments`` subcommand publishes the sub-window boundaries the analyzer
will later read back, so the write side and the read side share one definition.
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR.parent) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR.parent))

try:
    from .profile_analysis import derive_segments, render_segment_manifest
except ImportError:  # Direct execution remains useful for diagnostics.
    from profile_analysis import derive_segments, render_segment_manifest

SUBSCRIBED_PATTERN = re.compile(
    r"""Subscribed\s+to\s+topic\s*['"]?(?P<topic>[^'"\s]+)['"]?""", re.IGNORECASE
)
TOPIC_COUNT_PATTERN = re.compile(
    r"Topic:\s*(?P<topic>\S+)\s*\|\s*Type:\s*(?P<type>\S+)\s*\|\s*Count:\s*(?P<count>\d+)"
)
MESSAGE_TOTAL_PATTERN = re.compile(r"^Messages:\s*(?P<count>\d+)\s*$", re.MULTILINE)
PS_PID_PATTERN = re.compile(r"^\s*(?P<pid>\d+)\b")


def subscribed_topics(log_text: str) -> set[str]:
    return {match.group("topic") for match in SUBSCRIBED_PATTERN.finditer(log_text)}


def missing_subscriptions(log_text: str, topics: list[str]) -> list[str]:
    """Return the requested topics the recorder never reported subscribing to."""
    subscribed = subscribed_topics(log_text)
    return sorted(topic for topic in topics if topic not in subscribed)


def recorded_topic_counts(info_text: str) -> dict[str, int]:
    return {
        match.group("topic"): int(match.group("count"))
        for match in TOPIC_COUNT_PATTERN.finditer(info_text)
    }


def bag_completion_reasons(
    bag_dir: Path, info_text: str, expected_topics: list[str]
) -> list[str]:
    """Return every reason this bag may not be treated as readable evidence."""
    reasons: list[str] = []
    metadata = bag_dir / "metadata.yaml"
    if not metadata.is_file():
        reasons.append(
            f"{metadata} is missing; the recorder was cleaned up before it "
            "finalized and the bag is unreadable"
        )
    elif metadata.stat().st_size == 0:
        reasons.append(f"{metadata} is empty")
    total = MESSAGE_TOTAL_PATTERN.search(info_text)
    if total is None:
        reasons.append("ros2 bag info did not report a message total; bag is unreadable")
    elif int(total.group("count")) == 0:
        reasons.append("ros2 bag info reports zero messages")
    counts = recorded_topic_counts(info_text)
    for topic in expected_topics:
        if topic not in counts:
            reasons.append(f"{topic} is absent from the recorded bag")
        elif counts[topic] == 0:
            reasons.append(f"{topic} was subscribed but recorded zero messages")
    return reasons


def residual_pids(ps_output: str, tolerated_pids: list[int]) -> list[int]:
    """Return process-group members still alive after a group teardown."""
    tolerated = set(tolerated_pids)
    found = {
        int(match.group("pid"))
        for line in ps_output.splitlines()
        if (match := PS_PID_PATTERN.match(line))
    }
    return sorted(found - tolerated)


def _report(path: Path | None, lines: list[str]) -> None:
    rendered = "".join(f"{line}\n" for line in lines)
    if path is not None:
        path.write_text(rendered, encoding="utf-8")
    print(rendered, end="")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", type=Path)
    subparsers = parser.add_subparsers(dest="gate", required=True)

    subscriptions = subparsers.add_parser("subscriptions")
    subscriptions.add_argument("log", type=Path)
    subscriptions.add_argument("topics", nargs="+")

    bag = subparsers.add_parser("bag")
    bag.add_argument("bag_dir", type=Path)
    bag.add_argument("info", type=Path)
    bag.add_argument("topics", nargs="*", default=[])

    residual = subparsers.add_parser("residual")
    residual.add_argument("ps_output", type=Path)
    residual.add_argument("tolerated", nargs="*", type=int, default=[])

    segments = subparsers.add_parser("segments")
    segments.add_argument("t0_monotonic_ns", type=int)
    segments.add_argument("t1_monotonic_ns", type=int)
    segments.add_argument("--latch-monotonic-ns", type=int)
    segments.add_argument("--drain-s", type=float, default=0.0)
    segments.add_argument("--minimum-duration-s", type=float, default=0.0)
    segments.add_argument("--moving-name", default="motion")
    segments.add_argument("--settled-name", default="settled")
    segments.add_argument("--boundary-source", default="observed")

    args = parser.parse_args()
    if args.gate == "segments":
        rendered = render_segment_manifest(
            derive_segments(
                args.t0_monotonic_ns,
                args.t1_monotonic_ns,
                args.latch_monotonic_ns,
                args.drain_s,
                args.minimum_duration_s,
                args.moving_name,
                args.settled_name,
                args.boundary_source,
            )
        )
        if args.report is not None:
            args.report.write_text(rendered, encoding="utf-8")
        print(rendered, end="")
        return

    lines: list[str] = [f"gate={args.gate}"]
    failed = False
    try:
        if args.gate == "subscriptions":
            missing = missing_subscriptions(
                args.log.read_text(encoding="utf-8", errors="replace"), args.topics
            )
            lines.append(f"requested_topics={len(args.topics)}")
            lines.append(f"missing_topic_count={len(missing)}")
            for topic in missing:
                lines.append(f"missing_topic={topic}")
            failed = bool(missing)
        elif args.gate == "bag":
            info_text = args.info.read_text(encoding="utf-8", errors="replace")
            reasons = bag_completion_reasons(args.bag_dir, info_text, args.topics)
            for topic, count in sorted(recorded_topic_counts(info_text).items()):
                lines.append(f"recorded_message_count[{topic}]={count}")
            lines.append(f"failure_count={len(reasons)}")
            for reason in reasons:
                lines.append(f"failure={reason}")
            failed = bool(reasons)
        else:
            residuals = residual_pids(
                args.ps_output.read_text(encoding="utf-8", errors="replace"),
                args.tolerated,
            )
            lines.append(f"residual_process_count={len(residuals)}")
            for pid in residuals:
                lines.append(f"residual_pid={pid}")
            failed = bool(residuals)
    except OSError as error:
        lines.append(f"failure={error}")
        failed = True
    lines.append(f"gate_pass={'false' if failed else 'true'}")
    _report(args.report, lines)
    if failed:
        print(f"{args.gate} gate failed", file=sys.stderr)
        raise SystemExit(1)


if __name__ == "__main__":
    main()
