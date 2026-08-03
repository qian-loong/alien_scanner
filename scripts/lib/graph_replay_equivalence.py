#!/usr/bin/env python3
"""Prove that a bag replay reproduced the logical work of the direct run.

Replay only substitutes for the live graph if the nodes under test end up having
done the same thing. Two families of evidence are compared:

* **Identity keys** must match exactly. The final OctoMap payload digest is the
  strongest of these: any dropped or reordered observation changes it.
* **Counted keys** may differ by at most an explicitly declared tolerance, which
  is written into the report. A tolerance that is not zero is therefore visible
  in the evidence rather than hidden in the runner.

A mismatch means the replay lost data or distorted timing; the run is then not
evidence for anything and is marked invalid rather than quietly accepted.
"""

from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass, field
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR.parent) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR.parent))

try:
    from .profile_analysis import parse_manifest
except ImportError:  # Direct execution remains useful for diagnostics.
    from profile_analysis import parse_manifest

IDENTITY_KEYS = (
    "final_map_epoch",
    "mapper_contract_fingerprint",
    "final_octomap_data_sha256",
    "final_known_bounds_min",
    "final_known_bounds_max",
)
COUNTED_KEYS = (
    "observation_count",
    "final_revision",
    "final_octomap_data_bytes",
)


@dataclass
class EquivalenceReport:
    equivalent: bool
    count_tolerance: int
    mismatches: list[str] = field(default_factory=list)
    missing_keys: list[str] = field(default_factory=list)
    compared: dict[str, tuple[str, str]] = field(default_factory=dict)

    def render(self) -> str:
        lines = [
            f"replay_equivalent={'true' if self.equivalent else 'false'}",
            f"count_tolerance={self.count_tolerance}",
            f"compared_key_count={len(self.compared)}",
        ]
        for key in sorted(self.compared):
            baseline, observed = self.compared[key]
            lines.append(f"baseline[{key}]={baseline}")
            lines.append(f"observed[{key}]={observed}")
        for key in self.missing_keys:
            lines.append(f"missing_key={key}")
        for mismatch in self.mismatches:
            lines.append(f"mismatch={mismatch}")
        return "".join(f"{line}\n" for line in lines)


def compare_counts(
    baseline: dict[str, str], observed: dict[str, str], count_tolerance: int = 0
) -> EquivalenceReport:
    if count_tolerance < 0:
        raise ValueError("count tolerance must not be negative")
    report = EquivalenceReport(True, count_tolerance)
    for key in IDENTITY_KEYS + COUNTED_KEYS:
        if key not in baseline or key not in observed:
            report.missing_keys.append(key)
            report.equivalent = False
            continue
        report.compared[key] = (baseline[key], observed[key])
    for key in IDENTITY_KEYS:
        if key in report.compared and baseline[key] != observed[key]:
            report.mismatches.append(
                f"{key}: direct run {baseline[key]!r} vs replay {observed[key]!r}"
            )
            report.equivalent = False
    for key in COUNTED_KEYS:
        if key not in report.compared:
            continue
        try:
            expected = int(baseline[key])
            actual = int(observed[key])
        except ValueError:
            report.mismatches.append(f"{key}: value is not an integer count")
            report.equivalent = False
            continue
        if abs(actual - expected) > count_tolerance:
            report.mismatches.append(
                f"{key}: direct run {expected} vs replay {actual} "
                f"(tolerance {count_tolerance})"
            )
            report.equivalent = False
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("baseline", type=Path)
    parser.add_argument("observed", type=Path)
    parser.add_argument("report", type=Path)
    parser.add_argument("--count-tolerance", type=int, default=0)
    args = parser.parse_args()

    try:
        report = compare_counts(
            parse_manifest(args.baseline),
            parse_manifest(args.observed),
            args.count_tolerance,
        )
    except (OSError, ValueError) as error:
        args.report.write_text(
            f"replay_equivalent=false\nfailure={error}\n", encoding="utf-8"
        )
        print(error, file=sys.stderr)
        raise SystemExit(1) from error

    rendered = report.render()
    args.report.write_text(rendered, encoding="utf-8")
    print(rendered, end="")
    if not report.equivalent:
        print("replay is not equivalent to the direct run", file=sys.stderr)
        raise SystemExit(1)


if __name__ == "__main__":
    main()
