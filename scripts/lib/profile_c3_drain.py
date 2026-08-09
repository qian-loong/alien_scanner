#!/usr/bin/env python3
"""Wait for a profiling run's C3 producer and update streams to converge."""

from __future__ import annotations

import csv
import sys
import time
from pathlib import Path


def _rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def _integer(row: dict[str, str], key: str) -> int:
    return int(row[key])


def _write(path: Path, values: dict[str, object]) -> None:
    path.write_text(
        "".join(f"{key}={value}\n" for key, value in values.items()),
        encoding="utf-8",
    )


def monitor(run_dir: Path, mode: str, start_ns: int, timeout_s: float, output: Path) -> int:
    begin = time.monotonic_ns()
    deadline = begin + int(timeout_s * 1_000_000_000)
    base = {
        "drain_start_monotonic_ns": start_ns,
        "drain_stable_samples": 0,
    }

    if mode == "disabled":
        try:
            updates = _rows(run_dir / "map_updates.csv")
            diagnostics = _rows(run_dir / "map_update_producer_diagnostics.csv")
        except (OSError, csv.Error, ValueError):
            updates = diagnostics = [{"invalid": "1"}]
        converged = not updates and not diagnostics
        end_ns = start_ns
        values = {
            **base,
            "drain_end_monotonic_ns": end_ns,
            "drain_duration_ns": end_ns - start_ns,
            "drain_applicable": "false",
            "drain_converged": "not_applicable" if converged else "false",
            "drain_latest_revision": "none",
            "drain_published_revision": "none",
            "drain_update_revision": "none",
        }
        _write(output, values)
        return 0 if converged else 1

    latest: dict[str, object] = {}
    stable = 0
    while time.monotonic_ns() < deadline:
        try:
            states = _rows(run_dir / "states.csv")
            updates = _rows(run_dir / "map_updates.csv")
            diagnostics = [
                row
                for row in _rows(run_dir / "map_update_producer_diagnostics.csv")
                if _integer(row, "receipt_monotonic_ns") >= start_ns
            ]
            if states and updates and len(diagnostics) >= 2:
                latest_state = max(states, key=lambda row: _integer(row, "revision"))
                latest_update = max(
                    updates, key=lambda row: _integer(row, "receipt_monotonic_ns")
                )
                last_two = diagnostics[-2:]
                published = [_integer(row, "published_revision") for row in last_two]
                latest_revision = _integer(latest_state, "revision")
                update_revision = _integer(latest_update, "new_revision")
                if (
                    all(_integer(row, "pending") == 0 for row in last_two)
                    and all(_integer(row, "in_flight") == 0 for row in last_two)
                    and published[0] == published[1] == latest_revision == update_revision
                ):
                    stable = 2
                    latest = {
                        "drain_latest_revision": latest_revision,
                        "drain_published_revision": published[1],
                        "drain_update_revision": update_revision,
                        "_end_ns": _integer(last_two[-1], "receipt_monotonic_ns"),
                    }
                    break
                stable = 0
                latest = {
                    "drain_latest_revision": latest_revision,
                    "drain_published_revision": published[-1],
                    "drain_update_revision": update_revision,
                }
        except (OSError, csv.Error, KeyError, TypeError, ValueError):
            stable = 0
        time.sleep(0.1)

    converged = stable >= 2
    end_ns = int(latest.pop("_end_ns")) if converged else time.monotonic_ns()
    values = {
        **base,
        "drain_end_monotonic_ns": end_ns,
        "drain_duration_ns": end_ns - start_ns,
        "drain_applicable": "true",
        "drain_converged": str(converged).lower(),
        "drain_stable_samples": stable,
        **latest,
    }
    values.setdefault("drain_latest_revision", "none")
    values.setdefault("drain_published_revision", "none")
    values.setdefault("drain_update_revision", "none")
    _write(output, values)
    return 0 if converged else 1


def main(argv: list[str]) -> int:
    if len(argv) != 7 or argv[1] != "monitor":
        print(
            "usage: profile_c3_drain.py monitor <run-dir> <mode> <start-ns> <timeout-s> <output>",
            file=sys.stderr,
        )
        return 2
    return monitor(
        Path(argv[2]),
        argv[3],
        int(argv[4]),
        float(argv[5]),
        Path(argv[6]),
    )


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
