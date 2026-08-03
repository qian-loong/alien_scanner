#!/usr/bin/env python3
"""Validate and summarize one run or three independent C2 profiling runs."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from lib.local_map_profile_analysis import aggregate_runs, analyze_run  # noqa: E402


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dirs", nargs="+", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if len(args.run_dirs) not in {1, 3}:
        parser.error("provide one run directory or exactly three independent runs")

    try:
        results = [analyze_run(run_dir) for run_dir in args.run_dirs]
        payload = {"runs": results, "aggregate": aggregate_runs(results)}
    except (KeyError, OSError, ValueError) as error:
        parser.error(str(error))

    rendered = json.dumps(payload, indent=2, sort_keys=True) + "\n"
    if args.output is not None:
        args.output.write_text(rendered, encoding="utf-8")
    print(rendered, end="")


if __name__ == "__main__":
    main()
