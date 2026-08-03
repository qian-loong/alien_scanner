#!/usr/bin/env python3
"""Analyze unprobed, callback-only, and full C2 stage calibration runs."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from lib.stage_latency_calibration import (  # noqa: E402
    analyze_calibration,
    quality_values,
    write_quality,
)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("unprobed", type=Path)
    parser.add_argument("callback", type=Path)
    parser.add_argument("full", type=Path)
    parser.add_argument(
        "--callback-replicate",
        type=Path,
        action="append",
        default=[],
        help="additional callback-only run for median p99 evaluation (repeatable)",
    )
    parser.add_argument(
        "--full-replicate",
        type=Path,
        action="append",
        default=[],
        help="additional full-probe run for median p99 evaluation (repeatable)",
    )
    parser.add_argument(
        "--unprobed-replicate",
        type=Path,
        action="append",
        default=[],
        help="additional unprobed run for median CPU evaluation (repeatable)",
    )
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--quality", type=Path, required=True)
    args = parser.parse_args()

    if args.output.resolve() == args.quality.resolve():
        parser.error("output and quality paths must be distinct")
    for path in (args.output, args.quality):
        if path.exists() or path.is_symlink():
            parser.error(f"output path already exists: {path}")

    try:
        payload = analyze_calibration(
            args.unprobed,
            args.callback,
            args.full,
            tuple(args.callback_replicate),
            tuple(args.full_replicate),
            tuple(args.unprobed_replicate),
        )
        rendered = json.dumps(payload, indent=2, sort_keys=True) + "\n"
        args.output.write_text(rendered, encoding="utf-8")
        write_quality(args.quality, quality_values(payload))
    except (KeyError, OSError, ValueError) as error:
        for path in (args.output, args.quality):
            if path.is_file() or path.is_symlink():
                path.unlink()
        parser.error(str(error))

    print(rendered, end="")
    return 0 if payload["gate_pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
