#!/usr/bin/env python3

import argparse
import json
import math
import statistics
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from lib.profile_analysis import (  # noqa: E402
    parse_manifest,
    parse_memory_samples,
    parse_pidstat,
    percentile_nearest_rank,
    require_independent_evidence,
    resolve_distinct_run_dirs,
    slope_kib_per_minute,
)


def analyze_run(run_dir: Path) -> dict:
    manifest = parse_manifest(run_dir / "run-manifest.txt")
    if manifest.get("valid") != "true" or manifest.get("normal_completion") != "true":
        raise ValueError(f"{run_dir}: run is not valid and normally completed")
    if manifest.get("mode") != "plain-sample":
        raise ValueError(f"{run_dir}: run mode is not plain-sample")
    requested_duration_s = int(manifest["duration_requested_s"])
    if requested_duration_s < 300:
        raise ValueError(f"{run_dir}: requested duration is shorter than 300 seconds")
    workload = parse_manifest(run_dir / "workload-counts.txt")
    duration_s = float(workload["duration_actual_s"])
    if not math.isfinite(duration_s) or duration_s < requested_duration_s:
        raise ValueError(f"{run_dir}: actual workload duration is incomplete")
    counts = {sensor_id: int(workload[sensor_id]) for sensor_id in ("front", "rear", "top")}
    if int(workload["unknown"]) != 0:
        raise ValueError(f"{run_dir}: workload contains unknown sensor IDs")
    if sum(counts.values()) != int(workload["total"]):
        raise ValueError(f"{run_dir}: workload total does not match per-sensor counts")
    required_total = math.ceil(duration_s * 27.0)
    required_each = math.ceil(duration_s * 9.0)
    if (
        int(workload["required_total"]) != required_total
        or int(workload["required_each"]) != required_each
        or int(workload["total"]) < required_total
        or any(count < required_each for count in counts.values())
    ):
        raise ValueError(f"{run_dir}: workload count gate is not satisfied")
    samples = parse_memory_samples(run_dir, manifest, duration_s)
    metrics = {}
    for key in ("uss_kib", "pss_kib", "rss_kib", "smaps_rss_kib", "smaps_pss_kib"):
        points = [(sample["timestamp_ns"], sample[key]) for sample in samples]
        values = [value for _, value in points]
        metrics[key] = {
            "mean_kib": round(statistics.fmean(values), 3),
            "min_kib": min(values),
            "max_kib": max(values),
            "first_kib": values[0],
            "last_kib": values[-1],
            "slope_kib_per_min": round(slope_kib_per_minute(points), 6),
        }
    cpu_values, rss_values = parse_pidstat(run_dir, manifest["tracee_pid"], duration_s)
    return {
        "run": run_dir.name,
        "tracee_pid": int(manifest["tracee_pid"]),
        "duration_s": duration_s,
        "steady_memory_samples": len(samples),
        "memory": metrics,
        "pidstat_steady_samples": len(cpu_values),
        "cpu_mean_percent": round(statistics.fmean(cpu_values), 6),
        "cpu_p95_percent": percentile_nearest_rank(cpu_values, 0.95),
        "pidstat_rss_mean_kib": round(statistics.fmean(rss_values), 3),
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dirs", nargs=3, type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    try:
        resolved_run_dirs = resolve_distinct_run_dirs(args.run_dirs)
        require_independent_evidence(resolved_run_dirs)
    except ValueError as error:
        parser.error(str(error))
    runs = [analyze_run(path) for path in resolved_run_dirs]
    threshold_kib_per_min = 1024.0
    result = {
        "steady_window": "60s through measurement end",
        "growth_threshold_kib_per_min": threshold_kib_per_min,
        "runs": runs,
        "all_rss_slopes_above_threshold": all(
            run["memory"]["rss_kib"]["slope_kib_per_min"] > threshold_kib_per_min
            for run in runs
        ),
        "all_pss_slopes_above_threshold": all(
            run["memory"]["pss_kib"]["slope_kib_per_min"] > threshold_kib_per_min
            for run in runs
        ),
    }
    result["suspected_sustained_growth"] = (
        result["all_rss_slopes_above_threshold"]
        or result["all_pss_slopes_above_threshold"]
    )
    serialized = json.dumps(result, indent=2) + "\n"
    if args.output:
        args.output.write_text(serialized, encoding="utf-8")
    print(serialized, end="")


if __name__ == "__main__":
    main()
