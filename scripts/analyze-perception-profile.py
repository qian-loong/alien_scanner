#!/usr/bin/env python3

import argparse
import json
import math
import re
import statistics
from pathlib import Path


def parse_manifest(path: Path) -> dict[str, str]:
    values = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        if "=" in line:
            key, value = line.split("=", 1)
            values[key] = value
    return values


def slope_kib_per_minute(points: list[tuple[int, int]]) -> float:
    origin_ns = points[0][0]
    times_s = [(timestamp - origin_ns) / 1_000_000_000 for timestamp, _ in points]
    values_kib = [value for _, value in points]
    mean_time = statistics.fmean(times_s)
    mean_value = statistics.fmean(values_kib)
    denominator = sum((value - mean_time) ** 2 for value in times_s)
    if denominator == 0:
        return 0.0
    numerator = sum(
        (time_s - mean_time) * (value - mean_value)
        for time_s, value in zip(times_s, values_kib)
    )
    return numerator / denominator * 60.0


def percentile_nearest_rank(values: list[float], percentile: float) -> float:
    ordered = sorted(values)
    index = max(0, math.ceil(percentile * len(ordered)) - 1)
    return ordered[index]


def parse_memory_samples(
    run_dir: Path, manifest: dict[str, str], duration_s: float
) -> list[dict[str, int]]:
    tracee_pid = manifest["tracee_pid"]
    steady_start_ns = int(manifest["t0_monotonic_ns"]) + 60_000_000_000
    window_end_ns = int(manifest["t1_monotonic_ns"])
    samples = []
    for block in (run_dir / "smem-smaps.txt").read_text(encoding="utf-8").split("\n\n"):
        time_match = re.search(r"sample_monotonic_ns=(\d+)", block)
        smem_match = re.search(
            rf"^\s*{tracee_pid}\s+(\d+)\s+(\d+)\s+(\d+)\s+",
            block,
            re.MULTILINE,
        )
        smaps_rss = re.search(r"^Rss:\s+(\d+) kB$", block, re.MULTILINE)
        smaps_pss = re.search(r"^Pss:\s+(\d+) kB$", block, re.MULTILINE)
        if not (time_match and smem_match and smaps_rss and smaps_pss):
            continue
        timestamp_ns = int(time_match.group(1))
        if steady_start_ns <= timestamp_ns <= window_end_ns:
            samples.append(
                {
                    "timestamp_ns": timestamp_ns,
                    "uss_kib": int(smem_match.group(1)),
                    "pss_kib": int(smem_match.group(2)),
                    "rss_kib": int(smem_match.group(3)),
                    "smaps_rss_kib": int(smaps_rss.group(1)),
                    "smaps_pss_kib": int(smaps_pss.group(1)),
                }
            )
    timestamps = [sample["timestamp_ns"] for sample in samples]
    if any(right <= left for left, right in zip(timestamps, timestamps[1:])):
        raise ValueError(f"{run_dir}: memory sample timestamps are not strictly increasing")
    expected_samples = max(20, math.floor(max(0.0, duration_s - 60.0) / 10.0) - 1)
    if len(samples) < expected_samples:
        raise ValueError(
            f"{run_dir}: fewer than {expected_samples} steady memory samples"
        )
    return samples


def parse_pidstat(
    run_dir: Path, tracee_pid: str, duration_s: float
) -> tuple[list[float], list[int]]:
    cpu_values = []
    rss_values = []
    for line in (run_dir / "pidstat.txt").read_text(encoding="utf-8").splitlines():
        fields = line.split()
        if (
            len(fields) >= 15
            and re.fullmatch(r"\d{2}:\d{2}:\d{2}", fields[0])
            and fields[2] == tracee_pid
        ):
            cpu_values.append(float(fields[7]))
            rss_values.append(int(fields[12]))
    steady_cpu_values = cpu_values[60:]
    steady_rss_values = rss_values[60:]
    if any(not math.isfinite(value) or value < 0 for value in steady_cpu_values):
        raise ValueError(f"{run_dir}: pidstat contains an invalid CPU sample")
    if any(value < 0 for value in steady_rss_values):
        raise ValueError(f"{run_dir}: pidstat contains an invalid RSS sample")
    expected_steady_samples = max(1, math.floor(duration_s) - 61)
    if len(steady_cpu_values) < expected_steady_samples:
        raise ValueError(
            f"{run_dir}: fewer than {expected_steady_samples} steady pidstat samples"
        )
    return steady_cpu_values, steady_rss_values


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


def resolve_distinct_run_dirs(run_dirs: list[Path]) -> list[Path]:
    resolved = [path.resolve() for path in run_dirs]
    if len(set(resolved)) != 3:
        raise ValueError("the three run directories must be distinct")
    return resolved


def require_independent_evidence(run_dirs: list[Path]) -> None:
    identities = set()
    for run_dir in run_dirs:
        manifest = parse_manifest(run_dir / "run-manifest.txt")
        identities.add(
            (
                int(manifest["tracee_pid"]),
                int(manifest["t0_monotonic_ns"]),
                int(manifest["t1_monotonic_ns"]),
            )
        )
    if len(identities) != 3:
        raise ValueError(
            "the three run directories must contain independent evidence identities"
        )


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
