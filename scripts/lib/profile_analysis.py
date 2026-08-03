"""Shared manifest, sampler, and statistics primitives for profiling analysis."""

from __future__ import annotations

import math
import re
import statistics
from pathlib import Path


def parse_manifest(path: Path) -> dict[str, str]:
    values: dict[str, str] = {}
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


def parse_smem_smaps_samples(path: Path, tracee_pid: str) -> list[dict[str, int]]:
    samples: list[dict[str, int]] = []
    for block in path.read_text(encoding="utf-8").split("\n\n"):
        time_match = re.search(r"sample_monotonic_ns=(\d+)", block)
        smem_match = re.search(
            rf"^\s*{re.escape(tracee_pid)}\s+(\d+)\s+(\d+)\s+(\d+)\s+",
            block,
            re.MULTILINE,
        )
        smaps_rss = re.search(r"^Rss:\s+(\d+) kB$", block, re.MULTILINE)
        smaps_pss = re.search(r"^Pss:\s+(\d+) kB$", block, re.MULTILINE)
        if not (time_match and smem_match and smaps_rss and smaps_pss):
            continue
        samples.append(
            {
                "timestamp_ns": int(time_match.group(1)),
                "uss_kib": int(smem_match.group(1)),
                "pss_kib": int(smem_match.group(2)),
                "rss_kib": int(smem_match.group(3)),
                "smaps_rss_kib": int(smaps_rss.group(1)),
                "smaps_pss_kib": int(smaps_pss.group(1)),
            }
        )
    return samples


def parse_memory_samples(
    run_dir: Path, manifest: dict[str, str], duration_s: float
) -> list[dict[str, int]]:
    steady_start_ns = int(manifest["t0_monotonic_ns"]) + 60_000_000_000
    window_end_ns = int(manifest["t1_monotonic_ns"])
    samples = [
        sample
        for sample in parse_smem_smaps_samples(
            run_dir / "smem-smaps.txt", manifest["tracee_pid"]
        )
        if steady_start_ns <= sample["timestamp_ns"] <= window_end_ns
    ]
    timestamps = [sample["timestamp_ns"] for sample in samples]
    if any(right <= left for left, right in zip(timestamps, timestamps[1:])):
        raise ValueError(f"{run_dir}: memory sample timestamps are not strictly increasing")
    expected_samples = max(20, math.floor(max(0.0, duration_s - 60.0) / 10.0) - 1)
    if len(samples) < expected_samples:
        raise ValueError(
            f"{run_dir}: fewer than {expected_samples} steady memory samples"
        )
    return samples


def parse_pidstat_samples(
    path: Path, tracee_pid: str
) -> tuple[list[float], list[int]]:
    cpu_values: list[float] = []
    rss_values: list[int] = []
    for line in path.read_text(encoding="utf-8").splitlines():
        fields = line.split()
        if (
            len(fields) >= 15
            and re.fullmatch(r"\d{2}:\d{2}:\d{2}", fields[0])
            and fields[2] == tracee_pid
        ):
            cpu_values.append(float(fields[7]))
            rss_values.append(int(fields[12]))
    return cpu_values, rss_values


def parse_pidstat(
    run_dir: Path, tracee_pid: str, duration_s: float
) -> tuple[list[float], list[int]]:
    cpu_values, rss_values = parse_pidstat_samples(
        run_dir / "pidstat.txt", tracee_pid
    )
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


def resolve_distinct_run_dirs(run_dirs: list[Path]) -> list[Path]:
    resolved = [path.resolve() for path in run_dirs]
    if len(set(resolved)) != 3:
        raise ValueError("the three run directories must be distinct")
    return resolved


def evidence_identity(run_dir: Path) -> tuple[int, int, int]:
    manifest = parse_manifest(run_dir / "run-manifest.txt")
    return (
        int(manifest["tracee_pid"]),
        int(manifest["t0_monotonic_ns"]),
        int(manifest["t1_monotonic_ns"]),
    )


def require_independent_evidence(run_dirs: list[Path]) -> None:
    if len({evidence_identity(run_dir) for run_dir in run_dirs}) != 3:
        raise ValueError(
            "the three run directories must contain independent evidence identities"
        )
