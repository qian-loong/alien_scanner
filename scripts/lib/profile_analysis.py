"""Shared manifest, sampler, and statistics primitives for profiling analysis."""

from __future__ import annotations

import math
import re
import statistics
from dataclasses import dataclass
from pathlib import Path

DEFAULT_SENSITIVITY_MULTIPLIER = 2.0


def parse_manifest_text(text: str) -> dict[str, str]:
    values: dict[str, str] = {}
    for line in text.splitlines():
        if "=" in line:
            key, value = line.split("=", 1)
            values[key] = value
    return values


def parse_manifest(path: Path) -> dict[str, str]:
    return parse_manifest_text(path.read_text(encoding="utf-8"))


@dataclass(frozen=True)
class SlopeEstimate:
    """An ordinary-least-squares growth rate together with what it can resolve.

    ``detectable_kib_per_min`` is the smallest slope this particular measurement
    could have told apart from a flat line, derived from the residual scatter of
    its own samples and the length of its own window. Reporting a slope without
    it would let "no growth was observed" masquerade as "no growth exists".
    """

    slope_kib_per_min: float
    stderr_kib_per_min: float
    residual_stddev_kib: float
    detectable_kib_per_min: float
    sample_count: int
    span_s: float
    sensitivity_multiplier: float

    @property
    def resolvable(self) -> bool:
        return math.isfinite(self.detectable_kib_per_min)


def estimate_slope(
    points: list[tuple[int, int]],
    sensitivity_multiplier: float = DEFAULT_SENSITIVITY_MULTIPLIER,
) -> SlopeEstimate:
    """Fit KiB against time and report the sensitivity floor of that fit."""
    if sensitivity_multiplier <= 0:
        raise ValueError("sensitivity multiplier must be positive")
    origin_ns = points[0][0]
    times_s = [(timestamp - origin_ns) / 1_000_000_000 for timestamp, _ in points]
    values_kib = [float(value) for _, value in points]
    count = len(points)
    span_s = times_s[-1] - times_s[0] if count else 0.0
    mean_time = statistics.fmean(times_s) if count else 0.0
    mean_value = statistics.fmean(values_kib) if count else 0.0
    denominator = sum((value - mean_time) ** 2 for value in times_s)
    if denominator == 0:
        return SlopeEstimate(
            0.0, math.inf, math.inf, math.inf, count, span_s, sensitivity_multiplier
        )
    numerator = sum(
        (time_s - mean_time) * (value - mean_value)
        for time_s, value in zip(times_s, values_kib)
    )
    slope_per_s = numerator / denominator
    intercept = mean_value - slope_per_s * mean_time
    if count <= 2:
        # With two points a line fits exactly; the residuals carry no information
        # about noise, so nothing about the sensitivity floor may be claimed.
        return SlopeEstimate(
            slope_per_s * 60.0,
            math.inf,
            math.inf,
            math.inf,
            count,
            span_s,
            sensitivity_multiplier,
        )
    residual_sum_squares = sum(
        (value - (intercept + slope_per_s * time_s)) ** 2
        for time_s, value in zip(times_s, values_kib)
    )
    residual_stddev = math.sqrt(residual_sum_squares / (count - 2))
    stderr_per_s = residual_stddev / math.sqrt(denominator)
    return SlopeEstimate(
        slope_per_s * 60.0,
        stderr_per_s * 60.0,
        residual_stddev,
        sensitivity_multiplier * stderr_per_s * 60.0,
        count,
        span_s,
        sensitivity_multiplier,
    )


def slope_kib_per_minute(points: list[tuple[int, int]]) -> float:
    return estimate_slope(points).slope_kib_per_min


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


@dataclass(frozen=True)
class Segment:
    """A named sub-window of a run whose statistics are computed on their own.

    A run can contain phases with completely different physics - a moving phase
    whose map is legitimately growing, and a settled phase in which any growth
    is suspicious. Averaging across such phases produces a number that describes
    neither, so a segment is always summarized separately and never merged.
    """

    name: str
    t0_ns: int
    t1_ns: int
    boundary_source: str

    @property
    def duration_s(self) -> float:
        return (self.t1_ns - self.t0_ns) / 1_000_000_000

    def contains(self, timestamp_ns: int) -> bool:
        return self.t0_ns <= timestamp_ns <= self.t1_ns

    def encloses(self, begin_ns: int, end_ns: int) -> bool:
        return self.t0_ns <= begin_ns and end_ns <= self.t1_ns


def parse_segments(
    manifest: dict[str, str], t0_ns: int, t1_ns: int, source: str = "run manifest"
) -> list[Segment]:
    """Read segment boundaries a run declared. No duration is assumed here."""
    raw_count = manifest.get("segment_count", "").strip()
    if not raw_count:
        return []
    try:
        count = int(raw_count)
    except ValueError as error:
        raise ValueError(f"{source}: segment_count is not an integer") from error
    if count < 0:
        raise ValueError(f"{source}: segment_count is negative")
    segments: list[Segment] = []
    for index in range(count):
        prefix = f"segment_{index}_"
        try:
            name = manifest[prefix + "name"]
            begin_ns = int(manifest[prefix + "t0_monotonic_ns"])
            end_ns = int(manifest[prefix + "t1_monotonic_ns"])
        except (KeyError, ValueError) as error:
            raise ValueError(f"{source}: segment {index} is incomplete") from error
        if not name:
            raise ValueError(f"{source}: segment {index} has an empty name")
        if end_ns <= begin_ns:
            raise ValueError(f"{source}: segment {name} is empty")
        if begin_ns < t0_ns or end_ns > t1_ns:
            raise ValueError(f"{source}: segment {name} escapes the formal window")
        if segments and begin_ns < segments[-1].t1_ns:
            raise ValueError(
                f"{source}: segment {name} overlaps {segments[-1].name}; "
                "segment statistics may not share samples"
            )
        segments.append(
            Segment(name, begin_ns, end_ns, manifest.get(prefix + "boundary_source", ""))
        )
    names = [segment.name for segment in segments]
    if len(set(names)) != len(names):
        raise ValueError(f"{source}: segment names repeat")
    return segments


def derive_segments(
    t0_ns: int,
    t1_ns: int,
    latch_ns: int | None,
    drain_s: float = 0.0,
    minimum_duration_s: float = 0.0,
    moving_name: str = "motion",
    settled_name: str = "settled",
    boundary_source: str = "observed",
) -> list[Segment]:
    """Split a window at an observed transition, discarding the drain between.

    The transition is an observation, not an assumption: without one there is
    nothing to split on and the window stays whole. The drain interval after the
    transition belongs to neither phase - messages already in flight are still
    being processed - so it is left out of both segments rather than assigned to
    one of them. A segment too short to support statistics is dropped instead of
    being emitted and rejected downstream.
    """
    if latch_ns is None or not t0_ns < latch_ns < t1_ns:
        return []
    drain_ns = int(round(drain_s * 1_000_000_000))
    minimum_ns = int(round(minimum_duration_s * 1_000_000_000))
    candidates = [
        Segment(moving_name, t0_ns, latch_ns, boundary_source),
        Segment(settled_name, min(latch_ns + drain_ns, t1_ns), t1_ns, boundary_source),
    ]
    return [
        segment
        for segment in candidates
        if segment.t1_ns - segment.t0_ns >= max(minimum_ns, 1)
    ]


def render_segment_manifest(segments: list[Segment]) -> str:
    lines = [f"segment_count={len(segments)}"]
    for index, segment in enumerate(segments):
        lines.append(f"segment_{index}_name={segment.name}")
        lines.append(f"segment_{index}_t0_monotonic_ns={segment.t0_ns}")
        lines.append(f"segment_{index}_t1_monotonic_ns={segment.t1_ns}")
        lines.append(f"segment_{index}_boundary_source={segment.boundary_source}")
    return "".join(f"{line}\n" for line in lines)


def pidstat_sample_window(
    index: int, start_monotonic_ns: int, interval_s: float
) -> tuple[int, int]:
    """Return the interval a pidstat sample averages over.

    pidstat reports the mean over the interval that just elapsed, so sample
    ``index`` describes ``[start + index * interval, start + (index + 1) *
    interval]``. A sample straddling a segment boundary describes both segments
    and therefore belongs to neither.
    """
    interval_ns = int(round(interval_s * 1_000_000_000))
    begin_ns = start_monotonic_ns + index * interval_ns
    return begin_ns, begin_ns + interval_ns


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
