#!/usr/bin/env python3
"""Summarize one or three graph-wide CPU/memory profiling runs.

One run answers "how much CPU does each node use and where is the bottleneck".
Three independent runs answer "does any node grow without bound".

System memory is reported as a **sum of PSS across measured nodes taken inside
one sampling round**. RSS is never summed: shared library pages are counted in
every process that maps them, so an RSS sum is not a system memory figure.

A run may declare segments - named sub-windows with different physics, such as a
phase whose map is legitimately growing and a phase in which it is saturated.
Each segment is summarized on its own and never averaged together with another,
because a figure spanning both describes neither. Segment boundaries are read
from the run manifest; no duration is assumed here.

Every growth rate is published together with the smallest rate the measurement
could actually have resolved, so "no growth was observed" can never be read as
"no growth exists".
"""

from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
from pathlib import Path
from typing import Any

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from lib.graph_sampler import (  # noqa: E402
    HEADROOM_FILENAME,
    IDENTITIES_FILENAME,
    MEASURED_ROLE,
    SAMPLE_COLUMNS,
    SAMPLES_FILENAME,
    GraphIdentityError,
    NodeIdentity,
    load_frozen_identities,
)
from lib.local_map_profile_analysis import read_csv_rows  # noqa: E402
from lib.profile_analysis import (  # noqa: E402
    Segment,
    estimate_slope,
    parse_manifest,
    parse_pidstat_samples,
    parse_segments,
    percentile_nearest_rank,
    pidstat_sample_window,
    resolve_distinct_run_dirs,
    # Re-exported so that this analyzer and the single-PID one are provably
    # fitting growth with the same regression rather than two look-alikes.
    slope_kib_per_minute,
)

GROWTH_THRESHOLD_KIB_PER_MINUTE = 1024.0
MEMORY_KEYS = ("rss_kib", "pss_kib", "uss_kib")
REPLAY_EQUIVALENCE_FILENAME = "replay-equivalence.txt"
REPLAY_MODE = "graph-replay"
REPLAY_LOOP_MODE = "graph-replay-loop"
RSS_SUM_POLICY = (
    "rss_is_never_summed: shared pages are counted once per mapping process, "
    "so only PSS may be added across nodes"
)
SEGMENT_POLICY = (
    "segment_statistics_are_never_merged: a sample that straddles a boundary "
    "describes both segments and is therefore counted in neither"
)
SENSITIVITY_NOTE = (
    "a slope below detectable_slope_kib_per_min was not measurable in this "
    "window; absence of observed growth bounds the leak rate at that figure "
    "and does not establish that there is none"
)
CPU_UNCERTAINTY_RATIO = 0.30
CPU_NOTE = (
    "CPU percentages carry about +/-30 percent host-contention pollution on this "
    "measurement environment; only effects far above that are conclusive"
)


def _integer(row: dict[str, str], key: str, source: Path) -> int:
    try:
        return int(row[key])
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError(f"{source}: invalid integer in {key}") from error


def _round(value: float, digits: int) -> float:
    return round(float(value), digits)


def _finite(value: float, digits: int) -> float | None:
    """Non-finite figures become null: JSON cannot carry them and, more to the
    point, an unresolvable sensitivity floor must not read as a number."""
    return _round(value, digits) if math.isfinite(value) else None


def _growth_metrics(points: list[tuple[int, int]]) -> dict[str, Any]:
    estimate = estimate_slope(points)
    return {
        "slope_kib_per_min": _round(estimate.slope_kib_per_min, 3),
        "slope_stderr_kib_per_min": _finite(estimate.stderr_kib_per_min, 3),
        "residual_stddev_kib": _finite(estimate.residual_stddev_kib, 3),
        "detectable_slope_kib_per_min": _finite(estimate.detectable_kib_per_min, 3),
        "sensitivity_multiplier": estimate.sensitivity_multiplier,
        "sample_count": estimate.sample_count,
        "span_s": _round(estimate.span_s, 3),
        "growth_is_resolvable": estimate.resolvable,
        "sensitivity_note": SENSITIVITY_NOTE,
    }


def _replay_equivalence(run_dir: Path, manifest: dict[str, str]) -> dict[str, Any] | None:
    """Reject a replay run whose logical work differed from the direct run.

    Looped replay deliberately repeats the same data to hold the map saturated,
    so its totals cannot equal a single direct pass. Such a run is admitted, but
    its equivalence is reported as not established, which confines it to growth
    evidence and keeps it from standing in for the direct measurement.
    """
    path = run_dir / REPLAY_EQUIVALENCE_FILENAME
    mode = manifest.get("mode", "")
    if not path.is_file():
        if mode == REPLAY_LOOP_MODE:
            return {
                "equivalent": None,
                "not_established_reason": (
                    "looped replay repeats the same data, so counts cannot match a "
                    "single direct pass; this run is evidence for growth only"
                ),
            }
        if mode == REPLAY_MODE:
            raise ValueError(
                f"{run_dir}: a replay run must carry {REPLAY_EQUIVALENCE_FILENAME} "
                "proving it matched the direct run"
            )
        return None
    values = parse_manifest(path)
    if values.get("replay_equivalent") != "true":
        mismatches = [
            line.split("=", 1)[1]
            for line in path.read_text(encoding="utf-8").splitlines()
            if line.startswith(("mismatch=", "missing_key=", "failure="))
        ]
        raise ValueError(
            f"{run_dir}: replay did not reproduce the direct run; "
            + ("; ".join(mismatches) or "no detail recorded")
        )
    return {
        "equivalent": True,
        "count_tolerance": values.get("count_tolerance", ""),
        "compared_key_count": values.get("compared_key_count", ""),
    }


def _manifest_window(run_dir: Path) -> tuple[dict[str, str], int, int]:
    manifest = parse_manifest(run_dir / "run-manifest.txt")
    if manifest.get("valid") != "true" or manifest.get("normal_completion") != "true":
        raise ValueError(f"{run_dir}: run is not valid and normally completed")
    try:
        t0_ns = int(manifest["t0_monotonic_ns"])
        t1_ns = int(manifest["t1_monotonic_ns"])
    except (KeyError, ValueError) as error:
        raise ValueError(f"{run_dir}: manifest lacks a formal window") from error
    if t1_ns <= t0_ns:
        raise ValueError(f"{run_dir}: formal window is empty")
    return manifest, t0_ns, t1_ns



def _load_rounds(
    run_dir: Path, identities: list[NodeIdentity], t0_ns: int, t1_ns: int
) -> list[tuple[int, dict[str, dict[str, str]]]]:
    """Group samples into rounds and keep only complete in-window rounds."""
    samples_path = run_dir / SAMPLES_FILENAME
    rows = read_csv_rows(samples_path, SAMPLE_COLUMNS)
    frozen = {identity.node_name: identity for identity in identities}
    grouped: dict[int, dict[str, dict[str, str]]] = {}
    for row in rows:
        node_name = row["node_name"]
        identity = frozen.get(node_name)
        if identity is None:
            raise ValueError(f"{samples_path}: sample for unfrozen node {node_name}")
        if _integer(row, "pid", samples_path) != identity.pid:
            raise ValueError(f"{samples_path}: {node_name} sample has a foreign PID")
        if row["role"] != identity.role or row["tier"] != identity.tier:
            raise ValueError(
                f"{samples_path}: {node_name} sample contradicts its frozen role/tier"
            )
        index = _integer(row, "sample_index", samples_path)
        round_rows = grouped.setdefault(index, {})
        if node_name in round_rows:
            raise ValueError(
                f"{samples_path}: sample_index {index} repeats node {node_name}"
            )
        round_rows[node_name] = row

    expected = set(frozen)
    selected: list[tuple[int, dict[str, dict[str, str]]]] = []
    for index in sorted(grouped):
        round_rows = grouped[index]
        receipts = [
            _integer(row, "receipt_monotonic_ns", samples_path)
            for row in round_rows.values()
        ]
        if all(receipt < t0_ns for receipt in receipts) or all(
            receipt > t1_ns for receipt in receipts
        ):
            continue
        if not all(t0_ns <= receipt <= t1_ns for receipt in receipts):
            raise ValueError(
                f"{samples_path}: sample_index {index} straddles the formal window"
            )
        missing = sorted(expected - set(round_rows))
        if missing:
            raise ValueError(
                f"{samples_path}: sample_index {index} is missing "
                f"{', '.join(missing)}; a PSS total may not be summed from an "
                "incomplete round"
            )
        selected.append((index, round_rows))

    if len(selected) < 2:
        raise ValueError(f"{samples_path}: fewer than two complete in-window rounds")
    indices = [index for index, _rows in selected]
    if indices[-1] - indices[0] + 1 != len(indices):
        raise ValueError(f"{samples_path}: sample_index has a gap; sampling was interrupted")
    return selected


def _validate_headroom(
    run_dir: Path, rounds: list[tuple[int, dict[str, dict[str, str]]]], node_count: int
) -> dict[str, Any]:
    headroom_path = run_dir / HEADROOM_FILENAME
    rows = {
        _integer(row, "sample_index", headroom_path): row
        for row in read_csv_rows(
            headroom_path,
            ("sample_index", "node_count", "mem_available_kib", "oom", "oom_kill"),
        )
    }
    selected = []
    for index, _round_rows in rounds:
        row = rows.get(index)
        if row is None:
            raise ValueError(f"{headroom_path}: sample_index {index} has no headroom row")
        if _integer(row, "node_count", headroom_path) != node_count:
            raise ValueError(
                f"{headroom_path}: sample_index {index} recorded an incomplete round"
            )
        selected.append(row)
    oom_events = _integer(selected[-1], "oom", headroom_path) + _integer(
        selected[-1], "oom_kill", headroom_path
    )
    available = [_integer(row, "mem_available_kib", headroom_path) for row in selected]
    return {
        "mem_available_kib_min": min(available),
        "oom_events_at_window_end": oom_events,
    }


def _memory_points(
    node_name: str,
    key: str,
    rounds: list[tuple[int, dict[str, dict[str, str]]]],
    samples_path: Path,
) -> list[tuple[int, int]]:
    points = [
        (
            _integer(round_rows[node_name], "receipt_monotonic_ns", samples_path),
            _integer(round_rows[node_name], key, samples_path),
        )
        for _index, round_rows in rounds
    ]
    timestamps = [timestamp for timestamp, _value in points]
    if any(right <= left for left, right in zip(timestamps, timestamps[1:])):
        raise ValueError(f"{samples_path}: {node_name} receipts are not increasing")
    return points


def _memory_metrics(
    node_name: str,
    rounds: list[tuple[int, dict[str, dict[str, str]]]],
    samples_path: Path,
) -> dict[str, Any]:
    metrics: dict[str, Any] = {}
    for key in MEMORY_KEYS:
        points = _memory_points(node_name, key, rounds, samples_path)
        values = [value for _timestamp, value in points]
        growth = _growth_metrics(points)
        metrics[key] = {
            "mean": _round(statistics.fmean(values), 1),
            "peak": max(values),
            "slope_kib_per_min": growth["slope_kib_per_min"],
            "growth": growth,
        }
    return metrics


def _cpu_samples(run_dir: Path, identities: list[NodeIdentity]) -> dict[str, list[float]]:
    pidstat_path = run_dir / "pidstat.txt"
    per_node: dict[str, list[float]] = {}
    for identity in identities:
        cpu_values, _rss_values = parse_pidstat_samples(pidstat_path, str(identity.pid))
        if not cpu_values:
            raise ValueError(
                f"{pidstat_path}: no samples for {identity.node_name} (PID {identity.pid})"
            )
        if any(not math.isfinite(value) or value < 0 for value in cpu_values):
            raise ValueError(f"{pidstat_path}: {identity.node_name} has an invalid CPU sample")
        per_node[identity.node_name] = cpu_values
    counts = {len(values) for values in per_node.values()}
    if len(counts) != 1:
        raise ValueError(
            f"{pidstat_path}: nodes have unequal sample counts; sampling was interrupted"
        )
    return per_node


def _cpu_summary(values: list[float]) -> dict[str, Any]:
    return {
        "sample_count": len(values),
        "mean_percent": _round(statistics.fmean(values), 1),
        "p95_percent": _round(percentile_nearest_rank(values, 0.95), 1),
        "peak_percent": _round(max(values), 1),
    }


def _cpu_metrics(run_dir: Path, identities: list[NodeIdentity]) -> dict[str, Any]:
    return {
        node_name: _cpu_summary(values)
        for node_name, values in _cpu_samples(run_dir, identities).items()
    }


def _segment_cpu_indices(
    segment: Segment, sample_count: int, manifest: dict[str, str], run_dir: Path
) -> list[int]:
    """Select the pidstat samples whose whole averaging interval is inside the
    segment. A straddling sample averages across the boundary and belongs to
    neither side."""
    try:
        start_ns = int(manifest["pidstat_start_monotonic_ns"])
        interval_s = float(manifest["pidstat_interval_s"])
    except (KeyError, ValueError) as error:
        raise ValueError(
            f"{run_dir}: segment CPU statistics need pidstat_start_monotonic_ns "
            "and pidstat_interval_s in the manifest"
        ) from error
    if interval_s <= 0:
        raise ValueError(f"{run_dir}: pidstat_interval_s must be positive")
    return [
        index
        for index in range(sample_count)
        if segment.encloses(*pidstat_sample_window(index, start_ns, interval_s))
    ]


def _pss_total(
    names: list[str],
    rounds: list[tuple[int, dict[str, dict[str, str]]]],
    samples_path: Path,
) -> dict[str, Any] | None:
    if not names:
        return None
    points = [
        (
            min(
                _integer(round_rows[name], "receipt_monotonic_ns", samples_path)
                for name in names
            ),
            sum(_integer(round_rows[name], "pss_kib", samples_path) for name in names),
        )
        for _index, round_rows in rounds
    ]
    values = [value for _timestamp, value in points]
    growth = _growth_metrics(points)
    return {
        "node_count": len(names),
        "mean_kib": _round(statistics.fmean(values), 1),
        "peak_kib": max(values),
        "slope_kib_per_min": growth["slope_kib_per_min"],
        "growth": growth,
        "summed_within_one_sampling_round": True,
    }


def _segment_report(
    segment: Segment,
    identities: list[NodeIdentity],
    measured: list[NodeIdentity],
    rounds: list[tuple[int, dict[str, dict[str, str]]]],
    cpu_samples: dict[str, list[float]],
    manifest: dict[str, str],
    run_dir: Path,
    samples_path: Path,
) -> dict[str, Any]:
    selected = [
        entry
        for entry in rounds
        if all(
            segment.contains(_integer(row, "receipt_monotonic_ns", samples_path))
            for row in entry[1].values()
        )
    ]
    if len(selected) < 2:
        raise ValueError(
            f"{run_dir}: segment {segment.name} holds fewer than two complete "
            "sampling rounds, so it supports no statistics of its own"
        )
    sample_count = len(next(iter(cpu_samples.values())))
    cpu_indices = _segment_cpu_indices(segment, sample_count, manifest, run_dir)
    if not cpu_indices:
        raise ValueError(
            f"{run_dir}: segment {segment.name} encloses no whole pidstat interval"
        )
    nodes = {
        identity.node_name: {
            "role": identity.role,
            "tier": identity.tier,
            "cpu": _cpu_summary(
                [cpu_samples[identity.node_name][index] for index in cpu_indices]
            ),
            "memory": _memory_metrics(identity.node_name, selected, samples_path),
        }
        for identity in identities
    }
    return {
        "name": segment.name,
        "t0_monotonic_ns": segment.t0_ns,
        "t1_monotonic_ns": segment.t1_ns,
        "duration_s": _round(segment.duration_s, 3),
        "boundary_source": segment.boundary_source,
        "round_count": len(selected),
        "cpu_sample_count": len(cpu_indices),
        "cpu_samples_excluded_at_boundaries": sample_count - len(cpu_indices),
        "nodes": nodes,
        "system_total_pss_measured": _pss_total(
            [identity.node_name for identity in measured], selected, samples_path
        ),
        "segment_policy": SEGMENT_POLICY,
    }


def _leak_sensitivity(entries: list[dict[str, Any]]) -> dict[str, Any]:
    """The coarsest per-node PSS sensitivity floor bounds what the run can claim."""
    floors = [entry["growth"]["detectable_slope_kib_per_min"] for entry in entries]
    resolvable = all(floor is not None for floor in floors)
    return {
        "pss_detectable_slope_kib_per_min": max(floors) if resolvable else None,
        "all_nodes_resolvable": resolvable,
        "note": SENSITIVITY_NOTE,
    }



def analyze_run(run_dir: Path) -> dict[str, Any]:
    manifest, t0_ns, t1_ns = _manifest_window(run_dir)
    replay_equivalence = _replay_equivalence(run_dir, manifest)
    identities = load_frozen_identities(run_dir / IDENTITIES_FILENAME)
    rounds = _load_rounds(run_dir, identities, t0_ns, t1_ns)
    samples_path = run_dir / SAMPLES_FILENAME

    measured = [identity for identity in identities if identity.role == MEASURED_ROLE]
    support = [identity for identity in identities if identity.role != MEASURED_ROLE]
    cpu_samples = _cpu_samples(run_dir, identities)
    cpu = {name: _cpu_summary(values) for name, values in cpu_samples.items()}

    nodes = {
        identity.node_name: {
            "pid": identity.pid,
            "role": identity.role,
            "tier": identity.tier,
            "cpu": cpu[identity.node_name],
            "memory": _memory_metrics(identity.node_name, rounds, samples_path),
        }
        for identity in identities
    }

    segments = [
        _segment_report(
            segment,
            identities,
            measured,
            rounds,
            cpu_samples,
            manifest,
            run_dir,
            samples_path,
        )
        for segment in parse_segments(manifest, t0_ns, t1_ns, str(run_dir))
    ]

    cpu_ranking = sorted(
        (
            {
                "node_name": identity.node_name,
                "tier": identity.tier,
                "cpu_mean_percent": cpu[identity.node_name]["mean_percent"],
            }
            for identity in measured
        ),
        key=lambda entry: (-entry["cpu_mean_percent"], entry["node_name"]),
    )
    cpu_by_tier: dict[str, float] = {}
    for identity in measured:
        cpu_by_tier[identity.tier] = _round(
            cpu_by_tier.get(identity.tier, 0.0) + cpu[identity.node_name]["mean_percent"],
            1,
        )

    return {
        "run": run_dir.name,
        "t0_monotonic_ns": t0_ns,
        "t1_monotonic_ns": t1_ns,
        "duration_s": _round((t1_ns - t0_ns) / 1_000_000_000, 3),
        "round_count": len(rounds),
        "measured_pids": sorted(identity.pid for identity in measured),
        "nodes": nodes,
        "segments": segments,
        "segment_names": [segment["name"] for segment in segments],
        "segment_policy": SEGMENT_POLICY,
        "cpu_ranking": cpu_ranking,
        "cpu_mean_percent_by_tier": cpu_by_tier,
        "cpu_percent_uncertainty_ratio": CPU_UNCERTAINTY_RATIO,
        "cpu_percent_note": CPU_NOTE,
        "system_total_pss_measured": _pss_total(
            [identity.node_name for identity in measured], rounds, samples_path
        ),
        "support_role_total_pss": _pss_total(
            [identity.node_name for identity in support], rounds, samples_path
        ),
        "leak_sensitivity": _leak_sensitivity(
            [nodes[identity.node_name]["memory"]["pss_kib"] for identity in measured]
        ),
        "leak_sensitivity_by_segment": {
            segment["name"]: _leak_sensitivity(
                [
                    segment["nodes"][identity.node_name]["memory"]["pss_kib"]
                    for identity in measured
                ]
            )
            for segment in segments
        },
        "replay_equivalence": replay_equivalence,
        "rss_sum_policy": RSS_SUM_POLICY,
        "headroom": _validate_headroom(run_dir, rounds, len(identities)),
        "mode": manifest.get("mode", ""),
    }


def _evidence_identity(result: dict[str, Any]) -> tuple:
    return (
        tuple(result["measured_pids"]),
        result["t0_monotonic_ns"],
        result["t1_monotonic_ns"],
    )


def aggregate_runs(results: list[dict[str, Any]]) -> dict[str, Any] | None:
    if len(results) != 3:
        return None
    if len({_evidence_identity(result) for result in results}) != 3:
        raise ValueError("three-run aggregate contains duplicate evidence identities")
    node_sets = {tuple(sorted(result["nodes"])) for result in results}
    if len(node_sets) != 1:
        raise ValueError("three-run aggregate requires the same node set in every run")

    per_node: dict[str, Any] = {}
    for node_name in sorted(next(iter(node_sets))):
        slopes = {
            key: [result["nodes"][node_name]["memory"][key]["slope_kib_per_min"] for result in results]
            for key in ("rss_kib", "pss_kib")
        }
        above = {
            key: all(value > GROWTH_THRESHOLD_KIB_PER_MINUTE for value in values)
            for key, values in slopes.items()
        }
        floors = [
            result["nodes"][node_name]["memory"]["pss_kib"]["growth"][
                "detectable_slope_kib_per_min"
            ]
            for result in results
        ]
        per_node[node_name] = {
            "tier": results[0]["nodes"][node_name]["tier"],
            "slopes_kib_per_min": slopes,
            "all_runs_above_threshold": above,
            "suspected_sustained_growth": above["rss_kib"] or above["pss_kib"],
            "pss_detectable_slope_kib_per_min_by_run": floors,
            "pss_detectable_slope_kib_per_min": (
                max(floors) if all(floor is not None for floor in floors) else None
            ),
        }

    system_slopes = [
        result["system_total_pss_measured"]["slope_kib_per_min"] for result in results
    ]
    coarsest = [
        entry["pss_detectable_slope_kib_per_min"] for entry in per_node.values()
    ]
    return {
        "run_count": 3,
        "growth_threshold_kib_per_min": GROWTH_THRESHOLD_KIB_PER_MINUTE,
        "nodes": per_node,
        "system_total_pss_slopes_kib_per_min": system_slopes,
        "system_total_pss_suspected_sustained_growth": all(
            value > GROWTH_THRESHOLD_KIB_PER_MINUTE for value in system_slopes
        ),
        "any_node_suspected_sustained_growth": any(
            entry["suspected_sustained_growth"] for entry in per_node.values()
        ),
        "leak_sensitivity": {
            "pss_detectable_slope_kib_per_min": (
                max(coarsest) if all(floor is not None for floor in coarsest) else None
            ),
            "all_nodes_resolvable": all(floor is not None for floor in coarsest),
            "note": SENSITIVITY_NOTE,
        },
        "segment_names": sorted(
            {name for result in results for name in result["segment_names"]}
        ),
        "rss_sum_policy": RSS_SUM_POLICY,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dirs", nargs="+", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if len(args.run_dirs) not in {1, 3}:
        parser.error("provide one run directory or exactly three independent runs")

    try:
        run_dirs = (
            resolve_distinct_run_dirs(args.run_dirs)
            if len(args.run_dirs) == 3
            else args.run_dirs
        )
        results = [analyze_run(run_dir) for run_dir in run_dirs]
        payload = {"runs": results, "aggregate": aggregate_runs(results)}
    except (GraphIdentityError, KeyError, OSError, ValueError) as error:
        parser.error(str(error))

    rendered = json.dumps(payload, indent=2, sort_keys=True) + "\n"
    if args.output is not None:
        args.output.write_text(rendered, encoding="utf-8")
    print(rendered, end="")


if __name__ == "__main__":
    main()
