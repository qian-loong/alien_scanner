"""C2 local-map profiling evidence validation and attribution."""

from __future__ import annotations

import ast
import csv
import math
import statistics
from pathlib import Path
from typing import Any

from .profile_analysis import (
    parse_manifest,
    parse_pidstat,
    parse_pidstat_samples,
    percentile_nearest_rank,
    slope_kib_per_minute,
)
from .stage_latency_analysis import EVENT_SET_STAGES


EXPECTED_RATE_HZ = 10.0
MINIMUM_RATE_RATIO = 0.95
MAX_WINDOW_COUNT_SKEW = 2
MAX_CHECKPOINT_SKEW_NS = 2_000_000_000
BOUNDED_GROWTH_THRESHOLD_KIB_PER_MINUTE = 1024.0
CAPACITY_BUCKET_SIZE = 200
C3_MODES = {"disabled", "enabled", "keyframe-only"}
C3_ZERO_HASH = "0" * 64
C3_V2_IDENTITY = {
    "protocol_version": 2,
    "canonical_encoding_version": 1,
    "hash_algorithm": 1,
    "content_identity_scheme": 2,
    "content_identity_chunk_edge": 16,
    "content_identity_coordinate_key_version": 1,
    "content_identity_node_encoding_version": 1,
}

REQUIRED_STAGE_NAMES = EVENT_SET_STAGES["full"]


def _yaml_scalar(value: str) -> Any:
    value = value.strip()
    if value in {"", "null", "Null", "NULL", "~"}:
        return None
    if value.lower() in {"true", "false"}:
        return value.lower() == "true"
    try:
        return ast.literal_eval(value)
    except (SyntaxError, ValueError):
        return value.strip("\"'")


def parse_flat_yaml(path: Path) -> dict[str, Any]:
    """Parse generated key/value YAML into dotted keys without a PyYAML dependency."""

    result: dict[str, Any] = {}
    stack: list[tuple[int, str]] = []
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        if not raw_line.strip() or raw_line.lstrip().startswith("#"):
            continue
        indent = len(raw_line) - len(raw_line.lstrip(" "))
        stripped = raw_line.strip()
        if ":" not in stripped:
            raise ValueError(f"{path}: unsupported YAML line: {raw_line}")
        key, raw_value = stripped.split(":", 1)
        while stack and stack[-1][0] >= indent:
            stack.pop()
        prefix = ".".join(item[1] for item in stack)
        dotted_key = f"{prefix}.{key}" if prefix else key
        if raw_value.strip():
            result[dotted_key] = _yaml_scalar(raw_value)
        else:
            stack.append((indent, key))
    return result


def read_csv_rows(path: Path, required_columns: tuple[str, ...]) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        fieldnames = set(reader.fieldnames or ())
        missing = set(required_columns) - fieldnames
        if missing:
            raise ValueError(f"{path}: missing columns: {', '.join(sorted(missing))}")
        return list(reader)


def _integer(row: dict[str, str], key: str, source: Path) -> int:
    try:
        return int(row[key])
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError(f"{source}: invalid integer in {key}") from error


def _number(row: dict[str, str], key: str, source: Path) -> float:
    try:
        value = float(row[key])
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError(f"{source}: invalid number in {key}") from error
    if not math.isfinite(value):
        raise ValueError(f"{source}: non-finite number in {key}")
    return value


def _bounds(row: dict[str, str], source: Path) -> tuple[float, ...] | None:
    if "has_bounds" in row and _integer(row, "has_bounds", source) == 0:
        return None
    keys = ("min_x", "min_y", "min_z", "max_x", "max_y", "max_z")
    if all(not row.get(key, "") for key in keys):
        return None
    return tuple(_number(row, key, source) for key in keys)


def _same_bounds(left: tuple[float, ...] | None, right: tuple[float, ...] | None) -> bool:
    if left is None or right is None:
        return left is right
    return all(math.isclose(a, b, rel_tol=0.0, abs_tol=1e-6) for a, b in zip(left, right))


def _strictly_increasing(values: list[int], description: str) -> None:
    if any(right <= left for left, right in zip(values, values[1:])):
        raise ValueError(f"{description} are not strictly increasing")


def _require_contiguous(values: list[int], description: str) -> None:
    _strictly_increasing(values, description)
    if values and values[-1] - values[0] + 1 != len(values):
        raise ValueError(f"{description} contain a gap")


def _window_rows(
    rows: list[dict[str, str]], path: Path, t0_ns: int, t1_ns: int
) -> list[dict[str, str]]:
    return [
        row
        for row in rows
        if t0_ns <= _integer(row, "receipt_monotonic_ns", path) <= t1_ns
    ]


def _state_by_revision(
    rows: list[dict[str, str]], path: Path
) -> dict[int, dict[str, str]]:
    states: dict[int, dict[str, str]] = {}
    identity_keys = (
        "map_epoch",
        "stamp_ns",
        "fingerprint",
        "has_bounds",
        "min_x",
        "min_y",
        "min_z",
        "max_x",
        "max_y",
        "max_z",
    )
    for row in rows:
        revision = _integer(row, "revision", path)
        if revision <= 0:
            continue
        previous = states.get(revision)
        if previous is not None and any(previous[key] != row[key] for key in identity_keys):
            raise ValueError(f"{path}: conflicting heartbeat data for revision {revision}")
        states.setdefault(revision, row)
    return states


def _load_oracle(run_dir: Path) -> tuple[dict[str, Any], list[dict[str, str]]]:
    manifest_path = run_dir / "oracle_manifest.yaml"
    checkpoints_path = run_dir / "oracle_checkpoints.csv"
    manifest = parse_flat_yaml(manifest_path)
    checkpoints = read_csv_rows(
        checkpoints_path,
        (
            "sequence",
            "stamp_ns",
            "payload_digest",
            "map_epoch",
            "revision",
            "known",
            "free",
            "occupied",
            "fingerprint",
            "min_x",
            "min_y",
            "min_z",
            "max_x",
            "max_y",
            "max_z",
        ),
    )
    revisions = [_integer(row, "revision", checkpoints_path) for row in checkpoints]
    _strictly_increasing(revisions, f"{checkpoints_path} revisions")
    return manifest, checkpoints


def _validate_oracle_join(
    state_rows: dict[int, dict[str, str]],
    oracle_rows: list[dict[str, str]],
    states_path: Path,
    oracle_path: Path,
) -> list[dict[str, str]]:
    joined: list[dict[str, str]] = []
    for oracle in oracle_rows:
        revision = _integer(oracle, "revision", oracle_path)
        state = state_rows.get(revision)
        if state is None:
            continue
        for key in ("map_epoch", "revision", "stamp_ns", "fingerprint"):
            if state[key] != oracle[key]:
                raise ValueError(
                    f"revision {revision}: production/oracle {key} mismatch"
                )
        if not _same_bounds(_bounds(state, states_path), _bounds(oracle, oracle_path)):
            raise ValueError(f"revision {revision}: production/oracle bounds mismatch")
        joined.append(oracle)
    if not joined:
        raise ValueError("no formal-window production state joined an oracle checkpoint")
    return joined


def _memory_metrics(
    run_dir: Path,
    t0_ns: int,
    t1_ns: int,
    first_revision: int,
    last_revision: int,
    oracle_by_revision: dict[int, dict[str, str]],
) -> dict[str, Any] | None:
    samples_path = run_dir / "resource-samples.csv"
    if not samples_path.exists():
        return None
    samples = _window_rows(
        read_csv_rows(
            samples_path,
            (
                "receipt_monotonic_ns",
                "state_receipt_monotonic_ns",
                "map_epoch",
                "revision",
                "stamp_ns",
                "rss_kib",
                "pss_kib",
                "uss_kib",
            ),
        ),
        samples_path,
        t0_ns,
        t1_ns,
    )
    timestamps = [_integer(row, "receipt_monotonic_ns", samples_path) for row in samples]
    _strictly_increasing(timestamps, f"{samples_path} timestamps")
    if len(samples) < 2:
        raise ValueError(f"{samples_path}: fewer than two formal-window samples")

    result: dict[str, Any] = {"sample_count": len(samples)}
    for key in ("rss_kib", "pss_kib", "uss_kib"):
        points = [
            (
                _integer(row, "receipt_monotonic_ns", samples_path),
                _integer(row, key, samples_path),
            )
            for row in samples
        ]
        result[f"{key}_mean"] = statistics.fmean(value for _, value in points)
        result[f"{key}_peak"] = max(value for _, value in points)
        result[f"{key}_slope_per_minute"] = slope_kib_per_minute(points)

    checkpoints_path = run_dir / "memory-checkpoints.csv"
    if not checkpoints_path.exists():
        result["checkpoint_count"] = 0
        return result
    checkpoint_rows = read_csv_rows(
        checkpoints_path,
        (
            "receipt_monotonic_ns",
            "state_receipt_monotonic_ns",
            "map_epoch",
            "revision",
            "stamp_ns",
            "rss_kib",
            "pss_kib",
            "uss_kib",
        ),
    )
    checkpoints = [
        row
        for row in checkpoint_rows
        if t0_ns
        <= _integer(row, "receipt_monotonic_ns", checkpoints_path)
        <= t1_ns
        and t0_ns
        <= _integer(row, "state_receipt_monotonic_ns", checkpoints_path)
        <= t1_ns
    ]
    joined: list[tuple[dict[str, str], dict[str, str]]] = []
    for checkpoint in checkpoints:
        revision = _integer(checkpoint, "revision", checkpoints_path)
        oracle = oracle_by_revision.get(revision)
        if oracle is None:
            continue
        for key in ("map_epoch", "revision", "stamp_ns"):
            if checkpoint[key] != oracle[key]:
                raise ValueError(
                    f"memory checkpoint revision {revision}: oracle {key} mismatch"
                )
        skew_ns = _integer(checkpoint, "receipt_monotonic_ns", checkpoints_path) - _integer(
            checkpoint, "state_receipt_monotonic_ns", checkpoints_path
        )
        if skew_ns < 0 or skew_ns > MAX_CHECKPOINT_SKEW_NS:
            raise ValueError(
                f"memory checkpoint revision {revision}: state/resource skew is too large"
            )
        joined.append((checkpoint, oracle))
    expected_revisions = {
        revision
        for revision in oracle_by_revision
        if first_revision <= revision <= last_revision and revision % 100 == 0
    }
    joined_revisions = {
        _integer(checkpoint, "revision", checkpoints_path)
        for checkpoint, _oracle in joined
    }
    missing_revisions = sorted(expected_revisions - joined_revisions)
    if missing_revisions:
        missing = ", ".join(str(revision) for revision in missing_revisions)
        raise ValueError(
            f"{checkpoints_path}: missing formal-window memory checkpoints: {missing}"
        )
    result["checkpoint_count"] = len(joined)
    if len(joined) >= 2:
        first_memory, first_oracle = joined[0]
        last_memory, last_oracle = joined[-1]
        known_delta = _integer(last_oracle, "known", run_dir / "oracle_checkpoints.csv") - _integer(
            first_oracle, "known", run_dir / "oracle_checkpoints.csv"
        )
        revision_delta = _integer(last_oracle, "revision", run_dir / "oracle_checkpoints.csv") - _integer(
            first_oracle, "revision", run_dir / "oracle_checkpoints.csv"
        )
        result["known_delta"] = known_delta
        result["revision_delta"] = revision_delta
        for key in ("rss_kib", "pss_kib", "uss_kib"):
            delta_kib = _integer(last_memory, key, checkpoints_path) - _integer(
                first_memory, key, checkpoints_path
            )
            result[f"{key}_delta"] = delta_kib
            result[f"{key}_per_revision"] = (
                delta_kib / revision_delta if revision_delta else None
            )
            result[f"{key}_bytes_per_known"] = (
                delta_kib * 1024.0 / known_delta if known_delta > 0 else None
            )
    return result


def _cpu_metrics(
    run_dir: Path, tracee_pid: str, duration_s: float
) -> dict[str, Any] | None:
    """Summarize tracee-only pidstat CPU samples for the formal window."""

    pidstat_path = run_dir / "pidstat.txt"
    if not pidstat_path.exists():
        return None
    if duration_s >= 300.0:
        values, _rss_values = parse_pidstat(run_dir, tracee_pid, duration_s)
        leading_samples_excluded = 60
        sample_scope = "steady_after_60s"
    else:
        values, _rss_values = parse_pidstat_samples(pidstat_path, tracee_pid)
        if any(not math.isfinite(value) or value < 0 for value in values):
            raise ValueError(f"{run_dir}: pidstat contains an invalid CPU sample")
        expected_samples = max(1, math.floor(duration_s) - 2)
        if len(values) < expected_samples:
            raise ValueError(
                f"{run_dir}: fewer than {expected_samples} formal-window pidstat samples"
            )
        leading_samples_excluded = 0
        sample_scope = "full_short_window"

    return {
        "sample_count": len(values),
        "sample_scope": sample_scope,
        "leading_samples_excluded": leading_samples_excluded,
        "mean_percent": statistics.fmean(values),
        "p50_percent": percentile_nearest_rank(values, 0.50),
        "p95_percent": percentile_nearest_rank(values, 0.95),
        "max_percent": max(values),
    }


def _latency_metrics(
    run_dir: Path,
    first_revision: int,
    last_revision: int,
    tracee_pid: int,
    manifest_event_set: str,
) -> dict[str, Any]:
    quality = parse_manifest(run_dir / "stage-probe-quality.txt")
    if quality.get("event_schema_version") != "1":
        raise ValueError("stage probe event schema is missing or unsupported")
    if quality.get("provider") != "perception_local_map_stage":
        raise ValueError("stage probe provider does not match the profiling-only UST provider")
    if int(quality.get("target_pid", "-1")) != tracee_pid:
        raise ValueError("stage probe target PID does not match the tracee")
    if int(quality.get("loss_counter_count", "0")) <= 0:
        raise ValueError("stage probe lacks an explicit LTTng loss counter")
    quality_event_set = quality.get("event_set", "full")
    if quality_event_set not in EVENT_SET_STAGES:
        raise ValueError("stage probe event set is unsupported")
    if manifest_event_set != quality_event_set:
        raise ValueError("stage probe event set does not match the run manifest")
    for key in (
        "lost_events",
        "unmatched_entries",
        "unmatched_returns",
        "nesting_mismatches",
        "incomplete_callbacks",
        "duplicate_stage_samples",
        "invalid_durations",
        "unexpected_event_set_events",
    ):
        default = "0" if key == "unexpected_event_set_events" else "-1"
        if int(quality.get(key, default)) != 0:
            raise ValueError(f"stage probe has nonzero {key}")
    if quality.get("normal_completion") != "true" or quality.get("gate_pass") != "true":
        raise ValueError("stage probe did not complete normally and pass its raw evidence gate")

    path = run_dir / "stage-latency.csv"
    rows = read_csv_rows(
        path,
        (
            "stage",
            "duration_ns",
            "revision",
            "callback_id",
            "vtid",
            "begin_realtime_ns",
            "end_realtime_ns",
        ),
    )
    selected_rows = [
        row
        for row in rows
        if first_revision <= _integer(row, "revision", path) <= last_revision
    ]
    by_callback: dict[int, list[dict[str, str]]] = {}
    for row in selected_rows:
        callback_id = _integer(row, "callback_id", path)
        if callback_id <= 0:
            raise ValueError("stage probe contains a non-positive callback correlation ID")
        begin_ns = _integer(row, "begin_realtime_ns", path)
        end_ns = _integer(row, "end_realtime_ns", path)
        if end_ns <= begin_ns or end_ns - begin_ns != _integer(row, "duration_ns", path):
            raise ValueError("stage probe contains an invalid raw duration boundary")
        by_callback.setdefault(callback_id, []).append(row)

    expected_stages = EVENT_SET_STAGES[quality_event_set]
    for callback_rows in by_callback.values():
        stages = [row["stage"] for row in callback_rows]
        if sorted(stages) != sorted(expected_stages):
            raise ValueError("stage probe callback does not contain exactly one sample per stage")
        if len({_integer(row, "revision", path) for row in callback_rows}) != 1:
            raise ValueError("stage probe callback rows disagree on revision")
        if len({_integer(row, "vtid", path) for row in callback_rows}) != 1:
            raise ValueError("stage probe callback crossed thread boundaries")

    metrics: dict[str, Any] = {"event_set": quality_event_set}
    for stage in expected_stages:
        values = [
            _integer(row, "duration_ns", path)
            for row in selected_rows
            if row["stage"] == stage
        ]
        if not values or any(value <= 0 for value in values):
            raise ValueError(f"stage probe is missing positive {stage} samples")
        metrics[stage] = {
            "count": len(values),
            "mean_ns": statistics.fmean(values),
            "p50_ns": percentile_nearest_rank(values, 0.50),
            "p95_ns": percentile_nearest_rank(values, 0.95),
            "p99_ns": percentile_nearest_rank(values, 0.99),
            "max_ns": max(values),
        }
    callback_count = metrics["callback"]["count"]
    for stage in expected_stages[1:]:
        if metrics[stage]["count"] != callback_count:
            raise ValueError("stage probe sample counts do not match applied callbacks")
    if callback_count != int(quality.get("complete_applied_callbacks", "-1")):
        raise ValueError("stage probe CSV count disagrees with its raw pairing quality")
    if abs(callback_count - (last_revision - first_revision + 1)) > MAX_WINDOW_COUNT_SKEW:
        raise ValueError("stage probe callback count does not match applied revisions")
    return metrics


def _validate_tool_report(run_dir: Path, mode: str) -> None:
    quality_files = {
        "perf-stat": ("perf-stat-quality.txt", "gate_pass"),
        "perf-record": ("perf-quality.txt", "gate_pass"),
        "heaptrack": ("heaptrack-quality.txt", "gate_pass"),
        "valgrind-massif": ("massif-quality.txt", "gate_pass"),
        "valgrind-memcheck": ("memcheck-quality.txt", "target_verified"),
    }
    if mode in quality_files:
        filename, key = quality_files[mode]
        values = parse_manifest(run_dir / filename)
        if values.get(key) != "true":
            raise ValueError(f"{run_dir / filename}: profiler report gate failed")
    if mode == "ros-trace":
        counts = parse_manifest(run_dir / "trace-counts.txt")
        if any(int(counts.get(key, "0")) <= 0 for key in ("trace_callback", "trace_take", "trace_publish")):
            raise ValueError("ROS trace is missing target callback/take/publish evidence")


def _capacity_summary(
    oracle_manifest: dict[str, Any], first_revision: int, last_revision: int
) -> dict[str, Any]:
    status = oracle_manifest.get("capacity.status")
    if status not in {"covered", "not_reached", "crossing_without_post_window"}:
        raise ValueError(f"invalid oracle capacity status: {status}")
    crossing = oracle_manifest.get("capacity.crossing_revision")
    required_end = oracle_manifest.get("capacity.required_end_revision")
    result: dict[str, Any] = {
        "status": status,
        "crossing_revision": crossing,
        "required_end_revision": required_end,
        "segmented_evidence_available": False,
    }
    if status == "covered":
        if not isinstance(crossing, int) or not isinstance(required_end, int):
            raise ValueError("covered capacity plan lacks integer revisions")
        if crossing < 300 or required_end != crossing + 300:
            raise ValueError("covered capacity plan has invalid boundaries")
        buckets = {
            "pre": [crossing - 299, crossing - 100],
            "crossing": [crossing - 99, crossing + 100],
            "post": [crossing + 101, crossing + 300],
        }
        if any(last - first + 1 != CAPACITY_BUCKET_SIZE for first, last in buckets.values()):
            raise ValueError("capacity bucket is not exactly 200 revisions")
        if first_revision <= buckets["pre"][0] and last_revision >= required_end:
            result["segmented_evidence_available"] = True
            result["buckets"] = buckets
    return result


def _validate_c3_evidence(
    run_dir: Path,
    run_manifest: dict[str, str],
    states_path: Path,
) -> dict[str, Any]:
    """Validate the C2-to-C3 publication chain and its convergence evidence."""
    mode = run_manifest.get("c3_mode", "")
    if mode not in C3_MODES:
        raise ValueError(f"{run_dir}: invalid or missing c3_mode in run manifest")

    sink_manifest_path = run_dir / "sink_manifest.yaml"
    sink_manifest = parse_flat_yaml(sink_manifest_path)
    if sink_manifest.get("schema") != "alien-scanner/perception-profile-sink/v2":
        raise ValueError(f"{sink_manifest_path}: sink manifest schema is not v2")
    if sink_manifest.get("c3_mode") != mode:
        raise ValueError(f"{run_dir}: sink and run manifests disagree on c3_mode")
    if sink_manifest.get("summary.normal_completion") is not True:
        raise ValueError(f"{sink_manifest_path}: sink did not finalize normally")

    parameters_path = run_dir / "c3-runtime-parameters.txt"
    parameters = parse_manifest(parameters_path)
    if parameters.get("schema") != "alien-scanner/perception-c3-runtime-parameters/v1":
        raise ValueError(f"{parameters_path}: C3 parameter schema is missing")
    if parameters.get("c3_mode") != mode or parameters.get("sink.c3_mode") != mode:
        raise ValueError(f"{run_dir}: runtime parameters disagree on c3_mode")
    expected_enabled = "false" if mode == "disabled" else "true"
    expected_delta = "false" if mode == "keyframe-only" else "true"
    if parameters.get("map_update_enabled") != expected_enabled:
        raise ValueError(f"{parameters_path}: map_update_enabled does not match c3_mode")
    if parameters.get("map_update.delta_enabled") != expected_delta:
        raise ValueError(f"{parameters_path}: delta_enabled does not match c3_mode")
    if parameters.get("map_update_topic") != "/profile/local_map/updates":
        raise ValueError(f"{parameters_path}: map-update topic is not fixed")
    if parameters.get("sink.map_update_topic") != parameters.get("map_update_topic"):
        raise ValueError(f"{parameters_path}: sink and target map-update topics disagree")

    updates_path = run_dir / "map_updates.csv"
    updates = read_csv_rows(
        updates_path,
        (
            "receipt_monotonic_ns",
            "protocol_version",
            "canonical_encoding_version",
            "hash_algorithm",
            "content_identity_scheme",
            "content_identity_chunk_edge",
            "content_identity_coordinate_key_version",
            "content_identity_node_encoding_version",
            "update_kind",
            "vehicle_id",
            "mapper_session_boot_ns",
            "mapper_session_suffix",
            "map_epoch",
            "base_revision",
            "new_revision",
            "revision_span",
            "observed_coalesced_receipt_count",
            "known_cell_count",
            "operation_count",
            "canonical_payload_bytes",
            "base_content_hash",
            "content_hash",
            "update_hash",
        ),
    )
    producer_path = run_dir / "map_update_producer_diagnostics.csv"
    producer_rows = read_csv_rows(
        producer_path,
        (
            "receipt_monotonic_ns",
            "pending",
            "in_flight",
            "pending_revision",
            "in_flight_revision",
            "published_revision",
            "published_keyframes",
            "published_deltas",
            "revision_only_deltas",
            "publish_failures",
            "resource_rejections",
            "snapshot_cells",
            "delta_operations",
            "payload_bytes",
            "acquire_duration_ns",
            "materialize_duration_ns",
            "traversal_duration_ns",
            "canonicalize_duration_ns",
            "geometry_fingerprint_duration_ns",
            "prepare_duration_ns",
            "validation_duration_ns",
            "diff_duration_ns",
            "encode_duration_ns",
            "store_candidate_duration_ns",
            "merkle_duration_ns",
            "update_hash_duration_ns",
            "publish_duration_ns",
        ),
    )
    summary_keys = (
        "map_update_count",
        "map_update_keyframe_count",
        "map_update_delta_count",
        "map_update_revision_only_delta_count",
        "producer_diagnostic_count",
    )
    summary = {
        key: int(sink_manifest.get(f"summary.{key}", "-1"))
        for key in summary_keys
    }
    if any(value < 0 for value in summary.values()):
        raise ValueError(f"{sink_manifest_path}: C3 summary counters are incomplete")
    if summary["map_update_count"] != len(updates):
        raise ValueError(f"{run_dir}: sink map_update_count disagrees with map_updates.csv")
    if summary["producer_diagnostic_count"] != len(producer_rows):
        raise ValueError(
            f"{run_dir}: sink producer_diagnostic_count disagrees with diagnostics CSV"
        )

    drain_path = run_dir / "drain-manifest.txt"
    drain = parse_manifest(drain_path)
    if drain.get("drain_applicable") != ("false" if mode == "disabled" else "true"):
        raise ValueError(f"{drain_path}: drain applicability does not match c3_mode")

    if mode == "disabled":
        if updates or producer_rows:
            raise ValueError(f"{run_dir}: disabled C3 mode contains map-update evidence")
        if any(summary[key] != 0 for key in summary_keys):
            raise ValueError(f"{run_dir}: disabled C3 summary counters are nonzero")
        if drain.get("drain_converged") != "not_applicable":
            raise ValueError(f"{drain_path}: disabled drain must be not_applicable")
        return {
            "mode": mode,
            "map_update_count": 0,
            "published_keyframes": 0,
            "published_deltas": 0,
            "revision_only_deltas": 0,
            "producer_diagnostic_count": 0,
            "timing_sample_count": 0,
            "drain_converged": "not_applicable",
        }

    if not updates or not producer_rows:
        raise ValueError(f"{run_dir}: enabled C3 mode has no update or producer evidence")
    update_receipts = [_integer(row, "receipt_monotonic_ns", updates_path) for row in updates]
    _strictly_increasing(update_receipts, f"{updates_path} receipt timestamps")
    identities = {
        (
            row["vehicle_id"],
            _integer(row, "mapper_session_boot_ns", updates_path),
            _integer(row, "mapper_session_suffix", updates_path),
            _integer(row, "map_epoch", updates_path),
        )
        for row in updates
    }
    if len(identities) != 1:
        raise ValueError(f"{updates_path}: source identity changed within one profiling run")

    previous: dict[str, str] | None = None
    keyframes = deltas = revision_only = 0
    for row in updates:
        for field, expected in C3_V2_IDENTITY.items():
            if _integer(row, field, updates_path) != expected:
                raise ValueError(
                    f"{updates_path}: update does not use the production v2 identity"
                )
        kind = _integer(row, "update_kind", updates_path)
        base_revision = _integer(row, "base_revision", updates_path)
        new_revision = _integer(row, "new_revision", updates_path)
        if new_revision <= base_revision:
            raise ValueError(f"{updates_path}: update revisions do not advance")
        if previous is not None and new_revision <= int(previous["new_revision"]):
            raise ValueError(f"{updates_path}: update revisions do not advance")
        if _integer(row, "revision_span", updates_path) != new_revision - base_revision:
            raise ValueError(f"{updates_path}: revision_span is inconsistent")
        if any(
            len(row[key]) != 64 or any(character not in "0123456789abcdef" for character in row[key])
            for key in ("base_content_hash", "content_hash", "update_hash")
        ):
            raise ValueError(f"{updates_path}: update contains an invalid hash")
        if kind == 1:
            keyframes += 1
            if base_revision != 0 or row["base_content_hash"] != C3_ZERO_HASH:
                raise ValueError(f"{updates_path}: keyframe carries a non-empty base")
        elif kind == 2:
            deltas += 1
            if previous is None:
                raise ValueError(f"{updates_path}: first update cannot be a delta")
            if base_revision != int(previous["new_revision"]):
                raise ValueError(f"{updates_path}: delta base revision is not chained")
            if row["base_content_hash"] != previous["content_hash"]:
                raise ValueError(f"{updates_path}: delta base content hash is not chained")
            if _integer(row, "operation_count", updates_path) == 0:
                revision_only += 1
        else:
            raise ValueError(f"{updates_path}: profiling stream contains non keyframe/delta kind")
        previous = row
    if mode == "enabled" and (keyframes < 1 or deltas < 1):
        raise ValueError(f"{run_dir}: enabled C3 mode lacks both keyframe and delta evidence")
    if mode == "keyframe-only" and deltas:
        raise ValueError(f"{run_dir}: keyframe-only mode contains delta evidence")
    if summary["map_update_keyframe_count"] != keyframes:
        raise ValueError(f"{run_dir}: sink keyframe count disagrees with map_updates.csv")
    if summary["map_update_delta_count"] != deltas:
        raise ValueError(f"{run_dir}: sink delta count disagrees with map_updates.csv")
    if summary["map_update_revision_only_delta_count"] != revision_only:
        raise ValueError(
            f"{run_dir}: sink revision-only delta count disagrees with map_updates.csv"
        )

    diagnostic_receipts = [_integer(row, "receipt_monotonic_ns", producer_path) for row in producer_rows]
    _strictly_increasing(diagnostic_receipts, f"{producer_path} receipt timestamps")
    previous_published = -1
    by_revision: dict[int, dict[str, str]] = {}
    for row in producer_rows:
        pending = _integer(row, "pending", producer_path)
        in_flight = _integer(row, "in_flight", producer_path)
        if pending not in {0, 1} or in_flight not in {0, 1}:
            raise ValueError(f"{producer_path}: pending/in_flight is not boolean")
        published = _integer(row, "published_revision", producer_path)
        if published < previous_published:
            raise ValueError(f"{producer_path}: published revision regressed")
        previous_published = published
        for key in (
            "pending_revision", "in_flight_revision", "published_keyframes",
            "published_deltas", "revision_only_deltas", "publish_failures",
            "resource_rejections", "snapshot_cells", "delta_operations", "payload_bytes",
        ):
            if _integer(row, key, producer_path) < 0:
                raise ValueError(f"{producer_path}: negative producer counter")
        if published > 0:
            by_revision[published] = row
    if not by_revision:
        raise ValueError(f"{producer_path}: no published revision timing evidence")
    final_diag = producer_rows[-1]
    published_keyframes = _integer(final_diag, "published_keyframes", producer_path)
    published_deltas = _integer(final_diag, "published_deltas", producer_path)
    final_published = _integer(final_diag, "published_revision", producer_path)
    if len(updates) != published_keyframes + published_deltas:
        raise ValueError(f"{run_dir}: published update counter is not conserved")
    if published_keyframes != keyframes or published_deltas != deltas:
        raise ValueError(f"{run_dir}: producer and map update kind counters disagree")
    if _integer(final_diag, "revision_only_deltas", producer_path) != revision_only:
        raise ValueError(f"{run_dir}: revision-only delta counter is not conserved")
    if _integer(final_diag, "publish_failures", producer_path) != 0:
        raise ValueError(f"{run_dir}: producer reported a publish failure")
    if _integer(final_diag, "resource_rejections", producer_path) != 0:
        raise ValueError(f"{run_dir}: producer reported a resource rejection")

    timing_fields = (
        "acquire_duration_ns",
        "materialize_duration_ns",
        "traversal_duration_ns",
        "canonicalize_duration_ns",
        "geometry_fingerprint_duration_ns",
        "prepare_duration_ns",
        "validation_duration_ns",
        "diff_duration_ns",
        "encode_duration_ns",
        "store_candidate_duration_ns",
        "merkle_duration_ns",
        "update_hash_duration_ns",
        "publish_duration_ns",
    )
    timing: dict[str, dict[str, float | int]] = {}
    unique_rows = [by_revision[revision] for revision in sorted(by_revision)]
    for key in timing_fields:
        values = [_integer(row, key, producer_path) for row in unique_rows]
        if any(value < 0 for value in values):
            raise ValueError(f"{producer_path}: negative producer duration")
        timing[key] = {
            "count": len(values),
            "mean_ns": statistics.fmean(values),
            "p50_ns": percentile_nearest_rank(values, 0.50),
            "p95_ns": percentile_nearest_rank(values, 0.95),
            "p99_ns": percentile_nearest_rank(values, 0.99),
            "max_ns": max(values),
        }

    states = read_csv_rows(
        states_path,
        ("revision", "receipt_monotonic_ns"),
    )
    latest_state_revision = max(_integer(row, "revision", states_path) for row in states)
    latest_update_revision = _integer(updates[-1], "new_revision", updates_path)
    if final_published != latest_state_revision or latest_update_revision != latest_state_revision:
        raise ValueError(f"{run_dir}: final C3 revision did not converge to latest state")
    if drain.get("drain_converged") != "true" or int(drain.get("drain_stable_samples", "0")) < 2:
        raise ValueError(f"{drain_path}: C3 drain did not provide two stable samples")
    for key in ("drain_latest_revision", "drain_published_revision", "drain_update_revision"):
        if int(drain.get(key, "-1")) != latest_state_revision:
            raise ValueError(f"{drain_path}: {key} does not match latest revision")
    return {
        "mode": mode,
        "map_update_count": len(updates),
        "published_keyframes": keyframes,
        "published_deltas": deltas,
        "revision_only_deltas": revision_only,
        "producer_diagnostic_count": len(producer_rows),
        "timing_sample_count": len(by_revision),
        "timing": timing,
        "drain_converged": "true",
    }


def analyze_run(run_dir: Path) -> dict[str, Any]:
    run_dir = run_dir.resolve()
    manifest_path = run_dir / "run-manifest.txt"
    manifest = parse_manifest(manifest_path)
    if manifest.get("valid") != "true" or manifest.get("normal_completion") != "true":
        raise ValueError(f"{run_dir}: run is not valid and normally completed")
    mode = manifest.get("mode", "")
    workload = manifest.get("workload", "")
    if workload not in {"bounded", "expanding"}:
        raise ValueError(f"{run_dir}: invalid workload {workload!r}")
    t0_ns = int(manifest["t0_monotonic_ns"])
    t1_ns = int(manifest["t1_monotonic_ns"])
    if t1_ns <= t0_ns:
        raise ValueError(f"{run_dir}: invalid formal window")
    actual_duration_s = (t1_ns - t0_ns) / 1_000_000_000
    requested_duration_s = float(manifest["duration_requested_s"])
    if mode != "capacity-ramp" and actual_duration_s + 0.25 < requested_duration_s:
        raise ValueError(f"{run_dir}: formal window is shorter than requested")
    oracle_manifest, oracle_rows = _load_oracle(run_dir)
    if oracle_manifest.get("mode") != workload:
        raise ValueError(f"{run_dir}: oracle mode does not match workload")
    for key in ("rejected", "backend_fault", "no_evidence"):
        if int(oracle_manifest.get(key, -1)) != 0:
            raise ValueError(f"{run_dir}: oracle reports nonzero {key}")

    observations_path = run_dir / "observations.csv"
    observations = _window_rows(
        read_csv_rows(
            observations_path,
            (
                "receipt_monotonic_ns",
                "sequence",
                "stamp_ns",
                "payload_digest",
                "expected_digest",
                "schema_valid",
                "digest_matches",
            ),
        ),
        observations_path,
        t0_ns,
        t1_ns,
    )
    sequences = [_integer(row, "sequence", observations_path) for row in observations]
    _require_contiguous(sequences, f"{observations_path} formal-window sequences")
    if any(
        _integer(row, "schema_valid", observations_path) != 1
        or _integer(row, "digest_matches", observations_path) != 1
        for row in observations
    ):
        raise ValueError(f"{observations_path}: schema or digest mismatch")
    required_count = max(
        1, math.floor(actual_duration_s * EXPECTED_RATE_HZ * MINIMUM_RATE_RATIO)
    )
    if len(observations) < required_count:
        raise ValueError(
            f"{run_dir}: observation workload {len(observations)} is below {required_count}"
        )

    states_path = run_dir / "states.csv"
    formal_states = _window_rows(
        read_csv_rows(
            states_path,
            (
                "receipt_monotonic_ns",
                "state_sequence",
                "map_epoch",
                "revision",
                "stamp_ns",
                "fingerprint",
                "changed_cell_count",
                "has_bounds",
                "min_x",
                "min_y",
                "min_z",
                "max_x",
                "max_y",
                "max_z",
            ),
        ),
        states_path,
        t0_ns,
        t1_ns,
    )
    state_rows = _state_by_revision(formal_states, states_path)
    revisions = sorted(state_rows)
    _require_contiguous(revisions, f"{states_path} formal-window revisions")
    if len(revisions) < required_count:
        raise ValueError(f"{run_dir}: revision workload is below {required_count}")
    if abs(len(revisions) - len(observations)) > MAX_WINDOW_COUNT_SKEW:
        raise ValueError(f"{run_dir}: observation/revision window count skew is too large")
    epochs = {_integer(row, "map_epoch", states_path) for row in state_rows.values()}
    fingerprints = {row["fingerprint"] for row in state_rows.values()}
    if len(epochs) != 1:
        raise ValueError(f"{run_dir}: map epoch changed during the formal window")
    if len(fingerprints) != 1 or not next(iter(fingerprints)):
        raise ValueError(f"{run_dir}: mapper fingerprint changed or is empty")

    health_path = run_dir / "health.csv"
    health_rows = _window_rows(
        read_csv_rows(
            health_path,
            (
                "receipt_monotonic_ns",
                "stamp_ns",
                "state",
                "producer_source_id",
                "session_boot_ns",
                "session_suffix",
                "fingerprint",
                "full_no_return",
                "active_sensor_count",
            ),
        ),
        health_path,
        t0_ns,
        t1_ns,
    )
    if not health_rows:
        raise ValueError(f"{run_dir}: formal window has no health evidence")
    health_sessions = {
        (
            row["producer_source_id"],
            _integer(row, "session_boot_ns", health_path),
            _integer(row, "session_suffix", health_path),
        )
        for row in health_rows
    }
    if len(health_sessions) != 1 or next(iter(health_sessions))[0] != "perception_profile_fixture":
        raise ValueError(f"{run_dir}: health producer session changed or is unexpected")
    if any(
        _integer(row, "state", health_path) not in {0, 1}
        or row["fingerprint"] != next(iter(fingerprints))
        or _integer(row, "full_no_return", health_path) != 1
        or _integer(row, "active_sensor_count", health_path) != 1
        for row in health_rows
    ):
        raise ValueError(f"{run_dir}: health/capability/fingerprint gate failed")

    joined_oracle = _validate_oracle_join(
        state_rows, oracle_rows, states_path, run_dir / "oracle_checkpoints.csv"
    )
    joined_counts = [
        (
            _integer(row, "known", run_dir / "oracle_checkpoints.csv"),
            _integer(row, "free", run_dir / "oracle_checkpoints.csv"),
            _integer(row, "occupied", run_dir / "oracle_checkpoints.csv"),
        )
        for row in joined_oracle
    ]
    joined_bounds = [
        _bounds(row, run_dir / "oracle_checkpoints.csv") for row in joined_oracle
    ]
    if workload == "bounded":
        plateau_start = oracle_manifest.get("plateau_start_revision")
        if not isinstance(plateau_start, int):
            raise ValueError(f"{run_dir}: bounded oracle did not prove a plateau")
        if revisions[0] < plateau_start + 600:
            raise ValueError(f"{run_dir}: formal window began before plateau proof completed")
        if len(set(joined_counts)) != 1 or any(
            not _same_bounds(joined_bounds[0], bounds) for bounds in joined_bounds[1:]
        ):
            raise ValueError(f"{run_dir}: bounded map changed after plateau")
    else:
        known = [counts[0] for counts in joined_counts]
        if len(known) < 2 or known[-1] <= known[0]:
            raise ValueError(f"{run_dir}: expanding map did not increase exact known voxels")
        if any(right < left for left, right in zip(known, known[1:])):
            raise ValueError(f"{run_dir}: expanding exact known voxels regressed")
        if joined_bounds[0] is None or joined_bounds[-1] is None or joined_bounds[-1][3] <= joined_bounds[0][3]:
            raise ValueError(f"{run_dir}: expanding X bounds did not grow")

    diagnostics_path = run_dir / "diagnostics.csv"
    diagnostics = _window_rows(
        read_csv_rows(
            diagnostics_path,
            ("receipt_monotonic_ns", "stamp_ns", "level", "name", "message"),
        ),
        diagnostics_path,
        t0_ns,
        t1_ns,
    )
    if any(_integer(row, "level", diagnostics_path) >= 1 for row in diagnostics):
        raise ValueError(f"{run_dir}: warning/error diagnostic occurred in the formal window")

    snapshots_path = run_dir / "snapshots.csv"
    snapshots = _window_rows(
        read_csv_rows(
            snapshots_path,
            (
                "receipt_monotonic_ns",
                "stamp_ns",
                "ordinal",
                "binary",
                "resolution_m",
                "data_bytes",
            ),
        ),
        snapshots_path,
        t0_ns,
        t1_ns,
    )
    if any(
        _integer(row, "binary", snapshots_path) != 1
        or _integer(row, "data_bytes", snapshots_path) <= 0
        for row in snapshots
    ):
        raise ValueError(f"{run_dir}: invalid OctoMap snapshot evidence")
    if len(snapshots) + MAX_WINDOW_COUNT_SKEW < len(revisions):
        raise ValueError(f"{run_dir}: OctoMap snapshot count is below applied revisions")

    c3 = _validate_c3_evidence(run_dir, manifest, states_path)

    oracle_by_revision = {
        _integer(row, "revision", run_dir / "oracle_checkpoints.csv"): row
        for row in oracle_rows
    }
    memory = _memory_metrics(
        run_dir,
        t0_ns,
        t1_ns,
        revisions[0],
        revisions[-1],
        oracle_by_revision,
    )
    capacity = _capacity_summary(oracle_manifest, revisions[0], revisions[-1])
    if mode == "capacity-ramp" and capacity["status"] == "covered" and not capacity["segmented_evidence_available"]:
        raise ValueError(f"{run_dir}: covered capacity ramp lacks the complete post window")

    _validate_tool_report(run_dir, mode)
    latency = (
        _latency_metrics(
            run_dir,
            revisions[0],
            revisions[-1],
            int(manifest["tracee_pid"]),
            manifest.get("stage_event_set", "full"),
        )
        if mode == "stage-latency"
        else None
    )
    cpu = (
        None
        if mode == "stage-latency" or "workspace_closure_install_base" in manifest
        else _cpu_metrics(run_dir, manifest["tracee_pid"], actual_duration_s)
    )
    return {
        "run_dir": str(run_dir),
        "mode": mode,
        "workload": workload,
        "tracee_pid": int(manifest["tracee_pid"]),
        "t0_monotonic_ns": t0_ns,
        "t1_monotonic_ns": t1_ns,
        "duration_actual_s": actual_duration_s,
        "observation_count": len(observations),
        "revision_count": len(revisions),
        "first_revision": revisions[0],
        "last_revision": revisions[-1],
        "map_epoch": next(iter(epochs)),
        "fingerprint": next(iter(fingerprints)),
        "oracle_checkpoint_count": len(joined_oracle),
        "final_known": joined_counts[-1][0],
        "final_free": joined_counts[-1][1],
        "final_occupied": joined_counts[-1][2],
        "snapshot_count": len(snapshots),
        "c3_mode": c3["mode"],
        "c3": c3,
        "cpu": cpu,
        "memory": memory,
        "capacity": capacity,
        "latency": latency,
    }


def aggregate_runs(results: list[dict[str, Any]]) -> dict[str, Any] | None:
    if len(results) != 3:
        return None
    if any(result["mode"] != "plain-sample" for result in results):
        raise ValueError("three-run aggregate accepts only plain-sample evidence")
    if len({result["workload"] for result in results}) != 1:
        raise ValueError("three-run aggregate requires one workload type")
    if len({result["c3_mode"] for result in results}) != 1:
        raise ValueError("three-run aggregate requires one C3 mode")
    identities = {
        (result["tracee_pid"], result["t0_monotonic_ns"], result["t1_monotonic_ns"])
        for result in results
    }
    if len(identities) != 3:
        raise ValueError("three-run aggregate contains duplicate evidence identities")
    if any(result["memory"] is None for result in results):
        raise ValueError("three-run aggregate requires resource samples")
    if any(result.get("cpu") is None for result in results):
        raise ValueError("three-run aggregate requires pidstat CPU samples")

    workload = results[0]["workload"]
    aggregate: dict[str, Any] = {
        "run_count": 3,
        "workload": workload,
        "c3_mode": results[0]["c3_mode"],
        "cpu_percent": {
            key: [result["cpu"][key] for result in results]
            for key in ("mean_percent", "p95_percent", "max_percent")
        },
    }
    if workload == "bounded":
        slope_keys = (
            "rss_kib_slope_per_minute",
            "pss_kib_slope_per_minute",
            "uss_kib_slope_per_minute",
        )
        slopes = {
            key: [result["memory"][key] for result in results] for key in slope_keys
        }
        aggregate["slopes_kib_per_minute"] = slopes
        aggregate["suspected_sustained_growth"] = all(
            value > BOUNDED_GROWTH_THRESHOLD_KIB_PER_MINUTE
            for values in slopes.values()
            for value in values
        )
    else:
        aggregate["bytes_per_known"] = {
            memory_name: [
                result["memory"].get(f"{memory_name}_kib_bytes_per_known")
                for result in results
            ]
            for memory_name in ("rss", "pss", "uss")
        }
    return aggregate
