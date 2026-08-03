"""Validate unprobed/callback/full C2 stage-latency calibration evidence."""

from __future__ import annotations

import math
import re
import statistics
from pathlib import Path
from typing import Any

from .local_map_build_provenance import (
    load_and_validate_workspace_closure,
    load_source_identity,
    sha256_file,
)
from .local_map_profile_analysis import analyze_run
from .profile_analysis import parse_manifest


MINIMUM_DURATION_S = 120.0
MINIMUM_STAGE_CALLBACKS = 1200
PIDSTAT_BRACKET_MARGIN_NS = 2_000_000_000
CPU_RELATIVE_LIMIT = 0.05
CPU_ABSOLUTE_FLOOR_PERCENTAGE_POINTS = 0.5
LATENCY_RELATIVE_LIMIT = 0.10
LATENCY_ABSOLUTE_FLOOR_NS = 50_000

COMMON_PROVENANCE_FIELDS = (
    "source_diff_sha256",
    "source_untracked_sha256",
    "source_untracked_archive_sha256",
    "profile_script_sha256",
    "profile_runner_common_sha256",
    "profile_role_monitor_sha256",
    "profile_sampler_sha256",
    "stage_latency_analysis_sha256",
    "profile_build_provenance_sha256",
    "analysis_script_sha256",
    "workload_yaml_sha256",
    "rmw",
    "cpu_target",
    "cpu_helpers",
    "cpu_allowed",
    "target_affinity",
)

ROLE_NAMES = (
    "launcher",
    "tracee",
    "sink",
    "fixture",
    "pidstat",
    "checkpoint_sampler",
)

TARGET_BUILD_FIELDS = (
    "install_prefix",
    "target_sha256",
    "target_build_id",
    "target_build_type",
    "target_compile_flags_verified",
    "target_stage_latency_option",
)

CLOSURE_MANIFEST_FIELDS = (
    "paired_source_identity_sha256",
    "workspace_closure_install_base",
    "workspace_closure_build_base",
    "workspace_closure_manifest_sha256",
    "workspace_dependency_comparison_sha256",
    "profiling_prefix",
    "profiling_helper_set_sha256",
)

SOURCE_ARTIFACT_FIELDS = {
    "source_diff_sha256": "source-diff.patch",
    "source_untracked_sha256": "source-untracked.txt",
    "source_untracked_archive_sha256": "source-untracked.tar.gz",
}


def _require_manifest_fields(
    manifest: dict[str, str], fields: tuple[str, ...], run_dir: Path
) -> None:
    missing = [field for field in fields if not manifest.get(field)]
    if missing:
        raise ValueError(
            f"{run_dir}: manifest lacks required calibration provenance: "
            + ", ".join(missing)
        )


def _positive_integer(manifest: dict[str, str], key: str, run_dir: Path) -> int:
    try:
        value = int(manifest[key])
    except (KeyError, ValueError) as error:
        raise ValueError(f"{run_dir}: invalid manifest integer {key}") from error
    if value <= 0:
        raise ValueError(f"{run_dir}: non-positive manifest integer {key}")
    return value


def _validate_role_provenance(
    run_dir: Path, manifest: dict[str, str], analyzed: dict[str, Any]
) -> None:
    identities: dict[str, tuple[int, int, int]] = {}
    for role in ROLE_NAMES:
        identities[role] = tuple(
            _positive_integer(manifest, f"{role}_{suffix}", run_dir)
            for suffix in ("pid", "starttime", "pgid")
        )
    if _positive_integer(manifest, "tracee_pid", run_dir) != analyzed["tracee_pid"]:
        raise ValueError(f"{run_dir}: tracee role does not match analyzed target PID")
    if identities["launcher"] != identities["tracee"]:
        raise ValueError(f"{run_dir}: direct launcher and tracee identities differ")
    if any(pid != pgid for pid, _, pgid in identities.values()):
        raise ValueError(f"{run_dir}: calibration role lacks an isolated process group")
    helper_pids = [identities[role][0] for role in ROLE_NAMES if role != "tracee"]
    if len(helper_pids) != len(set(helper_pids)):
        raise ValueError(f"{run_dir}: calibration roles do not have distinct PIDs")


def _validate_target_builds(
    manifests: list[dict[str, str]], run_dirs: list[Path]
) -> None:
    for manifest, run_dir in zip(manifests, run_dirs):
        _require_manifest_fields(manifest, TARGET_BUILD_FIELDS, run_dir)
        if manifest["target_build_type"] != "RelWithDebInfo":
            raise ValueError(f"{run_dir}: target is not a RelWithDebInfo build")
        if manifest["target_compile_flags_verified"] != "true":
            raise ValueError(f"{run_dir}: target compile flags are not verified")
        if not re.fullmatch(r"[0-9a-f]{64}", manifest["target_sha256"]):
            raise ValueError(f"{run_dir}: target SHA-256 is invalid")
        if not re.fullmatch(r"[0-9a-f]+", manifest["target_build_id"]):
            raise ValueError(f"{run_dir}: target build ID is invalid")

    unprobed, callback, full = manifests
    if unprobed["target_stage_latency_option"] != "OFF":
        raise ValueError("unprobed target must record the stage latency option as OFF")
    if callback["target_stage_latency_option"] != "ON" or full[
        "target_stage_latency_option"
    ] != "ON":
        raise ValueError("callback/full targets must record the stage latency option as ON")
    if unprobed["target_sha256"] == callback["target_sha256"]:
        raise ValueError("unprobed and latency targets must be distinct ELFs")
    for key in ("install_prefix", "target_sha256", "target_build_id"):
        if callback[key] != full[key]:
            raise ValueError(f"callback/full target provenance differs in {key}")
    if unprobed["install_prefix"] == callback["install_prefix"]:
        raise ValueError("unprobed and latency runs must use distinct install prefixes")


def _validate_shared_provenance(
    manifests: list[dict[str, str]], run_dirs: list[Path]
) -> None:
    for manifest, run_dir in zip(manifests, run_dirs):
        _require_manifest_fields(
            manifest, ("source_revision", *COMMON_PROVENANCE_FIELDS), run_dir
        )
    for field in COMMON_PROVENANCE_FIELDS:
        if len({manifest[field] for manifest in manifests}) != 1:
            raise ValueError(f"calibration runs differ in {field}")

    revisions = {manifest["source_revision"] for manifest in manifests}
    if len(revisions) != 1:
        for manifest, run_dir in zip(manifests, run_dirs):
            _require_manifest_fields(manifest, ("source_tree_sha256",), run_dir)
        if len({manifest["source_tree_sha256"] for manifest in manifests}) != 1:
            raise ValueError(
                "calibration source revisions differ without an equivalent source tree hash"
            )


def _validate_workspace_closures(
    manifests: list[dict[str, str]], run_dirs: list[Path]
) -> list[dict[str, Any]]:
    payloads: list[dict[str, Any]] = []
    for manifest, run_dir in zip(manifests, run_dirs):
        _require_manifest_fields(manifest, CLOSURE_MANIFEST_FIELDS, run_dir)
        for field in (
            "paired_source_identity_sha256",
            "workspace_closure_manifest_sha256",
            "workspace_dependency_comparison_sha256",
            "profiling_helper_set_sha256",
        ):
            if re.fullmatch(r"[0-9a-f]{64}", manifest[field]) is None:
                raise ValueError(f"{run_dir}: manifest field {field} is not a SHA-256")
        closure_path = run_dir / "target-workspace-closure.json"
        if sha256_file(closure_path) != manifest["workspace_closure_manifest_sha256"]:
            raise ValueError(f"{run_dir}: closure manifest SHA-256 mismatch")
        payload = load_and_validate_workspace_closure(closure_path)
        source_identity = load_source_identity(
            run_dir / "paired-source-identity.txt"
        )
        for field, filename in SOURCE_ARTIFACT_FIELDS.items():
            if sha256_file(run_dir / filename) != source_identity[field]:
                raise ValueError(
                    f"{run_dir}: source artifact {filename} does not match paired identity"
                )
        expected_values = {
            "source_revision": source_identity["source_revision"],
            "source_diff_sha256": source_identity["source_diff_sha256"],
            "source_untracked_sha256": source_identity["source_untracked_sha256"],
            "source_untracked_archive_sha256": source_identity[
                "source_untracked_archive_sha256"
            ],
            "paired_source_identity_sha256": payload[
                "paired_source_identity_sha256"
            ],
            "workspace_closure_install_base": payload["closure_install_base"],
            "workspace_closure_build_base": payload["closure_build_base"],
            "workspace_dependency_comparison_sha256": payload[
                "dependency_comparison_sha256"
            ],
            "profiling_prefix": str(
                Path(payload["closure_install_base"]) / "perception_profiling"
            ),
            "profiling_helper_set_sha256": payload["helper_set_sha256"],
            "workload_yaml_sha256": payload["workload"]["sha256"],
        }
        for field, expected in expected_values.items():
            if manifest[field] != expected:
                raise ValueError(f"{run_dir}: closure manifest field {field} mismatch")
        packages = [dependency["package"] for dependency in payload["dependencies"]]
        if packages != ["perception_core", "perception_interfaces"]:
            raise ValueError(f"{run_dir}: workspace dependency closure is not fixed-domain")
        payloads.append(payload)

    for field in CLOSURE_MANIFEST_FIELDS:
        if len({manifest[field] for manifest in manifests}) != 1:
            raise ValueError(f"calibration closure provenance differs in {field}")
    return payloads


def _pidstat_cpu(
    run_dir: Path, manifest: dict[str, str], analyzed: dict[str, Any]
) -> dict[str, Any]:
    _require_manifest_fields(
        manifest,
        (
            "pidstat_start_monotonic_ns",
            "pidstat_stop_monotonic_ns",
            "pidstat_interval_s",
        ),
        run_dir,
    )
    start_ns = _positive_integer(manifest, "pidstat_start_monotonic_ns", run_dir)
    stop_ns = _positive_integer(manifest, "pidstat_stop_monotonic_ns", run_dir)
    interval_s = _positive_integer(manifest, "pidstat_interval_s", run_dir)
    if interval_s != 1:
        raise ValueError(f"{run_dir}: pidstat interval must be exactly one second")
    t0_ns = analyzed["t0_monotonic_ns"]
    t1_ns = analyzed["t1_monotonic_ns"]
    if not (
        start_ns <= t0_ns < t1_ns <= stop_ns
        and t0_ns - start_ns <= PIDSTAT_BRACKET_MARGIN_NS
        and stop_ns - t1_ns <= PIDSTAT_BRACKET_MARGIN_NS
    ):
        raise ValueError(f"{run_dir}: pidstat does not tightly bracket the formal window")

    tracee_pid = str(analyzed["tracee_pid"])
    sample_pids: set[str] = set()
    cpu_values: list[float] = []
    for line in (run_dir / "pidstat.txt").read_text(encoding="utf-8").splitlines():
        fields = line.split()
        if len(fields) < 15 or re.fullmatch(r"\d{2}:\d{2}:\d{2}", fields[0]) is None:
            continue
        sample_pids.add(fields[2])
        try:
            cpu = float(fields[7])
        except ValueError as error:
            raise ValueError(f"{run_dir}: pidstat contains an invalid CPU sample") from error
        if fields[2] == tracee_pid:
            cpu_values.append(cpu)

    if sample_pids != {tracee_pid}:
        raise ValueError(f"{run_dir}: pidstat samples do not belong only to the tracee PID")
    if any(not math.isfinite(value) or value < 0.0 or value > 100.0 for value in cpu_values):
        raise ValueError(f"{run_dir}: pidstat CPU sample is outside [0, 100]")
    duration_s = analyzed["duration_actual_s"]
    minimum_samples = max(1, math.floor(duration_s / interval_s) - 1)
    maximum_samples = math.ceil(duration_s / interval_s) + 3
    if not minimum_samples <= len(cpu_values) <= maximum_samples:
        raise ValueError(
            f"{run_dir}: pidstat sample count {len(cpu_values)} is outside "
            f"[{minimum_samples}, {maximum_samples}]"
        )
    return {
        "sample_count": len(cpu_values),
        "mean_percent": statistics.fmean(cpu_values),
        "start_margin_ns": t0_ns - start_ns,
        "stop_margin_ns": stop_ns - t1_ns,
    }


def _comparison(
    baseline: float, measured: float, relative_limit: float, absolute_floor: float
) -> dict[str, Any]:
    delta = abs(measured - baseline)
    threshold = max(abs(baseline) * relative_limit, absolute_floor)
    relative_percent = None if baseline == 0.0 else delta / abs(baseline) * 100.0
    return {
        "baseline": baseline,
        "measured": measured,
        "absolute_delta": delta,
        "relative_delta_percent": relative_percent,
        "threshold": threshold,
        "pass": delta <= threshold,
    }


def analyze_calibration(
    unprobed_dir: Path,
    callback_dir: Path,
    full_dir: Path,
    callback_replicate_dirs: tuple[Path, ...] = (),
    full_replicate_dirs: tuple[Path, ...] = (),
    unprobed_replicate_dirs: tuple[Path, ...] = (),
) -> dict[str, Any]:
    run_dirs = [path.resolve() for path in (unprobed_dir, callback_dir, full_dir)]
    callback_replicates = [path.resolve() for path in callback_replicate_dirs]
    full_replicates = [path.resolve() for path in full_replicate_dirs]
    unprobed_replicates = [path.resolve() for path in unprobed_replicate_dirs]
    replicate_dirs = callback_replicates + full_replicates + unprobed_replicates
    all_dirs = run_dirs + replicate_dirs
    if len(set(all_dirs)) != len(all_dirs):
        raise ValueError("calibration run directories must be distinct")
    if callback_replicates or full_replicates:
        callback_total = 1 + len(callback_replicates)
        full_total = 1 + len(full_replicates)
        if callback_total != full_total or callback_total < 3:
            raise ValueError(
                "median p99 evaluation requires equal callback/full run counts of"
                " at least three"
            )
    if unprobed_replicates:
        if 1 + len(unprobed_replicates) < 3 or 1 + len(full_replicates) < 3:
            raise ValueError(
                "median CPU evaluation requires at least three unprobed and"
                " three full runs"
            )
    analyzed = [analyze_run(path) for path in all_dirs]
    manifests = [parse_manifest(path / "run-manifest.txt") for path in all_dirs]

    expected_roles = (
        ("plain-sample", None),
        ("stage-latency", "callback"),
        ("stage-latency", "full"),
        *((("stage-latency", "callback"),) * len(callback_replicates)),
        *((("stage-latency", "full"),) * len(full_replicates)),
        *((("plain-sample", None),) * len(unprobed_replicates)),
    )
    for run_dir, result, (mode, event_set) in zip(all_dirs, analyzed, expected_roles):
        if result["mode"] != mode:
            raise ValueError(f"{run_dir}: calibration role requires mode {mode}")
        actual_event_set = result["latency"]["event_set"] if result["latency"] else None
        if actual_event_set != event_set:
            raise ValueError(f"{run_dir}: calibration event-set role is incorrect")
        if result["workload"] != "bounded":
            raise ValueError(f"{run_dir}: calibration requires the bounded workload")
        if result["duration_actual_s"] < MINIMUM_DURATION_S:
            raise ValueError(f"{run_dir}: calibration duration is below 120 seconds")

    if len({result["fingerprint"] for result in analyzed}) != 1:
        raise ValueError("calibration mapper fingerprints differ")
    identities = {
        (result["tracee_pid"], result["t0_monotonic_ns"], result["t1_monotonic_ns"])
        for result in analyzed
    }
    if len(identities) != len(all_dirs):
        raise ValueError("calibration runs do not contain independent evidence identities")
    for run_dir, manifest, result in zip(all_dirs, manifests, analyzed):
        _validate_role_provenance(run_dir, manifest, result)
    _validate_shared_provenance(manifests, all_dirs)
    closure_payloads = _validate_workspace_closures(manifests, all_dirs)
    _validate_target_builds(manifests[:3], run_dirs)
    for manifest, run_dir, reference in zip(
        manifests[3:],
        replicate_dirs,
        [manifests[1]] * len(callback_replicates)
        + [manifests[2]] * len(full_replicates)
        + [manifests[0]] * len(unprobed_replicates),
    ):
        _require_manifest_fields(manifest, TARGET_BUILD_FIELDS, run_dir)
        for field in TARGET_BUILD_FIELDS:
            if manifest[field] != reference[field]:
                raise ValueError(
                    f"{run_dir}: replicate target provenance differs in {field}"
                )

    stage_indices = list(range(1, 3 + len(callback_replicates) + len(full_replicates)))
    for index in stage_indices:
        callback_count = analyzed[index]["latency"]["callback"]["count"]
        if callback_count < MINIMUM_STAGE_CALLBACKS:
            label = str(all_dirs[index])
            raise ValueError(f"{label} stage evidence has fewer than 1200 callbacks")

    cpu = {
        label: _pidstat_cpu(run_dir, manifest, result)
        for label, run_dir, manifest, result in zip(
            ("unprobed", "callback", "full"), run_dirs, manifests, analyzed
        )
    }
    replicate_cpu = [
        _pidstat_cpu(run_dir, manifest, result)
        for run_dir, manifest, result in zip(
            replicate_dirs, manifests[3:], analyzed[3:]
        )
    ]
    callback_rep_count = len(callback_replicates)
    full_rep_count = len(full_replicates)
    unprobed_cpu_values = [cpu["unprobed"]["mean_percent"]] + [
        info["mean_percent"]
        for info in replicate_cpu[callback_rep_count + full_rep_count :]
    ]
    full_cpu_values = [cpu["full"]["mean_percent"]] + [
        info["mean_percent"]
        for info in replicate_cpu[callback_rep_count : callback_rep_count + full_rep_count]
    ]
    if unprobed_replicates:
        cpu_comparison = _comparison(
            statistics.median(unprobed_cpu_values),
            statistics.median(full_cpu_values),
            CPU_RELATIVE_LIMIT,
            CPU_ABSOLUTE_FLOOR_PERCENTAGE_POINTS,
        )
    else:
        cpu_comparison = _comparison(
            cpu["unprobed"]["mean_percent"],
            cpu["full"]["mean_percent"],
            CPU_RELATIVE_LIMIT,
            CPU_ABSOLUTE_FLOOR_PERCENTAGE_POINTS,
        )

    callback_p99_values = [analyzed[1]["latency"]["callback"]["p99_ns"]] + [
        analyzed[3 + offset]["latency"]["callback"]["p99_ns"]
        for offset in range(len(callback_replicates))
    ]
    full_p99_values = [analyzed[2]["latency"]["callback"]["p99_ns"]] + [
        analyzed[3 + len(callback_replicates) + offset]["latency"]["callback"]["p99_ns"]
        for offset in range(len(full_replicates))
    ]

    latency_comparisons: dict[str, dict[str, Any]] = {}
    for percentile in ("p50_ns", "p95_ns"):
        latency_comparisons[percentile] = _comparison(
            analyzed[1]["latency"]["callback"][percentile],
            analyzed[2]["latency"]["callback"][percentile],
            LATENCY_RELATIVE_LIMIT,
            LATENCY_ABSOLUTE_FLOOR_NS,
        )
    if callback_replicates:
        latency_comparisons["p99_ns"] = _comparison(
            statistics.median(callback_p99_values),
            statistics.median(full_p99_values),
            LATENCY_RELATIVE_LIMIT,
            LATENCY_ABSOLUTE_FLOOR_NS,
        )
    else:
        latency_comparisons["p99_ns"] = _comparison(
            analyzed[1]["latency"]["callback"]["p99_ns"],
            analyzed[2]["latency"]["callback"]["p99_ns"],
            LATENCY_RELATIVE_LIMIT,
            LATENCY_ABSOLUTE_FLOOR_NS,
        )

    gate_pass = cpu_comparison["pass"] and all(
        comparison["pass"] for comparison in latency_comparisons.values()
    )
    payload: dict[str, Any] = {
        "schema_version": 4 if unprobed_replicates else (3 if replicate_dirs else 2),
        "gate_pass": gate_pass,
        "workspace_closure": {
            "paired_source_identity_sha256": closure_payloads[0][
                "paired_source_identity_sha256"
            ],
            "install_base": closure_payloads[0]["closure_install_base"],
            "build_base": closure_payloads[0]["closure_build_base"],
            "manifest_sha256": manifests[0][
                "workspace_closure_manifest_sha256"
            ],
            "dependency_comparison_sha256": closure_payloads[0][
                "dependency_comparison_sha256"
            ],
            "helper_set_sha256": closure_payloads[0]["helper_set_sha256"],
        },
        "runs": {
            label: {
                "run_dir": str(run_dir),
                "tracee_pid": result["tracee_pid"],
                "duration_actual_s": result["duration_actual_s"],
                "observation_count": result["observation_count"],
                "revision_count": result["revision_count"],
                "target_sha256": manifest["target_sha256"],
                "target_build_id": manifest["target_build_id"],
                "cpu": cpu[label],
            }
            for label, run_dir, manifest, result in zip(
                ("unprobed", "callback", "full"), run_dirs, manifests, analyzed
            )
        },
        "cpu_full_vs_unprobed": cpu_comparison,
        "callback_full_vs_callback_only": {
            "callback_only_count": analyzed[1]["latency"]["callback"]["count"],
            "full_count": analyzed[2]["latency"]["callback"]["count"],
            "percentiles": latency_comparisons,
        },
    }
    if callback_replicates:
        payload["p99_evaluation"] = {
            "method": "median_of_runs",
            "callback_run_count": len(callback_p99_values),
            "full_run_count": len(full_p99_values),
            "callback_p99_ns": callback_p99_values,
            "full_p99_ns": full_p99_values,
            "callback_p99_median_ns": statistics.median(callback_p99_values),
            "full_p99_median_ns": statistics.median(full_p99_values),
        }
        payload["replicates"] = {
            "callback": [
                {
                    "run_dir": str(run_dir),
                    "tracee_pid": result["tracee_pid"],
                    "duration_actual_s": result["duration_actual_s"],
                    "target_sha256": manifest["target_sha256"],
                    "cpu": cpu_info,
                }
                for run_dir, manifest, result, cpu_info in zip(
                    callback_replicates,
                    manifests[3 : 3 + len(callback_replicates)],
                    analyzed[3 : 3 + len(callback_replicates)],
                    replicate_cpu[: len(callback_replicates)],
                )
            ],
            "full": [
                {
                    "run_dir": str(run_dir),
                    "tracee_pid": result["tracee_pid"],
                    "duration_actual_s": result["duration_actual_s"],
                    "target_sha256": manifest["target_sha256"],
                    "cpu": cpu_info,
                }
                for run_dir, manifest, result, cpu_info in zip(
                    full_replicates,
                    manifests[3 + callback_rep_count : 3 + callback_rep_count + full_rep_count],
                    analyzed[3 + callback_rep_count : 3 + callback_rep_count + full_rep_count],
                    replicate_cpu[callback_rep_count : callback_rep_count + full_rep_count],
                )
            ],
            "unprobed": [
                {
                    "run_dir": str(run_dir),
                    "tracee_pid": result["tracee_pid"],
                    "duration_actual_s": result["duration_actual_s"],
                    "target_sha256": manifest["target_sha256"],
                    "cpu": cpu_info,
                }
                for run_dir, manifest, result, cpu_info in zip(
                    unprobed_replicates,
                    manifests[3 + callback_rep_count + full_rep_count :],
                    analyzed[3 + callback_rep_count + full_rep_count :],
                    replicate_cpu[callback_rep_count + full_rep_count :],
                )
            ],
        }
    else:
        payload["p99_evaluation"] = {
            "method": "single_pair",
            "callback_run_count": 1,
            "full_run_count": 1,
        }
    if unprobed_replicates:
        payload["cpu_evaluation"] = {
            "method": "median_of_runs",
            "unprobed_run_count": len(unprobed_cpu_values),
            "full_run_count": len(full_cpu_values),
            "unprobed_cpu_percent": unprobed_cpu_values,
            "full_cpu_percent": full_cpu_values,
            "unprobed_cpu_median_percent": statistics.median(unprobed_cpu_values),
            "full_cpu_median_percent": statistics.median(full_cpu_values),
        }
    else:
        payload["cpu_evaluation"] = {
            "method": "single_pair",
            "unprobed_run_count": 1,
            "full_run_count": 1,
        }
    return payload


def quality_values(payload: dict[str, Any]) -> dict[str, Any]:
    cpu = payload["cpu_full_vs_unprobed"]
    values: dict[str, Any] = {
        "schema_version": payload["schema_version"],
        "paired_source_identity_sha256": payload["workspace_closure"][
            "paired_source_identity_sha256"
        ],
        "workspace_closure_manifest_sha256": payload["workspace_closure"][
            "manifest_sha256"
        ],
        "workspace_dependency_comparison_sha256": payload["workspace_closure"][
            "dependency_comparison_sha256"
        ],
        "profiling_helper_set_sha256": payload["workspace_closure"][
            "helper_set_sha256"
        ],
        "unprobed_cpu_sample_count": payload["runs"]["unprobed"]["cpu"]["sample_count"],
        "callback_cpu_sample_count": payload["runs"]["callback"]["cpu"]["sample_count"],
        "full_cpu_sample_count": payload["runs"]["full"]["cpu"]["sample_count"],
        "unprobed_cpu_mean_percent": payload["runs"]["unprobed"]["cpu"]["mean_percent"],
        "callback_cpu_mean_percent": payload["runs"]["callback"]["cpu"]["mean_percent"],
        "full_cpu_mean_percent": payload["runs"]["full"]["cpu"]["mean_percent"],
        "cpu_absolute_delta_percentage_points": cpu["absolute_delta"],
        "cpu_relative_delta_percent": (
            "unavailable" if cpu["relative_delta_percent"] is None else cpu["relative_delta_percent"]
        ),
        "cpu_threshold_percentage_points": cpu["threshold"],
        "cpu_gate_pass": cpu["pass"],
    }
    for percentile, comparison in payload["callback_full_vs_callback_only"][
        "percentiles"
    ].items():
        prefix = percentile.removesuffix("_ns")
        values[f"callback_{prefix}_absolute_delta_ns"] = comparison["absolute_delta"]
        values[f"callback_{prefix}_threshold_ns"] = comparison["threshold"]
        values[f"callback_{prefix}_gate_pass"] = comparison["pass"]
    evaluation = payload.get("p99_evaluation") or {}
    values["p99_evaluation_method"] = evaluation.get("method", "single_pair")
    values["p99_callback_run_count"] = evaluation.get("callback_run_count", 1)
    values["p99_full_run_count"] = evaluation.get("full_run_count", 1)
    if evaluation.get("method") == "median_of_runs":
        values["callback_p99_median_ns"] = evaluation["callback_p99_median_ns"]
        values["full_p99_median_ns"] = evaluation["full_p99_median_ns"]
    cpu_evaluation = payload.get("cpu_evaluation") or {}
    values["cpu_evaluation_method"] = cpu_evaluation.get("method", "single_pair")
    values["cpu_unprobed_run_count"] = cpu_evaluation.get("unprobed_run_count", 1)
    values["cpu_full_run_count"] = cpu_evaluation.get("full_run_count", 1)
    if cpu_evaluation.get("method") == "median_of_runs":
        values["unprobed_cpu_median_percent"] = cpu_evaluation[
            "unprobed_cpu_median_percent"
        ]
        values["full_cpu_median_percent"] = cpu_evaluation["full_cpu_median_percent"]
    values["gate_pass"] = payload["gate_pass"]
    return values


def write_quality(path: Path, values: dict[str, Any]) -> None:
    with path.open("w", encoding="utf-8") as stream:
        for key, value in values.items():
            if isinstance(value, bool):
                rendered = str(value).lower()
            else:
                rendered = str(value)
            stream.write(f"{key}={rendered}\n")
