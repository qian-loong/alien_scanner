#!/usr/bin/env python3

from __future__ import annotations

import csv
import hashlib
import importlib.util
import json
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
ANALYZER_PATH = SCRIPT_DIR / "analyze-local-map-profile.py"
CALIBRATION_CLI_PATH = SCRIPT_DIR / "analyze-local-map-stage-calibration.py"
RUNNER_PATH = SCRIPT_DIR / "profile-local-map.sh"
SHELL_COMMON_PATH = SCRIPT_DIR / "lib" / "profile-runner-common.sh"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from lib import local_map_build_provenance as BUILD_PROVENANCE
from lib import local_map_profile_analysis as ANALYSIS
from lib import profile_c3_drain as C3_DRAIN
from lib import profile_local_map_sampler as SAMPLER
from lib import stage_latency_analysis as STAGE
from lib import stage_latency_calibration as CALIBRATION

SPEC = importlib.util.spec_from_file_location("analyze_local_map_profile", ANALYZER_PATH)
assert SPEC is not None and SPEC.loader is not None
ANALYZER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(ANALYZER)


def write_csv(path: Path, fieldnames: tuple[str, ...], rows: list[dict]) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


class AnalyzeLocalMapProfileTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temp = tempfile.TemporaryDirectory(dir=SCRIPT_DIR)
        self.root = Path(self.temp.name)

    def tearDown(self) -> None:
        self.temp.cleanup()

    def make_run(
        self,
        name: str,
        *,
        workload: str = "bounded",
        mode: str = "plain-sample",
        first_revision: int = 1000,
        count: int = 100,
        tracee_pid: int = 4200,
        t0_ns: int = 1_000_000_000,
        c3_mode: str = "disabled",
        capacity_status: str = "not_reached",
        crossing_revision: int | None = None,
        stage_event_set: str = "full",
    ) -> Path:
        run_dir = self.root / name
        run_dir.mkdir()
        duration_s = count / 10.0
        t1_ns = t0_ns + int(duration_s * 1_000_000_000)
        manifest_lines = [
            f"mode={mode}",
            f"workload={workload}",
            f"c3_mode={c3_mode}",
            f"duration_requested_s={int(duration_s)}",
            f"tracee_pid={tracee_pid}",
            f"t0_monotonic_ns={t0_ns}",
            f"t1_monotonic_ns={t1_ns}",
        ]
        if mode == "stage-latency":
            manifest_lines.append(f"stage_event_set={stage_event_set}")
        manifest_lines.extend(("valid=true", "normal_completion=true"))
        run_dir.joinpath("run-manifest.txt").write_text(
            "\n".join(manifest_lines)
            + "\n",
            encoding="utf-8",
        )

        fingerprint = "a" * 64
        bounds = (-1.0, -2.0, -3.0, 10.0, 2.0, 3.0)
        observations = []
        states = []
        snapshots = []
        for offset in range(count):
            revision = first_revision + offset
            sequence = revision
            receipt = t0_ns + 10_000_000 + offset * 100_000_000
            stamp = 20_000_000_000 + sequence * 100_000_000
            observations.append(
                {
                    "receipt_monotonic_ns": receipt,
                    "sequence": sequence,
                    "stamp_ns": stamp,
                    "payload_digest": f"digest-{sequence}",
                    "expected_digest": f"digest-{sequence}",
                    "schema_valid": 1,
                    "digest_matches": 1,
                }
            )
            states.append(
                {
                    "receipt_monotonic_ns": receipt + 1_000_000,
                    "state_sequence": offset + 1,
                    "map_epoch": 1,
                    "revision": revision,
                    "stamp_ns": stamp,
                    "fingerprint": fingerprint,
                    "changed_cell_count": 10,
                    "has_bounds": 1,
                    "min_x": bounds[0],
                    "min_y": bounds[1],
                    "min_z": bounds[2],
                    "max_x": bounds[3] + (offset / 10.0 if workload == "expanding" else 0),
                    "max_y": bounds[4],
                    "max_z": bounds[5],
                }
            )
            snapshots.append(
                {
                    "receipt_monotonic_ns": receipt + 2_000_000,
                    "stamp_ns": stamp,
                    "ordinal": offset + 1,
                    "binary": 1,
                    "resolution_m": 0.2,
                    "data_bytes": 1000 + offset,
                }
            )

        write_csv(
            run_dir / "observations.csv",
            (
                "receipt_monotonic_ns",
                "sequence",
                "stamp_ns",
                "payload_digest",
                "expected_digest",
                "schema_valid",
                "digest_matches",
            ),
            observations,
        )
        write_csv(
            run_dir / "states.csv",
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
            states,
        )
        write_csv(
            run_dir / "snapshots.csv",
            (
                "receipt_monotonic_ns",
                "stamp_ns",
                "ordinal",
                "binary",
                "resolution_m",
                "data_bytes",
            ),
            snapshots,
        )
        write_csv(
            run_dir / "diagnostics.csv",
            ("receipt_monotonic_ns", "stamp_ns", "level", "name", "message"),
            [],
        )
        write_csv(
            run_dir / "health.csv",
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
            [
                {
                    "receipt_monotonic_ns": t0_ns + 100_000_000,
                    "stamp_ns": 20_000_000_000,
                    "state": 0,
                    "producer_source_id": "perception_profile_fixture",
                    "session_boot_ns": 123456,
                    "session_suffix": 7,
                    "fingerprint": fingerprint,
                    "full_no_return": 1,
                    "active_sensor_count": 1,
                },
                {
                    "receipt_monotonic_ns": t1_ns - 100_000_000,
                    "stamp_ns": 20_100_000_000,
                    "state": 0,
                    "producer_source_id": "perception_profile_fixture",
                    "session_boot_ns": 123456,
                    "session_suffix": 7,
                    "fingerprint": fingerprint,
                    "full_no_return": 1,
                    "active_sensor_count": 1,
                },
            ],
        )

        map_updates = []
        producer_diagnostics = []
        if c3_mode != "disabled":
            published_keyframes = 0
            published_deltas = 0
            previous_hash = "0" * 64
            for offset, state in enumerate(states):
                revision = state["revision"]
                keyframe = c3_mode == "keyframe-only" or offset == 0
                if keyframe:
                    published_keyframes += 1
                    kind = 1
                    base_revision = 0
                    base_hash = "0" * 64
                else:
                    published_deltas += 1
                    kind = 2
                    base_revision = revision - 1
                    base_hash = previous_hash
                content_hash = f"{revision:064x}"[-64:]
                update_receipt = int(state["receipt_monotonic_ns"]) + 3_000_000
                map_updates.append(
                    {
                        "receipt_monotonic_ns": update_receipt,
                        "stamp_ns": state["stamp_ns"],
                        "protocol_version": 2,
                        "canonical_encoding_version": 1,
                        "hash_algorithm": 1,
                        "content_identity_scheme": 2,
                        "content_identity_chunk_edge": 16,
                        "content_identity_coordinate_key_version": 1,
                        "content_identity_node_encoding_version": 1,
                        "update_kind": kind,
                        "vehicle_id": "profile-vehicle",
                        "mapper_session_boot_ns": 123456,
                        "mapper_session_suffix": 9,
                        "map_epoch": 1,
                        "base_revision": base_revision,
                        "new_revision": revision,
                        "revision_span": revision - base_revision,
                        "observed_coalesced_receipt_count": 0,
                        "known_cell_count": 2000,
                        "operation_count": 0 if keyframe else 1,
                        "canonical_payload_bytes": 1000 if keyframe else 24,
                        "base_content_hash": base_hash,
                        "content_hash": content_hash,
                        "update_hash": f"{revision + 1:064x}"[-64:],
                    }
                )
                producer_diagnostics.append(
                    self.producer_diagnostic_row(
                        update_receipt + 1_000_000,
                        revision,
                        published_keyframes,
                        published_deltas,
                    )
                )
                previous_hash = content_hash
            producer_diagnostics.extend(
                (
                    self.producer_diagnostic_row(
                        t1_ns + 100_000_000,
                        first_revision + count - 1,
                        published_keyframes,
                        published_deltas,
                    ),
                    self.producer_diagnostic_row(
                        t1_ns + 300_000_000,
                        first_revision + count - 1,
                        published_keyframes,
                        published_deltas,
                    ),
                )
            )

        write_csv(
            run_dir / "map_updates.csv",
            (
                "receipt_monotonic_ns", "stamp_ns", "protocol_version",
                "canonical_encoding_version", "hash_algorithm",
                "content_identity_scheme", "content_identity_chunk_edge",
                "content_identity_coordinate_key_version",
                "content_identity_node_encoding_version", "update_kind", "vehicle_id",
                "mapper_session_boot_ns", "mapper_session_suffix", "map_epoch",
                "base_revision", "new_revision", "revision_span",
                "observed_coalesced_receipt_count", "known_cell_count",
                "operation_count", "canonical_payload_bytes", "base_content_hash",
                "content_hash", "update_hash",
            ),
            map_updates,
        )
        write_csv(
            run_dir / "map_update_producer_diagnostics.csv",
            (
                "receipt_monotonic_ns", "stamp_ns", "level", "name", "message",
                "pending", "in_flight", "pending_revision", "in_flight_revision",
                "published_revision", "coalesced_receipts", "superseded_receipts",
                "published_keyframes", "published_deltas", "revision_only_deltas",
                "publish_failures", "resource_rejections", "snapshot_cells",
                "delta_operations", "payload_bytes", "acquire_duration_ns",
                "materialize_duration_ns", "traversal_duration_ns",
                "canonicalize_duration_ns", "geometry_fingerprint_duration_ns",
                "prepare_duration_ns", "validation_duration_ns", "diff_duration_ns",
                "encode_duration_ns", "store_candidate_duration_ns",
                "merkle_duration_ns", "update_hash_duration_ns", "publish_duration_ns",
            ),
            producer_diagnostics,
        )
        keyframe_count = sum(row["update_kind"] == 1 for row in map_updates)
        delta_count = sum(row["update_kind"] == 2 for row in map_updates)
        run_dir.joinpath("sink_manifest.yaml").write_text(
            "\n".join(
                (
                    "schema: alien-scanner/perception-profile-sink/v2",
                    f"mode: {workload}",
                    f"c3_mode: {c3_mode}",
                    "summary:",
                    f"  map_update_count: {len(map_updates)}",
                    f"  map_update_keyframe_count: {keyframe_count}",
                    f"  map_update_delta_count: {delta_count}",
                    "  map_update_revision_only_delta_count: 0",
                    f"  producer_diagnostic_count: {len(producer_diagnostics)}",
                    "  normal_completion: true",
                )
            )
            + "\n",
            encoding="utf-8",
        )
        run_dir.joinpath("c3-runtime-parameters.txt").write_text(
            "schema=alien-scanner/perception-c3-runtime-parameters/v1\n"
            f"c3_mode={c3_mode}\n"
            f"map_update_enabled={'false' if c3_mode == 'disabled' else 'true'}\n"
            f"map_update.delta_enabled={'false' if c3_mode == 'keyframe-only' else 'true'}\n"
            "map_update_topic=/profile/local_map/updates\n"
            f"sink.c3_mode={c3_mode}\n"
            "sink.map_update_topic=/profile/local_map/updates\n",
            encoding="utf-8",
        )
        if c3_mode == "disabled":
            drain_lines = (
                "drain_start_monotonic_ns=1", "drain_end_monotonic_ns=2",
                "drain_duration_ns=1", "drain_applicable=false",
                "drain_converged=not_applicable", "drain_stable_samples=0",
                "drain_latest_revision=none", "drain_published_revision=none",
                "drain_update_revision=none",
            )
        else:
            latest = first_revision + count - 1
            drain_lines = (
                f"drain_start_monotonic_ns={t1_ns}",
                f"drain_end_monotonic_ns={t1_ns + 300_000_000}",
                "drain_duration_ns=300000000", "drain_applicable=true",
                "drain_converged=true", "drain_stable_samples=2",
                f"drain_latest_revision={latest}",
                f"drain_published_revision={latest}",
                f"drain_update_revision={latest}",
            )
        run_dir.joinpath("drain-manifest.txt").write_text(
            "".join(f"{line}\n" for line in drain_lines), encoding="utf-8"
        )

        oracle_indices = sorted({0, count // 2, count - 1})
        oracle_rows = []
        for index in oracle_indices:
            state = states[index]
            known = 2000 if workload == "bounded" else 2000 + index * 25
            oracle_rows.append(
                {
                    "sequence": state["revision"],
                    "stamp_ns": state["stamp_ns"],
                    "payload_digest": f"digest-{state['revision']}",
                    "map_epoch": state["map_epoch"],
                    "revision": state["revision"],
                    "known": known,
                    "free": known - 100,
                    "occupied": 100,
                    "fingerprint": state["fingerprint"],
                    "min_x": state["min_x"],
                    "min_y": state["min_y"],
                    "min_z": state["min_z"],
                    "max_x": state["max_x"],
                    "max_y": state["max_y"],
                    "max_z": state["max_z"],
                }
            )
        write_csv(
            run_dir / "oracle_checkpoints.csv",
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
            oracle_rows,
        )
        required_end = crossing_revision + 300 if crossing_revision is not None else None
        run_dir.joinpath("oracle_manifest.yaml").write_text(
            "\n".join(
                (
                    "schema: alien-scanner/perception-profile-oracle/v1",
                    f"mode: {workload}",
                    f"accepted: {count}",
                    f"applied: {count}",
                    "no_evidence: 0",
                    "rejected: 0",
                    "unavailable: 0",
                    "backend_fault: 0",
                    "final_map_epoch: 1",
                    f"final_revision: {first_revision + count - 1}",
                    "plateau_start_revision: 300" if workload == "bounded" else "plateau_start_revision: null",
                    "capacity:",
                    f"  status: {capacity_status}",
                    f"  crossing_revision: {crossing_revision if crossing_revision is not None else 'null'}",
                    f"  required_end_revision: {required_end if required_end is not None else 'null'}",
                )
            )
            + "\n",
            encoding="utf-8",
        )
        write_csv(
            run_dir / "oracle_milestones.csv",
            ("threshold", "sequence", "stamp_ns", "revision", "known"),
            [],
        )

        resource_rows = []
        for index in (5, count // 2, count - 5):
            state = states[index]
            resource_rows.append(
                {
                    "receipt_monotonic_ns": state["receipt_monotonic_ns"] + 5_000_000,
                    "state_receipt_monotonic_ns": state["receipt_monotonic_ns"],
                    "map_epoch": 1,
                    "revision": state["revision"],
                    "stamp_ns": state["stamp_ns"],
                    "rss_kib": 20000 + index,
                    "pss_kib": 15000 + index,
                    "uss_kib": 12000 + index,
                    "cgroup_current_bytes": 100000000,
                    "cgroup_max_bytes": 1000000000,
                    "mem_available_kib": 8000000,
                    "oom": 0,
                    "oom_kill": 0,
                }
            )
        write_csv(run_dir / "resource-samples.csv", SAMPLER.RESOURCE_COLUMNS, resource_rows)
        checkpoint_rows = []
        for index in oracle_indices:
            state = states[index]
            checkpoint_rows.append(
                {
                    "receipt_monotonic_ns": state["receipt_monotonic_ns"] + 10_000_000,
                    "state_receipt_monotonic_ns": state["receipt_monotonic_ns"],
                    "map_epoch": state["map_epoch"],
                    "revision": state["revision"],
                    "stamp_ns": state["stamp_ns"],
                    "rss_kib": 20000 + (state["revision"] - first_revision),
                    "pss_kib": 15000 + (state["revision"] - first_revision),
                    "uss_kib": 12000 + (state["revision"] - first_revision),
                    "cgroup_current_bytes": 100000000,
                    "cgroup_max_bytes": 1000000000,
                    "mem_available_kib": 8000000,
                    "oom": 0,
                    "oom_kill": 0,
                }
            )
        write_csv(run_dir / "memory-checkpoints.csv", SAMPLER.RESOURCE_COLUMNS, checkpoint_rows)
        self.write_pidstat(run_dir, tracee_pid, max(1, int(duration_s)), 40.0)
        return run_dir

    @staticmethod
    def producer_diagnostic_row(
        receipt: int,
        revision: int,
        keyframes: int,
        deltas: int,
    ) -> dict[str, object]:
        return {
            "receipt_monotonic_ns": receipt,
            "stamp_ns": receipt,
            "level": 0,
            "name": "/profile_local_map: map_update_producer",
            "message": "running",
            "pending": 0,
            "in_flight": 0,
            "pending_revision": 0,
            "in_flight_revision": 0,
            "published_revision": revision,
            "coalesced_receipts": 0,
            "superseded_receipts": 0,
            "published_keyframes": keyframes,
            "published_deltas": deltas,
            "revision_only_deltas": 0,
            "publish_failures": 0,
            "resource_rejections": 0,
            "snapshot_cells": 2000,
            "delta_operations": 1 if deltas else 0,
            "payload_bytes": 24 if deltas else 1000,
            "acquire_duration_ns": revision,
            "materialize_duration_ns": revision + 1,
            "traversal_duration_ns": revision + 2,
            "canonicalize_duration_ns": revision + 3,
            "geometry_fingerprint_duration_ns": revision + 4,
            "prepare_duration_ns": revision + 5,
            "validation_duration_ns": revision + 6,
            "diff_duration_ns": revision + 7,
            "encode_duration_ns": revision + 8,
            "store_candidate_duration_ns": revision + 9,
            "merkle_duration_ns": revision + 10,
            "update_hash_duration_ns": revision + 11,
            "publish_duration_ns": revision + 12,
        }

    def mutate_csv(self, path: Path, mutator) -> None:
        with path.open(newline="", encoding="utf-8") as stream:
            reader = csv.DictReader(stream)
            fieldnames = tuple(reader.fieldnames or ())
            rows = list(reader)
        mutator(rows)
        write_csv(path, fieldnames, rows)

    def append_manifest(self, run_dir: Path, values: dict[str, object]) -> None:
        with (run_dir / "run-manifest.txt").open("a", encoding="utf-8") as stream:
            for key, value in values.items():
                stream.write(f"{key}={value}\n")

    def write_pidstat(
        self, run_dir: Path, pid: int, sample_count: int, cpu_percent: float
    ) -> None:
        rows = []
        for index in range(sample_count):
            total_seconds = 12 * 3600 + index
            hour = total_seconds // 3600
            minute = (total_seconds % 3600) // 60
            second = total_seconds % 60
            rows.append(
                f"{hour:02d}:{minute:02d}:{second:02d} 0 {pid} 0.00 0.00 0.00 "
                f"0.00 {cpu_percent:.2f} 0 0.00 0.00 100000 50000 0.10 target"
            )
        (run_dir / "pidstat.txt").write_text("\n".join(rows) + "\n", encoding="utf-8")

    def make_build_evidence(
        self,
        name: str,
        *,
        build_type: str = "RelWithDebInfo",
        stage_option: str = "OFF",
        build_profile: str = "performance",
        omit_flag: str | None = None,
    ) -> Path:
        build_dir = self.root / name
        build_dir.mkdir()
        flags = (
            BUILD_PROVENANCE.PERFORMANCE_FLAGS
            if build_profile == "performance"
            else BUILD_PROVENANCE.SANITIZER_FLAGS
        )
        ordered_flags = sorted(flag for flag in flags if flag != omit_flag)
        build_dir.joinpath("CMakeCache.txt").write_text(
            f"CMAKE_BUILD_TYPE:STRING={build_type}\n"
            "PERCEPTION_LOCAL_MAP_ENABLE_STAGE_LATENCY_TRACEPOINTS:BOOL="
            f"{stage_option}\n",
            encoding="utf-8",
        )
        build_dir.joinpath("compile_commands.json").write_text(
            json.dumps(
                [
                    {
                        "directory": str(build_dir),
                        "command": " ".join(
                            (
                                "/usr/bin/c++",
                                *ordered_flags,
                                "-o",
                                "CMakeFiles/perception_local_map_node.dir/src/"
                                "PerceptionLocalMapNode.cpp.o",
                                "-c",
                                "/src/PerceptionLocalMapNode.cpp",
                            )
                        ),
                        "file": "/src/PerceptionLocalMapNode.cpp",
                        "output": "CMakeFiles/perception_local_map_node.dir/src/"
                        "PerceptionLocalMapNode.cpp.o",
                    }
                ]
            ),
            encoding="utf-8",
        )
        return build_dir

    def make_workspace_closure(
        self,
        name: str,
        *,
        dependency_optimization: str = "-O2",
        dependency_build_type: str = "RelWithDebInfo",
        omit_dependency_flag: str | None = None,
    ) -> tuple[Path, dict[str, object]]:
        root = self.root / name
        install_base = root / "install"
        build_base = root / "build"
        install_base.mkdir(parents=True)
        build_base.mkdir()

        compile_flags = tuple(
            flag
            for flag in (
                dependency_optimization,
                "-g",
                "-DNDEBUG",
                "-fno-omit-frame-pointer",
            )
            if flag != omit_dependency_flag
        )
        for package in BUILD_PROVENANCE.WORKSPACE_DEPENDENCY_PACKAGES:
            package_install = install_base / package
            package_build = build_base / package
            package_install.mkdir()
            package_build.mkdir()
            package_build.joinpath("CMakeCache.txt").write_text(
                f"CMAKE_BUILD_TYPE:STRING={dependency_build_type}\n"
                f"CMAKE_INSTALL_PREFIX:PATH={package_install}\n",
                encoding="utf-8",
            )
            if package == "perception_core":
                source = Path(
                    "/workspaces/alien-scanner/ws/src/alien_perception/"
                    "perception_core/src/core.cpp"
                )
                output = "CMakeFiles/perception_core.dir/src/core.cpp.o"
            else:
                source = package_build / "rosidl_generator_c/generated.c"
                output = (
                    "CMakeFiles/perception_interfaces__rosidl_generator_c.dir/"
                    "rosidl_generator_c/generated.c.o"
                )
            package_build.joinpath("compile_commands.json").write_text(
                json.dumps(
                    [
                        {
                            "directory": str(package_build),
                            "arguments": [
                                "/usr/bin/c++",
                                *compile_flags,
                                "-o",
                                output,
                                "-c",
                                str(source),
                            ],
                            "file": str(source),
                            "output": output,
                        }
                    ]
                ),
                encoding="utf-8",
            )

        core_include = install_base / "perception_core/include/perception_core"
        core_include.mkdir(parents=True)
        core_include.joinpath("core.hpp").write_text("#pragma once\n", encoding="utf-8")
        core_lib = install_base / "perception_core/lib"
        core_lib.mkdir()
        core_lib.joinpath("libperception_core.a").write_bytes(b"archive\n")

        interfaces_include = (
            install_base
            / "perception_interfaces/include/perception_interfaces/perception_interfaces/msg"
        )
        interfaces_include.mkdir(parents=True)
        interfaces_include.joinpath("generated.hpp").write_text(
            "#pragma once\n", encoding="utf-8"
        )
        interfaces_lib = install_base / "perception_interfaces/lib"
        interfaces_lib.mkdir()
        shutil.copy2(
            Path(sys.executable),
            interfaces_lib / "libperception_interfaces__rosidl_typesupport_cpp.so",
        )

        profiling_prefix = install_base / "perception_profiling"
        profiling_prefix.joinpath("lib/perception_profiling").mkdir(parents=True)
        for executable in BUILD_PROVENANCE.PROFILING_HELPERS.values():
            shutil.copy2(Path(sys.executable), profiling_prefix / executable)
        workload = profiling_prefix / BUILD_PROVENANCE.PROFILING_WORKLOAD
        workload.parent.mkdir(parents=True)
        workload.write_text("profile_local_map: {}\n", encoding="utf-8")

        diff = root / "source-diff.patch"
        untracked = root / "source-untracked.txt"
        archive = root / "source-untracked.tar.gz"
        diff.write_bytes(b"diff\n")
        untracked.write_bytes(b"scripts/new.py\n")
        archive.write_bytes(b"archive\n")
        source_identity = BUILD_PROVENANCE.source_identity_from_files(
            "1" * 40, diff, untracked, archive
        )
        source_identity_path = root / "paired-source-identity.txt"
        BUILD_PROVENANCE.write_values(source_identity_path, source_identity)

        payload = BUILD_PROVENANCE.capture_workspace_closure(
            install_base, build_base, source_identity_path
        )
        manifest_path = root / "target-workspace-closure.json"
        BUILD_PROVENANCE.write_workspace_closure(manifest_path, payload)
        return manifest_path, payload

    def make_target_resolution_evidence(
        self,
        name: str,
        closure_payload: dict[str, object],
        *,
        compile_suffix: str = "",
        link_suffix: str = "",
    ) -> tuple[Path, Path]:
        build_dir = self.root / name
        link_dir = build_dir / "CMakeFiles/perception_local_map_node.dir"
        link_dir.mkdir(parents=True)
        install_base = Path(str(closure_payload["closure_install_base"]))
        build_dir.joinpath("CMakeCache.txt").write_text(
            "CMAKE_BUILD_TYPE:STRING=RelWithDebInfo\n", encoding="utf-8"
        )
        build_dir.joinpath("compile_commands.json").write_text(
            json.dumps(
                [
                    {
                        "file": "/workspaces/alien-scanner/ws/src/alien_perception/"
                        "perception_local_map/src/PerceptionLocalMapNode.cpp",
                        "output": "CMakeFiles/perception_local_map_node.dir/src/"
                        "PerceptionLocalMapNode.cpp.o",
                        "command": "/usr/bin/c++ "
                        f"-I{install_base / 'perception_core/include'} "
                        f"-I{install_base / 'perception_interfaces/include'} "
                        f"{compile_suffix}",
                    },
                    {
                        "file": "/workspaces/alien-scanner/ws/src/alien_perception/"
                        "perception_local_map/test/TestCaveFullRayScene.cpp",
                        "output": "CMakeFiles/TestCaveFullRayScene.dir/test/"
                        "TestCaveFullRayScene.cpp.o",
                        "command": "/usr/bin/c++ "
                        f"-I{install_base / 'cave_world/include'} "
                        f"-I{install_base / 'drone_scanner/include'} "
                        f"-I{install_base / 'perception_core/include'}",
                    },
                ]
            ),
            encoding="utf-8",
        )
        link_dir.joinpath("link.txt").write_text(
            "/usr/bin/c++ target.o "
            f"{install_base / 'perception_core/lib/libperception_core.a'} "
            f"-L{install_base / 'perception_interfaces/lib'} "
            "-lperception_interfaces__rosidl_typesupport_cpp "
            f"{link_suffix}\n",
            encoding="utf-8",
        )
        return build_dir, Path(sys.executable)

    def add_stage_evidence(
        self,
        run_dir: Path,
        event_set: str,
        sample_count: int,
        callback_durations: list[int] | None = None,
    ) -> None:
        manifest = ANALYSIS.parse_manifest(run_dir / "run-manifest.txt")
        tracee_pid = int(manifest["tracee_pid"])
        first_revision = 1000
        callback_values = callback_durations or [1_000_000] * sample_count
        self.assertEqual(sample_count, len(callback_values))
        stages = STAGE.EVENT_SET_STAGES[event_set]
        rows = []
        for index in range(sample_count):
            for stage_index, stage in enumerate(stages):
                duration = callback_values[index] if stage == "callback" else 100_000
                begin_ns = 2_000_000_000 + index * 2_000_000 + stage_index * 200_000
                rows.append(
                    {
                        "stage": stage,
                        "duration_ns": duration,
                        "revision": first_revision + index,
                        "callback_id": index + 1,
                        "vtid": tracee_pid,
                        "begin_realtime_ns": begin_ns,
                        "end_realtime_ns": begin_ns + duration,
                    }
                )
        write_csv(
            run_dir / "stage-latency.csv",
            (
                "stage",
                "duration_ns",
                "revision",
                "callback_id",
                "vtid",
                "begin_realtime_ns",
                "end_realtime_ns",
            ),
            rows,
        )
        (run_dir / "stage-probe-quality.txt").write_text(
            "event_schema_version=1\n"
            "provider=perception_local_map_stage\n"
            f"event_set={event_set}\n"
            f"target_pid={tracee_pid}\n"
            "loss_counter_count=1\n"
            "lost_events=0\n"
            "unmatched_entries=0\n"
            "unmatched_returns=0\n"
            "nesting_mismatches=0\n"
            "incomplete_callbacks=0\n"
            "duplicate_stage_samples=0\n"
            "invalid_durations=0\n"
            "unexpected_event_set_events=0\n"
            f"complete_applied_callbacks={sample_count}\n"
            "normal_completion=true\n"
            "gate_pass=true\n",
            encoding="utf-8",
        )

    def add_calibration_provenance(
        self,
        run_dir: Path,
        *,
        role: str,
        cpu_percent: float,
        closure_manifest: Path,
        closure_payload: dict[str, object],
    ) -> None:
        manifest = ANALYSIS.parse_manifest(run_dir / "run-manifest.txt")
        tracee_pid = int(manifest["tracee_pid"])
        t0_ns = int(manifest["t0_monotonic_ns"])
        t1_ns = int(manifest["t1_monotonic_ns"])
        common_hash = "c" * 64
        stage_target = role != "unprobed"
        source_identity = BUILD_PROVENANCE.load_source_identity(
            closure_manifest.parent / "paired-source-identity.txt"
        )
        values: dict[str, object] = {
            "source_revision": source_identity["source_revision"],
            "source_diff_sha256": source_identity["source_diff_sha256"],
            "source_untracked_sha256": source_identity["source_untracked_sha256"],
            "source_untracked_archive_sha256": source_identity[
                "source_untracked_archive_sha256"
            ],
            "profile_script_sha256": common_hash,
            "profile_runner_common_sha256": common_hash,
            "profile_role_monitor_sha256": common_hash,
            "profile_sampler_sha256": common_hash,
            "stage_latency_analysis_sha256": common_hash,
            "profile_build_provenance_sha256": common_hash,
            "analysis_script_sha256": common_hash,
            "workload_yaml_sha256": closure_payload["workload"]["sha256"],
            "rmw": "rmw_fastrtps_cpp",
            "cpu_target": 0,
            "cpu_helpers": 1,
            "cpu_allowed": "0-21",
            "target_affinity": 0,
            "install_prefix": "/tmp/stage/install" if stage_target else "/tmp/prod/install",
            "target_sha256": ("2" if stage_target else "1") * 64,
            "target_build_id": ("b" if stage_target else "a") * 40,
            "target_build_type": "RelWithDebInfo",
            "target_compile_flags_verified": "true",
            "target_stage_latency_option": "ON" if stage_target else "OFF",
            "pidstat_start_monotonic_ns": t0_ns - 500_000_000,
            "pidstat_stop_monotonic_ns": t1_ns + 500_000_000,
            "pidstat_interval_s": 1,
            "paired_source_identity_sha256": closure_payload[
                "paired_source_identity_sha256"
            ],
            "workspace_closure_install_base": closure_payload[
                "closure_install_base"
            ],
            "workspace_closure_build_base": closure_payload[
                "closure_build_base"
            ],
            "workspace_closure_manifest_sha256": hashlib.sha256(
                closure_manifest.read_bytes()
            ).hexdigest(),
            "workspace_dependency_comparison_sha256": closure_payload[
                "dependency_comparison_sha256"
            ],
            "profiling_prefix": str(
                Path(str(closure_payload["closure_install_base"]))
                / "perception_profiling"
            ),
            "profiling_helper_set_sha256": closure_payload["helper_set_sha256"],
        }
        for offset, name in enumerate(CALIBRATION.ROLE_NAMES, start=1):
            pid = tracee_pid if name in {"launcher", "tracee"} else tracee_pid + offset * 10
            values[f"{name}_pid"] = pid
            values[f"{name}_starttime"] = (
                100_001 if name in {"launcher", "tracee"} else 100_000 + offset
            )
            values[f"{name}_pgid"] = pid
        self.append_manifest(run_dir, values)
        shutil.copy2(closure_manifest, run_dir / "target-workspace-closure.json")
        shutil.copy2(
            closure_manifest.parent / "paired-source-identity.txt",
            run_dir / "paired-source-identity.txt",
        )
        for filename in (
            "source-diff.patch",
            "source-untracked.txt",
            "source-untracked.tar.gz",
        ):
            shutil.copy2(closure_manifest.parent / filename, run_dir / filename)
        self.write_pidstat(run_dir, tracee_pid, 120, cpu_percent)

    def make_calibration_runs(
        self,
        *,
        count: int = 1200,
        callback_stage_count: int | None = None,
        full_stage_count: int | None = None,
        unprobed_cpu: float = 40.0,
        callback_cpu: float = 40.5,
        full_cpu: float = 41.0,
        full_callback_durations: list[int] | None = None,
    ) -> tuple[Path, Path, Path]:
        closure_manifest, closure_payload = self.make_workspace_closure(
            "calibration-closure"
        )
        self._calibration_closure = (closure_manifest, closure_payload)
        runs = (
            self.make_run(
                "calibration-unprobed",
                mode="plain-sample",
                count=count,
                tracee_pid=4200,
                t0_ns=10_000_000_000,
            ),
            self.make_run(
                "calibration-callback",
                mode="stage-latency",
                stage_event_set="callback",
                count=count,
                tracee_pid=4300,
                t0_ns=200_000_000_000,
            ),
            self.make_run(
                "calibration-full",
                mode="stage-latency",
                stage_event_set="full",
                count=count,
                tracee_pid=4400,
                t0_ns=400_000_000_000,
            ),
        )
        for run_dir, role, cpu in zip(
            runs,
            ("unprobed", "callback", "full"),
            (unprobed_cpu, callback_cpu, full_cpu),
        ):
            self.add_calibration_provenance(
                run_dir,
                role=role,
                cpu_percent=cpu,
                closure_manifest=closure_manifest,
                closure_payload=closure_payload,
            )
        self.add_stage_evidence(
            runs[1], "callback", callback_stage_count or count
        )
        self.add_stage_evidence(
            runs[2],
            "full",
            full_stage_count or count,
            full_callback_durations,
        )
        return runs

    def make_calibration_replicate(
        self,
        name: str,
        *,
        event_set: str,
        tracee_pid: int,
        t0_ns: int,
        count: int = 1200,
        cpu_percent: float = 40.5,
        callback_durations: list[int] | None = None,
    ) -> Path:
        closure_manifest, closure_payload = self._calibration_closure
        if event_set == "unprobed":
            run_dir = self.make_run(
                name,
                mode="plain-sample",
                count=count,
                tracee_pid=tracee_pid,
                t0_ns=t0_ns,
            )
            self.add_calibration_provenance(
                run_dir,
                role="unprobed",
                cpu_percent=cpu_percent,
                closure_manifest=closure_manifest,
                closure_payload=closure_payload,
            )
            return run_dir
        run_dir = self.make_run(
            name,
            mode="stage-latency",
            stage_event_set=event_set,
            count=count,
            tracee_pid=tracee_pid,
            t0_ns=t0_ns,
        )
        self.add_calibration_provenance(
            run_dir,
            role=event_set,
            cpu_percent=cpu_percent,
            closure_manifest=closure_manifest,
            closure_payload=closure_payload,
        )
        self.add_stage_evidence(run_dir, event_set, count, callback_durations)
        return run_dir

    def test_valid_bounded_run_uses_oracle_plateau_and_memory(self) -> None:
        result = ANALYSIS.analyze_run(self.make_run("bounded"))
        self.assertEqual("bounded", result["workload"])
        self.assertEqual("disabled", result["c3_mode"])
        self.assertEqual(100, result["observation_count"])
        self.assertEqual(3, result["oracle_checkpoint_count"])
        self.assertEqual(2000, result["final_known"])
        self.assertEqual(10, result["cpu"]["sample_count"])
        self.assertEqual("full_short_window", result["cpu"]["sample_scope"])
        self.assertEqual(40.0, result["cpu"]["mean_percent"])
        self.assertEqual(3, result["memory"]["sample_count"])
        self.assertIsNone(result["memory"]["rss_kib_bytes_per_known"])

    def test_cpu_metrics_use_formal_samples_and_steady_300_second_window(self) -> None:
        short_run = self.root / "cpu-short"
        short_run.mkdir()
        self.write_pidstat(short_run, 4200, 24, 40.0)
        short = ANALYSIS._cpu_metrics(short_run, "4200", 25.0)
        self.assertEqual(24, short["sample_count"])
        self.assertEqual("full_short_window", short["sample_scope"])
        self.assertEqual(0, short["leading_samples_excluded"])

        formal_run = self.root / "cpu-formal"
        formal_run.mkdir()
        self.write_pidstat(formal_run, 4300, 300, 41.0)
        formal = ANALYSIS._cpu_metrics(formal_run, "4300", 300.0)
        self.assertEqual(240, formal["sample_count"])
        self.assertEqual("steady_after_60s", formal["sample_scope"])
        self.assertEqual(60, formal["leading_samples_excluded"])
        self.assertEqual(41.0, formal["mean_percent"])

    def test_c3_enabled_and_keyframe_only_evidence(self) -> None:
        enabled = ANALYSIS.analyze_run(
            self.make_run("c3-enabled", count=20, c3_mode="enabled")
        )
        self.assertEqual("enabled", enabled["c3_mode"])
        self.assertEqual(20, enabled["c3"]["map_update_count"])
        self.assertEqual(1, enabled["c3"]["published_keyframes"])
        self.assertEqual(19, enabled["c3"]["published_deltas"])
        self.assertEqual(20, enabled["c3"]["timing_sample_count"])
        self.assertEqual("true", enabled["c3"]["drain_converged"])

        keyframes = ANALYSIS.analyze_run(
            self.make_run("c3-keyframes", count=20, c3_mode="keyframe-only")
        )
        self.assertEqual("keyframe-only", keyframes["c3_mode"])
        self.assertEqual(20, keyframes["c3"]["published_keyframes"])
        self.assertEqual(0, keyframes["c3"]["published_deltas"])

    def test_c3_evidence_rejects_mode_chain_counter_and_drain_faults(self) -> None:
        run_dir = self.make_run("c3-mode-mismatch", count=20, c3_mode="enabled")
        manifest = run_dir / "sink_manifest.yaml"
        manifest.write_text(
            manifest.read_text(encoding="utf-8").replace(
                "c3_mode: enabled", "c3_mode: keyframe-only"
            ),
            encoding="utf-8",
        )
        with self.assertRaisesRegex(ValueError, "disagree on c3_mode"):
            ANALYSIS.analyze_run(run_dir)

        run_dir = self.make_run("c3-chain", count=20, c3_mode="enabled")
        self.mutate_csv(
            run_dir / "map_updates.csv",
            lambda rows: rows[1].update(
                {"base_revision": "999", "revision_span": "2"}
            ),
        )
        with self.assertRaisesRegex(ValueError, "base revision is not chained"):
            ANALYSIS.analyze_run(run_dir)

        run_dir = self.make_run("c3-keyframe-delta", count=20, c3_mode="keyframe-only")

        def inject_delta(rows):
            rows[1].update(
                {
                    "update_kind": "2",
                    "base_revision": rows[0]["new_revision"],
                    "base_content_hash": rows[0]["content_hash"],
                    "revision_span": "1",
                    "operation_count": "1",
                }
            )

        self.mutate_csv(run_dir / "map_updates.csv", inject_delta)
        with self.assertRaisesRegex(ValueError, "keyframe-only mode contains delta"):
            ANALYSIS.analyze_run(run_dir)

        run_dir = self.make_run("c3-counter", count=20, c3_mode="enabled")
        self.mutate_csv(
            run_dir / "map_update_producer_diagnostics.csv",
            lambda rows: rows[-1].update({"published_deltas": "18"}),
        )
        with self.assertRaisesRegex(ValueError, "counter is not conserved"):
            ANALYSIS.analyze_run(run_dir)

        run_dir = self.make_run("c3-drain", count=20, c3_mode="enabled")
        drain = run_dir / "drain-manifest.txt"
        drain.write_text(
            drain.read_text(encoding="utf-8").replace(
                "drain_converged=true", "drain_converged=false"
            ),
            encoding="utf-8",
        )
        with self.assertRaisesRegex(ValueError, "two stable samples"):
            ANALYSIS.analyze_run(run_dir)

    def test_c3_drain_monitor_handles_enabled_and_disabled_modes(self) -> None:
        enabled = self.make_run("drain-enabled", count=20, c3_mode="enabled")
        output = self.root / "drain-enabled-result.txt"
        start_ns = 1_000_000_000 + 2_000_000_000
        self.assertEqual(0, C3_DRAIN.monitor(enabled, "enabled", start_ns, 0.2, output))
        self.assertIn("drain_converged=true", output.read_text(encoding="utf-8"))

        disabled = self.make_run("drain-disabled", count=20)
        output = self.root / "drain-disabled-result.txt"
        self.assertEqual(0, C3_DRAIN.monitor(disabled, "disabled", 1, 0.2, output))
        self.assertIn("drain_converged=not_applicable", output.read_text(encoding="utf-8"))

    def test_bounded_run_rejects_unproven_or_changing_plateau(self) -> None:
        run_dir = self.make_run("not-plateau")
        manifest = run_dir / "oracle_manifest.yaml"
        manifest.write_text(
            manifest.read_text(encoding="utf-8").replace(
                "plateau_start_revision: 300", "plateau_start_revision: null"
            ),
            encoding="utf-8",
        )
        with self.assertRaisesRegex(ValueError, "did not prove a plateau"):
            ANALYSIS.analyze_run(run_dir)

        run_dir = self.make_run("changing")
        self.mutate_csv(
            run_dir / "oracle_checkpoints.csv",
            lambda rows: rows[-1].update({"known": "2001"}),
        )
        with self.assertRaisesRegex(ValueError, "changed after plateau"):
            ANALYSIS.analyze_run(run_dir)

    def test_expanding_requires_known_and_x_growth(self) -> None:
        run_dir = self.make_run("expanding", workload="expanding")
        result = ANALYSIS.analyze_run(run_dir)
        self.assertGreater(result["memory"]["rss_kib_bytes_per_known"], 0)

        self.mutate_csv(
            run_dir / "oracle_checkpoints.csv",
            lambda rows: [row.update({"known": "2000"}) for row in rows],
        )
        with self.assertRaisesRegex(ValueError, "did not increase exact known"):
            ANALYSIS.analyze_run(run_dir)

    def test_epoch_fingerprint_revision_and_stamp_fail_closed(self) -> None:
        cases = (
            ("epoch", lambda rows: rows[50].update({"map_epoch": "2"}), "map epoch changed"),
            ("fingerprint", lambda rows: rows[50].update({"fingerprint": "b" * 64}), "fingerprint changed"),
            ("revision", lambda rows: rows.pop(50), "revisions contain a gap"),
            ("stamp", lambda rows: rows[0].update({"stamp_ns": "123"}), "stamp_ns mismatch"),
        )
        for name, mutation, message in cases:
            with self.subTest(name=name):
                run_dir = self.make_run(name)
                self.mutate_csv(run_dir / "states.csv", mutation)
                with self.assertRaisesRegex(ValueError, message):
                    ANALYSIS.analyze_run(run_dir)

    def test_checkpoint_skew_and_diagnostic_reject(self) -> None:
        run_dir = self.make_run("skew")
        self.mutate_csv(
            run_dir / "memory-checkpoints.csv",
            lambda rows: rows[0].update(
                {
                    "receipt_monotonic_ns": str(
                        int(rows[0]["state_receipt_monotonic_ns"])
                        + ANALYSIS.MAX_CHECKPOINT_SKEW_NS
                        + 1
                    )
                }
            ),
        )
        with self.assertRaisesRegex(ValueError, "skew is too large"):
            ANALYSIS.analyze_run(run_dir)

        run_dir = self.make_run("diagnostic")
        write_csv(
            run_dir / "diagnostics.csv",
            ("receipt_monotonic_ns", "stamp_ns", "level", "name", "message"),
            [
                {
                    "receipt_monotonic_ns": 2_000_000_000,
                    "stamp_ns": 1,
                    "level": 1,
                    "name": "mapper",
                    "message": "rejected",
                }
            ],
        )
        with self.assertRaisesRegex(ValueError, "warning/error diagnostic"):
            ANALYSIS.analyze_run(run_dir)

    def test_prewindow_checkpoint_is_ignored_but_cannot_hide_missing_evidence(self) -> None:
        run_dir = self.make_run("prewindow-checkpoint")

        def add_prewindow_copy(rows):
            stale = dict(rows[0])
            stale["receipt_monotonic_ns"] = "1000000001"
            stale["state_receipt_monotonic_ns"] = "-69000000000"
            rows.insert(0, stale)

        self.mutate_csv(run_dir / "memory-checkpoints.csv", add_prewindow_copy)
        result = ANALYSIS.analyze_run(run_dir)
        self.assertEqual(3, result["memory"]["checkpoint_count"])

        self.mutate_csv(
            run_dir / "memory-checkpoints.csv",
            lambda rows: rows.pop(1),
        )
        with self.assertRaisesRegex(
            ValueError, "missing formal-window memory checkpoints: 1000"
        ):
            ANALYSIS.analyze_run(run_dir)

    def test_health_session_and_full_ray_capability_are_frozen(self) -> None:
        for name, mutation, message in (
            (
                "health-session",
                lambda rows: rows[-1].update({"session_suffix": "8"}),
                "producer session changed",
            ),
            (
                "health-capability",
                lambda rows: rows[-1].update({"full_no_return": "0"}),
                "health/capability/fingerprint gate failed",
            ),
        ):
            with self.subTest(name=name):
                run_dir = self.make_run(name)
                self.mutate_csv(run_dir / "health.csv", mutation)
                with self.assertRaisesRegex(ValueError, message):
                    ANALYSIS.analyze_run(run_dir)

    def test_capacity_covered_requires_all_three_registered_buckets(self) -> None:
        run_dir = self.make_run(
            "covered",
            workload="expanding",
            mode="capacity-ramp",
            first_revision=700,
            count=701,
            capacity_status="covered",
            crossing_revision=1050,
        )
        result = ANALYSIS.analyze_run(run_dir)
        capacity = result["capacity"]
        self.assertTrue(capacity["segmented_evidence_available"])
        self.assertEqual([751, 950], capacity["buckets"]["pre"])
        self.assertEqual([951, 1150], capacity["buckets"]["crossing"])
        self.assertEqual([1151, 1350], capacity["buckets"]["post"])

        run_dir = self.make_run(
            "covered-short",
            workload="expanding",
            mode="capacity-ramp",
            first_revision=900,
            count=300,
            capacity_status="covered",
            crossing_revision=1050,
        )
        with self.assertRaisesRegex(ValueError, "lacks the complete post window"):
            ANALYSIS.analyze_run(run_dir)

    def test_stage_latency_requires_complete_matched_samples(self) -> None:
        run_dir = self.make_run("latency", mode="stage-latency")
        run_dir.joinpath("stage-probe-quality.txt").write_text(
            "event_schema_version=1\n"
            "provider=perception_local_map_stage\n"
            "target_pid=4200\n"
            "loss_counter_count=2\n"
            "lost_events=0\n"
            "unmatched_entries=0\n"
            "unmatched_returns=0\n"
            "nesting_mismatches=0\n"
            "incomplete_callbacks=0\n"
            "duplicate_stage_samples=0\n"
            "invalid_durations=0\n"
            "complete_applied_callbacks=100\n"
            "normal_completion=true\n"
            "gate_pass=true\n",
            encoding="utf-8",
        )
        rows = [
            {
                "stage": stage,
                "duration_ns": 1000 + index,
                "revision": 1000 + index,
                "callback_id": index + 1,
                "vtid": 4200,
                "begin_realtime_ns": 2_000_000_000 + index * 2_000_000,
                "end_realtime_ns": 2_000_001_000
                + index * 2_000_000
                + index,
            }
            for stage in ANALYSIS.REQUIRED_STAGE_NAMES
            for index in range(100)
        ]
        write_csv(
            run_dir / "stage-latency.csv",
            (
                "stage",
                "duration_ns",
                "revision",
                "callback_id",
                "vtid",
                "begin_realtime_ns",
                "end_realtime_ns",
            ),
            rows,
        )
        result = ANALYSIS.analyze_run(run_dir)
        self.assertEqual(100, result["latency"]["mapper_apply"]["count"])
        self.assertEqual(1099, result["latency"]["callback"]["max_ns"])

        quality = run_dir.joinpath("stage-probe-quality.txt").read_text(encoding="utf-8")
        run_dir.joinpath("stage-probe-quality.txt").write_text(
            quality.replace("lost_events=0", "lost_events=1"), encoding="utf-8"
        )
        with self.assertRaisesRegex(ValueError, "nonzero lost_events"):
            ANALYSIS.analyze_run(run_dir)

    def test_stage_latency_callback_event_set_outputs_only_callback_metrics(self) -> None:
        run_dir = self.make_run(
            "latency-callback", mode="stage-latency", stage_event_set="callback"
        )
        run_dir.joinpath("stage-probe-quality.txt").write_text(
            "event_schema_version=1\n"
            "provider=perception_local_map_stage\n"
            "event_set=callback\n"
            "target_pid=4200\n"
            "loss_counter_count=1\n"
            "lost_events=0\n"
            "unmatched_entries=0\n"
            "unmatched_returns=0\n"
            "nesting_mismatches=0\n"
            "incomplete_callbacks=0\n"
            "duplicate_stage_samples=0\n"
            "invalid_durations=0\n"
            "unexpected_event_set_events=0\n"
            "complete_applied_callbacks=100\n"
            "normal_completion=true\n"
            "gate_pass=true\n",
            encoding="utf-8",
        )
        rows = [
            {
                "stage": "callback",
                "duration_ns": 1000 + index,
                "revision": 1000 + index,
                "callback_id": index + 1,
                "vtid": 4200,
                "begin_realtime_ns": 2_000_000_000 + index * 2_000_000,
                "end_realtime_ns": 2_000_001_000 + index * 2_000_000 + index,
            }
            for index in range(100)
        ]
        stage_path = run_dir / "stage-latency.csv"
        write_csv(
            stage_path,
            (
                "stage",
                "duration_ns",
                "revision",
                "callback_id",
                "vtid",
                "begin_realtime_ns",
                "end_realtime_ns",
            ),
            rows,
        )
        result = ANALYSIS.analyze_run(run_dir)
        self.assertEqual({"event_set", "callback"}, set(result["latency"]))
        self.assertEqual("callback", result["latency"]["event_set"])
        self.assertEqual(100, result["latency"]["callback"]["count"])

        quality_path = run_dir / "stage-probe-quality.txt"
        quality = quality_path.read_text(encoding="utf-8")
        quality_path.write_text(
            quality.replace("event_set=callback", "event_set=full"), encoding="utf-8"
        )
        with self.assertRaisesRegex(ValueError, "does not match the run manifest"):
            ANALYSIS.analyze_run(run_dir)
        quality_path.write_text(quality, encoding="utf-8")

        rows.append({**rows[0], "stage": "mapper_apply"})
        write_csv(stage_path, tuple(rows[0]), rows)
        with self.assertRaisesRegex(ValueError, "exactly one sample per stage"):
            ANALYSIS.analyze_run(run_dir)

    def test_stage_trace_pairing_filters_pid_and_non_applied_callbacks(self) -> None:
        events: list[STAGE.TraceEvent] = []

        def event(
            timestamp_ns: int,
            stage: str,
            phase: str,
            callback_id: int,
            *,
            vpid: int = 4200,
            applied: int = -1,
            revision: int = 0,
        ) -> None:
            events.append(
                STAGE.TraceEvent(
                    timestamp_ns,
                    vpid,
                    4201,
                    stage,
                    phase,
                    callback_id,
                    revision,
                    applied,
                )
            )

        def callback(base: int, callback_id: int, revision: int, applied: bool = True) -> None:
            event(base, "callback", "begin", callback_id, applied=0)
            event(base + 10, "mapper_apply", "begin", callback_id)
            event(base + 20, "mapper_apply", "end", callback_id)
            if applied:
                event(base + 30, "snapshot_total", "begin", callback_id, revision=revision)
                event(base + 40, "read_transaction", "begin", callback_id, revision=revision)
                event(base + 50, "read_transaction", "end", callback_id, revision=revision)
                event(base + 60, "snapshot_serialization", "begin", callback_id)
                event(base + 70, "snapshot_serialization", "end", callback_id)
                event(base + 80, "snapshot_total", "end", callback_id, revision=revision)
                event(base + 90, "state_publication", "begin", callback_id)
                event(base + 100, "state_publication", "end", callback_id)
            event(
                base + 110,
                "callback",
                "end",
                callback_id,
                applied=1 if applied else 0,
                revision=revision if applied else 0,
            )

        callback(1_100, 1, 77)
        callback(1_300, 2, 0, applied=False)
        event(1_500, "callback", "begin", 3, vpid=9999, applied=0)
        event(1_510, "callback", "end", 3, vpid=9999, applied=1, revision=88)
        pairs, counters = STAGE.pair_stage_events(events, 4200, 1_000, 2_000)
        self.assertEqual(6, len(pairs))
        self.assertEqual({77}, {pair.revision for pair in pairs})
        self.assertEqual(1, counters["complete_applied_callbacks"])
        self.assertEqual(0, counters["unmatched_entries"])
        self.assertEqual(0, counters["unmatched_returns"])

        callback_pairs, callback_counters = STAGE.pair_stage_events(
            events, 4200, 1_000, 2_000, "callback"
        )
        self.assertEqual(1, len(callback_pairs))
        self.assertEqual("callback", callback_pairs[0].stage)
        self.assertEqual(12, callback_counters["unexpected_event_set_events"])

        callback_only_events = [item for item in events if item.stage == "callback"]
        callback_pairs, callback_counters = STAGE.pair_stage_events(
            callback_only_events, 4200, 1_000, 2_000, "callback"
        )
        self.assertEqual(1, len(callback_pairs))
        self.assertEqual(0, callback_counters["unexpected_event_set_events"])
        self.assertEqual(0, callback_counters["incomplete_callbacks"])
        self.assertEqual(0, callback_counters["duplicate_stage_samples"])

        duplicate_events = callback_only_events + [
            STAGE.TraceEvent(1_600, 4200, 4201, "callback", "begin", 1, 0, 0),
            STAGE.TraceEvent(1_610, 4200, 4201, "callback", "end", 1, 77, 1),
        ]
        duplicate_pairs, duplicate_counters = STAGE.pair_stage_events(
            duplicate_events, 4200, 1_000, 2_000, "callback"
        )
        self.assertEqual([], duplicate_pairs)
        self.assertEqual(1, duplicate_counters["incomplete_callbacks"])
        self.assertEqual(1, duplicate_counters["duplicate_stage_samples"])
        with self.assertRaisesRegex(ValueError, "invalid stage event set"):
            STAGE.pair_stage_events(callback_only_events, 4200, 1_000, 2_000, "unknown")

    def test_stage_trace_pairing_reports_unmatched_and_loss_evidence(self) -> None:
        event = STAGE.TraceEvent(
            1_500, 4200, 4201, "callback", "end", 9, 9, 1
        )
        pairs, counters = STAGE.pair_stage_events([event], 4200, 1_000, 2_000)
        self.assertEqual([], pairs)
        self.assertEqual(1, counters["unmatched_returns"])

        stats = self.root / "stage-session-final.txt"
        stats.write_text(
            "Discarded events: 0\nLost packets: 0\nDiscarded events: 2\n",
            encoding="utf-8",
        )
        self.assertEqual((2, 3), STAGE.parse_loss_counters(stats))

        line = (
            "[1720000000.123456789] (+0.000000001) host "
            "perception_local_map_stage:callback_end: { vpid = 4200, vtid = 4201 }, "
            "{ callback_id = 9, revision = 77, applied = 1 }"
        )
        parsed = STAGE.parse_pretty_event(line)
        self.assertIsNotNone(parsed)
        self.assertEqual(1_720_000_000_123_456_789, parsed.timestamp_ns)
        self.assertEqual((4200, 4201, 77, 1), (
            parsed.vpid, parsed.vtid, parsed.revision, parsed.applied
        ))

    def test_stage_tracepoints_are_opt_in_and_runner_owns_the_ust_session(self) -> None:
        cmake = (SCRIPT_DIR.parent / "ws/src/alien_perception/perception_local_map/CMakeLists.txt").read_text(
            encoding="utf-8"
        )
        self.assertRegex(
            cmake,
            r"PERCEPTION_LOCAL_MAP_ENABLE_STAGE_LATENCY_TRACEPOINTS\s+"
            r'"Compile profiling-only LTTng-UST stage latency tracepoints"\s+OFF',
        )
        runner = RUNNER_PATH.read_text(encoding="utf-8")
        self.assertNotIn("ALIEN_PROFILE_STAGE_PROBE", runner)
        self.assertNotIn("perception_local_map_stage:*", runner)
        self.assertIn("STAGE_PROVIDER_EVENTS=(", runner)
        self.assertIn("STAGE_SELECTED_EVENTS=(callback_begin callback_end)", runner)
        self.assertIn("stage_event_set=${STAGE_EVENT_SET}", runner)
        self.assertIn("validate_stage_session_event_set", runner)
        self.assertIn("stage_latency_analysis", runner)
        self.assertIn("STAGE_TRACE_SUBBUF_SIZE_BYTES=262144", runner)
        self.assertIn("STAGE_TRACE_NUM_SUBBUF=4", runner)
        self.assertIn("STAGE_TRACE_SHM_MARGIN_BYTES=8388608", runner)
        self.assertNotIn("--subbuf-size=8M", runner)
        self.assertIn("lttng_ust_shm_headroom", runner)
        self.assertIn(
            'babeltrace2 convert --clock-seconds \\\n'
            '                "${trace_root}" -w "${OUTPUT_DIR}/stage-trace-events.txt"',
            runner,
        )
        self.assertNotIn(
            "babeltrace2 convert --clock-seconds --fields=all --names=all", runner
        )
        self.assertLess(
            runner.index('prepare_stage_trace ||'), runner.index("start_target ||")
        )
        self.assertLess(
            runner.index("start_target ||"), runner.index('validate_stage_trace ||')
        )

    def test_stage_trace_shm_headroom_is_recomputable_and_fails_closed(self) -> None:
        command = (
            f'source "{SHELL_COMMON_PATH}"; '
            "lttng_ust_shm_headroom 22 262144 4 8388608 31457280"
        )
        result = subprocess.run(
            ["bash", "-c", command], text=True, capture_output=True, check=False
        )
        self.assertEqual(0, result.returncode, result.stderr)
        self.assertIn("stage_trace_shm_required_bytes=31457280", result.stdout)
        self.assertIn("stage_trace_shm_gate_pass=true", result.stdout)

        result = subprocess.run(
            ["bash", "-c", command.rsplit(" ", 1)[0] + " 31457279"],
            text=True,
            capture_output=True,
            check=False,
        )
        self.assertEqual(1, result.returncode, result.stderr)
        self.assertIn("stage_trace_shm_required_bytes=31457280", result.stdout)
        self.assertIn("stage_trace_shm_gate_pass=false", result.stdout)

    def test_build_provenance_uses_real_cache_and_compile_commands(self) -> None:
        performance = self.make_build_evidence("performance")
        values = BUILD_PROVENANCE.validate_build(performance, "performance", "OFF")
        self.assertEqual("RelWithDebInfo", values["target_build_type"])
        self.assertEqual("true", values["target_compile_flags_verified"])
        self.assertEqual("OFF", values["target_stage_latency_option"])

        stage = self.make_build_evidence("stage", stage_option="ON")
        self.assertEqual(
            "ON",
            BUILD_PROVENANCE.validate_build(stage, "performance", "ON")[
                "target_stage_latency_option"
            ],
        )
        sanitizer = self.make_build_evidence(
            "sanitizer", build_profile="sanitizer"
        )
        BUILD_PROVENANCE.validate_build(sanitizer, "sanitizer", "OFF")

        wrong_type = self.make_build_evidence("wrong-type", build_type="Release")
        with self.assertRaisesRegex(ValueError, "expected RelWithDebInfo"):
            BUILD_PROVENANCE.validate_build(wrong_type, "performance", "OFF")
        missing_flag = self.make_build_evidence(
            "missing-flag", omit_flag="-fno-omit-frame-pointer"
        )
        with self.assertRaisesRegex(ValueError, "lacks required flags"):
            BUILD_PROVENANCE.validate_build(missing_flag, "performance", "OFF")
        with self.assertRaisesRegex(ValueError, "option is OFF, expected ON"):
            BUILD_PROVENANCE.validate_build(performance, "performance", "ON")

    def test_workspace_closure_is_canonical_and_recomputable(self) -> None:
        manifest_path, payload = self.make_workspace_closure("closure-canonical")
        self.assertEqual(
            BUILD_PROVENANCE.canonical_json_bytes(payload), manifest_path.read_bytes()
        )
        self.assertEqual(
            payload,
            BUILD_PROVENANCE.load_and_validate_workspace_closure(manifest_path),
        )
        self.assertEqual(
            ["perception_core", "perception_interfaces"],
            [entry["package"] for entry in payload["dependencies"]],
        )
        for dependency in payload["dependencies"]:
            relative_paths = [
                artifact["relative_path"] for artifact in dependency["artifacts"]
            ]
            self.assertEqual(sorted(relative_paths), relative_paths)
        self.assertEqual(
            sorted(BUILD_PROVENANCE.PROFILING_HELPERS),
            [helper["name"] for helper in payload["helpers"]],
        )
        self.assertEqual(
            sorted(helper["relative_path"] for helper in payload["helpers"]),
            [helper["relative_path"] for helper in payload["helpers"]],
        )

        identity_path = manifest_path.parent / "paired-source-identity.txt"
        identity = BUILD_PROVENANCE.load_source_identity(identity_path)
        identity["source_revision"] = "invalid"
        identity["paired_source_identity_sha256"] = BUILD_PROVENANCE._canonical_sha256(
            {
                key: identity[key]
                for key in BUILD_PROVENANCE.SOURCE_IDENTITY_KEYS[:-1]
            }
        )
        BUILD_PROVENANCE.write_values(identity_path, identity)
        with self.assertRaisesRegex(ValueError, "revision is not a lowercase Git SHA"):
            BUILD_PROVENANCE.load_source_identity(identity_path)

    def test_workspace_closure_rejects_build_profile_and_path_faults(self) -> None:
        with self.assertRaisesRegex(ValueError, "required flags|optimization"):
            self.make_workspace_closure(
                "closure-o3", dependency_optimization="-O3"
            )
        with self.assertRaisesRegex(ValueError, "build type is not RelWithDebInfo"):
            self.make_workspace_closure(
                "closure-release", dependency_build_type="Release"
            )
        with self.assertRaisesRegex(ValueError, "lacks required flags"):
            self.make_workspace_closure(
                "closure-frame-pointer",
                omit_dependency_flag="-fno-omit-frame-pointer",
            )

        manifest_path, payload = self.make_workspace_closure("closure-build-escape")
        build_base = Path(str(payload["closure_build_base"]))
        escaped_build = self.root / "escaped-core-build"
        shutil.copytree(build_base / "perception_core", escaped_build)
        shutil.rmtree(build_base / "perception_core")
        (build_base / "perception_core").symlink_to(
            escaped_build, target_is_directory=True
        )
        with self.assertRaisesRegex(ValueError, "outside closure build base"):
            BUILD_PROVENANCE.capture_workspace_closure(
                Path(str(payload["closure_install_base"])),
                build_base,
                manifest_path.parent / "paired-source-identity.txt",
            )

    def test_workspace_closure_rejects_artifact_faults(self) -> None:

        manifest_path, payload = self.make_workspace_closure("closure-mutation")
        install_base = Path(str(payload["closure_install_base"]))
        helper = (
            install_base
            / "perception_profiling"
            / BUILD_PROVENANCE.PROFILING_HELPERS["fixture"]
        )
        helper.write_bytes(helper.read_bytes() + b"mutated")
        with self.assertRaisesRegex(ValueError, "closure artifact changed"):
            BUILD_PROVENANCE.load_and_validate_workspace_closure(manifest_path)

        escape_root = self.root / "closure-escape"
        manifest_path, payload = self.make_workspace_closure("closure-escape")
        install_base = Path(str(payload["closure_install_base"]))
        header = install_base / "perception_core/include/perception_core/core.hpp"
        outside = escape_root / "outside.hpp"
        outside.write_text("#pragma once\n", encoding="utf-8")
        header.unlink()
        header.symlink_to(outside)
        with self.assertRaisesRegex(ValueError, "outside closure install base"):
            BUILD_PROVENANCE.capture_workspace_closure(
                install_base,
                Path(str(payload["closure_build_base"])),
                escape_root / "paired-source-identity.txt",
            )

        manifest_path, payload = self.make_workspace_closure("closure-duplicate")
        install_base = Path(str(payload["closure_install_base"]))
        header_root = install_base / "perception_core/include/perception_core"
        header_root.joinpath("duplicate.hpp").symlink_to(header_root / "core.hpp")
        with self.assertRaisesRegex(ValueError, "duplicate artifacts"):
            BUILD_PROVENANCE.capture_workspace_closure(
                install_base,
                Path(str(payload["closure_build_base"])),
                manifest_path.parent / "paired-source-identity.txt",
            )

        manifest_path, payload = self.make_workspace_closure("closure-missing")
        install_base = Path(str(payload["closure_install_base"]))
        missing = (
            install_base
            / "perception_profiling"
            / BUILD_PROVENANCE.PROFILING_HELPERS["sink"]
        )
        missing.unlink()
        with self.assertRaisesRegex(ValueError, "artifact is missing"):
            BUILD_PROVENANCE.capture_workspace_closure(
                install_base,
                Path(str(payload["closure_build_base"])),
                manifest_path.parent / "paired-source-identity.txt",
            )

        manifest_path, payload = self.make_workspace_closure("closure-helper-build-id")
        install_base = Path(str(payload["closure_install_base"]))
        helper = (
            install_base
            / "perception_profiling"
            / BUILD_PROVENANCE.PROFILING_HELPERS["oracle"]
        )
        helper.write_text("not an ELF\n", encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "lacks an ELF build ID"):
            BUILD_PROVENANCE.capture_workspace_closure(
                install_base,
                Path(str(payload["closure_build_base"])),
                manifest_path.parent / "paired-source-identity.txt",
            )

    def test_target_closure_rejects_main_install_and_extra_workspace_dependency(
        self,
    ) -> None:
        _, payload = self.make_workspace_closure("target-resolution-closure")
        install_base = Path(str(payload["closure_install_base"]))
        build_dir, target = self.make_target_resolution_evidence(
            "target-resolution-valid", payload
        )
        BUILD_PROVENANCE._validate_target_closure_resolution(
            build_dir, target, install_base
        )

        build_dir, target = self.make_target_resolution_evidence(
            "target-resolution-main-install",
            payload,
            compile_suffix="-I/workspaces/alien-scanner/ws/install/perception_core/include",
        )
        with self.assertRaisesRegex(ValueError, "forbidden main-workspace install"):
            BUILD_PROVENANCE._validate_target_closure_resolution(
                build_dir, target, install_base
            )

        build_dir, target = self.make_target_resolution_evidence(
            "target-resolution-extra-package",
            payload,
            link_suffix=str(install_base / "cave_world/lib/libcave_world.a"),
        )
        with self.assertRaisesRegex(ValueError, "unexpected workspace packages: cave_world"):
            BUILD_PROVENANCE._validate_target_closure_resolution(
                build_dir, target, install_base
            )

        build_dir, target = self.make_target_resolution_evidence(
            "target-resolution-production-compile-extra",
            payload,
            compile_suffix=f"-I{install_base / 'perception_fixtures/include'}",
        )
        with self.assertRaisesRegex(
            ValueError, "unexpected workspace packages: perception_fixtures"
        ):
            BUILD_PROVENANCE._validate_target_closure_resolution(
                build_dir, target, install_base
            )

    def test_runner_stage_prefix_split_is_explicit_and_stage_only(self) -> None:
        runner = RUNNER_PATH.read_text(encoding="utf-8")
        production_setup = 'source "${CLOSURE_INSTALL_BASE}/setup.bash"'
        target_local_setup = 'source "${INSTALL_PREFIX}/local_setup.bash"'
        self.assertIn(production_setup, runner)
        self.assertIn(target_local_setup, runner)
        self.assertLess(runner.index(production_setup), runner.index(target_local_setup))
        self.assertIn(
            "stage-latency requires closure install/build bases and paired source identity",
            runner,
        )
        self.assertIn(
            'elif [[ -n "${PAIRED_SOURCE_IDENTITY}" ]]; then',
            runner,
        )
        self.assertIn(
            "non-stage modes require target and helper packages from one install prefix",
            runner,
        )
        self.assertIn("capture-closure", runner)
        self.assertIn("target-workspace-closure.json", runner)
        self.assertIn("/workspaces/alien-scanner/ws/install/*", runner)
        self.assertLess(
            runner.index("capture-closure"),
            runner.index('taskset -c 1 "${ORACLE_EXE}"'),
        )

        fake_install = self.root / "stage-install"
        fake_install.mkdir()
        for setup in ("setup.bash", "local_setup.bash"):
            fake_install.joinpath(setup).write_text("true\n", encoding="utf-8")
        result = subprocess.run(
            [
                "bash",
                str(RUNNER_PATH),
                "stage-latency",
                str(fake_install),
                str(self.root / "stage-output"),
                "1",
                "bounded",
                "callback",
            ],
            text=True,
            capture_output=True,
            check=False,
        )
        self.assertEqual(2, result.returncode)
        self.assertIn("requires closure install/build bases", result.stderr)
        self.assertFalse((self.root / "stage-output").exists())

    def test_runner_records_real_build_affinity_and_pidstat_provenance(self) -> None:
        runner = RUNNER_PATH.read_text(encoding="utf-8")
        for text in (
            'PROFILE_BUILD_PROVENANCE="${SCRIPT_DIR}/lib/local_map_build_provenance.py"',
            "profile_build_provenance_sha256=",
            "install_prefix=${INSTALL_PREFIX}",
            "target_build_artifact_build_id=${TARGET_BUILD_ARTIFACT_BUILD_ID}",
            'target_affinity="$(process_affinity_list "${TRACEE_PID}"',
            "target_affinity=${target_affinity}",
            "pidstat_start_monotonic_ns=${PIDSTAT_START_MONOTONIC_NS}",
            "pidstat_stop_monotonic_ns=${PIDSTAT_STOP_MONOTONIC_NS}",
            "pidstat_interval_s=${PIDSTAT_INTERVAL_S}",
            "validate_pidstat_bracket",
            "create_deterministic_source_archive",
        ):
            self.assertIn(text, runner)

        flow = runner[runner.index("start_target || exit 1") :]
        self.assertLess(flow.index("start_checkpoint_sampler ||"), flow.index("start_pidstat ||"))
        self.assertLess(flow.index("start_pidstat ||"), flow.index('T0_MONOTONIC_NS="'))
        self.assertLess(flow.index('T1_MONOTONIC_NS="'), flow.index('PIDSTAT_STOP_MONOTONIC_NS="'))
        self.assertLess(
            flow.index('PIDSTAT_STOP_MONOTONIC_NS="'),
            flow.index('stop_group "${PIDSTAT_PID}"'),
        )

    def test_pidstat_bracket_and_source_archive_helpers_fail_closed(self) -> None:
        common = SHELL_COMMON_PATH.as_posix()
        valid = subprocess.run(
            [
                "bash",
                "-c",
                f'source "{common}"; validate_pidstat_bracket 100 150 250 300 100',
            ],
            text=True,
            capture_output=True,
            check=False,
        )
        self.assertEqual(0, valid.returncode, valid.stderr)
        for arguments in ("100 250 200 300 100", "100 250 300 350 100"):
            with self.subTest(arguments=arguments):
                invalid = subprocess.run(
                    [
                        "bash",
                        "-c",
                        f'source "{common}"; validate_pidstat_bracket {arguments}',
                    ],
                    text=True,
                    capture_output=True,
                    check=False,
                )
                self.assertNotEqual(0, invalid.returncode)

        source_root = self.root / "archive-source"
        source_root.mkdir()
        source_root.joinpath("payload.txt").write_text("payload\n", encoding="utf-8")
        file_list = self.root / "archive-files.txt"
        file_list.write_bytes(b"payload.txt\n")
        first = self.root / "first.tar.gz"
        second = self.root / "second.tar.gz"
        for output in (first, second):
            result = subprocess.run(
                [
                    "bash",
                    "-c",
                    f'source "{common}"; create_deterministic_source_archive '
                    f'"archive-source" "archive-files.txt" "{output.name}"',
                ],
                text=True,
                capture_output=True,
                check=False,
                cwd=self.root,
            )
            self.assertEqual(0, result.returncode, result.stderr)
            source_root.joinpath("payload.txt").touch()
        self.assertEqual(first.read_bytes(), second.read_bytes())

    def test_stage_calibration_accepts_complete_three_role_evidence(self) -> None:
        runs = self.make_calibration_runs()
        result = CALIBRATION.analyze_calibration(*runs)
        self.assertTrue(result["gate_pass"])
        self.assertEqual(2, result["schema_version"])
        self.assertEqual(120, result["runs"]["unprobed"]["cpu"]["sample_count"])
        self.assertEqual(40.0, result["runs"]["unprobed"]["cpu"]["mean_percent"])
        self.assertEqual(
            1.0, result["cpu_full_vs_unprobed"]["absolute_delta"]
        )
        self.assertEqual(
            2.0, result["cpu_full_vs_unprobed"]["threshold"]
        )
        self.assertEqual(
            1200,
            result["callback_full_vs_callback_only"]["callback_only_count"],
        )
        quality = CALIBRATION.quality_values(result)
        self.assertTrue(quality["cpu_gate_pass"])
        self.assertTrue(quality["callback_p99_gate_pass"])
        self.assertTrue(quality["gate_pass"])

    def test_stage_calibration_rejects_missing_closure_before_pidstat(self) -> None:
        runs = self.make_calibration_runs()
        manifest_path = runs[0] / "run-manifest.txt"
        manifest_path.write_text(
            "\n".join(
                line
                for line in manifest_path.read_text(encoding="utf-8").splitlines()
                if not line.startswith("workspace_closure_manifest_sha256=")
            )
            + "\n",
            encoding="utf-8",
        )
        (runs[0] / "pidstat.txt").unlink()
        with self.assertRaisesRegex(
            ValueError, "workspace_closure_manifest_sha256"
        ):
            CALIBRATION.analyze_calibration(*runs)

    def test_preserved_mixed_build_calibration_is_rejected_for_missing_closure(self) -> None:
        runs = tuple(
            Path(path)
            for path in (
                "/tmp/alien-c2-calibration-unprobed-20260728",
                "/tmp/alien-c2-calibration-callback-20260728",
                "/tmp/alien-c2-calibration-full-20260728",
            )
        )
        if not all(path.is_dir() for path in runs):
            self.skipTest("preserved mixed-build evidence is not available")
        with self.assertRaisesRegex(ValueError, "closure"):
            CALIBRATION.analyze_calibration(*runs)

    def test_stage_calibration_rejects_role_duration_and_callback_count_errors(self) -> None:
        runs = self.make_calibration_runs()
        with self.assertRaisesRegex(ValueError, "event-set role is incorrect"):
            CALIBRATION.analyze_calibration(runs[0], runs[2], runs[1])

        original = ANALYSIS.parse_manifest(runs[0] / "run-manifest.txt")
        self.append_manifest(runs[0], {"tracee_starttime": 999_999})
        with self.assertRaisesRegex(ValueError, "launcher and tracee identities differ"):
            CALIBRATION.analyze_calibration(*runs)
        self.append_manifest(
            runs[0], {"tracee_starttime": original["tracee_starttime"]}
        )
        self.append_manifest(runs[0], {"fixture_pgid": 999_999})
        with self.assertRaisesRegex(ValueError, "lacks an isolated process group"):
            CALIBRATION.analyze_calibration(*runs)

        self.tearDown()
        self.setUp()
        short_runs = self.make_calibration_runs(count=1199)
        with self.assertRaisesRegex(ValueError, "duration is below 120 seconds"):
            CALIBRATION.analyze_calibration(*short_runs)

        self.tearDown()
        self.setUp()
        low_sample_runs = self.make_calibration_runs(callback_stage_count=1199)
        with self.assertRaisesRegex(ValueError, "fewer than 1200 callbacks"):
            CALIBRATION.analyze_calibration(*low_sample_runs)

    def test_stage_calibration_cpu_gate_uses_absolute_bidirectional_delta(self) -> None:
        runs = self.make_calibration_runs()
        for measured in (43.0, 37.0):
            with self.subTest(measured=measured):
                self.write_pidstat(runs[2], 4400, 120, measured)
                result = CALIBRATION.analyze_calibration(*runs)
                comparison = result["cpu_full_vs_unprobed"]
                self.assertFalse(result["gate_pass"])
                self.assertEqual(3.0, comparison["absolute_delta"])
                self.assertEqual(2.0, comparison["threshold"])
                self.assertEqual(7.5, comparison["relative_delta_percent"])
                self.assertFalse(comparison["pass"])

    def test_stage_calibration_checks_each_callback_percentile(self) -> None:
        runs = self.make_calibration_runs()
        patterns = {
            "p50_ns": [1_000_000] * 599 + [1_200_000] * 601,
            "p95_ns": [1_000_000] * 1139 + [1_200_000] * 61,
            "p99_ns": [1_000_000] * 1187 + [1_200_000] * 13,
        }
        for percentile, durations in patterns.items():
            with self.subTest(percentile=percentile):
                self.add_stage_evidence(runs[2], "full", 1200, durations)
                result = CALIBRATION.analyze_calibration(*runs)
                comparison = result["callback_full_vs_callback_only"]["percentiles"][
                    percentile
                ]
                self.assertEqual(200_000, comparison["absolute_delta"])
                self.assertEqual(100_000, comparison["threshold"])
                self.assertFalse(comparison["pass"])
                self.assertFalse(result["gate_pass"])

    def test_stage_calibration_median_p99_passes_despite_primary_outlier(self) -> None:
        high_p99 = [1_000_000] * 1187 + [1_200_000] * 13
        runs = self.make_calibration_runs(full_callback_durations=high_p99)
        single = CALIBRATION.analyze_calibration(*runs)
        self.assertFalse(single["gate_pass"])
        self.assertEqual(2, single["schema_version"])

        callback_reps = (
            self.make_calibration_replicate(
                "calibration-callback-r2",
                event_set="callback",
                tracee_pid=4500,
                t0_ns=600_000_000_000,
            ),
            self.make_calibration_replicate(
                "calibration-callback-r3",
                event_set="callback",
                tracee_pid=4600,
                t0_ns=800_000_000_000,
            ),
        )
        full_reps = (
            self.make_calibration_replicate(
                "calibration-full-r2",
                event_set="full",
                tracee_pid=4700,
                t0_ns=1_000_000_000_000,
            ),
            self.make_calibration_replicate(
                "calibration-full-r3",
                event_set="full",
                tracee_pid=4800,
                t0_ns=1_200_000_000_000,
            ),
        )
        result = CALIBRATION.analyze_calibration(*runs, callback_reps, full_reps)
        self.assertEqual(3, result["schema_version"])
        self.assertTrue(result["gate_pass"])
        evaluation = result["p99_evaluation"]
        self.assertEqual("median_of_runs", evaluation["method"])
        self.assertEqual(3, evaluation["callback_run_count"])
        self.assertEqual(1_000_000, evaluation["callback_p99_median_ns"])
        self.assertEqual(1_000_000, evaluation["full_p99_median_ns"])
        percentiles = result["callback_full_vs_callback_only"]["percentiles"]
        self.assertTrue(percentiles["p99_ns"]["pass"])
        self.assertEqual(0, percentiles["p99_ns"]["absolute_delta"])
        quality = CALIBRATION.quality_values(result)
        self.assertEqual("median_of_runs", quality["p99_evaluation_method"])
        self.assertEqual(3, quality["p99_full_run_count"])
        self.assertTrue(quality["gate_pass"])

    def test_stage_calibration_median_p99_fails_when_majority_is_high(self) -> None:
        high_p99 = [1_000_000] * 1187 + [1_200_000] * 13
        runs = self.make_calibration_runs(full_callback_durations=high_p99)
        callback_reps = (
            self.make_calibration_replicate(
                "calibration-callback-r2",
                event_set="callback",
                tracee_pid=4500,
                t0_ns=600_000_000_000,
            ),
            self.make_calibration_replicate(
                "calibration-callback-r3",
                event_set="callback",
                tracee_pid=4600,
                t0_ns=800_000_000_000,
            ),
        )
        full_reps = (
            self.make_calibration_replicate(
                "calibration-full-r2",
                event_set="full",
                tracee_pid=4700,
                t0_ns=1_000_000_000_000,
                callback_durations=high_p99,
            ),
            self.make_calibration_replicate(
                "calibration-full-r3",
                event_set="full",
                tracee_pid=4800,
                t0_ns=1_200_000_000_000,
            ),
        )
        result = CALIBRATION.analyze_calibration(*runs, callback_reps, full_reps)
        self.assertEqual(3, result["schema_version"])
        self.assertFalse(result["gate_pass"])
        percentiles = result["callback_full_vs_callback_only"]["percentiles"]
        self.assertFalse(percentiles["p99_ns"]["pass"])
        self.assertEqual(200_000, percentiles["p99_ns"]["absolute_delta"])

    def test_stage_calibration_rejects_bad_replicate_sets(self) -> None:
        runs = self.make_calibration_runs()
        callback_rep = self.make_calibration_replicate(
            "calibration-callback-r2",
            event_set="callback",
            tracee_pid=4500,
            t0_ns=600_000_000_000,
        )
        full_rep = self.make_calibration_replicate(
            "calibration-full-r2",
            event_set="full",
            tracee_pid=4700,
            t0_ns=1_000_000_000_000,
        )
        with self.assertRaisesRegex(ValueError, "at least three"):
            CALIBRATION.analyze_calibration(*runs, (callback_rep,), (full_rep,))

        callback_rep3 = self.make_calibration_replicate(
            "calibration-callback-r3",
            event_set="callback",
            tracee_pid=4600,
            t0_ns=800_000_000_000,
        )
        with self.assertRaisesRegex(ValueError, "equal callback/full run counts"):
            CALIBRATION.analyze_calibration(
                *runs, (callback_rep, callback_rep3), (full_rep,)
            )

        full_rep3 = self.make_calibration_replicate(
            "calibration-full-r3",
            event_set="full",
            tracee_pid=4800,
            t0_ns=1_200_000_000_000,
        )
        self.append_manifest(full_rep3, {"target_sha256": "3" * 64})
        with self.assertRaisesRegex(ValueError, "replicate target provenance differs"):
            CALIBRATION.analyze_calibration(
                *runs, (callback_rep, callback_rep3), (full_rep, full_rep3)
            )

        duplicate_identity = self.make_calibration_replicate(
            "calibration-callback-duplicate",
            event_set="callback",
            tracee_pid=4300,
            t0_ns=200_000_000_000,
        )
        with self.assertRaisesRegex(ValueError, "independent evidence identities"):
            CALIBRATION.analyze_calibration(
                *runs, (callback_rep, duplicate_identity), (full_rep, full_rep3)
            )

    def test_stage_calibration_cpu_median_absorbs_unprobed_outlier(self) -> None:
        # 主 unprobed 40.0 vs full 43.0 单点差 3.0 pp 超阈；中位数比较应吸收。
        runs = self.make_calibration_runs(full_cpu=43.0)
        single = CALIBRATION.analyze_calibration(*runs)
        self.assertFalse(single["cpu_full_vs_unprobed"]["pass"])

        callback_reps = tuple(
            self.make_calibration_replicate(
                f"calibration-callback-r{i}",
                event_set="callback",
                tracee_pid=4400 + i * 100,
                t0_ns=(400 + i * 200) * 1_000_000_000,
            )
            for i in (2, 3)
        )
        full_reps = tuple(
            self.make_calibration_replicate(
                f"calibration-full-r{i}",
                event_set="full",
                tracee_pid=5000 + i * 100,
                t0_ns=(1000 + i * 200) * 1_000_000_000,
                cpu_percent=40.5,
            )
            for i in (2, 3)
        )
        unprobed_reps = tuple(
            self.make_calibration_replicate(
                f"calibration-unprobed-r{i}",
                event_set="unprobed",
                tracee_pid=5600 + i * 100,
                t0_ns=(1600 + i * 200) * 1_000_000_000,
                cpu_percent=40.25,
            )
            for i in (2, 3)
        )
        result = CALIBRATION.analyze_calibration(
            *runs, callback_reps, full_reps, unprobed_reps
        )
        self.assertEqual(4, result["schema_version"])
        evaluation = result["cpu_evaluation"]
        self.assertEqual("median_of_runs", evaluation["method"])
        self.assertEqual(3, evaluation["unprobed_run_count"])
        # unprobed 中位数 40.25，full 中位数 40.5 → 差 0.25 < max(5%,0.5)=2.0125
        self.assertEqual(40.25, evaluation["unprobed_cpu_median_percent"])
        self.assertEqual(40.5, evaluation["full_cpu_median_percent"])
        self.assertTrue(result["cpu_full_vs_unprobed"]["pass"])
        self.assertTrue(result["gate_pass"])
        quality = CALIBRATION.quality_values(result)
        self.assertEqual("median_of_runs", quality["cpu_evaluation_method"])
        self.assertEqual(3, quality["cpu_unprobed_run_count"])
        self.assertTrue(quality["gate_pass"])

        with self.assertRaisesRegex(ValueError, "three unprobed and"):
            CALIBRATION.analyze_calibration(
                *runs, callback_reps, full_reps, unprobed_reps[:1]
            )

        # unprobed replicate 必须与主 unprobed 目标完全一致（含 stage 选项 OFF）。
        self.append_manifest(
            unprobed_reps[0], {"target_stage_latency_option": "ON"}
        )
        with self.assertRaisesRegex(ValueError, "replicate target provenance differs"):
            CALIBRATION.analyze_calibration(
                *runs, callback_reps, full_reps, unprobed_reps
            )

    def test_stage_calibration_cpu_median_fails_when_majority_high(self) -> None:
        runs = self.make_calibration_runs(full_cpu=43.0)
        callback_reps = tuple(
            self.make_calibration_replicate(
                f"calibration-callback-r{i}",
                event_set="callback",
                tracee_pid=4400 + i * 100,
                t0_ns=(400 + i * 200) * 1_000_000_000,
            )
            for i in (2, 3)
        )
        full_reps = tuple(
            self.make_calibration_replicate(
                f"calibration-full-r{i}",
                event_set="full",
                tracee_pid=5000 + i * 100,
                t0_ns=(1000 + i * 200) * 1_000_000_000,
                cpu_percent=43.5,
            )
            for i in (2, 3)
        )
        unprobed_reps = tuple(
            self.make_calibration_replicate(
                f"calibration-unprobed-r{i}",
                event_set="unprobed",
                tracee_pid=5600 + i * 100,
                t0_ns=(1600 + i * 200) * 1_000_000_000,
                cpu_percent=40.0,
            )
            for i in (2, 3)
        )
        result = CALIBRATION.analyze_calibration(
            *runs, callback_reps, full_reps, unprobed_reps
        )
        # unprobed 中位数 40.0，full 中位数 43.0 → 差 3.0 > 2.0 → fail
        self.assertEqual(4, result["schema_version"])
        self.assertFalse(result["cpu_full_vs_unprobed"]["pass"])
        self.assertFalse(result["gate_pass"])

    def test_stage_calibration_rejects_pidstat_scope_pid_and_sample_errors(self) -> None:
        runs = self.make_calibration_runs()
        self.write_pidstat(runs[0], 9999, 120, 40.0)
        with self.assertRaisesRegex(ValueError, "only to the tracee PID"):
            CALIBRATION.analyze_calibration(*runs)

        self.write_pidstat(runs[0], 4200, 10, 40.0)
        with self.assertRaisesRegex(ValueError, "sample count"):
            CALIBRATION.analyze_calibration(*runs)

        self.write_pidstat(runs[0], 4200, 120, 40.0)
        self.append_manifest(
            runs[0], {"pidstat_start_monotonic_ns": 7_000_000_000}
        )
        with self.assertRaisesRegex(ValueError, "tightly bracket"):
            CALIBRATION.analyze_calibration(*runs)

        self.append_manifest(
            runs[0],
            {
                "pidstat_start_monotonic_ns": 9_500_000_000,
                "pidstat_interval_s": 2,
            },
        )
        with self.assertRaisesRegex(ValueError, "exactly one second"):
            CALIBRATION.analyze_calibration(*runs)
        self.append_manifest(
            runs[0],
            {
                "pidstat_interval_s": 1,
                "pidstat_stop_monotonic_ns": 133_000_000_000,
            },
        )
        with self.assertRaisesRegex(ValueError, "tightly bracket"):
            CALIBRATION.analyze_calibration(*runs)
        self.append_manifest(
            runs[0], {"pidstat_stop_monotonic_ns": 130_500_000_000}
        )

        manifest_path = runs[0] / "run-manifest.txt"
        text = manifest_path.read_text(encoding="utf-8")
        manifest_path.write_text(
            "\n".join(
                line
                for line in text.splitlines()
                if not line.startswith("pidstat_start_monotonic_ns=")
            )
            + "\n",
            encoding="utf-8",
        )
        (runs[0] / "pidstat.txt").unlink()
        with self.assertRaisesRegex(ValueError, "pidstat_start_monotonic_ns"):
            CALIBRATION.analyze_calibration(*runs)

    def test_stage_calibration_rejects_build_source_and_config_mismatch(self) -> None:
        runs = self.make_calibration_runs()
        cases = (
            ("target_build_id", "d" * 40, "target provenance differs"),
            ("source_diff_sha256", "d" * 64, "source_diff_sha256"),
            ("workload_yaml_sha256", "e" * 64, "workload_yaml_sha256"),
            ("cpu_allowed", "0-20", "cpu_allowed"),
            ("target_affinity", "0-1", "target_affinity"),
            ("install_prefix", "/tmp/other/install", "target provenance differs"),
            (
                "paired_source_identity_sha256",
                "d" * 64,
                "paired_source_identity_sha256",
            ),
            (
                "workspace_dependency_comparison_sha256",
                "e" * 64,
                "workspace_dependency_comparison_sha256",
            ),
            (
                "profiling_helper_set_sha256",
                "f" * 64,
                "profiling_helper_set_sha256",
            ),
        )
        original = ANALYSIS.parse_manifest(runs[1] / "run-manifest.txt")
        for key, value, message in cases:
            with self.subTest(key=key):
                self.append_manifest(runs[1], {key: value})
                with self.assertRaisesRegex(ValueError, message):
                    CALIBRATION.analyze_calibration(*runs)
                self.append_manifest(runs[1], {key: original[key]})

        manifest_path = runs[2] / "run-manifest.txt"
        text = manifest_path.read_text(encoding="utf-8")
        manifest_path.write_text(
            "\n".join(
                line
                for line in text.splitlines()
                if not line.startswith("target_compile_flags_verified=")
            )
            + "\n",
            encoding="utf-8",
        )
        with self.assertRaisesRegex(ValueError, "target_compile_flags_verified"):
            CALIBRATION.analyze_calibration(*runs)

        self.append_manifest(runs[2], {"target_compile_flags_verified": "true"})
        self.append_manifest(runs[0], {"target_build_type": "Release"})
        with self.assertRaisesRegex(ValueError, "not a RelWithDebInfo build"):
            CALIBRATION.analyze_calibration(*runs)

        self.append_manifest(runs[0], {"target_build_type": "RelWithDebInfo"})
        self.append_manifest(runs[1], {"target_stage_latency_option": "OFF"})
        with self.assertRaisesRegex(ValueError, "must record the stage latency option as ON"):
            CALIBRATION.analyze_calibration(*runs)

    def test_stage_calibration_binds_runtime_source_and_workload_to_closure(self) -> None:
        runs = self.make_calibration_runs()
        for run_dir in runs:
            self.append_manifest(run_dir, {"source_diff_sha256": "d" * 64})
        with self.assertRaisesRegex(ValueError, "source_diff_sha256.*mismatch"):
            CALIBRATION.analyze_calibration(*runs)

        self.tearDown()
        self.setUp()
        runs = self.make_calibration_runs()
        (runs[0] / "source-diff.patch").write_bytes(b"mutated diff\n")
        with self.assertRaisesRegex(ValueError, "source artifact source-diff.patch"):
            CALIBRATION.analyze_calibration(*runs)

        self.tearDown()
        self.setUp()
        runs = self.make_calibration_runs()
        for run_dir in runs:
            self.append_manifest(run_dir, {"workload_yaml_sha256": "e" * 64})
        with self.assertRaisesRegex(ValueError, "workload_yaml_sha256.*mismatch"):
            CALIBRATION.analyze_calibration(*runs)

    def test_stage_calibration_cli_writes_json_quality_and_fails_closed(self) -> None:
        runs = self.make_calibration_runs()
        output = self.root / "calibration.json"
        quality = self.root / "calibration-quality.txt"
        result = subprocess.run(
            [
                sys.executable,
                str(CALIBRATION_CLI_PATH),
                *(str(path) for path in runs),
                "--output",
                str(output),
                "--quality",
                str(quality),
            ],
            text=True,
            capture_output=True,
            check=False,
        )
        self.assertEqual(0, result.returncode, result.stderr)
        self.assertTrue(json.loads(output.read_text(encoding="utf-8"))["gate_pass"])
        self.assertIn("gate_pass=true", quality.read_text(encoding="utf-8"))

        stale_output_hash = hashlib.sha256(output.read_bytes()).hexdigest()
        stale_quality_hash = hashlib.sha256(quality.read_bytes()).hexdigest()
        result = subprocess.run(
            [
                sys.executable,
                str(CALIBRATION_CLI_PATH),
                *(str(path) for path in runs),
                "--output",
                str(output),
                "--quality",
                str(quality),
            ],
            text=True,
            capture_output=True,
            check=False,
        )
        self.assertEqual(2, result.returncode)
        self.assertIn("output path already exists", result.stderr)
        self.assertEqual(stale_output_hash, hashlib.sha256(output.read_bytes()).hexdigest())
        self.assertEqual(stale_quality_hash, hashlib.sha256(quality.read_bytes()).hexdigest())

        invalid_output = self.root / "invalid-calibration.json"
        invalid_quality = self.root / "invalid-calibration-quality.txt"
        result = subprocess.run(
            [
                sys.executable,
                str(CALIBRATION_CLI_PATH),
                str(runs[0]),
                str(runs[2]),
                str(runs[1]),
                "--output",
                str(invalid_output),
                "--quality",
                str(invalid_quality),
            ],
            text=True,
            capture_output=True,
            check=False,
        )
        self.assertEqual(2, result.returncode)
        self.assertIn("event-set role is incorrect", result.stderr)
        self.assertFalse(invalid_output.exists())
        self.assertFalse(invalid_quality.exists())

        self.tearDown()
        self.setUp()
        failing_runs = self.make_calibration_runs(
            unprobed_cpu=40.0, full_cpu=43.0
        )
        failed_output = self.root / "failed-calibration.json"
        failed_quality = self.root / "failed-calibration-quality.txt"
        result = subprocess.run(
            [
                sys.executable,
                str(CALIBRATION_CLI_PATH),
                *(str(path) for path in failing_runs),
                "--output",
                str(failed_output),
                "--quality",
                str(failed_quality),
            ],
            text=True,
            capture_output=True,
            check=False,
        )
        self.assertEqual(1, result.returncode, result.stderr)
        self.assertFalse(json.loads(failed_output.read_text(encoding="utf-8"))["gate_pass"])
        self.assertIn("schema_version=2", failed_quality.read_text(encoding="utf-8"))
        self.assertIn("gate_pass=false", failed_quality.read_text(encoding="utf-8"))

    def test_three_run_aggregate_rejects_duplicate_evidence(self) -> None:
        runs = [
            self.make_run(f"run-{index}", tracee_pid=4200 + index, t0_ns=1_000_000_000 + index * 20_000_000_000)
            for index in range(3)
        ]
        results = [ANALYSIS.analyze_run(path) for path in runs]
        aggregate = ANALYSIS.aggregate_runs(results)
        self.assertEqual(3, aggregate["run_count"])
        self.assertEqual([40.0, 40.0, 40.0], aggregate["cpu_percent"]["mean_percent"])
        self.assertFalse(aggregate["suspected_sustained_growth"])
        results[2]["tracee_pid"] = results[0]["tracee_pid"]
        results[2]["t0_monotonic_ns"] = results[0]["t0_monotonic_ns"]
        results[2]["t1_monotonic_ns"] = results[0]["t1_monotonic_ns"]
        with self.assertRaisesRegex(ValueError, "duplicate evidence"):
            ANALYSIS.aggregate_runs(results)

    def test_three_run_aggregate_rejects_mixed_c3_modes(self) -> None:
        runs = [
            self.make_run(
                f"mixed-c3-{index}",
                count=20,
                c3_mode="enabled" if index == 2 else "disabled",
                tracee_pid=4300 + index,
                t0_ns=1_000_000_000 + index * 10_000_000_000,
            )
            for index in range(3)
        ]
        results = [ANALYSIS.analyze_run(path) for path in runs]
        with self.assertRaisesRegex(ValueError, "one C3 mode"):
            ANALYSIS.aggregate_runs(results)

    def test_sampler_parsers_and_capacity_safety(self) -> None:
        memory = SAMPLER.parse_smaps_rollup(
            "Rss: 100 kB\nPss: 80 kB\nPrivate_Clean: 20 kB\n"
            "Private_Dirty: 30 kB\nPrivate_Hugetlb: 4 kB\n"
        )
        self.assertEqual((100, 80, 54), (memory.rss_kib, memory.pss_kib, memory.uss_kib))
        baseline = SAMPLER.Headroom(100, 1000, 5_000_000, 0, 0)
        self.assertIsNone(SAMPLER.safety_reason(baseline, baseline))
        self.assertEqual(
            "cgroup_memory_at_or_above_80_percent",
            SAMPLER.safety_reason(SAMPLER.Headroom(800, 1000, 5_000_000, 0, 0), baseline),
        )
        self.assertEqual(
            "host_memavailable_at_or_below_2_gib",
            SAMPLER.safety_reason(SAMPLER.Headroom(100, 1000, 2_000_000, 0, 0), baseline),
        )
        self.assertEqual(
            "cgroup_oom_kill_incremented",
            SAMPLER.safety_reason(SAMPLER.Headroom(100, 1000, 5_000_000, 0, 1), baseline),
        )
        rows = [
            {"receipt_monotonic_ns": "99", "revision": "800"},
            {"receipt_monotonic_ns": "100", "revision": "801"},
        ]
        self.assertEqual(
            [rows[1]],
            SAMPLER.states_received_at_or_after(rows, 100),
        )
        with self.assertRaisesRegex(ValueError, "invalid receipt_monotonic_ns"):
            SAMPLER.states_received_at_or_after(
                [{"receipt_monotonic_ns": "invalid"}], 100
            )

    def test_cli_and_runner_reject_unknown_or_existing_output(self) -> None:
        result = subprocess.run(
            ["bash", str(RUNNER_PATH)], text=True, capture_output=True, check=False
        )
        self.assertEqual(2, result.returncode)
        self.assertIn("[bounded|expanding]", result.stderr)
        self.assertIn("[callback|full]", result.stderr)
        result = subprocess.run(
            ["bash", str(RUNNER_PATH), "unknown", "/tmp", str(self.root), "1"],
            text=True,
            capture_output=True,
            check=False,
        )
        self.assertEqual(2, result.returncode)

        result = subprocess.run(
            [
                "bash",
                str(RUNNER_PATH),
                "stage-latency",
                "/tmp",
                str(self.root / "invalid-stage-set"),
                "1",
                "bounded",
                "invalid",
            ],
            text=True,
            capture_output=True,
            check=False,
        )
        self.assertEqual(2, result.returncode)
        self.assertIn("event set must be callback or full", result.stderr)

        result = subprocess.run(
            [
                "bash",
                str(RUNNER_PATH),
                "plain-sample",
                "/tmp",
                str(self.root / "non-stage-set"),
                "1",
                "bounded",
                "callback",
            ],
            text=True,
            capture_output=True,
            check=False,
        )
        self.assertEqual(2, result.returncode)
        self.assertIn("valid only for stage-latency", result.stderr)

        runner = RUNNER_PATH.read_text(encoding="utf-8")
        self.assertIn('source "${PROFILE_RUNNER_COMMON}"', runner)
        self.assertIn("capacity-ramp", runner)
        self.assertIn("scan_accumulator|octomap_builder", runner)
        self.assertIn("cloud_map", runner)
        self.assertIn("normal_completion=${NORMAL_COMPLETION}", runner)
        self.assertIn("source_diff_sha256=", runner)
        self.assertNotIn("ros2 trace --list", runner)
        self.assertIn("list_ros_trace_events", runner)
        self.assertNotIn(
            'wait_for_process_identity "${LAUNCHER_PID}" heaptrack', runner
        )
        self.assertIn(
            'TRACEE_PID="$(find_matching_descendant "${LAUNCHER_PID}" || true)"',
            runner,
        )
        self.assertIn("process_cmdline_has_exact_arguments", runner)
        self.assertIn("heaptrack_process_model=", runner)
        self.assertIn(
            'heaptrack_print -f "${files[0]}" -M '
            '"${OUTPUT_DIR}/heaptrack-massif.out"',
            runner,
        )
        self.assertNotIn('heaptrack_print -M "${files[0]}"', runner)
        self.assertIn('"${PROFILE_REPORT_PARSER}" heaptrack-massif', runner)
        self.assertIn("TARGET_IDENTITY_RECORDED=false", runner)
        self.assertIn("stop_partial_target_startup", runner)
        self.assertIn(
            'if [[ "${TARGET_IDENTITY_RECORDED}" == true ]]; then', runner
        )
        stop_start = runner.index("stop_perf()\n{")
        stop_end = runner.index("\nstart_trace()", stop_start)
        stop_perf = runner[stop_start:stop_end]
        disable_index = stop_perf.index("perf_control disable")
        stop_index = stop_perf.index("perf_control stop")
        close_index = stop_perf.index("exec 8>&- 9>&-")
        signal_index = stop_perf.index(
            'signal_process INT "${TOOL_PID}" "${TOOL_PGID}" "${TOOL_STARTTIME}"'
        )
        timeout_index = stop_perf.index('if ! wait_for_dead "${TOOL_PID}" 20; then')
        forced_index = stop_perf.index("FORCED_STOP=true")
        reap_index = stop_perf.index('wait_child "${TOOL_PID}" perf || true')
        self.assertLess(disable_index, stop_index)
        self.assertLess(stop_index, close_index)
        self.assertLess(close_index, signal_index)
        self.assertLess(signal_index, timeout_index)
        self.assertLess(timeout_index, forced_index)
        self.assertLess(forced_index, reap_index)
        self.assertIn(
            'signal_process INT "${TOOL_PID}" "${TOOL_PGID}" "${TOOL_STARTTIME}"',
            stop_perf,
        )
        self.assertIn("if (( rc != 0 && rc != 130 )); then", stop_perf)
        self.assertIn("ROLE_EXIT_FAILURE=true", stop_perf)
        self.assertIn('invalidate "perf exited abnormally (${rc})"', stop_perf)
        self.assertNotIn("tool_exit_code=", stop_perf)
        common = SHELL_COMMON_PATH.read_text(encoding="utf-8")
        self.assertIn(
            'process_identity_matches "${pid}" "${starttime}" "${pgid}"', common
        )

    def test_analyzer_cli_writes_machine_readable_summary(self) -> None:
        run_dir = self.make_run("cli")
        output = self.root / "summary.json"
        result = subprocess.run(
            [sys.executable, str(ANALYZER_PATH), str(run_dir), "--output", str(output)],
            text=True,
            capture_output=True,
            check=False,
        )
        self.assertEqual(0, result.returncode, result.stderr)
        payload = json.loads(output.read_text(encoding="utf-8"))
        self.assertEqual("bounded", payload["runs"][0]["workload"])


if __name__ == "__main__":
    unittest.main()
