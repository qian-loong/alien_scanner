#!/usr/bin/env python3

import argparse
import csv
import hashlib
import json
import math
import os
import platform
import re
import shutil
import signal
import statistics
import subprocess
import sys
import time
from pathlib import Path


def parse_args():
    parser = argparse.ArgumentParser(description="Run the isolated C4 receiver resource workload")
    parser.add_argument("--source", required=True, type=Path)
    parser.add_argument("--receiver", required=True, type=Path)
    parser.add_argument(
        "--receiver-tool", choices=("plain", "heaptrack", "memcheck"), default="plain"
    )
    parser.add_argument(
        "--report-parser",
        type=Path,
        default=Path(__file__).resolve().parents[4] / "scripts/lib/profile_report_parsers.py",
    )
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument(
        "--workload-mode",
        choices=("bounded", "expanding", "keyframe-replacement"),
        default="bounded",
    )
    parser.add_argument("--source-count", type=int, default=2)
    parser.add_argument("--cells-per-source", type=int, default=10_000)
    parser.add_argument("--initial-cells-per-source", type=int, default=10_000)
    parser.add_argument("--delta-operations", type=int, default=256)
    parser.add_argument("--rate-hz-per-source", type=float, default=10.0)
    parser.add_argument("--qos-depth", type=int, default=4)
    parser.add_argument("--storage-mode", choices=("vector", "chunked"), default="vector")
    parser.add_argument("--chunk-edge", type=int, choices=(8, 16, 32), default=16)
    parser.add_argument("--chunk-bucket-count", type=int, default=256)
    parser.add_argument("--warmup-s", type=float, default=3.0)
    parser.add_argument("--ready-timeout-s", type=float, default=60.0)
    parser.add_argument("--window-s", type=float, default=30.0)
    parser.add_argument("--sample-period-s", type=float, default=1.0)
    parser.add_argument("--domain-id", type=int, default=174)
    parser.add_argument("--formal", action="store_true")
    parser.add_argument("--workspace-root", type=Path)
    parser.add_argument("--image-id", default=os.environ.get("ALIEN_PROFILE_IMAGE_ID", ""))
    return parser.parse_args()


def require(condition, message):
    if not condition:
        raise ValueError(message)


def validate(args):
    require(args.source.is_file(), f"source executable does not exist: {args.source}")
    require(args.receiver.is_file(), f"receiver executable does not exist: {args.receiver}")
    if args.receiver_tool == "heaptrack":
        require(shutil.which("heaptrack") is not None, "heaptrack is not installed")
        require(shutil.which("heaptrack_print") is not None, "heaptrack_print is not installed")
        require(args.report_parser.is_file(), f"missing report parser: {args.report_parser}")
    if args.receiver_tool == "memcheck":
        require(shutil.which("valgrind") is not None, "valgrind is not installed")
        require(Path("/usr/bin/valgrind.bin").is_file(), "valgrind.bin is not installed")
        require(args.report_parser.is_file(), f"missing report parser: {args.report_parser}")
    require(1 <= args.source_count <= 64, "source-count must be in [1, 64]")
    require(args.cells_per_source > 0, "cells-per-source must be positive")
    require(args.initial_cells_per_source > 0, "initial-cells-per-source must be positive")
    if args.workload_mode == "expanding":
        require(
            args.initial_cells_per_source <= args.cells_per_source,
            "expanding initial-cells-per-source must not exceed cells-per-source",
        )
    require(
        0 < args.delta_operations <= args.cells_per_source,
        "delta-operations must be positive and no larger than cells-per-source",
    )
    require(math.isfinite(args.rate_hz_per_source) and args.rate_hz_per_source > 0,
            "rate-hz-per-source must be finite and positive")
    require(args.qos_depth > 0, "qos-depth must be positive")
    require(
        0 < args.chunk_bucket_count <= (1 << 63) - 1,
        "chunk-bucket-count must fit a positive int64",
    )
    require(math.isfinite(args.warmup_s) and args.warmup_s >= 0, "warmup-s is invalid")
    require(
        math.isfinite(args.ready_timeout_s) and args.ready_timeout_s > 0,
        "ready-timeout-s is invalid",
    )
    require(math.isfinite(args.window_s) and args.window_s > 0, "window-s is invalid")
    require(
        math.isfinite(args.sample_period_s) and args.sample_period_s > 0,
        "sample-period-s is invalid",
    )
    if args.formal:
        require(
            args.receiver_tool == "plain",
            "formal sustained-growth runs must use an uninstrumented receiver",
        )
        require(args.window_s >= 300.0, "formal windows must be at least 300 seconds")
        require(
            args.workload_mode == "bounded",
            "formal sustained-growth runs must use the bounded workload",
        )
        require(args.workspace_root is not None, "formal runs require --workspace-root")
        require(
            re.fullmatch(r"sha256:[0-9a-f]{64}", args.image_id) is not None,
            "formal runs require --image-id in sha256:<64 hex> form",
        )


def prepare_output(args):
    if args.output_dir is None:
        import tempfile

        return Path(tempfile.mkdtemp(prefix="c4-resource-profile-"))
    output = args.output_dir.resolve()
    if output.exists() and any(output.iterdir()):
        raise ValueError(f"output directory is not empty: {output}")
    output.mkdir(parents=True, exist_ok=True)
    return output


def ros_args(parameters):
    result = ["--ros-args"]
    for key, value in parameters.items():
        result.extend(["-p", f"{key}:={value}"])
    return result


def proc_identity(pid):
    proc = Path("/proc") / str(pid)
    stat_fields = (proc / "stat").read_text(encoding="utf-8").rpartition(")")[2].split()
    return {
        "pid": pid,
        "state": stat_fields[0],
        "process_group_id": int(stat_fields[2]),
        "starttime_ticks": int(stat_fields[19]),
        "exe": str((proc / "exe").resolve()),
        "cmdline": [part.decode("utf-8") for part in (proc / "cmdline").read_bytes().split(b"\0") if part],
    }


def same_process(identity):
    try:
        current = proc_identity(identity["pid"])
        return (
            current["starttime_ticks"] == identity["starttime_ticks"]
            and current["process_group_id"] == identity["process_group_id"]
            and current["state"] != "Z"
        )
    except (FileNotFoundError, ProcessLookupError):
        return False


def require_live_process(identity, role):
    if not same_process(identity):
        raise RuntimeError(f"{role} PID/starttime/process-group identity changed")


def wait_for_process_identity(process, expected_exe, role, timeout=5.0):
    deadline = time.monotonic() + timeout
    expected = str(expected_exe.resolve())
    last_error = None
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise RuntimeError(f"{role} exited before its identity stabilized")
        try:
            identity = proc_identity(process.pid)
            if (
                identity["state"] != "Z"
                and identity["exe"] == expected
                and identity["cmdline"]
                and identity["cmdline"][0] == expected
            ):
                return identity
            last_error = identity
        except (FileNotFoundError, ProcessLookupError) as error:
            last_error = str(error)
        time.sleep(0.05)
    raise RuntimeError(f"{role} identity did not stabilize: {last_error}")


def direct_children(pid):
    children = []
    for candidate in Path("/proc").iterdir():
        if not candidate.name.isdigit():
            continue
        try:
            fields = (candidate / "stat").read_text(encoding="utf-8").rpartition(")")[2].split()
            if int(fields[1]) == pid and fields[0] != "Z":
                children.append(proc_identity(int(candidate.name)))
        except (FileNotFoundError, PermissionError, ProcessLookupError, ValueError):
            continue
    return children


def wait_for_unique_child_exe(parent, expected_exe, role, timeout=10.0):
    deadline = time.monotonic() + timeout
    expected = str(expected_exe.resolve())
    last_matches = []
    while time.monotonic() < deadline:
        if parent.poll() is not None:
            raise RuntimeError(f"{role} launcher exited before tracee identity stabilized")
        matches = [child for child in direct_children(parent.pid) if child["exe"] == expected]
        last_matches = matches
        if len(matches) == 1 and matches[0]["cmdline"]:
            return matches[0]
        if len(matches) > 1:
            raise RuntimeError(f"{role} launcher has multiple target descendants: {matches}")
        time.sleep(0.05)
    raise RuntimeError(f"{role} tracee identity did not stabilize: {last_matches}")


def wait_for_launcher_arguments(process, required_arguments, role, timeout=5.0):
    deadline = time.monotonic() + timeout
    last_identity = None
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise RuntimeError(f"{role} launcher exited before its identity stabilized")
        try:
            identity = proc_identity(process.pid)
            last_identity = identity
            if identity["state"] != "Z" and all(
                argument in identity["cmdline"] for argument in required_arguments
            ):
                return identity
        except (FileNotFoundError, ProcessLookupError):
            pass
        time.sleep(0.05)
    raise RuntimeError(f"{role} launcher arguments did not stabilize: {last_identity}")


def wait_for_launcher_exit(process, identity, timeout=30.0):
    try:
        return process.wait(timeout=timeout)
    except subprocess.TimeoutExpired as error:
        if same_process(identity):
            os.killpg(process.pid, signal.SIGTERM)
        raise RuntimeError("receiver instrumentation launcher did not finalize") from error


def stop_instrumented_receiver(process, launcher_identity, tracee_identity):
    if process.poll() is not None:
        raise RuntimeError("receiver instrumentation launcher exited before normal stop")
    require_live_process(launcher_identity, "receiver instrumentation launcher")
    require_live_process(tracee_identity, "receiver tracee")
    os.kill(tracee_identity["pid"], signal.SIGINT)
    return wait_for_launcher_exit(process, launcher_identity)


def read_smaps(pid):
    values = {}
    for line in (Path("/proc") / str(pid) / "smaps_rollup").read_text(encoding="utf-8").splitlines():
        fields = line.split()
        if len(fields) >= 2 and fields[0].endswith(":") and fields[1].isdigit():
            values[fields[0][:-1]] = int(fields[1])
    private_kib = values.get("Private_Clean", 0) + values.get("Private_Dirty", 0)
    return {
        "rss_kib": values.get("Rss", 0),
        "pss_kib": values.get("Pss", 0),
        "uss_kib": private_kib,
        "private_clean_kib": values.get("Private_Clean", 0),
        "private_dirty_kib": values.get("Private_Dirty", 0),
        "shared_clean_kib": values.get("Shared_Clean", 0),
        "shared_dirty_kib": values.get("Shared_Dirty", 0),
    }


def read_cpu_ticks(pid):
    fields = (Path("/proc") / str(pid) / "stat").read_text(encoding="utf-8").rpartition(")")[2].split()
    return int(fields[11]) + int(fields[12])


def read_smem(pid):
    completed = subprocess.run(
        ["smem", "-H", "-c", "pid rss pss uss"],
        check=False,
        capture_output=True,
        text=True,
    )
    if completed.returncode != 0:
        return {"returncode": completed.returncode, "raw": completed.stdout + completed.stderr}
    for raw in completed.stdout.splitlines():
        fields = raw.split()
        if len(fields) == 4 and fields[0].isdigit() and int(fields[0]) == pid:
            return {
                "returncode": 0,
                "pid": pid,
                "rss_kib": int(fields[1]),
                "pss_kib": int(fields[2]),
                "uss_kib": int(fields[3]),
                "raw": raw,
            }
    return {"returncode": 0, "pid": pid, "raw": completed.stdout, "missing_pid": True}


def sha256_file(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def command_output(command):
    completed = subprocess.run(command, check=False, capture_output=True, text=True)
    return {
        "command": command,
        "returncode": completed.returncode,
        "stdout": completed.stdout,
        "stderr": completed.stderr,
    }


def elf_provenance(path):
    resolved = path.resolve()
    readelf = command_output(["readelf", "-n", str(resolved)])
    build_id_match = re.search(r"Build ID:\s*([0-9a-f]+)", readelf["stdout"])
    ldd = command_output(["ldd", str(resolved)])
    return {
        "path": str(resolved),
        "sha256": sha256_file(resolved),
        "build_id": build_id_match.group(1) if build_id_match else None,
        "readelf_returncode": readelf["returncode"],
        "ldd_returncode": ldd["returncode"],
        "ldd": ldd["stdout"],
    }


def find_compile_command(executable, source_name):
    candidates = []
    for parent in (executable.resolve().parent, *executable.resolve().parents):
        candidate = parent / "compile_commands.json"
        if candidate.is_file():
            candidates.append(candidate)
        if len(candidates) >= 2:
            break
    for candidate in candidates:
        try:
            entries = json.loads(candidate.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            continue
        for entry in entries:
            if Path(entry.get("file", "")).name == source_name:
                return {"database": str(candidate), "entry": entry}
    return None


def write_artifact_hashes(output):
    rows = []
    for path in sorted(output.iterdir(), key=lambda value: value.name):
        if path.is_file() and path.name != "artifact-sha256.tsv":
            rows.append((path.name, sha256_file(path)))
    with (output / "artifact-sha256.tsv").open("w", encoding="utf-8") as stream:
        for name, digest in rows:
            stream.write(f"{digest}\t{name}\n")


def parse_kv(path):
    values = {}
    with path.open(encoding="utf-8") as stream:
        for raw in stream:
            key, value = raw.rstrip("\n").split("\t", 1)
            values[key] = int(value) if value.isdigit() else value
    return values


def parse_equals_kv(path):
    values = {}
    with path.open(encoding="utf-8") as stream:
        for raw in stream:
            key, value = raw.rstrip("\n").split("=", 1)
            if value.isdigit():
                values[key] = int(value)
            else:
                try:
                    values[key] = float(value)
                except ValueError:
                    values[key] = value
    return values


def sanitizer_evidence(environment, output, source_elf, receiver_elf):
    asan_options = environment.get("ASAN_OPTIONS", "")
    lsan_options = environment.get("LSAN_OPTIONS", "")
    enabled = (
        "libasan.so" in source_elf["ldd"]
        and "libasan.so" in receiver_elf["ldd"]
    )
    reports = sorted(
        path.name
        for path in output.iterdir()
        if path.is_file() and (path.name.startswith("asan.") or path.name.startswith("lsan."))
    )
    required_options = (
        "detect_leaks=1",
        "halt_on_error=1",
        "exitcode=23",
    )
    configured = not enabled or all(option in asan_options for option in required_options)
    return {
        "enabled": enabled,
        "configured": configured,
        "asan_options": asan_options or None,
        "lsan_options": lsan_options or None,
        "report_files": reports,
        "gate_pass": configured and not reports,
    }


def generate_heaptrack_report(args, output):
    artifacts = sorted(output.glob("heaptrack*.gz"))
    require(len(artifacts) == 1, "Heaptrack must produce exactly one primary artifact")
    artifact = artifacts[0]
    report = output / "heaptrack-report.txt"
    timeline = output / "heaptrack-massif.out"
    report_quality = output / "heaptrack-quality.txt"
    timeline_quality = output / "heaptrack-massif-quality.txt"

    with report.open("w", encoding="utf-8") as stream:
        completed = subprocess.run(
            ["heaptrack_print", str(artifact)],
            check=False,
            stdout=stream,
            stderr=subprocess.STDOUT,
            text=True,
        )
    require(completed.returncode == 0, "heaptrack_print failed")
    completed = subprocess.run(
        ["heaptrack_print", "-f", str(artifact), "-M", str(timeline)],
        check=False,
        capture_output=True,
        text=True,
    )
    (output / "heaptrack-massif.stdout.log").write_text(
        completed.stdout, encoding="utf-8"
    )
    (output / "heaptrack-massif.stderr.log").write_text(
        completed.stderr, encoding="utf-8"
    )
    require(completed.returncode == 0, "heaptrack Massif timeline generation failed")
    requested_duration = max(1, int(args.window_s))
    commands = (
        [
            sys.executable,
            str(args.report_parser),
            "heaptrack",
            str(report),
            str(report_quality),
            str(requested_duration),
        ],
        [
            sys.executable,
            str(args.report_parser),
            "heaptrack-massif",
            str(timeline),
            str(timeline_quality),
            str(args.receiver.resolve()),
            str(requested_duration),
        ],
    )
    for command in commands:
        completed = subprocess.run(command, check=False, capture_output=True, text=True)
        if completed.returncode != 0:
            raise RuntimeError(
                f"Heaptrack report quality gate failed: {completed.stderr.strip()}"
            )
    return {
        "primary_artifact": str(artifact),
        "primary_artifact_sha256": sha256_file(artifact),
        "report": parse_equals_kv(report_quality),
        "timeline": parse_equals_kv(timeline_quality),
    }


def generate_memcheck_report(args, output):
    artifacts = sorted(output.glob("memcheck.*.log"))
    require(len(artifacts) == 1, "Memcheck must produce exactly one primary log")
    artifact = artifacts[0]
    summary = output / "memcheck-summary.txt"
    quality = output / "memcheck-quality.txt"
    completed = subprocess.run(
        [
            sys.executable,
            str(args.report_parser),
            "memcheck",
            str(artifact),
            str(summary),
            str(quality),
            str(args.receiver.resolve()),
        ],
        check=False,
        capture_output=True,
        text=True,
    )
    if completed.returncode != 0:
        raise RuntimeError(f"Memcheck report parser failed: {completed.stderr.strip()}")
    return {
        "primary_artifact": str(artifact),
        "primary_artifact_sha256": sha256_file(artifact),
        "report": parse_equals_kv(quality),
    }


def parse_sources(path):
    with path.open(newline="", encoding="utf-8") as stream:
        return {row["source"]: row for row in csv.DictReader(stream, delimiter="\t")}


def slope_kib_per_minute(samples, field):
    if len(samples) < 2:
        return 0.0
    xs = [sample["elapsed_s"] / 60.0 for sample in samples]
    ys = [float(sample[field]) for sample in samples]
    x_mean = statistics.fmean(xs)
    y_mean = statistics.fmean(ys)
    denominator = sum((x - x_mean) ** 2 for x in xs)
    if denominator == 0.0:
        return 0.0
    return sum((x - x_mean) * (y - y_mean) for x, y in zip(xs, ys)) / denominator


def memory_summary(samples):
    result = {"sample_count": len(samples)}
    for field in ("rss_kib", "pss_kib", "uss_kib"):
        values = [sample[field] for sample in samples]
        result[f"{field[:-4]}_mean_kib"] = statistics.fmean(values) if values else 0.0
        result[f"{field[:-4]}_peak_kib"] = max(values, default=0)
        result[f"{field[:-4]}_slope_kib_per_min"] = slope_kib_per_minute(samples, field)
    return result


def stage_summary(receiver_values):
    callback_total = receiver_values.get("callback_total_ns", 0)
    if not isinstance(callback_total, int) or callback_total <= 0:
        return {}
    decode_total = receiver_values.get("decode_total_ns", 0)
    apply_total = receiver_values.get("apply_total_ns", 0)
    payload_decode_total = receiver_values.get("payload_decode_total_ns", 0)
    candidate_build_total = receiver_values.get("candidate_build_total_ns", 0)
    canonical_hash_total = receiver_values.get("canonical_hash_total_ns", 0)
    commit_total = receiver_values.get("commit_total_ns", 0)
    internal_total = (
        payload_decode_total + candidate_build_total + canonical_hash_total + commit_total
    )
    return {
        "decode_fraction_of_callback": decode_total / callback_total,
        "apply_fraction_of_callback": apply_total / callback_total,
        "payload_decode_fraction_of_callback": payload_decode_total / callback_total,
        "candidate_build_fraction_of_callback": candidate_build_total / callback_total,
        "canonical_hash_fraction_of_callback": canonical_hash_total / callback_total,
        "commit_fraction_of_callback": commit_total / callback_total,
        "apply_internal_other_fraction_of_callback": max(
            0.0, (apply_total - internal_total) / callback_total
        ),
        "other_fraction_of_callback": max(
            0.0, 1.0 - (decode_total + apply_total) / callback_total
        ),
    }


def stop_process(process, identity, timeout=10.0):
    if process.poll() is not None:
        return process.returncode
    if not same_process(identity):
        raise RuntimeError(f"process identity changed before stop: {identity}")
    os.killpg(process.pid, signal.SIGINT)
    try:
        return process.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        if same_process(identity):
            os.killpg(process.pid, signal.SIGTERM)
        try:
            return process.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            if same_process(identity):
                os.killpg(process.pid, signal.SIGKILL)
            return process.wait(timeout=5.0)


def stop_unidentified_spawned_process(process, timeout=10.0):
    if process.poll() is not None:
        return process.returncode
    try:
        process_group_id = os.getpgid(process.pid)
    except ProcessLookupError:
        return process.wait(timeout=1.0)
    require(
        process_group_id == process.pid and process_group_id != os.getpgrp(),
        "refusing to signal an unidentified process outside its isolated group",
    )
    try:
        os.killpg(process_group_id, signal.SIGINT)
    except ProcessLookupError:
        return process.wait(timeout=1.0)
    try:
        return process.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(process_group_id, signal.SIGTERM)
        except ProcessLookupError:
            return process.wait(timeout=1.0)
        try:
            return process.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(process_group_id, signal.SIGKILL)
            except ProcessLookupError:
                return process.wait(timeout=1.0)
            return process.wait(timeout=5.0)


def git_value(root, *args):
    if root is None:
        return None
    completed = subprocess.run(
        ["git", "-C", str(root), *args],
        check=False,
        capture_output=True,
        text=True,
    )
    return completed.stdout.strip() if completed.returncode == 0 else None


def main():
    args = parse_args()
    validate(args)
    output = prepare_output(args)
    print(f"artifacts={output}", flush=True)

    source_summary = output / "source-summary.tsv"
    source_rows = output / "source-per-source.tsv"
    receiver_summary = output / "receiver-summary.tsv"
    receiver_rows = output / "receiver-per-source.tsv"
    receiver_ready = output / "receiver-ready.tsv"
    source_log = (output / "source.log").open("w", encoding="utf-8")
    receiver_log = (output / "receiver.log").open("w", encoding="utf-8")
    samples = []
    role_samples = []
    source = None
    receiver = None
    source_identity = None
    receiver_identity = None
    receiver_launcher_identity = None
    source_exit = None
    receiver_exit = None
    smem_crosscheck = None

    environment = os.environ.copy()
    environment["ROS_DOMAIN_ID"] = str(args.domain_id)
    environment.setdefault("RCUTILS_COLORIZED_OUTPUT", "0")
    topic = f"/c4/profile/domain_{args.domain_id}/routed_updates"

    receiver_target_command = [
        str(args.receiver),
        *ros_args({
            "expected_sources": args.source_count,
            "qos_depth": args.qos_depth,
            "target_cells_per_source": args.cells_per_source,
            "summary_path": receiver_summary,
            "per_source_path": receiver_rows,
            "ready_path": receiver_ready,
            "input_topic": topic,
            "storage_mode": args.storage_mode,
            "chunk_edge": args.chunk_edge,
            "chunk_bucket_count": args.chunk_bucket_count,
        }),
    ]
    source_command = [
        str(args.source),
        *ros_args({
            "workload_mode": args.workload_mode,
            "source_count": args.source_count,
            "cells_per_source": args.cells_per_source,
            "initial_cells_per_source": args.initial_cells_per_source,
            "delta_operations": args.delta_operations,
            "rate_hz_per_source": args.rate_hz_per_source,
            "qos_depth": args.qos_depth,
            "summary_path": source_summary,
            "per_source_path": source_rows,
            "output_topic": topic,
        }),
    ]

    try:
        receiver_command = receiver_target_command
        if args.receiver_tool == "heaptrack":
            receiver_command = [
                shutil.which("heaptrack"),
                "-o",
                str(output / "heaptrack.%p"),
                *receiver_target_command,
            ]
        elif args.receiver_tool == "memcheck":
            receiver_command = [
                shutil.which("valgrind"),
                "--tool=memcheck",
                "--leak-check=full",
                "--show-leak-kinds=all",
                "--errors-for-leak-kinds=definite,indirect,possible",
                "--track-origins=yes",
                "--error-exitcode=42",
                f"--log-file={output / 'memcheck.%p.log'}",
                *receiver_target_command,
            ]
        receiver = subprocess.Popen(
            receiver_command,
            stdout=receiver_log,
            stderr=subprocess.STDOUT,
            env=environment,
            start_new_session=True,
        )
        if args.receiver_tool == "heaptrack":
            receiver_launcher_identity = wait_for_launcher_arguments(
                receiver,
                [shutil.which("heaptrack"), str(args.receiver)],
                "heaptrack",
            )
            receiver_identity = wait_for_unique_child_exe(
                receiver, args.receiver, "heaptrack receiver"
            )
        elif args.receiver_tool == "memcheck":
            receiver_identity = wait_for_process_identity(
                receiver, Path("/usr/bin/valgrind.bin"), "memcheck receiver"
            )
            require(
                str(args.receiver.resolve()) in receiver_identity["cmdline"],
                "memcheck receiver command does not identify the target ELF",
            )
            receiver_launcher_identity = receiver_identity
        else:
            receiver_identity = wait_for_process_identity(
                receiver, args.receiver, "receiver"
            )
            receiver_launcher_identity = receiver_identity
        source = subprocess.Popen(
            source_command,
            stdout=source_log,
            stderr=subprocess.STDOUT,
            env=environment,
            start_new_session=True,
        )
        source_identity = wait_for_process_identity(source, args.source, "source")

        ready_deadline = time.monotonic() + args.ready_timeout_s
        while not receiver_ready.is_file() or receiver_ready.stat().st_size == 0:
            if source.poll() is not None or receiver.poll() is not None:
                raise RuntimeError("a workload process exited before receiver readiness")
            require_live_process(source_identity, "source")
            require_live_process(receiver_identity, "receiver")
            if time.monotonic() >= ready_deadline:
                raise RuntimeError("receiver did not reach the declared target map size")
            time.sleep(0.1)

        warmup_deadline = time.monotonic() + args.warmup_s
        while time.monotonic() < warmup_deadline:
            if source.poll() is not None or receiver.poll() is not None:
                raise RuntimeError("a workload process exited during warmup")
            require_live_process(source_identity, "source")
            require_live_process(receiver_identity, "receiver")
            time.sleep(min(0.2, max(0.0, warmup_deadline - time.monotonic())))

        start = time.monotonic()
        start_ticks = read_cpu_ticks(receiver_identity["pid"])
        deadline = start + args.window_s
        next_sample = start
        while time.monotonic() < deadline:
            if source.poll() is not None or receiver.poll() is not None:
                raise RuntimeError("a workload process exited during the measurement window")
            now = time.monotonic()
            if now >= next_sample:
                require_live_process(source_identity, "source")
                require_live_process(receiver_identity, "receiver")
                sample = {
                    "elapsed_s": now - start,
                    **read_smaps(receiver_identity["pid"]),
                }
                samples.append(sample)
                if smem_crosscheck is None:
                    smem_crosscheck = read_smem(receiver_identity["pid"])
                role_samples.append({
                    "elapsed_s": now - start,
                    "source_alive": True,
                    "receiver_alive": True,
                    "source_starttime_ticks": source_identity["starttime_ticks"],
                    "receiver_starttime_ticks": receiver_identity["starttime_ticks"],
                })
                next_sample += args.sample_period_s
            time.sleep(min(0.05, max(0.0, min(next_sample, deadline) - time.monotonic())))
        end_ticks = read_cpu_ticks(receiver_identity["pid"])
        elapsed = time.monotonic() - start

        source_exit = stop_process(source, source_identity)
        time.sleep(1.0)
        if args.receiver_tool in ("heaptrack", "memcheck"):
            receiver_exit = stop_instrumented_receiver(
                receiver, receiver_launcher_identity, receiver_identity
            )
        else:
            receiver_exit = stop_process(receiver, receiver_identity)
    finally:
        if source is not None and source.poll() is None:
            if source_identity is not None:
                stop_process(source, source_identity)
            else:
                stop_unidentified_spawned_process(source)
        if receiver is not None and receiver.poll() is None:
            if receiver_identity is None:
                stop_unidentified_spawned_process(receiver)
            elif (
                args.receiver_tool in ("heaptrack", "memcheck")
                and receiver_launcher_identity is not None
            ):
                stop_instrumented_receiver(
                    receiver, receiver_launcher_identity, receiver_identity
                )
            else:
                stop_process(receiver, receiver_identity)
        source_log.close()
        receiver_log.close()

    require(source_exit == 0, f"source exited with {source_exit}")
    require(
        receiver_exit == 0 or (args.receiver_tool == "memcheck" and receiver_exit == 42),
        f"receiver exited with {receiver_exit}",
    )
    instrumentation = None
    if args.receiver_tool == "heaptrack":
        instrumentation = generate_heaptrack_report(args, output)
    elif args.receiver_tool == "memcheck":
        instrumentation = generate_memcheck_report(args, output)
    for artifact in (
        source_summary,
        source_rows,
        receiver_summary,
        receiver_rows,
        receiver_ready,
    ):
        require(artifact.is_file() and artifact.stat().st_size > 0, f"missing artifact: {artifact}")

    source_values = parse_kv(source_summary)
    receiver_values = parse_kv(receiver_summary)
    ready_values = parse_kv(receiver_ready)
    sent = parse_sources(source_rows)
    received = parse_sources(receiver_rows)
    mismatches = []
    if set(sent) != set(received):
        mismatches.append("source identity sets differ")
    for name in sorted(set(sent) & set(received)):
        source_row = sent[name]
        receiver_row = received[name]
        if int(source_row["final_cells"]) != args.cells_per_source:
            mismatches.append(
                f"{name}: final_cells={source_row['final_cells']} != "
                f"target={args.cells_per_source}"
            )
        fields = (
            ("messages_sent", "messages_received"),
            ("messages_sent", "messages_applied"),
            ("final_revision", "final_revision"),
            ("final_cells", "final_cells"),
            ("final_content_hash", "final_content_hash"),
            ("payload_bytes_sent", "payload_bytes_received"),
        )
        for source_field, receiver_field in fields:
            if source_row[source_field] != receiver_row[receiver_field]:
                mismatches.append(
                    f"{name}: {source_field}={source_row[source_field]} != "
                    f"{receiver_field}={receiver_row[receiver_field]}"
                )

    tick_hz = os.sysconf(os.sysconf_names["SC_CLK_TCK"])
    cpu_seconds = (end_ticks - start_ticks) / tick_hz
    memory = memory_summary(samples)
    source_elf = elf_provenance(args.source)
    receiver_elf = elf_provenance(args.receiver)
    sanitizer = sanitizer_evidence(environment, output, source_elf, receiver_elf)
    source_compile_command = find_compile_command(
        args.source, "C4ResourceProfileSource.cpp"
    )
    receiver_compile_command = find_compile_command(
        args.receiver, "C4ResourceProfileReceiver.cpp"
    )
    source_path = Path(__file__).resolve().with_name("C4ResourceProfileSource.cpp")
    receiver_path = Path(__file__).resolve().with_name("C4ResourceProfileReceiver.cpp")
    source_hash = sha256_file(source_path) if source_path.is_file() else None
    receiver_hash = sha256_file(receiver_path) if receiver_path.is_file() else None
    runner_hash = sha256_file(Path(__file__).resolve())
    provenance_complete = (
        source_elf["build_id"] is not None
        and receiver_elf["build_id"] is not None
        and source_elf["ldd_returncode"] == 0
        and receiver_elf["ldd_returncode"] == 0
        and source_compile_command is not None
        and receiver_compile_command is not None
        and source_hash is not None
        and receiver_hash is not None
    )
    instrumentation_valid = True
    if args.receiver_tool == "heaptrack":
        instrumentation_valid = (
            instrumentation["report"].get("gate_pass") == "true"
            and instrumentation["timeline"].get("gate_pass") == "true"
        )
    elif args.receiver_tool == "memcheck":
        instrumentation_valid = instrumentation["report"].get("gate_pass") == "true"
    valid = (
        not mismatches
        and instrumentation_valid
        and sanitizer["gate_pass"]
        and source_values.get("workload_mode") == args.workload_mode
        and source_values.get("conversion_failures") == 0
        and source_values.get("serialization_failures") == 0
        and source_values.get("endpoint_count_anomalies") == 0
        and receiver_values.get("messages_rejected") == 0
        and receiver_values.get("messages_duplicate") == 0
        and receiver_values.get("origin_clock_anomalies") == 0
        and receiver_values.get("endpoint_count_anomalies") == 0
        and receiver_values.get("sources_seen") == args.source_count
        and receiver_values.get("storage_mode") == args.storage_mode
        and receiver_values.get("chunk_edge") == args.chunk_edge
        and receiver_values.get("chunk_bucket_count") == args.chunk_bucket_count
        and ready_values.get("sources_ready") == args.source_count
        and ready_values.get("cells_per_source") == args.cells_per_source
        and smem_crosscheck is not None
        and not smem_crosscheck.get("missing_pid", False)
        and smem_crosscheck.get("returncode") == 0
        and len(samples) >= max(2, int(args.window_s / args.sample_period_s) - 1)
    )
    if args.formal:
        valid = (
            valid
            and provenance_complete
            and memory["pss_slope_kib_per_min"] < 1024.0
            and memory["uss_slope_kib_per_min"] < 1024.0
        )

    git_status = git_value(args.workspace_root, "status", "--porcelain")
    analysis = {
        "schema_version": 1,
        "valid": valid,
        "formal": args.formal,
        "mismatches": mismatches,
        "workload": {
            "mode": args.workload_mode,
            "source_count": args.source_count,
            "cells_per_source": args.cells_per_source,
            "initial_cells_per_source": args.initial_cells_per_source,
            "delta_operations": args.delta_operations,
            "rate_hz_per_source": args.rate_hz_per_source,
            "qos_depth": args.qos_depth,
            "storage_mode": args.storage_mode,
            "chunk_edge": args.chunk_edge,
            "chunk_bucket_count": args.chunk_bucket_count,
            "warmup_s": args.warmup_s,
            "ready_timeout_s": args.ready_timeout_s,
            "window_s": args.window_s,
            "sample_period_s": args.sample_period_s,
            "ros_domain_id": args.domain_id,
        },
        "environment": {
            "kernel": platform.release(),
            "platform": platform.platform(),
            "rmw_implementation": environment.get("RMW_IMPLEMENTATION", "default"),
            "image_id": args.image_id or None,
            "git_commit": git_value(args.workspace_root, "rev-parse", "HEAD"),
            "git_dirty": None if git_status is None else bool(git_status),
            "cpu_count": os.cpu_count(),
            "cpu_affinity": sorted(os.sched_getaffinity(0)),
            "cgroup": Path("/proc/self/cgroup").read_text(encoding="utf-8").splitlines(),
        },
        "processes": {
            "source": source_identity,
            "receiver": receiver_identity,
            "receiver_launcher": receiver_launcher_identity,
        },
        "provenance": {
            "complete": provenance_complete,
            "runner_sha256": runner_hash,
            "report_parser_sha256": (
                sha256_file(args.report_parser) if args.report_parser.is_file() else None
            ),
            "source_fixture_sha256": source_hash,
            "receiver_fixture_sha256": receiver_hash,
            "source_elf": source_elf,
            "receiver_elf": receiver_elf,
            "source_compile_command": source_compile_command,
            "receiver_compile_command": receiver_compile_command,
        },
        "source": source_values,
        "receiver": receiver_values,
        "receiver_tool": args.receiver_tool,
        "instrumentation": instrumentation,
        "sanitizer": sanitizer,
        "stage_breakdown": stage_summary(receiver_values),
        "receiver_ready": ready_values,
        "memory": memory,
        "smem_crosscheck": smem_crosscheck,
        "receiver_cpu": {
            "cpu_seconds": cpu_seconds,
            "window_elapsed_s": elapsed,
            "single_core_percent": 100.0 * cpu_seconds / elapsed,
            "wording": "record-only; host contention makes absolute CPU uncertain by about +/-30%",
        },
        "network_bytes": {
            "application_cdr_bytes": source_values.get("application_serialized_bytes", 0),
            "payload_bytes": source_values.get("payload_bytes_sent", 0),
            "scope": "application CDR only; excludes DDS/RTPS, IP, and link-layer overhead",
        },
    }
    with (output / "memory-samples.tsv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=samples[0].keys(), delimiter="\t")
        writer.writeheader()
        writer.writerows(samples)
    with (output / "role-monitor.tsv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=role_samples[0].keys(), delimiter="\t")
        writer.writeheader()
        writer.writerows(role_samples)
    (output / "analysis-summary.json").write_text(
        json.dumps(analysis, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    write_artifact_hashes(output)
    print(json.dumps(analysis, indent=2, sort_keys=True))
    return 0 if valid else 2


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as error:
        print(f"c4 resource profile failed: {error}", file=sys.stderr)
        sys.exit(1)
