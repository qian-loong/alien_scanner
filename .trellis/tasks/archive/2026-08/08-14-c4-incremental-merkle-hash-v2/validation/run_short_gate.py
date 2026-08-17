#!/usr/bin/env python3

import argparse
import csv
import hashlib
import json
import os
import platform
import subprocess
import tempfile
import time
from datetime import datetime, timezone
from pathlib import Path


DEFAULT_EXECUTABLE = Path(
    "/workspaces/alien-scanner/ws/install/perception_profiling/lib/"
    "perception_profiling/perception_merkle_profile"
)
REPOSITORY = Path("/workspaces/alien-scanner")
SOURCE_PATHS = (
    ".trellis/tasks/08-14-c4-incremental-merkle-hash-v2/validation/run_short_gate.py",
    "ws/src/alien_perception/perception_map_update/CMakeLists.txt",
    "ws/src/alien_perception/perception_map_update/include/perception_map_update/CellSnapshotStore.hpp",
    "ws/src/alien_perception/perception_map_update/include/perception_map_update/MerklePatricia.hpp",
    "ws/src/alien_perception/perception_map_update/include/perception_map_update/MerklePrototypeApplier.hpp",
    "ws/src/alien_perception/perception_map_update/include/perception_map_update/MerklePrototypeProtocol.hpp",
    "ws/src/alien_perception/perception_map_update/src/CellSnapshotStore.cpp",
    "ws/src/alien_perception/perception_map_update/src/MerklePatricia.cpp",
    "ws/src/alien_perception/perception_map_update/src/MerklePrototypeApplier.cpp",
    "ws/src/alien_perception/perception_map_update/src/MerklePrototypeProtocol.cpp",
    "ws/src/alien_perception/perception_map_update/src/Sha256DigestSink.hpp",
    "ws/src/alien_perception/perception_map_update/test/TestCellSnapshotStore.cpp",
    "ws/src/alien_perception/perception_map_update/test/TestMerklePatricia.cpp",
    "ws/src/alien_perception/perception_map_update/test/TestMerklePrototypeApplier.cpp",
    "ws/src/alien_perception/perception_map_update/test/TestMerklePrototypeProtocol.cpp",
    "ws/src/alien_perception/perception_profiling/CMakeLists.txt",
    "ws/src/alien_perception/perception_profiling/src/MerkleHashBenchmarkMain.cpp",
)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--mode", choices=("correctness", "flat", "merkle"), required=True)
    parser.add_argument("--pattern", choices=("update", "insert"), default="update")
    parser.add_argument("--flat-storage", choices=("vector", "chunked"), default="chunked")
    parser.add_argument("--cells", type=int, required=True)
    parser.add_argument("--iterations", type=int, required=True)
    parser.add_argument("--warmup", type=int, required=True)
    parser.add_argument("--touched-chunks", type=int, required=True)
    parser.add_argument("--hold-seconds", type=int, default=5)
    parser.add_argument("--samples", type=int, default=12)
    parser.add_argument("--sample-interval", type=float, default=0.2)
    parser.add_argument("--docker-image-id", required=True)
    parser.add_argument("--executable", type=Path, default=DEFAULT_EXECUTABLE)
    args = parser.parse_args()
    for name in ("cells", "iterations", "warmup", "touched_chunks", "samples"):
        if getattr(args, name) <= 0:
            parser.error(f"--{name.replace('_', '-')} must be positive")
    if args.hold_seconds <= 0 or args.sample_interval <= 0:
        parser.error("hold duration and sample interval must be positive")
    if args.samples * args.sample_interval >= args.hold_seconds:
        parser.error("memory sampling window must fit inside --hold-seconds")
    if not args.docker_image_id.startswith("sha256:") or len(args.docker_image_id) != 71:
        parser.error("--docker-image-id must be sha256:<64 hex>")
    return args


def run_text(command):
    return subprocess.run(command, check=True, text=True, capture_output=True).stdout.strip()


def sha256_file(path):
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def process_identity(pid):
    proc = Path("/proc") / str(pid)
    fields = (proc / "stat").read_text(encoding="utf-8").rpartition(")")[2].split()
    return {
        "pid": pid,
        "state": fields[0],
        "process_group_id": int(fields[2]),
        "starttime_ticks": int(fields[19]),
        "exe": str((proc / "exe").resolve()),
        "cmdline": [
            item.decode("utf-8")
            for item in (proc / "cmdline").read_bytes().split(b"\0")
            if item
        ],
    }


def require_same_process(expected):
    current = process_identity(expected["pid"])
    if (
        current["state"] == "Z"
        or current["starttime_ticks"] != expected["starttime_ticks"]
        or current["process_group_id"] != expected["process_group_id"]
        or current["exe"] != expected["exe"]
    ):
        raise RuntimeError("benchmark PID/starttime/process identity changed")


def read_smaps(pid):
    path = Path("/proc") / str(pid) / "smaps_rollup"
    raw = path.read_text(encoding="utf-8")
    values = {}
    for line in raw.splitlines():
        fields = line.split()
        if len(fields) >= 2 and fields[0].endswith(":") and fields[1].isdigit():
            values[fields[0][:-1]] = int(fields[1])
    return raw, {
        "rss_kib": values.get("Rss", 0),
        "pss_kib": values.get("Pss", 0),
        "uss_kib": values.get("Private_Clean", 0) + values.get("Private_Dirty", 0),
        "private_clean_kib": values.get("Private_Clean", 0),
        "private_dirty_kib": values.get("Private_Dirty", 0),
        "shared_clean_kib": values.get("Shared_Clean", 0),
        "shared_dirty_kib": values.get("Shared_Dirty", 0),
    }


def wait_for_output(process, summary_path, timeout):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if summary_path.is_file():
            return
        if process.poll() is not None:
            raise RuntimeError(f"benchmark exited before hold window: {process.returncode}")
        time.sleep(0.02)
    raise RuntimeError("timed out waiting for benchmark hold window")


def build_metadata(args, command, identity):
    cache = REPOSITORY / "ws/build/perception_profiling/CMakeCache.txt"
    build_type = next(
        line.partition("=")[2]
        for line in cache.read_text(encoding="utf-8").splitlines()
        if line.startswith("CMAKE_BUILD_TYPE:")
    )
    source_hashes = {}
    for relative in SOURCE_PATHS:
        path = REPOSITORY / relative
        source_hashes[relative] = sha256_file(path)
    build_id_output = run_text(["readelf", "-n", str(args.executable)])
    build_id = next(
        line.split("Build ID:", 1)[1].strip()
        for line in build_id_output.splitlines()
        if "Build ID:" in line
    )
    relevant_status = subprocess.run(
        ["git", "-C", str(REPOSITORY), "status", "--porcelain=v1", "--", *SOURCE_PATHS],
        check=True,
        text=True,
        capture_output=True,
    ).stdout.splitlines()
    return {
        "captured_at_utc": datetime.now(timezone.utc).isoformat(),
        "command": command,
        "process": identity,
        "executable_sha256": sha256_file(args.executable),
        "executable_build_id": build_id,
        "docker_image_id": args.docker_image_id,
        "git_head": run_text(["git", "-C", str(REPOSITORY), "rev-parse", "HEAD"]),
        "relevant_git_status": relevant_status,
        "source_sha256": source_hashes,
        "cmake_build_type": build_type,
        "compiler": run_text(["c++", "--version"]).splitlines()[0],
        "openssl": run_text(["openssl", "version"]),
        "platform": platform.platform(),
        "kernel": platform.release(),
        "clock_ticks_per_second": os.sysconf(os.sysconf_names["SC_CLK_TCK"]),
    }


def main():
    args = parse_args()
    output_dir = args.output_dir.resolve()
    if output_dir.exists():
        raise ValueError("--output-dir must not already exist")
    if not output_dir.parent.is_dir():
        raise ValueError("--output-dir parent must already exist")
    executable = args.executable.resolve(strict=True)
    args.executable = executable
    command = [
        str(executable),
        "--output-dir", str(output_dir),
        "--mode", args.mode,
        "--pattern", args.pattern,
        "--flat-storage", args.flat_storage,
        "--cells", str(args.cells),
        "--iterations", str(args.iterations),
        "--warmup", str(args.warmup),
        "--touched-chunks", str(args.touched_chunks),
        "--hold-seconds", str(args.hold_seconds),
    ]
    stdout_temp = tempfile.NamedTemporaryFile(
        mode="w", encoding="utf-8", dir=output_dir.parent, prefix="merkle-stdout-", delete=False
    )
    stderr_temp = tempfile.NamedTemporaryFile(
        mode="w", encoding="utf-8", dir=output_dir.parent, prefix="merkle-stderr-", delete=False
    )
    process = None
    try:
        process = subprocess.Popen(command, stdout=stdout_temp, stderr=stderr_temp, start_new_session=True)
        identity = process_identity(process.pid)
        expected_executable = str(executable)
        if identity["exe"] != expected_executable or identity["cmdline"][0] != expected_executable:
            raise RuntimeError("benchmark executable identity mismatch")
        summary_path = output_dir / "merkle-hash-benchmark-summary.json"
        wait_for_output(process, summary_path, timeout=120.0)
        samples = []
        for index in range(args.samples):
            require_same_process(identity)
            raw, values = read_smaps(identity["pid"])
            (output_dir / f"smaps-rollup-{index:02d}.txt").write_text(raw, encoding="utf-8")
            samples.append({"sample": index, "monotonic_ns": time.monotonic_ns(), **values})
            time.sleep(args.sample_interval)
        return_code = process.wait(timeout=args.hold_seconds + 5)
        if return_code != 0:
            raise RuntimeError(f"benchmark exited with status {return_code}")
        stdout_temp.close()
        stderr_temp.close()
        Path(stdout_temp.name).replace(output_dir / "stdout.txt")
        Path(stderr_temp.name).replace(output_dir / "stderr.txt")
        with (output_dir / "memory-rollup.csv").open("w", encoding="utf-8", newline="") as output:
            writer = csv.DictWriter(output, fieldnames=samples[0].keys())
            writer.writeheader()
            writer.writerows(samples)
        metadata = build_metadata(args, command, identity)
        metadata["memory_samples"] = len(samples)
        metadata["sample_interval_seconds"] = args.sample_interval
        (output_dir / "run-manifest.json").write_text(
            json.dumps(metadata, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
    finally:
        if process is not None and process.poll() is None:
            try:
                require_same_process(identity)
                process.terminate()
                process.wait(timeout=5)
            except (NameError, FileNotFoundError, ProcessLookupError, RuntimeError, subprocess.TimeoutExpired):
                pass
        if not stdout_temp.closed:
            stdout_temp.close()
        if not stderr_temp.closed:
            stderr_temp.close()


if __name__ == "__main__":
    main()
