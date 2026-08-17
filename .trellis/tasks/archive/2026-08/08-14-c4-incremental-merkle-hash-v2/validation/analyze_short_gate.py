#!/usr/bin/env python3

import argparse
import csv
import hashlib
import json
import math
import statistics
from pathlib import Path


ROOT = Path(__file__).resolve().parent


def read_csv(path):
    with path.open(encoding="utf-8", newline="") as source:
        return list(csv.DictReader(source))


def integers(rows, column):
    return [int(row[column]) for row in rows]


def mean(values):
    return sum(values) / len(values) if values else 0.0


def percentile(values, numerator=95, denominator=100):
    if not values:
        return 0
    ordered = sorted(values)
    rank = max(1, math.ceil(len(ordered) * numerator / denominator))
    return ordered[rank - 1]


def sha256_file(path):
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def canonical_digest(value):
    encoded = json.dumps(value, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def load_run(path):
    summary = json.loads((path / "merkle-hash-benchmark-summary.json").read_text(encoding="utf-8"))
    manifest = json.loads((path / "run-manifest.json").read_text(encoding="utf-8"))
    samples = read_csv(path / "merkle-hash-benchmark.csv")
    memory = read_csv(path / "memory-rollup.csv")
    if len(samples) != summary["iterations"]:
        raise RuntimeError(f"{path.name}: CSV row count does not match iterations")
    if len(memory) != manifest["memory_samples"]:
        raise RuntimeError(f"{path.name}: memory row count does not match manifest")
    if (path / "stderr.txt").read_text(encoding="utf-8"):
        raise RuntimeError(f"{path.name}: stderr is not empty")
    command_values = manifest["command"]
    if len(command_values) % 2 == 0:
        raise RuntimeError(f"{path.name}: benchmark command is not option/value pairs")
    command_options = dict(zip(command_values[1::2], command_values[2::2]))
    expected_options = {
        "--mode": summary["mode"],
        "--pattern": summary["pattern"],
        "--flat-storage": summary["flat_storage"],
        "--cells": str(summary["cells"]),
        "--iterations": str(summary["iterations"]),
        "--warmup": str(summary["warmup"]),
        "--touched-chunks": str(summary["touched_chunks"]),
        "--hold-seconds": str(summary["hold_seconds"]),
    }
    if any(command_options.get(key) != value for key, value in expected_options.items()):
        raise RuntimeError(f"{path.name}: manifest command and summary arguments differ")
    if summary["mode"] == "correctness":
        if not summary["validation_performed"]:
            raise RuntimeError(f"{path.name}: correctness validation was not performed")
        if not summary["all_flat_hash_match"] or not summary["all_content_match"]:
            raise RuntimeError(f"{path.name}: correctness summary failed")
        if any(row["validation_performed"] != "1" for row in samples):
            raise RuntimeError(f"{path.name}: a correctness sample skipped validation")
        if any(row["flat_hash_match"] != "1" or row["content_match"] != "1" for row in samples):
            raise RuntimeError(f"{path.name}: a correctness sample diverged")
    elif any(row["validation_performed"] != "0" for row in samples):
        raise RuntimeError(f"{path.name}: performance mode mixed in correctness work")
    if summary["mode"] == "merkle":
        flat_columns = ("flat_cow_apply_ns", "flat_hash_ns", "flat_total_apply_ns")
        if any(int(row[column]) != 0 for row in samples for column in flat_columns):
            raise RuntimeError(f"{path.name}: Merkle performance mode computed flat work")
        if any(summary[column] != 0 for column in (
            "flat_keyframe_storage_ns", "flat_keyframe_hash_ns", "flat_keyframe_total_ns"
        )):
            raise RuntimeError(f"{path.name}: Merkle performance mode computed a flat keyframe")
    if summary["mode"] == "flat":
        merkle_columns = (
            "merkle_apply_ns", "merkle_allocated_nodes", "merkle_storage_ns",
            "merkle_mutation_build_ns", "merkle_tree_ns", "merkle_commit_ns",
            "merkle_leaf_hash_ns", "merkle_branch_hash_ns", "merkle_content_hash_ns",
            "merkle_path_nodes", "merkle_owned_bytes", "merkle_candidate_owned_bytes",
            "storage_candidate_owned_bytes",
        )
        if any(int(row[column]) != 0 for row in samples for column in merkle_columns):
            raise RuntimeError(f"{path.name}: flat performance mode computed Merkle work")
        if any(summary[column] != 0 for column in (
            "merkle_keyframe_storage_ns", "merkle_keyframe_tree_ns",
            "merkle_keyframe_commit_ns", "keyframe_leaf_count", "keyframe_node_count",
            "keyframe_owned_bytes",
        )):
            raise RuntimeError(f"{path.name}: flat performance mode computed a Merkle keyframe")
    if summary["flat_total_apply_mean_ns"] != int(mean(integers(samples, "flat_total_apply_ns"))):
        raise RuntimeError(f"{path.name}: flat summary mean differs from raw CSV")
    if summary["merkle_apply_mean_ns"] != int(mean(integers(samples, "merkle_apply_ns"))):
        raise RuntimeError(f"{path.name}: Merkle summary mean differs from raw CSV")
    smaps_files = list(path.glob("smaps-rollup-*.txt"))
    if len(smaps_files) != manifest["memory_samples"]:
        raise RuntimeError(f"{path.name}: raw smaps sample count differs from manifest")
    pss = integers(memory, "pss_kib")
    uss = integers(memory, "uss_kib")
    rss = integers(memory, "rss_kib")
    return manifest, {
        "run": path.name,
        **{key: summary[key] for key in (
            "mode", "pattern", "flat_storage", "cells", "iterations", "warmup",
            "touched_chunks", "hold_seconds", "flat_keyframe_storage_ns",
            "flat_keyframe_hash_ns", "flat_keyframe_total_ns",
            "merkle_keyframe_storage_ns", "merkle_keyframe_tree_ns",
            "merkle_keyframe_commit_ns", "keyframe_leaf_count", "keyframe_node_count",
            "keyframe_owned_bytes", "validation_performed", "all_flat_hash_match",
            "all_content_match",
        )},
        "flat_cow_apply_mean_ns": mean(integers(samples, "flat_cow_apply_ns")),
        "flat_hash_mean_ns": mean(integers(samples, "flat_hash_ns")),
        "flat_total_apply_mean_ns": mean(integers(samples, "flat_total_apply_ns")),
        "flat_total_apply_p95_ns": percentile(integers(samples, "flat_total_apply_ns")),
        "merkle_apply_mean_ns": mean(integers(samples, "merkle_apply_ns")),
        "merkle_apply_p95_ns": percentile(integers(samples, "merkle_apply_ns")),
        "merkle_storage_mean_ns": mean(integers(samples, "merkle_storage_ns")),
        "merkle_mutation_build_mean_ns": mean(integers(samples, "merkle_mutation_build_ns")),
        "merkle_tree_mean_ns": mean(integers(samples, "merkle_tree_ns")),
        "merkle_commit_mean_ns": mean(integers(samples, "merkle_commit_ns")),
        "merkle_leaf_hash_mean_ns": mean(integers(samples, "merkle_leaf_hash_ns")),
        "merkle_branch_hash_mean_ns": mean(integers(samples, "merkle_branch_hash_ns")),
        "merkle_content_hash_mean_ns": mean(integers(samples, "merkle_content_hash_ns")),
        "merkle_allocated_nodes_mean": mean(integers(samples, "merkle_allocated_nodes")),
        "merkle_path_nodes_mean": mean(integers(samples, "merkle_path_nodes")),
        "merkle_owned_bytes_last": int(samples[-1]["merkle_owned_bytes"]),
        "merkle_candidate_owned_bytes_p95": percentile(
            integers(samples, "merkle_candidate_owned_bytes")
        ),
        "storage_candidate_owned_bytes_p95": percentile(
            integers(samples, "storage_candidate_owned_bytes")
        ),
        "pss_median_kib": statistics.median(pss),
        "pss_p95_kib": percentile(pss),
        "uss_median_kib": statistics.median(uss),
        "uss_p95_kib": percentile(uss),
        "rss_median_kib": statistics.median(rss),
        "rss_p95_kib": percentile(rss),
    }


def parse_args():
    parser = argparse.ArgumentParser(
        description="Validate and aggregate the incremental Merkle v2 short Gate."
    )
    parser.add_argument(
        "--raw-root",
        type=Path,
        default=ROOT,
        help="directory containing the authoritative evidence-v3-* run directories",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=ROOT,
        help="existing directory that receives the aggregate JSON and CSV",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    raw_root = args.raw_root.resolve(strict=True)
    output_dir = args.output_dir.resolve(strict=True)
    paths = sorted(path for path in raw_root.glob("evidence-v3-*") if path.is_dir())
    if len(paths) != 21:
        raise RuntimeError(f"expected 21 final evidence runs, found {len(paths)}")
    manifests = []
    rows = []
    for path in paths:
        manifest, row = load_run(path)
        manifests.append(manifest)
        rows.append(row)
    identity_fields = (
        "executable_sha256", "executable_build_id", "docker_image_id", "git_head",
        "cmake_build_type", "compiler", "openssl", "kernel",
    )
    common_identity = {}
    for field in identity_fields:
        values = {manifest[field] for manifest in manifests}
        if len(values) != 1:
            raise RuntimeError(f"run identity mismatch for {field}: {sorted(values)}")
        common_identity[field] = values.pop()
    source_digests = {canonical_digest(manifest["source_sha256"]) for manifest in manifests}
    if len(source_digests) != 1:
        raise RuntimeError("source manifest differs across final evidence runs")
    common_identity["source_manifest_sha256"] = source_digests.pop()
    output = {
        "analyzer_sha256": sha256_file(Path(__file__).resolve()),
        "run_count": len(rows),
        "identity": common_identity,
        "runs": rows,
    }
    (output_dir / "short-gate-aggregate.json").write_text(
        json.dumps(output, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    fieldnames = list(rows[0].keys())
    with (output_dir / "short-gate-aggregate.csv").open(
        "w", encoding="utf-8", newline=""
    ) as target:
        writer = csv.DictWriter(target, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


if __name__ == "__main__":
    main()
