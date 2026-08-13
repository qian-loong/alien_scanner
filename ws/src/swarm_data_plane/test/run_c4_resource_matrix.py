#!/usr/bin/env python3

import argparse
import hashlib
import json
import subprocess
import sys
from pathlib import Path


DEFAULT_SCENARIOS = (
    ("sources-1x100k", 1, 100_000),
    ("sources-2x100k", 2, 100_000),
    ("sources-4x100k", 4, 100_000),
    ("sources-8x100k", 8, 100_000),
    ("scale-2x10k", 2, 10_000),
    ("scale-2x500k", 2, 500_000),
)
SCENARIO_NAMES = tuple(scenario[0] for scenario in DEFAULT_SCENARIOS)


def parse_args():
    parser = argparse.ArgumentParser(description="Run the serial C4 resource matrix")
    parser.add_argument("--runner", required=True, type=Path)
    parser.add_argument("--source", required=True, type=Path)
    parser.add_argument("--receiver", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--workspace-root", required=True, type=Path)
    parser.add_argument("--image-id", required=True)
    parser.add_argument("--repetitions", type=int, default=1)
    parser.add_argument("--warmup-s", type=float, default=3.0)
    parser.add_argument("--window-s", type=float, default=30.0)
    parser.add_argument("--sample-period-s", type=float, default=1.0)
    parser.add_argument("--rate-hz-per-source", type=float, default=10.0)
    parser.add_argument("--delta-operations", type=int, default=256)
    parser.add_argument("--domain-id-base", type=int, default=190)
    parser.add_argument(
        "--scenario",
        action="append",
        choices=SCENARIO_NAMES,
        dest="scenarios",
        help="run only the selected scenario; repeat to select multiple scenarios",
    )
    parser.add_argument("--formal", action="store_true")
    return parser.parse_args()


def require(condition, message):
    if not condition:
        raise ValueError(message)


def validate(args):
    for path in (args.runner, args.source, args.receiver):
        require(path.is_file(), f"missing executable or script: {path}")
    require(args.repetitions > 0, "repetitions must be positive")
    if args.scenarios:
        require(
            len(set(args.scenarios)) == len(args.scenarios),
            "scenario filters must not contain duplicates",
        )
    if args.formal:
        require(args.repetitions == 3, "formal matrix requires exactly three repetitions")
        require(args.window_s >= 300.0, "formal matrix windows must be at least 300 seconds")
    output = args.output_dir.resolve()
    require(not output.exists() or not any(output.iterdir()), f"output is not empty: {output}")
    output.mkdir(parents=True, exist_ok=True)
    return output


def sha256_file(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main():
    args = parse_args()
    output = validate(args)
    runs = []
    domain_id = args.domain_id_base
    selected_names = set(args.scenarios or SCENARIO_NAMES)
    selected_scenarios = tuple(
        entry for entry in DEFAULT_SCENARIOS if entry[0] in selected_names
    )
    for scenario, sources, cells in selected_scenarios:
        for repetition in range(1, args.repetitions + 1):
            run_dir = output / f"{scenario}-run{repetition}"
            evidence_dir = run_dir / "evidence"
            command = [
                sys.executable,
                str(args.runner),
                "--source", str(args.source),
                "--receiver", str(args.receiver),
                "--output-dir", str(evidence_dir),
                "--workload-mode", "bounded",
                "--source-count", str(sources),
                "--cells-per-source", str(cells),
                "--initial-cells-per-source", str(cells),
                "--delta-operations", str(args.delta_operations),
                "--rate-hz-per-source", str(args.rate_hz_per_source),
                "--warmup-s", str(args.warmup_s),
                "--window-s", str(args.window_s),
                "--sample-period-s", str(args.sample_period_s),
                "--domain-id", str(domain_id),
                "--workspace-root", str(args.workspace_root),
                "--image-id", args.image_id,
            ]
            if args.formal:
                command.append("--formal")
            run_dir.mkdir(parents=True, exist_ok=False)
            stdout_path = run_dir / "runner.stdout.log"
            stderr_path = run_dir / "runner.stderr.log"
            with stdout_path.open("w", encoding="utf-8") as stdout_stream, stderr_path.open(
                "w", encoding="utf-8"
            ) as stderr_stream:
                completed = subprocess.run(
                    command,
                    check=False,
                    stdout=stdout_stream,
                    stderr=stderr_stream,
                )
            if completed.returncode != 0:
                raise RuntimeError(
                    f"matrix run failed: {scenario} run {repetition}, "
                    f"exit={completed.returncode}, stderr={stderr_path}"
                )
            summary_path = evidence_dir / "analysis-summary.json"
            summary = json.loads(summary_path.read_text(encoding="utf-8"))
            require(summary.get("valid") is True, f"invalid run: {run_dir}")
            runs.append({
                "scenario": scenario,
                "repetition": repetition,
                "domain_id": domain_id,
                "summary_path": str(summary_path),
                "summary_sha256": sha256_file(summary_path),
                "receiver_pid": summary["processes"]["receiver"]["pid"],
                "receiver_starttime_ticks": summary["processes"]["receiver"]["starttime_ticks"],
                "receiver_elf_sha256": summary["provenance"]["receiver_elf"]["sha256"],
                "receiver_build_id": summary["provenance"]["receiver_elf"]["build_id"],
                "source_elf_sha256": summary["provenance"]["source_elf"]["sha256"],
                "source_build_id": summary["provenance"]["source_elf"]["build_id"],
                "pss_mean_kib": summary["memory"]["pss_mean_kib"],
                "pss_peak_kib": summary["memory"]["pss_peak_kib"],
                "uss_mean_kib": summary["memory"]["uss_mean_kib"],
                "apply_p95_upper_ns": summary["receiver"]["apply_p95_upper_ns"],
                "apply_fraction_of_callback": summary["stage_breakdown"][
                    "apply_fraction_of_callback"
                ],
                "messages_applied": summary["receiver"]["messages_applied"],
            })
            domain_id += 1

    receiver_identities = {
        (row["receiver_pid"], row["receiver_starttime_ticks"]) for row in runs
    }
    require(len(receiver_identities) == len(runs), "receiver evidence identities are not unique")
    for field in (
        "receiver_elf_sha256",
        "receiver_build_id",
        "source_elf_sha256",
        "source_build_id",
    ):
        require(len({row[field] for row in runs}) == 1, f"matrix mixed {field}")

    matrix = {
        "schema_version": 1,
        "valid": True,
        "formal": args.formal,
        "runner_sha256": sha256_file(args.runner),
        "selected_scenarios": [entry[0] for entry in selected_scenarios],
        "run_count": len(runs),
        "runs": runs,
    }
    summary_path = output / "matrix-summary.json"
    summary_path.write_text(
        json.dumps(matrix, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps(matrix, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as error:
        print(f"c4 resource matrix failed: {error}", file=sys.stderr)
        sys.exit(1)
