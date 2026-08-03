#!/usr/bin/env python3

import argparse
import gzip
import json
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class PackageSpec:
    name: str
    gcov_subdir: str
    minimum_percent: float


@dataclass(frozen=True)
class CoverageSummary:
    covered: int
    total: int

    @property
    def percent(self) -> float:
        return 100.0 * self.covered / self.total


def parse_package_spec(value: str) -> PackageSpec:
    fields = value.split(":")
    if len(fields) != 3 or not fields[0] or not fields[1]:
        raise argparse.ArgumentTypeError(
            "package must be PACKAGE:GCOV_SUBDIR:MINIMUM_PERCENT"
        )
    try:
        minimum_percent = float(fields[2])
    except ValueError as error:
        raise argparse.ArgumentTypeError("minimum percent must be numeric") from error
    if not 0.0 <= minimum_percent <= 100.0:
        raise argparse.ArgumentTypeError("minimum percent must be within [0, 100]")
    return PackageSpec(fields[0], fields[1], minimum_percent)


def relative_to_or_none(path: Path, root: Path) -> Path | None:
    try:
        return path.relative_to(root)
    except ValueError:
        return None


def is_production_source(path: Path, package_root: Path) -> bool:
    relative = relative_to_or_none(path, package_root)
    if relative is None or not relative.parts or relative.parts[0] not in {
        "include",
        "src",
    }:
        return False
    excluded_directories = {"test", "tests", "generated", "external"}
    if any(part.lower() in excluded_directories for part in relative.parts[:-1]):
        return False
    return not path.name.lower().endswith("node.cpp")


def summarize_package(
    package_root: Path, gcov_directory: Path
) -> CoverageSummary:
    package_root = package_root.resolve()
    gcov_files = sorted(gcov_directory.rglob("*.gcov.json.gz"))
    if not gcov_files:
        raise ValueError(f"no gcov JSON files found under {gcov_directory}")

    executable_lines: dict[tuple[Path, int], bool] = {}
    for gcov_file in gcov_files:
        with gzip.open(gcov_file, "rt", encoding="utf-8") as stream:
            document = json.load(stream)
        working_directory = Path(document["current_working_directory"])
        for source in document["files"]:
            source_path = Path(source["file"])
            if not source_path.is_absolute():
                source_path = working_directory / source_path
            source_path = source_path.resolve()
            if not is_production_source(source_path, package_root):
                continue
            for line in source["lines"]:
                line_number = int(line["line_number"])
                count = int(line["count"])
                if line_number <= 0 or count < 0:
                    raise ValueError(
                        f"invalid gcov line record in {gcov_file}: {line}"
                    )
                key = (source_path, line_number)
                executable_lines[key] = executable_lines.get(key, False) or count > 0

    if not executable_lines:
        raise ValueError(f"no production executable lines found for {package_root}")
    return CoverageSummary(
        covered=sum(executable_lines.values()),
        total=len(executable_lines),
    )


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Summarize package production lines from gcov JSON files."
    )
    parser.add_argument("--source-root", type=Path, required=True)
    parser.add_argument("--gcov-root", type=Path, required=True)
    parser.add_argument(
        "--package",
        action="append",
        type=parse_package_spec,
        required=True,
        metavar="PACKAGE:GCOV_SUBDIR:MINIMUM_PERCENT",
    )
    return parser


def main() -> int:
    arguments = build_argument_parser().parse_args()
    failed = False
    for package in arguments.package:
        summary = summarize_package(
            arguments.source_root / package.name,
            arguments.gcov_root / package.gcov_subdir,
        )
        passed = summary.percent > package.minimum_percent
        failed = failed or not passed
        status = "PASS" if passed else "FAIL"
        print(
            f"{package.name:<20} {summary.covered} / {summary.total} lines = "
            f"{summary.percent:.2f}% (required > {package.minimum_percent:g}%) "
            f"{status}"
        )
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
