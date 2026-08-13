#!/usr/bin/env python3
"""Reusable validators for profiling control and tool reports."""

from __future__ import annotations

import argparse
import csv
import math
import re
import sys
from collections.abc import Mapping, Sequence
from pathlib import Path


C1_SENSOR_RATES_HZ = {"front": 9.0, "rear": 9.0, "top": 9.0}
C1_WORKSPACE_SYMBOL_PREFIXES = (
    "PerceptionInputNode::",
    "Perception::",
    "PerceptionAdapters::",
    "PerceptionCore::",
)


def write_key_values(path: Path, values: Mapping[str, object]) -> None:
    with path.open("w", encoding="utf-8") as stream:
        for key, value in values.items():
            stream.write(f"{key}={value}\n")


def parse_perf_control_window(
    control_path: Path, t0_ns: int, t1_ns: int, quality_path: Path
) -> dict[str, object]:
    values: dict[str, str] = {}
    with control_path.open(encoding="utf-8") as stream:
        for line in stream:
            if "=" not in line:
                continue
            key, value = line.strip().split("=", 1)
            if key in values:
                raise ValueError(f"duplicate perf control evidence: {key}")
            values[key] = value

    for command in ("enable", "disable", "stop"):
        if values.get(f"{command}_ack") != "ack":
            raise ValueError(f"missing perf {command} acknowledgement")

    enable = int(values["enable_ack_monotonic_ns"])
    disable = int(values["disable_ack_monotonic_ns"])
    stop = int(values["stop_ack_monotonic_ns"])
    if not enable <= t0_ns < t1_ns <= disable <= stop:
        raise ValueError(
            "perf control acknowledgements do not enclose the workload window"
        )

    quality = {
        "parse_valid": "true",
        "enable_ack_monotonic_ns": enable,
        "t0_monotonic_ns": t0_ns,
        "t1_monotonic_ns": t1_ns,
        "disable_ack_monotonic_ns": disable,
        "stop_ack_monotonic_ns": stop,
        "gate_pass": "true",
    }
    write_key_values(quality_path, quality)
    return quality


def parse_workload_window(
    measurement_path: Path,
    start_line: int,
    end_line: int,
    t0_ns: int,
    t1_ns: int,
    output_path: Path,
    sensor_rates_hz: Mapping[str, float],
) -> dict[str, object]:
    duration_s = (t1_ns - t0_ns) / 1_000_000_000
    counts = dict.fromkeys(sensor_rates_hz, 0)
    unknown: list[str] = []
    with measurement_path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.reader(stream))
    for row in rows[start_line:end_line]:
        if not row:
            continue
        value = row[-1].strip().strip('"')
        if value in counts:
            counts[value] += 1
        elif value and value not in {"sensor_id", "---"}:
            unknown.append(value)

    total = sum(counts.values())
    required_counts = {
        sensor_id: math.ceil(duration_s * rate_hz)
        for sensor_id, rate_hz in sensor_rates_hz.items()
    }
    required_total = math.ceil(duration_s * sum(sensor_rates_hz.values()))
    values: dict[str, object] = {
        "duration_actual_s": f"{duration_s:.9f}",
        "line_start": start_line,
        "line_end": end_line,
    }
    values.update(counts)
    values["total"] = total
    values["required_total"] = required_total
    if len(set(required_counts.values())) == 1:
        values["required_each"] = next(iter(required_counts.values()))
    else:
        values.update(
            (f"required_{sensor_id}", count)
            for sensor_id, count in required_counts.items()
        )
    values["unknown"] = len(unknown)
    write_key_values(output_path, values)

    if (
        duration_s <= 0
        or total < required_total
        or any(counts[key] < required_counts[key] for key in counts)
        or unknown
    ):
        raise ValueError("measurement-window workload count gate failed")
    return values


def parse_c1_workload_window(
    measurement_path: Path,
    start_line: int,
    end_line: int,
    t0_ns: int,
    t1_ns: int,
    output_path: Path,
) -> dict[str, object]:
    return parse_workload_window(
        measurement_path,
        start_line,
        end_line,
        t0_ns,
        t1_ns,
        output_path,
        C1_SENSOR_RATES_HZ,
    )


def parse_perf_stat_report(stat_path: Path, quality_path: Path) -> dict[str, str]:
    expected_events = (
        "task-clock",
        "context-switches",
        "cpu-migrations",
        "page-faults",
        "cycles:u",
        "instructions:u",
        "branches:u",
        "branch-misses:u",
        "cache-references:u",
        "cache-misses:u",
    )
    required_software_events = set(expected_events[:4])
    unavailable_values = {"<not supported>", "<not counted>"}
    rows: dict[str, tuple[str, ...]] = {}

    with stat_path.open(newline="", encoding="utf-8", errors="replace") as stream:
        for row in csv.reader(stream):
            if len(row) < 5:
                continue
            event = row[2].strip()
            if event in expected_events:
                if event in rows:
                    raise ValueError(f"duplicate perf stat event: {event}")
                rows[event] = tuple(field.strip() for field in row)

    missing = [event for event in expected_events if event not in rows]
    if missing:
        raise ValueError("missing perf stat events: " + ", ".join(missing))

    quality: dict[str, str] = {"parse_valid": "true"}
    for event in expected_events:
        row = rows[event]
        value_text = row[0]
        key = re.sub(r"[^a-z0-9]+", "_", event.lower()).strip("_")
        if value_text in unavailable_values:
            if event in required_software_events:
                raise ValueError(f"required software event is unavailable: {event}")
            quality[f"{key}_status"] = value_text.strip("<>").replace(" ", "_")
            continue
        try:
            value = float(value_text)
            runtime_ns = float(row[3])
            running_percent = float(row[4])
        except ValueError as error:
            raise ValueError(f"unparseable perf stat row for {event}: {row}") from error
        if not all(math.isfinite(item) for item in (value, runtime_ns, running_percent)):
            raise ValueError(f"non-finite perf stat row for {event}: {row}")
        if value < 0 or runtime_ns <= 0 or not 0 < running_percent <= 100.0:
            raise ValueError(f"invalid perf stat counters for {event}: {row}")
        if event == "task-clock" and value <= 0:
            raise ValueError("task-clock must be positive")
        quality[f"{key}_status"] = "supported"
        quality[f"{key}_value"] = value_text
        quality[f"{key}_runtime_ns"] = row[3]
        quality[f"{key}_running_percent"] = row[4]
    quality["gate_pass"] = "true"
    write_key_values(quality_path, quality)
    return quality


def parse_perf_record_report(
    symbols_path: Path,
    report_path: Path,
    quality_path: Path,
    top_path: Path,
    requested_duration_s: int,
    workspace_symbol_prefixes: Sequence[str] = C1_WORKSPACE_SYMBOL_PREFIXES,
) -> dict[str, object]:
    lines = symbols_path.read_text(encoding="utf-8", errors="replace").splitlines()
    report_text = report_path.read_text(encoding="utf-8", errors="replace")
    sample_match = next(
        (
            re.search(r"^# Samples:\s+(\S+)", line)
            for line in lines
            if line.startswith("# Samples:")
        ),
        None,
    )
    header_samples = sample_match.group(1) if sample_match else "missing"
    lost_match = re.search(
        r"^# Total Lost Samples:\s+(\d+)\s*$", report_text, re.MULTILINE
    )
    lost_samples = int(lost_match.group(1)) if lost_match else -1
    rows: list[tuple[str, int, bool, bool]] = []
    for line in lines:
        parts = line.split("|")
        if len(parts) < 5 or not re.match(r"^\s*\d+(?:\.\d+)?%\s*$", parts[0]):
            continue
        try:
            row_samples = int(parts[1].strip())
        except ValueError:
            continue
        dso = parts[3].strip()
        symbol = parts[4].strip()
        normalized_symbol = re.sub(r"^\[[^]]+\]\s+", "", symbol).strip()
        is_unknown = (
            dso in {"[unknown]", "unknown"}
            or normalized_symbol in {"[unknown]", "unknown"}
            or normalized_symbol.startswith("0x")
            or normalized_symbol.lower().startswith("ffffffff")
        )
        is_workspace = any(
            prefix in normalized_symbol for prefix in workspace_symbol_prefixes
        )
        rows.append((line.rstrip(), row_samples, is_unknown, is_workspace))

    sample_count = sum(row[1] for row in rows)
    unknown_samples = sum(row[1] for row in rows if row[2])
    workspace_samples = sum(row[1] for row in rows if row[3])
    unknown_percent = 100.0 * unknown_samples / sample_count if sample_count else 100.0
    gate_enforced = requested_duration_s >= 120
    parse_valid = sample_match is not None and sample_count > 0 and lost_match is not None
    gate_pass = (
        parse_valid
        and lost_samples == 0
        and (
            not gate_enforced
            or (
                sample_count >= 1000
                and unknown_percent <= 20.0
                and workspace_samples > 0
            )
        )
    )
    quality: dict[str, object] = {
        "requested_duration_s": requested_duration_s,
        "gate_enforced": str(gate_enforced).lower(),
        "header_samples": header_samples,
        "samples": sample_count,
        "parsed_samples": sample_count,
        "unknown_samples": unknown_samples,
        "unknown_percent": f"{unknown_percent:.6f}",
        "workspace_samples": workspace_samples,
        "lost_samples": lost_samples,
        "parse_valid": str(parse_valid).lower(),
        "gate_pass": str(gate_pass).lower(),
    }
    write_key_values(quality_path, quality)
    with top_path.open("w", encoding="utf-8") as stream:
        for row, *_ in rows[:10]:
            stream.write(row + "\n")
    if not gate_pass:
        raise ValueError("perf record quality gate failed")
    return quality


def parse_heaptrack_quantity(value: str) -> int:
    match = re.fullmatch(r"([0-9]+(?:\.[0-9]+)?)([KMGTPE]?)(?:i?B)?", value)
    if match is None:
        raise ValueError(f"unparseable Heaptrack byte quantity: {value}")
    scale = 1024 ** " KMGTPE".index(match.group(2))
    result = float(match.group(1)) * scale
    if not math.isfinite(result) or result < 0:
        raise ValueError(f"invalid Heaptrack byte quantity: {value}")
    return round(result)


def parse_heaptrack_report(
    report_path: Path, quality_path: Path, requested_duration_s: int
) -> dict[str, object]:
    text = report_path.read_text(encoding="utf-8", errors="replace")
    required_sections = (
        "MOST CALLS TO ALLOCATION FUNCTIONS",
        "PEAK MEMORY CONSUMERS",
        "MOST TEMPORARY ALLOCATIONS",
    )
    missing_sections = [section for section in required_sections if section not in text]
    patterns = {
        "total_runtime_s": r"^total runtime:\s+([0-9.]+)s\.$",
        "allocation_calls": r"^calls to allocation functions:\s+(\d+)\s+\(",
        "temporary_allocations": r"^temporary memory allocations:\s+(\d+)\s+\(",
        "peak_heap": r"^peak heap memory consumption:\s+(\S+)\s*$",
        "peak_rss_with_overhead": r"^peak RSS \(including heaptrack overhead\):\s+(\S+)\s*$",
        "total_memory_leaked": r"^total memory leaked:\s+(\S+)\s*$",
    }
    values: dict[str, str] = {}
    for key, pattern in patterns.items():
        match = re.search(pattern, text, re.MULTILINE)
        if match is None:
            raise ValueError(f"missing Heaptrack summary: {key}")
        values[key] = match.group(1)
    if missing_sections:
        raise ValueError("missing Heaptrack sections: " + ", ".join(missing_sections))

    runtime = float(values["total_runtime_s"])
    if not math.isfinite(runtime) or runtime < requested_duration_s:
        raise ValueError("Heaptrack runtime is shorter than the formal window")
    peak_heap_bytes = parse_heaptrack_quantity(values["peak_heap"])
    peak_rss_bytes = parse_heaptrack_quantity(values["peak_rss_with_overhead"])
    leaked_bytes = parse_heaptrack_quantity(values["total_memory_leaked"])
    if int(values["allocation_calls"]) <= 0 or peak_heap_bytes <= 0 or peak_rss_bytes <= 0:
        raise ValueError("Heaptrack allocation and peak summaries must be positive")

    quality: dict[str, object] = {"parse_valid": "true", **values}
    quality.update(
        {
            "peak_heap_bytes": peak_heap_bytes,
            "peak_rss_with_overhead_bytes": peak_rss_bytes,
            "total_memory_leaked_bytes": leaked_bytes,
            "gate_pass": "true",
        }
    )
    write_key_values(quality_path, quality)
    return quality


def parse_heaptrack_massif_timeline(
    timeline_path: Path,
    quality_path: Path,
    expected_target: str,
    requested_duration_s: int,
) -> dict[str, object]:
    snapshots: list[dict[str, int | float]] = []
    current: dict[str, int | float] | None = None
    header: dict[str, str] = {}
    with timeline_path.open(encoding="utf-8", errors="replace") as stream:
        for raw_line in stream:
            line = raw_line.rstrip("\n")
            if line.startswith("desc:"):
                header["desc"] = line.split(":", 1)[1].strip()
            elif line.startswith("cmd:"):
                header["cmd"] = line.split(":", 1)[1].strip()
            elif line.startswith("time_unit:"):
                header["time_unit"] = line.split(":", 1)[1].strip()
            elif line.startswith("snapshot="):
                if current is not None:
                    snapshots.append(current)
                current = {"snapshot": int(line.split("=", 1)[1])}
            elif current is not None and "=" in line:
                key, value = line.split("=", 1)
                if key == "time":
                    current[key] = float(value)
                elif key in {"mem_heap_B", "mem_heap_extra_B", "mem_stacks_B"}:
                    current[key] = int(value)
    if current is not None:
        snapshots.append(current)

    required_keys = {"snapshot", "time", "mem_heap_B", "mem_heap_extra_B", "mem_stacks_B"}
    if len(snapshots) < 20:
        raise ValueError("Heaptrack Massif timeline produced fewer than 20 snapshots")
    if any(required_keys - snapshot.keys() for snapshot in snapshots):
        raise ValueError("Heaptrack Massif timeline snapshot is incomplete")
    if header.get("desc") != "heaptrack":
        raise ValueError("Heaptrack Massif timeline description is missing")
    if expected_target not in header.get("cmd", ""):
        raise ValueError("Heaptrack Massif timeline command does not identify the target ELF")
    if header.get("time_unit") != "s":
        raise ValueError("Heaptrack Massif timeline time unit is not seconds")

    ordinals = [int(snapshot["snapshot"]) for snapshot in snapshots]
    times = [float(snapshot["time"]) for snapshot in snapshots]
    if ordinals != list(range(len(snapshots))):
        raise ValueError("Heaptrack Massif timeline snapshots are not contiguous")
    if any(not math.isfinite(value) or value < 0 for value in times):
        raise ValueError("Heaptrack Massif timeline contains an invalid timestamp")
    if any(current_time < previous for previous, current_time in zip(times, times[1:])):
        raise ValueError("Heaptrack Massif timeline timestamps are not monotonic")

    def total_bytes(snapshot: Mapping[str, int | float]) -> int:
        return sum(
            int(snapshot[key])
            for key in ("mem_heap_B", "mem_heap_extra_B", "mem_stacks_B")
        )

    peak = max(snapshots, key=total_bytes)
    final = snapshots[-1]
    if total_bytes(peak) <= 0:
        raise ValueError("Heaptrack Massif timeline has no positive heap sample")
    if float(final["time"]) < requested_duration_s:
        raise ValueError("Heaptrack Massif timeline is shorter than the formal window")
    quality: dict[str, object] = {
        "parse_valid": "true",
        "snapshot_count": len(snapshots),
        "peak_snapshot": peak["snapshot"],
        "peak_time_s": f"{float(peak['time']):.6f}",
        "peak_total_bytes": total_bytes(peak),
        "final_snapshot": final["snapshot"],
        "final_time_s": f"{float(final['time']):.6f}",
        "final_total_bytes": total_bytes(final),
        "target_verified": "true",
        "gate_pass": "true",
    }
    write_key_values(quality_path, quality)
    return quality


def parse_massif_report(
    massif_path: Path,
    quality_path: Path,
    expected_target: str,
    requested_duration_s: int,
) -> dict[str, object]:
    snapshots: list[dict[str, int]] = []
    current: dict[str, int] | None = None
    header: dict[str, str] = {}
    peak_tree_count = 0
    with massif_path.open(encoding="utf-8", errors="replace") as stream:
        for raw_line in stream:
            line = raw_line.rstrip("\n")
            if line.startswith("cmd:"):
                header["cmd"] = line.split(":", 1)[1].strip()
            elif line.startswith("time_unit:"):
                header["time_unit"] = line.split(":", 1)[1].strip()
            elif line == "heap_tree=peak":
                peak_tree_count += 1
            if line.startswith("snapshot="):
                if current is not None:
                    snapshots.append(current)
                current = {"snapshot": int(line.split("=", 1)[1])}
            elif current is not None and "=" in line:
                key, value = line.split("=", 1)
                if key in {"time", "mem_heap_B", "mem_heap_extra_B", "mem_stacks_B"}:
                    current[key] = int(value)
    if current is not None:
        snapshots.append(current)

    required_keys = {"snapshot", "time", "mem_heap_B", "mem_heap_extra_B", "mem_stacks_B"}
    if len(snapshots) < 20:
        raise ValueError("Massif produced fewer than 20 snapshots")
    if any(required_keys - snapshot.keys() for snapshot in snapshots):
        raise ValueError("Massif snapshot is incomplete")
    if expected_target not in header.get("cmd", ""):
        raise ValueError("Massif command does not identify the target ELF")
    if header.get("time_unit") != "ms":
        raise ValueError("Massif time unit is not milliseconds")
    if peak_tree_count == 0 or max(snapshot["mem_stacks_B"] for snapshot in snapshots) <= 0:
        raise ValueError("Massif output does not contain stack-aware peak detail")

    def total_bytes(snapshot: Mapping[str, int]) -> int:
        return snapshot["mem_heap_B"] + snapshot["mem_heap_extra_B"] + snapshot["mem_stacks_B"]

    peak = max(snapshots, key=total_bytes)
    final = snapshots[-1]
    if final["time"] < requested_duration_s * 1000:
        raise ValueError("Massif timeline is shorter than the formal window")
    quality: dict[str, object] = {
        "parse_valid": "true",
        "snapshot_count": len(snapshots),
        "peak_snapshot": peak["snapshot"],
        "peak_time_ms": peak["time"],
        "peak_total_bytes": total_bytes(peak),
        "final_snapshot": final["snapshot"],
        "final_time_ms": final["time"],
        "final_total_bytes": total_bytes(final),
        "peak_tree_count": peak_tree_count,
        "target_verified": "true",
        "gate_pass": "true",
    }
    write_key_values(quality_path, quality)
    return quality


def parse_memcheck_report(
    log_path: Path,
    summary_path: Path,
    quality_path: Path,
    expected_target: str,
) -> dict[str, object]:
    text = log_path.read_text(encoding="utf-8", errors="replace")
    command_match = re.search(r"^==\d+== Command:\s+(.+)$", text, re.MULTILINE)
    if command_match is None or expected_target not in command_match.group(1):
        raise ValueError("Memcheck report does not identify the target ELF")

    required_patterns = (
        r"HEAP SUMMARY:",
        r"in use at exit:",
        r"total heap usage:",
        r"ERROR SUMMARY:\s+\d+ errors? from \d+ contexts?",
    )
    missing = [pattern for pattern in required_patterns if re.search(pattern, text) is None]
    if missing:
        raise ValueError("missing Memcheck sections: " + ", ".join(missing))

    all_freed = "All heap blocks were freed -- no leaks are possible" in text
    if not all_freed:
        leak_patterns = (
            r"LEAK SUMMARY:",
            r"definitely lost:\s+[\d,]+ bytes",
            r"indirectly lost:\s+[\d,]+ bytes",
            r"possibly lost:\s+[\d,]+ bytes",
            r"still reachable:\s+[\d,]+ bytes",
        )
        missing = [pattern for pattern in leak_patterns if re.search(pattern, text) is None]
        if missing:
            raise ValueError("missing Memcheck sections: " + ", ".join(missing))

    def bytes_for(label: str) -> int:
        match = re.search(rf"{label}:\s+([\d,]+) bytes", text)
        assert match is not None
        return int(match.group(1).replace(",", ""))

    def matching_line(fragment: str) -> str:
        return next(line for line in text.splitlines() if fragment in line)

    definite_bytes = 0 if all_freed else bytes_for("definitely lost")
    indirect_bytes = 0 if all_freed else bytes_for("indirectly lost")
    possible_bytes = 0 if all_freed else bytes_for("possibly lost")
    reachable_bytes = 0 if all_freed else bytes_for("still reachable")
    error_match = re.search(r"ERROR SUMMARY:\s+(\d+) errors?", text)
    assert error_match is not None
    error_count = int(error_match.group(1))
    invalid_access = re.search(
        r"Invalid (?:read|write|free|delete)|Mismatched free|Source and destination overlap",
        text,
    ) is not None
    uninitialized_access = re.search(
        r"Use of uninitialised value|Conditional jump or move depends on uninitialised|"
        r"Syscall param .* uninitialised|Uninitialised value was created",
        text,
    ) is not None
    other_error = (
        error_count > 0
        and not invalid_access
        and not uninitialized_access
        and definite_bytes == 0
        and indirect_bytes == 0
        and possible_bytes == 0
    )
    finding = (
        error_count > 0
        or definite_bytes > 0
        or indirect_bytes > 0
        or possible_bytes > 0
    )

    summary_fragments = [
        "HEAP SUMMARY:",
        "in use at exit:",
        "total heap usage:",
    ]
    if all_freed:
        summary_fragments.append("All heap blocks were freed -- no leaks are possible")
    else:
        summary_fragments.extend((
            "LEAK SUMMARY:",
            "definitely lost:",
            "indirectly lost:",
            "possibly lost:",
            "still reachable:",
        ))
    summary_fragments.append("ERROR SUMMARY:")
    with summary_path.open("w", encoding="utf-8") as stream:
        for fragment in summary_fragments:
            stream.write(matching_line(fragment) + "\n")
    quality: dict[str, object] = {
        "definite_lost_bytes": definite_bytes,
        "indirect_lost_bytes": indirect_bytes,
        "possibly_lost_bytes": possible_bytes,
        "still_reachable_bytes": reachable_bytes,
        "error_count": error_count,
        "invalid_access": str(invalid_access).lower(),
        "uninitialized_access": str(uninitialized_access).lower(),
        "other_error": str(other_error).lower(),
        "finding": str(finding).lower(),
        "all_heap_blocks_freed": str(all_freed).lower(),
        "target_verified": "true",
        "gate_pass": str(not finding).lower(),
    }
    write_key_values(quality_path, quality)
    return quality


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers(dest="command", required=True)

    perf_control = subparsers.add_parser("perf-control")
    perf_control.add_argument("control_path", type=Path)
    perf_control.add_argument("t0_ns", type=int)
    perf_control.add_argument("t1_ns", type=int)
    perf_control.add_argument("quality_path", type=Path)

    workload = subparsers.add_parser("c1-workload")
    workload.add_argument("measurement_path", type=Path)
    workload.add_argument("start_line", type=int)
    workload.add_argument("end_line", type=int)
    workload.add_argument("t0_ns", type=int)
    workload.add_argument("t1_ns", type=int)
    workload.add_argument("output_path", type=Path)

    perf_stat = subparsers.add_parser("perf-stat")
    perf_stat.add_argument("stat_path", type=Path)
    perf_stat.add_argument("quality_path", type=Path)

    perf_record = subparsers.add_parser("perf-record")
    perf_record.add_argument("symbols_path", type=Path)
    perf_record.add_argument("report_path", type=Path)
    perf_record.add_argument("quality_path", type=Path)
    perf_record.add_argument("top_path", type=Path)
    perf_record.add_argument("requested_duration_s", type=int)
    perf_record.add_argument("--workspace-prefix", action="append")

    heaptrack = subparsers.add_parser("heaptrack")
    heaptrack.add_argument("report_path", type=Path)
    heaptrack.add_argument("quality_path", type=Path)
    heaptrack.add_argument("requested_duration_s", type=int)

    heaptrack_massif = subparsers.add_parser("heaptrack-massif")
    heaptrack_massif.add_argument("timeline_path", type=Path)
    heaptrack_massif.add_argument("quality_path", type=Path)
    heaptrack_massif.add_argument("expected_target")
    heaptrack_massif.add_argument("requested_duration_s", type=int)

    massif = subparsers.add_parser("massif")
    massif.add_argument("massif_path", type=Path)
    massif.add_argument("quality_path", type=Path)
    massif.add_argument("expected_target")
    massif.add_argument("requested_duration_s", type=int)

    memcheck = subparsers.add_parser("memcheck")
    memcheck.add_argument("log_path", type=Path)
    memcheck.add_argument("summary_path", type=Path)
    memcheck.add_argument("quality_path", type=Path)
    memcheck.add_argument("expected_target")
    return parser


def main() -> None:
    parser = build_argument_parser()
    args = parser.parse_args()
    try:
        if args.command == "perf-control":
            parse_perf_control_window(
                args.control_path, args.t0_ns, args.t1_ns, args.quality_path
            )
        elif args.command == "c1-workload":
            parse_c1_workload_window(
                args.measurement_path,
                args.start_line,
                args.end_line,
                args.t0_ns,
                args.t1_ns,
                args.output_path,
            )
        elif args.command == "perf-stat":
            parse_perf_stat_report(args.stat_path, args.quality_path)
        elif args.command == "perf-record":
            parse_perf_record_report(
                args.symbols_path,
                args.report_path,
                args.quality_path,
                args.top_path,
                args.requested_duration_s,
                args.workspace_prefix or C1_WORKSPACE_SYMBOL_PREFIXES,
            )
        elif args.command == "heaptrack":
            parse_heaptrack_report(
                args.report_path, args.quality_path, args.requested_duration_s
            )
        elif args.command == "heaptrack-massif":
            parse_heaptrack_massif_timeline(
                args.timeline_path,
                args.quality_path,
                args.expected_target,
                args.requested_duration_s,
            )
        elif args.command == "massif":
            parse_massif_report(
                args.massif_path,
                args.quality_path,
                args.expected_target,
                args.requested_duration_s,
            )
        elif args.command == "memcheck":
            parse_memcheck_report(
                args.log_path,
                args.summary_path,
                args.quality_path,
                args.expected_target,
            )
        else:
            raise AssertionError(f"unhandled command: {args.command}")
    except (OSError, ValueError) as error:
        parser.exit(1, f"{error}\n")


if __name__ == "__main__":
    main()
