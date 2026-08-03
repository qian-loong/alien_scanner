"""Decode and pair profiling-only C2 LTTng-UST stage events."""

from __future__ import annotations

import argparse
import csv
import re
from collections import defaultdict
from dataclasses import dataclass
from decimal import Decimal, InvalidOperation
from pathlib import Path


PROVIDER = "perception_local_map_stage"
SCHEMA_VERSION = 1
REQUIRED_STAGES = (
    "callback",
    "mapper_apply",
    "state_publication",
    "read_transaction",
    "snapshot_serialization",
    "snapshot_total",
)
EVENT_SET_STAGES = {
    "callback": ("callback",),
    "full": REQUIRED_STAGES,
}

_EVENT_PATTERN = re.compile(
    rf"\b{PROVIDER}:(?P<stage>{'|'.join(REQUIRED_STAGES)})_(?P<phase>begin|end):"
)
_TIMESTAMP_PATTERN = re.compile(r"^\[(?P<seconds>-?[0-9]+)\.(?P<fraction>[0-9]{1,9})\]")


@dataclass(frozen=True)
class TraceEvent:
    timestamp_ns: int
    vpid: int
    vtid: int
    stage: str
    phase: str
    callback_id: int
    revision: int
    applied: int


@dataclass(frozen=True)
class PairedStage:
    stage: str
    callback_id: int
    vtid: int
    begin_ns: int
    end_ns: int
    revision: int
    applied: int

    @property
    def duration_ns(self) -> int:
        return self.end_ns - self.begin_ns


def _named_integer(line: str, name: str) -> int:
    match = re.search(rf"\b{re.escape(name)}\s*=\s*(-?[0-9]+)\b", line)
    if match is None:
        raise ValueError(f"stage trace event is missing integer field {name}")
    return int(match.group(1))


def parse_pretty_event(line: str) -> TraceEvent | None:
    event_match = _EVENT_PATTERN.search(line)
    if event_match is None:
        return None
    timestamp_match = _TIMESTAMP_PATTERN.search(line)
    if timestamp_match is None:
        raise ValueError("stage trace event is missing a seconds timestamp")
    fraction = timestamp_match.group("fraction").ljust(9, "0")
    timestamp_ns = int(timestamp_match.group("seconds")) * 1_000_000_000 + int(fraction)
    return TraceEvent(
        timestamp_ns=timestamp_ns,
        vpid=_named_integer(line, "vpid"),
        vtid=_named_integer(line, "vtid"),
        stage=event_match.group("stage"),
        phase=event_match.group("phase"),
        callback_id=_named_integer(line, "callback_id"),
        revision=_named_integer(line, "revision"),
        applied=_named_integer(line, "applied"),
    )


def parse_pretty_trace(path: Path) -> list[TraceEvent]:
    events: list[TraceEvent] = []
    for line_number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), start=1):
        try:
            event = parse_pretty_event(line)
        except ValueError as error:
            raise ValueError(f"{path}:{line_number}: {error}") from error
        if event is not None:
            events.append(event)
    return events


def parse_realtime_ns(value: str) -> int:
    try:
        seconds = Decimal(value)
    except InvalidOperation as error:
        raise ValueError(f"invalid realtime timestamp: {value}") from error
    return int(seconds * Decimal(1_000_000_000))


def parse_loss_counters(path: Path) -> tuple[int, int]:
    text = path.read_text(encoding="utf-8")
    values = [
        int(value)
        for value in re.findall(
            r"(?:Discarded events|Lost packets):\s*([0-9]+)", text, flags=re.IGNORECASE
        )
    ]
    return sum(values), len(values)


def pair_stage_events(
    events: list[TraceEvent],
    target_pid: int,
    t0_ns: int,
    t1_ns: int,
    event_set: str = "full",
) -> tuple[list[PairedStage], dict[str, int]]:
    if target_pid <= 0 or t0_ns < 0 or t1_ns <= t0_ns:
        raise ValueError("invalid target PID or formal window")
    try:
        required_stages = EVENT_SET_STAGES[event_set]
    except KeyError as error:
        raise ValueError(f"invalid stage event set: {event_set}") from error

    all_target_events = [event for event in events if event.vpid == target_pid]
    if any(
        current.timestamp_ns < previous.timestamp_ns
        for previous, current in zip(all_target_events, all_target_events[1:])
    ):
        raise ValueError("stage trace target events are not monotonic")
    target_events = [event for event in all_target_events if event.stage in required_stages]
    stacks: dict[int, list[TraceEvent]] = defaultdict(list)
    paired: list[PairedStage] = []
    counters = {
        "parsed_target_events": len(target_events),
        "unmatched_entries": 0,
        "unmatched_returns": 0,
        "nesting_mismatches": 0,
        "out_of_window_unmatched_entries": 0,
        "out_of_window_unmatched_returns": 0,
        "unexpected_event_set_events": len(all_target_events) - len(target_events),
    }

    for event in target_events:
        stack = stacks[event.vtid]
        if event.phase == "begin":
            stack.append(event)
            continue

        if not stack:
            key = (
                "unmatched_returns"
                if t0_ns <= event.timestamp_ns <= t1_ns
                else "out_of_window_unmatched_returns"
            )
            counters[key] += 1
            continue

        begin = stack[-1]
        if begin.stage != event.stage or begin.callback_id != event.callback_id:
            counters["nesting_mismatches"] += 1
            if t0_ns <= event.timestamp_ns <= t1_ns:
                counters["unmatched_returns"] += 1
            else:
                counters["out_of_window_unmatched_returns"] += 1
            continue

        stack.pop()
        paired.append(
            PairedStage(
                stage=event.stage,
                callback_id=event.callback_id,
                vtid=event.vtid,
                begin_ns=begin.timestamp_ns,
                end_ns=event.timestamp_ns,
                revision=event.revision,
                applied=event.applied,
            )
        )

    for stack in stacks.values():
        for begin in stack:
            key = (
                "unmatched_entries"
                if t0_ns <= begin.timestamp_ns <= t1_ns
                else "out_of_window_unmatched_entries"
            )
            counters[key] += 1

    callback_pairs = {
        pair.callback_id: pair
        for pair in paired
        if pair.stage == "callback"
        and pair.applied == 1
        and pair.begin_ns >= t0_ns
        and pair.end_ns <= t1_ns
    }
    counters["applied_callbacks"] = len(callback_pairs)
    selected: list[PairedStage] = []
    incomplete_callbacks = 0
    duplicate_stage_samples = 0
    invalid_durations = 0

    pairs_by_callback: dict[int, dict[str, list[PairedStage]]] = defaultdict(
        lambda: defaultdict(list)
    )
    for pair in paired:
        if pair.callback_id in callback_pairs:
            pairs_by_callback[pair.callback_id][pair.stage].append(pair)

    for callback_id, callback_pair in callback_pairs.items():
        stages = pairs_by_callback[callback_id]
        complete = True
        for stage in required_stages:
            samples = stages.get(stage, [])
            if len(samples) != 1:
                complete = False
                if len(samples) > 1:
                    duplicate_stage_samples += len(samples) - 1
                continue
            sample = samples[0]
            if sample.duration_ns <= 0:
                invalid_durations += 1
                complete = False
        if not complete:
            incomplete_callbacks += 1
            continue
        revision = callback_pair.revision
        for stage in required_stages:
            sample = stages[stage][0]
            selected.append(
                PairedStage(
                    stage=sample.stage,
                    callback_id=sample.callback_id,
                    vtid=sample.vtid,
                    begin_ns=sample.begin_ns,
                    end_ns=sample.end_ns,
                    revision=revision,
                    applied=1,
                )
            )

    counters["incomplete_callbacks"] = incomplete_callbacks
    counters["duplicate_stage_samples"] = duplicate_stage_samples
    counters["invalid_durations"] = invalid_durations
    counters["complete_applied_callbacks"] = (
        len(selected) // len(required_stages) if selected else 0
    )
    selected.sort(key=lambda pair: (pair.revision, required_stages.index(pair.stage)))
    return selected, counters


def write_latency_csv(path: Path, pairs: list[PairedStage]) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=(
                "stage",
                "duration_ns",
                "revision",
                "callback_id",
                "vtid",
                "begin_realtime_ns",
                "end_realtime_ns",
            ),
        )
        writer.writeheader()
        for pair in pairs:
            writer.writerow(
                {
                    "stage": pair.stage,
                    "duration_ns": pair.duration_ns,
                    "revision": pair.revision,
                    "callback_id": pair.callback_id,
                    "vtid": pair.vtid,
                    "begin_realtime_ns": pair.begin_ns,
                    "end_realtime_ns": pair.end_ns,
                }
            )


def write_quality(path: Path, values: dict[str, int | str | bool]) -> None:
    with path.open("w", encoding="utf-8") as stream:
        for key, value in values.items():
            if isinstance(value, bool):
                rendered = "true" if value else "false"
            else:
                rendered = str(value)
            stream.write(f"{key}={rendered}\n")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("trace_events", type=Path)
    parser.add_argument("session_stats", type=Path)
    parser.add_argument("target_pid", type=int)
    parser.add_argument("t0_realtime")
    parser.add_argument("t1_realtime")
    parser.add_argument("latency_csv", type=Path)
    parser.add_argument("quality", type=Path)
    parser.add_argument("--event-set", choices=tuple(EVENT_SET_STAGES), default="full")
    parser.add_argument("--normal-completion", action="store_true")
    args = parser.parse_args()

    events = parse_pretty_trace(args.trace_events)
    t0_ns = parse_realtime_ns(args.t0_realtime)
    t1_ns = parse_realtime_ns(args.t1_realtime)
    pairs, counters = pair_stage_events(
        events, args.target_pid, t0_ns, t1_ns, args.event_set
    )
    lost_events, loss_counter_count = parse_loss_counters(args.session_stats)
    complete_callbacks = counters["complete_applied_callbacks"]
    gate_pass = (
        args.normal_completion
        and loss_counter_count > 0
        and lost_events == 0
        and counters["unmatched_entries"] == 0
        and counters["unmatched_returns"] == 0
        and counters["nesting_mismatches"] == 0
        and counters["incomplete_callbacks"] == 0
        and counters["duplicate_stage_samples"] == 0
        and counters["invalid_durations"] == 0
        and counters["unexpected_event_set_events"] == 0
        and complete_callbacks > 0
    )
    write_latency_csv(args.latency_csv, pairs)
    write_quality(
        args.quality,
        {
            "event_schema_version": SCHEMA_VERSION,
            "provider": PROVIDER,
            "event_set": args.event_set,
            "target_pid": args.target_pid,
            "loss_counter_count": loss_counter_count,
            "lost_events": lost_events if loss_counter_count > 0 else -1,
            **counters,
            "normal_completion": args.normal_completion,
            "gate_pass": gate_pass,
        },
    )
    return 0 if gate_pass else 1


if __name__ == "__main__":
    raise SystemExit(main())
