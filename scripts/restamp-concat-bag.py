#!/usr/bin/env python3
"""Build a stamp-shifted concatenated bag for loop-style leak testing.

Why this exists: the mapper rejects any observation or pose whose stamp is not
strictly greater than its high-water mark (LocalObservationMapper.cpp:1013,
:1087), so a naive `ros2 bag play --loop` does no mapping work from the second
pass onward (measured 2026-08-02: revision frozen, 100% rejection). This tool
writes a new bag containing N copies of the source S3/S1 topics where copy k
has every header stamp, TF stamp and bag receive time shifted by k * period,
making the stream monotonic across copies.

Deliberately untouched: session identities (session_boot_time_ns / random
suffix) - they are identity, not time - and PoseEstimate.freshness_ns, which is
relative. /tf_static is written once (static transforms are timeless in tf2).
Source topic metadata is reused verbatim so QoS survives - /tf_static is
transient-local and a volatile republish would never reach the TF listener.

At each copy boundary the pose teleports back to the trajectory start; the
mapper correctly treats that as a pose discontinuity and resets the map epoch.
The resulting per-cycle build/reset sawtooth is the intended leak-test shape
(see docs/performance-memory-testing-cookbook.md section 4a).

--trim-s cuts each copy to its first N seconds. The scene's useful load stops
about 20 s in while pose/health/tf run on for another ~40 s; trimming to ~25 s
raises work density about 2.5x for the same wall clock.
"""

import argparse

from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py

DEFAULT_TOPICS = [
    "/cave_scene/perception/observations",
    "/cave_scene/perception/pose",
    "/cave_scene/perception/health",
    "/tf",
    "/tf_static",
]

TF_TOPICS = {"/tf", "/tf_static"}


def shift_time_msg(time_msg, offset_ns):
    total = time_msg.sec * 1_000_000_000 + time_msg.nanosec + offset_ns
    time_msg.sec = total // 1_000_000_000
    time_msg.nanosec = total % 1_000_000_000


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("source_bag")
    parser.add_argument("output_bag")
    parser.add_argument("--copies", type=int, default=2)
    parser.add_argument(
        "--gap-s", type=float, default=1.0,
        help="idle gap inserted between copies, seconds (default 1.0)")
    parser.add_argument(
        "--trim-s", type=float, default=None,
        help="keep only the first N seconds of each copy (default: full copy)")
    parser.add_argument(
        "--topics", nargs="+", default=DEFAULT_TOPICS,
        help="topics to carry over (default: the S3 cut plus TF)")
    args = parser.parse_args()

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=args.source_bag, storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""))
    source_topics = {t.name: t for t in reader.get_all_topics_and_types()}
    missing = [t for t in args.topics if t not in source_topics]
    if missing:
        raise SystemExit(f"source bag lacks topics: {missing}")

    messages = []  # (topic, recv_ns, raw)
    first_ns = None
    while reader.has_next():
        topic, raw, recv_ns = reader.read_next()
        if topic not in args.topics:
            continue
        messages.append((topic, recv_ns, raw))
        first_ns = recv_ns if first_ns is None else min(first_ns, recv_ns)
    del reader
    if not messages:
        raise SystemExit("no selected messages in source bag")

    if args.trim_s is not None:
        trim_ns = int(args.trim_s * 1e9)
        # /tf_static is a single early transient-local message; keep it
        # unconditionally so no trim value can strip the extrinsic.
        messages = [
            (topic, recv_ns, raw) for topic, recv_ns, raw in messages
            if topic == "/tf_static" or recv_ns - first_ns <= trim_ns]

    last_ns = max(recv_ns for _, recv_ns, _ in messages)
    period_ns = (last_ns - first_ns) + int(args.gap_s * 1e9)
    msg_classes = {
        t: get_message(source_topics[t].type) for t in args.topics}

    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=args.output_bag, storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""))
    for topic in args.topics:
        writer.create_topic(source_topics[topic])

    stamped_topics = set(args.topics) - TF_TOPICS
    written = {t: 0 for t in args.topics}
    for copy in range(args.copies):
        offset_ns = copy * period_ns
        for topic, recv_ns, raw in messages:
            if topic == "/tf_static" and copy > 0:
                continue  # static transforms are timeless; once is enough
            if copy == 0:
                out_raw = raw
            else:
                msg = deserialize_message(raw, msg_classes[topic])
                if topic in TF_TOPICS:
                    for transform in msg.transforms:
                        shift_time_msg(transform.header.stamp, offset_ns)
                elif topic in stamped_topics:
                    shift_time_msg(msg.header.stamp, offset_ns)
                out_raw = serialize_message(msg)
            writer.write(topic, out_raw, recv_ns + offset_ns)
            written[topic] += 1
    del writer

    print(f"period_ns={period_ns} ({period_ns / 1e9:.1f}s), "
          f"copies={args.copies}, "
          f"total={period_ns * args.copies / 1e9:.1f}s")
    for topic, count in written.items():
        print(f"  {topic}: {count}")


if __name__ == "__main__":
    main()
