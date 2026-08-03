#!/usr/bin/env python3
"""Capture the terminal map identity once, after the measurement window closes.

Replay equivalence needs to know what the run ended up with: the final revision
and the map's scale. A node subscribed for the whole window would answer that,
but it would also be an extra subscriber on hot topics for the entire measured
period - and route A requires zero perturbation. Since only the last value
matters, the subscription is made after t1 instead, where its cost lands outside
the window entirely.

`state` is latched-like in practice only in the sense that it is republished on
a heartbeat, so a late subscriber still receives a current value; the wait below
is bounded and failure is reported rather than silently producing an empty file.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path


def capture(state_topic: str, timeout_s: float) -> dict[str, str]:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

    from perception_interfaces.msg import LocalMapState

    captured: dict[str, str] = {}

    class Snapshot(Node):
        def __init__(self) -> None:
            super().__init__("graph_terminal_state_snapshot")
            # VOLATILE, not TRANSIENT_LOCAL. The publisher is volatile, and a
            # subscriber demanding stronger durability is simply incompatible -
            # rclpy warns and delivers nothing at all. Measured as
            # "offering incompatible QoS ... Last incompatible policy:
            # DURABILITY" followed by a ten-second wait for a message that could
            # never arrive.
            #
            # Nothing is lost by this: state is republished on a heartbeat, so a
            # late subscriber still receives a current value, which is all a
            # terminal snapshot needs.
            qos = QoSProfile(
                depth=1,
                history=HistoryPolicy.KEEP_LAST,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            )
            self.create_subscription(LocalMapState, state_topic, self._on_state, qos)

        def _on_state(self, message: LocalMapState) -> None:
            identity = getattr(message, "identity", None)
            revision = (
                getattr(identity, "revision", None)
                if identity is not None
                else getattr(message, "revision", None)
            )
            epoch = (
                getattr(identity, "map_epoch", None)
                if identity is not None
                else getattr(message, "map_epoch", None)
            )
            captured["terminal_revision"] = str(revision)
            captured["terminal_map_epoch"] = str(epoch)
            for field in ("known_cells", "free_cells", "occupied_cells"):
                value = getattr(message, field, None)
                if value is not None:
                    captured[f"terminal_{field}"] = str(value)

    rclpy.init()
    try:
        node = Snapshot()
        deadline = node.get_clock().now().nanoseconds + int(timeout_s * 1e9)
        while not captured and node.get_clock().now().nanoseconds < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        node.destroy_node()
    finally:
        rclpy.shutdown()
    return captured


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    parser.add_argument("--state-topic", required=True)
    parser.add_argument("--timeout-s", type=float, default=10.0)
    arguments = parser.parse_args()

    captured = capture(arguments.state_topic, arguments.timeout_s)
    if not captured:
        sys.stderr.write(
            f"no {arguments.state_topic} message arrived within "
            f"{arguments.timeout_s}s\n"
        )
        sys.exit(1)
    arguments.output.write_text(
        "".join(f"{key}={value}\n" for key, value in sorted(captured.items())),
        encoding="utf-8",
    )
    sys.stdout.write(f"captured {len(captured)} terminal fields\n")


if __name__ == "__main__":
    main()
