import csv
import math
import tempfile
import time
import unittest
from pathlib import Path

import launch
import launch_testing
import pytest
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from perception_interfaces.msg import LidarObservation, LocalMapState, PoseEstimate
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from tf2_msgs.msg import TFMessage


@pytest.mark.launch_test
def generate_test_description():
    output_directory = Path(tempfile.mkdtemp(prefix="alien-profile-integration-"))
    share = Path(get_package_share_directory("perception_profiling"))
    profile = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(str(share / "launch" / "profile.launch.xml")),
        launch_arguments={
            "mode": "bounded",
            "output_directory": str(output_directory),
            "sequence_limit": "120",
            "startup_delay_s": "1.0",
            "start_target": "true",
            "start_sink": "true",
        }.items(),
    )
    oracle = launch.actions.ExecuteProcess(
        cmd=[
            str(
                share.parents[1]
                / "lib"
                / "perception_profiling"
                / "perception_profile_oracle"
            ),
            "bounded",
            "120",
            str(output_directory),
        ],
        output="screen",
    )
    return (
        launch.LaunchDescription([profile, oracle, launch_testing.actions.ReadyToTest()]),
        {"profile_output": output_directory, "oracle_process": oracle},
    )


class TestProfileIntegration(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("test_profile_integration")
        cls.observations = []
        cls.pose_stamps = set()
        cls.states = []
        cls.transforms = {}
        cls.diagnostics = []

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=200,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        reliable_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=300,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        transient_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        cls.subscriptions = [
            cls.node.create_subscription(
                LidarObservation,
                "/profile/perception/observations",
                lambda message: cls.observations.append((time.monotonic(), message)),
                sensor_qos,
            ),
            cls.node.create_subscription(
                PoseEstimate,
                "/profile/perception/pose",
                lambda message: cls.pose_stamps.add(
                    message.header.stamp.sec * 1_000_000_000
                    + message.header.stamp.nanosec
                ),
                reliable_qos,
            ),
            cls.node.create_subscription(
                LocalMapState,
                "/profile/local_map/state",
                cls.states.append,
                reliable_qos,
            ),
            cls.node.create_subscription(
                TFMessage,
                "/tf_static",
                cls._on_tf_static,
                transient_qos,
            ),
            cls.node.create_subscription(
                DiagnosticArray,
                "/diagnostics",
                lambda message: cls.diagnostics.extend(message.status),
                reliable_qos,
            ),
        ]

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _on_tf_static(cls, message):
        for transform in message.transforms:
            cls.transforms[transform.child_frame_id] = transform

    def _spin_until(self, predicate, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if predicate():
                return True
        return predicate()

    def test_short_authoritative_profile_matches_exact_oracle(self, profile_output):
        self.assertTrue(
            self._spin_until(
                lambda: len(self.observations) >= 105
                and all(
                    {
                        message.header.stamp.sec * 1_000_000_000
                        + message.header.stamp.nanosec,
                        message.header.stamp.sec * 1_000_000_000
                        + message.header.stamp.nanosec
                        + 50_000_000,
                    }.issubset(self.pose_stamps)
                    for _receipt, message in self.observations[:100]
                )
                and any(state.revision >= 100 for state in self.states)
                and "profile_scan_link" in self.transforms,
                20.0,
            )
        )

        messages = [entry[1] for entry in self.observations]
        stamps = [
            message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
            for message in messages
        ]
        self.assertEqual(len(stamps), len(set(stamps)))
        self.assertTrue(
            all(right - left == 100_000_000 for left, right in zip(stamps, stamps[1:]))
        )
        self.assertEqual(messages[-1].data_type, LidarObservation.DATA_TYPE_SCAN_2D)
        self.assertEqual(
            messages[-1].ray_evidence, LidarObservation.RAY_EVIDENCE_FULL_RAY
        )
        self.assertEqual(len(messages[-1].ranges), 360)
        self.assertTrue(all(math.isinf(messages[-1].ranges[i]) for i in range(255, 286)))
        self.assertTrue(
            all(
                math.isfinite(value)
                for index, value in enumerate(messages[-1].ranges)
                if not 255 <= index <= 285
            )
        )
        receipt_span = self.observations[100][0] - self.observations[0][0]
        self.assertGreater(receipt_span, 8.0)
        self.assertLess(receipt_span, 12.5)

        self.assertGreaterEqual(len(self.pose_stamps), 200)
        for _observation_receipt, message in self.observations[:100]:
            stamp_ns = (
                message.header.stamp.sec * 1_000_000_000
                + message.header.stamp.nanosec
            )
            self.assertIn(stamp_ns, self.pose_stamps)
            lead_stamp_ns = stamp_ns + 50_000_000
            self.assertIn(lead_stamp_ns, self.pose_stamps)

        transform = self.transforms["profile_scan_link"]
        self.assertEqual(transform.header.frame_id, "base_link")
        translation = transform.transform.translation
        self.assertEqual((translation.x, translation.y, translation.z), (0.0, 0.0, 0.0))
        rotation = transform.transform.rotation
        self.assertEqual((rotation.x, rotation.y, rotation.z, rotation.w), (0.5, 0.5, 0.5, 0.5))

        final_state = max(self.states, key=lambda state: state.revision)
        self.assertEqual(final_state.map_epoch, 1)
        self.assertGreaterEqual(final_state.revision, 100)
        self.assertEqual(final_state.health, LocalMapState.STATE_HEALTHY)
        self.assertRegex(final_state.mapper_contract_fingerprint, r"^[0-9a-f]{64}$")
        self.assertEqual(final_state.maximum_active_ray_evidence, LocalMapState.RAY_EVIDENCE_FULL_RAY)

        unexpected = [
            status
            for status in self.diagnostics
            if status.level >= DiagnosticStatus.ERROR
            or (
                status.level == DiagnosticStatus.WARN
                and "recovery stability gate" not in status.message
                and "mapper input health is unavailable" not in status.message
            )
        ]
        self.assertEqual([], [(item.level, item.message) for item in unexpected])

        node_names = set(self.node.get_node_names())
        self.assertIn("perception_profile_fixture", node_names)
        self.assertIn("perception_profile_sink", node_names)
        self.assertIn("profile_local_map", node_names)
        self.assertNotIn("scan_accumulator", node_names)
        self.assertNotIn("octomap_builder", node_names)
        topic_names = {name for name, _types in self.node.get_topic_names_and_types()}
        self.assertNotIn("/scan_returns", topic_names)
        self.assertFalse(any("cloud_map" in name for name in topic_names))

        oracle_path = profile_output / "oracle_checkpoints.csv"
        states_path = profile_output / "states.csv"
        self.assertTrue(
            self._spin_until(lambda: oracle_path.exists() and states_path.exists(), 5.0)
        )
        with oracle_path.open(newline="", encoding="utf-8") as stream:
            oracle_rows = list(csv.DictReader(stream))
        with states_path.open(newline="", encoding="utf-8") as stream:
            state_rows = list(csv.DictReader(stream))
        checkpoint = next(row for row in oracle_rows if int(row["revision"]) == 100)
        production = next(
            row
            for row in state_rows
            if int(row["revision"]) == 100
            and int(row["stamp_ns"]) == int(checkpoint["stamp_ns"])
        )

        def joined(state_row, oracle_row):
            scalar_keys = ("map_epoch", "revision", "stamp_ns", "fingerprint")
            if any(state_row[key] != oracle_row[key] for key in scalar_keys):
                return False
            for key in ("min_x", "min_y", "min_z", "max_x", "max_y", "max_z"):
                if not math.isclose(
                    float(state_row[key]), float(oracle_row[key]), abs_tol=1e-12
                ):
                    return False
            return True

        self.assertTrue(joined(production, checkpoint))
        negative = dict(production)
        negative["revision"] = str(int(negative["revision"]) + 1)
        self.assertFalse(joined(negative, checkpoint))


@launch_testing.post_shutdown_test()
class TestProfileShutdown(unittest.TestCase):
    def test_oracle_exited_cleanly(self, proc_info, oracle_process):
        launch_testing.asserts.assertExitCodes(proc_info, process=oracle_process)

    def test_sink_records_normal_completion(self, profile_output):
        manifest_path = profile_output / "sink_manifest.yaml"
        self.assertTrue(manifest_path.exists())
        with manifest_path.open(encoding="utf-8") as stream:
            manifest = yaml.safe_load(stream)
        self.assertTrue(manifest["summary"]["normal_completion"])
