import os
from pathlib import Path
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import FrontendLaunchDescriptionSource
import launch_testing
from octomap_msgs.msg import Octomap
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import MarkerArray


TEST_DOMAIN_ID = str(200 + (os.getpid() % 20))
os.environ["ROS_DOMAIN_ID"] = TEST_DOMAIN_ID


@pytest.mark.launch_test
def generate_test_description():
    share = Path(get_package_share_directory("perception_profiling"))
    replay = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            str(share / "launch" / "map_update_replay_visualization.launch.xml")
        ),
        launch_arguments={
            "show_rviz": "false",
            "scenario_mode": "bounded",
            "sequence_count": "8",
            "visualization_publish_rate_hz": "10.0",
            "scenario_step_period_s": "0.2",
        }.items(),
    )
    return launch.LaunchDescription(
        [
            launch.actions.SetEnvironmentVariable("ROS_DOMAIN_ID", TEST_DOMAIN_ID),
            replay,
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestMapUpdateReplayVisualization(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("test_map_update_replay_visualization")
        cls.oracle_maps = []
        cls.reconstructed_maps = []
        cls.differences = []
        cls.map_markers = []
        cls.resync_markers = []
        cls.epoch_reset_markers = []
        cls.diagnostics = []
        cls.static_transforms = []

        latched = QoSProfile(depth=1)
        latched.reliability = ReliabilityPolicy.RELIABLE
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        cls.subscriptions = [
            cls.node.create_subscription(
                Octomap,
                "/map_update_replay/oracle_octomap",
                cls.oracle_maps.append,
                latched,
            ),
            cls.node.create_subscription(
                Octomap,
                "/map_update_replay/reconstructed_octomap",
                cls.reconstructed_maps.append,
                latched,
            ),
            cls.node.create_subscription(
                MarkerArray,
                "/map_update_replay/differences",
                cls.differences.append,
                latched,
            ),
            cls.node.create_subscription(
                MarkerArray,
                "/map_update_replay/map_markers",
                cls.map_markers.append,
                latched,
            ),
            cls.node.create_subscription(
                MarkerArray,
                "/map_update_replay/resync/markers",
                cls.resync_markers.append,
                latched,
            ),
            cls.node.create_subscription(
                MarkerArray,
                "/map_update_replay/epoch_reset/markers",
                cls.epoch_reset_markers.append,
                latched,
            ),
            cls.node.create_subscription(
                DiagnosticArray,
                "/map_update_replay/diagnostics",
                cls.diagnostics.append,
                latched,
            ),
            cls.node.create_subscription(
                TFMessage,
                "/tf_static",
                cls.static_transforms.append,
                latched,
            ),
        ]

    @classmethod
    def tearDownClass(cls):
        for subscription in cls.subscriptions:
            cls.node.destroy_subscription(subscription)
        cls.node.destroy_node()
        rclpy.shutdown()

    def _spin_until(self, predicate, timeout=25.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if predicate():
                return True
        return predicate()

    def test_final_maps_diagnostics_differences_and_frames(self):
        self.assertTrue(
            self._spin_until(
                lambda: self.oracle_maps
                and self.reconstructed_maps
                and self.differences
                and self.map_markers
                and self.resync_markers
                and self.epoch_reset_markers
                and self.diagnostics
                and self.static_transforms
            ),
            "replay visualization did not publish its latched final state",
        )

        oracle = self.oracle_maps[-1]
        reconstructed = self.reconstructed_maps[-1]
        self.assertEqual(oracle.header.frame_id, "replay_oracle_map")
        self.assertEqual(reconstructed.header.frame_id, "replay_reconstructed_map")
        self.assertEqual(oracle.id, reconstructed.id)
        self.assertEqual(oracle.binary, reconstructed.binary)
        self.assertEqual(oracle.resolution, reconstructed.resolution)
        self.assertGreater(len(oracle.data), 0)
        self.assertEqual(oracle.data, reconstructed.data)

        replay_statuses = [
            status
            for message in self.diagnostics
            for status in message.status
            if status.name.endswith("exact_revision_replay_oracle")
        ]
        self.assertTrue(replay_statuses)
        status = replay_statuses[-1]
        values = {entry.key: entry.value for entry in status.values}
        self.assertEqual(status.level, DiagnosticStatus.OK, status.message)
        self.assertEqual(values.get("match"), "true")
        self.assertEqual(values.get("input_sample_count"), "8")
        self.assertEqual(values.get("unavailable_input_count"), "1")
        self.assertEqual(values.get("checkpoint_count"), "7")
        self.assertEqual(values.get("keyframe_count"), "1")
        self.assertEqual(values.get("delta_count"), "6")
        self.assertEqual(values.get("missing_cells"), "0")
        self.assertEqual(values.get("unexpected_cells"), "0")
        self.assertEqual(values.get("state_mismatches"), "0")
        self.assertEqual(
            values.get("source_known_cells"),
            values.get("reconstructed_known_cells"),
        )

        difference = self.differences[-1]
        namespaces = {marker.ns for marker in difference.markers if marker.ns}
        self.assertEqual(
            namespaces,
            {
                "replay_diff/missing",
                "replay_diff/unexpected",
                "replay_diff/state",
            },
        )
        self.assertEqual(sum(len(marker.points) for marker in difference.markers), 0)

        map_markers = [
            marker for marker in self.map_markers[-1].markers if marker.ns
        ]
        self.assertEqual(
            {marker.ns for marker in map_markers},
            {
                "replay_map/oracle_occupied",
                "replay_map/reconstructed_occupied",
            },
        )
        self.assertTrue(all(marker.points for marker in map_markers))
        self.assertEqual(
            {marker.header.frame_id for marker in map_markers},
            {"replay_oracle_map", "replay_reconstructed_map"},
        )
        self.assertEqual(len(map_markers[0].points), len(map_markers[1].points))

        resync_statuses = [
            status
            for message in self.diagnostics
            for status in message.status
            if status.name.endswith("resync_recovery")
        ]
        epoch_statuses = [
            status
            for message in self.diagnostics
            for status in message.status
            if status.name.endswith("epoch_reset")
        ]
        self.assertTrue(resync_statuses)
        self.assertTrue(epoch_statuses)
        for scenario_status in (resync_statuses[-1], epoch_statuses[-1]):
            values = {entry.key: entry.value for entry in scenario_status.values}
            self.assertEqual(
                scenario_status.level, DiagnosticStatus.OK, scenario_status.message
            )
            self.assertEqual(values.get("passed"), "true")
            self.assertEqual(values.get("stage_count"), "4")
            self.assertEqual(values.get("final_receiver_state"), "ready")

        self.assertTrue(
            self._spin_until(
                lambda: self._stage_titles(self.resync_markers)
                >= {
                    "READY BASELINE",
                    "DELTA DROPPED",
                    "GAP REJECTED",
                    "RESYNC RECOVERED",
                }
                and self._stage_titles(self.epoch_reset_markers)
                >= {
                    "EPOCH 1 READY",
                    "EPOCH 2 ADMITTED",
                    "OLD EPOCH REJECTED",
                    "EPOCH 2 READY",
                },
                timeout=3.0,
            ),
            "dynamic acceptance scenes did not complete one four-stage cycle",
        )
        self._assert_dynamic_scene(
            self.resync_markers,
            "replay_resync",
            "replay_resync_view",
            {
                "READY BASELINE",
                "DELTA DROPPED",
                "GAP REJECTED",
                "RESYNC RECOVERED",
            },
        )
        self._assert_dynamic_scene(
            self.epoch_reset_markers,
            "replay_epoch_reset",
            "replay_epoch_reset_view",
            {
                "EPOCH 1 READY",
                "EPOCH 2 ADMITTED",
                "OLD EPOCH REJECTED",
                "EPOCH 2 READY",
            },
        )

        transforms = {
            transform.child_frame_id: transform
            for message in self.static_transforms
            for transform in message.transforms
        }
        self.assertTrue(
            {
                "replay_oracle_map",
                "replay_reconstructed_map",
                "replay_difference_map",
                "replay_resync_view",
                "replay_epoch_reset_view",
            }.issubset(transforms)
        )
        self.assertTrue(
            all(
                transforms[frame].header.frame_id == "replay_world"
                for frame in (
                    "replay_oracle_map",
                    "replay_reconstructed_map",
                    "replay_difference_map",
                    "replay_resync_view",
                    "replay_epoch_reset_view",
                )
            )
        )

    @staticmethod
    def _stage_titles(messages):
        return {
            marker.text.splitlines()[0]
            for message in messages
            for marker in message.markers
            if marker.ns.endswith("/status") and marker.text
        }

    def _assert_dynamic_scene(
        self, messages, marker_prefix, expected_frame, expected_titles
    ):
        self.assertEqual(self._stage_titles(messages), expected_titles)
        observed_revisions = {}
        for message in messages:
            markers = [marker for marker in message.markers if marker.ns]
            map_markers = [
                marker for marker in markers if marker.ns.endswith("/map")
            ]
            status_markers = [
                marker for marker in markers if marker.ns.endswith("/status")
            ]
            self.assertEqual(len(map_markers), 1)
            self.assertEqual(len(status_markers), 1)
            self.assertTrue(map_markers[0].points)
            self.assertEqual(
                {marker.header.frame_id for marker in markers}, {expected_frame}
            )
            title = status_markers[0].text.splitlines()[0]
            self.assertTrue(
                status_markers[0].ns.startswith(f"{marker_prefix}/stage_")
            )
            receiver_line = status_markers[0].text.splitlines()[1]
            revision = int(receiver_line.rsplit("revision ", 1)[1])
            observed_revisions[title] = revision

        if marker_prefix == "replay_resync":
            self.assertEqual(
                observed_revisions["READY BASELINE"],
                observed_revisions["DELTA DROPPED"],
            )
            self.assertEqual(
                observed_revisions["READY BASELINE"],
                observed_revisions["GAP REJECTED"],
            )
            self.assertGreater(
                observed_revisions["RESYNC RECOVERED"],
                observed_revisions["GAP REJECTED"],
            )
        else:
            self.assertEqual(
                observed_revisions["EPOCH 1 READY"],
                observed_revisions["EPOCH 2 ADMITTED"],
            )
            self.assertEqual(
                observed_revisions["EPOCH 1 READY"],
                observed_revisions["OLD EPOCH REJECTED"],
            )
            self.assertEqual(observed_revisions["EPOCH 2 READY"], 1)

    def test_visualization_is_republished_to_volatile_late_subscribers(self):
        self.assertTrue(
            self._spin_until(lambda: self.diagnostics),
            "replay did not complete before late subscriptions were created",
        )

        late_map_markers = []
        late_differences = []
        late_resync_markers = []
        late_epoch_reset_markers = []
        volatile = QoSProfile(depth=4)
        volatile.reliability = ReliabilityPolicy.RELIABLE
        volatile.durability = DurabilityPolicy.VOLATILE
        subscriptions = [
            self.node.create_subscription(
                MarkerArray,
                "/map_update_replay/map_markers",
                late_map_markers.append,
                volatile,
            ),
            self.node.create_subscription(
                MarkerArray,
                "/map_update_replay/differences",
                late_differences.append,
                volatile,
            ),
            self.node.create_subscription(
                MarkerArray,
                "/map_update_replay/resync/markers",
                late_resync_markers.append,
                volatile,
            ),
            self.node.create_subscription(
                MarkerArray,
                "/map_update_replay/epoch_reset/markers",
                late_epoch_reset_markers.append,
                volatile,
            ),
        ]
        try:
            self.assertTrue(
                self._spin_until(
                    lambda: len(late_map_markers) >= 2
                    and len(late_differences) >= 2
                    and len(late_resync_markers) >= 2
                    and len(late_epoch_reset_markers) >= 2,
                    timeout=3.0,
                ),
                "late volatile subscribers did not receive periodic visualization",
            )
            map_namespaces = {
                marker.ns for marker in late_map_markers[-1].markers if marker.ns
            }
            self.assertEqual(
                map_namespaces,
                {
                    "replay_map/oracle_occupied",
                    "replay_map/reconstructed_occupied",
                },
            )
            self.assertEqual(
                sum(len(marker.points) for marker in late_differences[-1].markers),
                0,
            )
            self.assertEqual(
                len(
                    [
                        marker
                        for marker in late_resync_markers[-1].markers
                        if marker.ns.endswith("/map")
                    ]
                ),
                1,
            )
            self.assertEqual(
                len(
                    [
                        marker
                        for marker in late_epoch_reset_markers[-1].markers
                        if marker.ns.endswith("/map")
                    ]
                ),
                1,
            )
            first_stamp = late_map_markers[-2].markers[-1].header.stamp
            second_stamp = late_map_markers[-1].markers[-1].header.stamp
            self.assertLess(
                (first_stamp.sec, first_stamp.nanosec),
                (second_stamp.sec, second_stamp.nanosec),
            )
        finally:
            for subscription in subscriptions:
                self.node.destroy_subscription(subscription)


@launch_testing.post_shutdown_test()
class TestMapUpdateReplayVisualizationShutdown(unittest.TestCase):
    def test_launch_testing_completed(self):
        self.assertTrue(True)
