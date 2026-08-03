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
from nav_msgs.msg import Odometry
from perception_interfaces.msg import HealthState, LidarObservation, LocalMapState, PoseEstimate
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan


TEST_DOMAIN_ID = str(160 + (os.getpid() % 20))
os.environ["ROS_DOMAIN_ID"] = TEST_DOMAIN_ID


@pytest.mark.launch_test
def generate_test_description():
    share = Path(get_package_share_directory("perception_profiling"))
    system = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            str(share / "launch" / "canonical_equivalence.launch.xml")
        )
    )
    return launch.LaunchDescription(
        [
            launch.actions.SetEnvironmentVariable("ROS_DOMAIN_ID", TEST_DOMAIN_ID),
            system,
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestCanonicalEquivalence(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("test_canonical_profile_equivalence")
        cls.raw_scans = []
        cls.released_scans = []
        cls.observations = []
        cls.observation_watermarks = []
        cls.poses = []
        cls.odometry = []
        cls.health = []
        cls.states = []
        cls.diagnostics = []
        cls.pose_watermark_ns = None

        sensor_qos = QoSProfile(depth=1000)
        sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        sensor_qos.durability = DurabilityPolicy.VOLATILE
        reliable_qos = QoSProfile(depth=1000)
        reliable_qos.reliability = ReliabilityPolicy.RELIABLE
        reliable_qos.durability = DurabilityPolicy.VOLATILE

        cls.subscriptions = [
            cls.node.create_subscription(
                LaserScan,
                "/cave_scene/raw_scan",
                cls.raw_scans.append,
                sensor_qos,
            ),
            cls.node.create_subscription(
                LaserScan,
                "/cave_scene/scan",
                cls.released_scans.append,
                sensor_qos,
            ),
            cls.node.create_subscription(
                LidarObservation,
                "/cave_scene/perception/observations",
                cls._on_observation,
                sensor_qos,
            ),
            cls.node.create_subscription(
                PoseEstimate,
                "/cave_scene/perception/pose",
                cls._on_pose,
                reliable_qos,
            ),
            cls.node.create_subscription(
                Odometry,
                "/cave_scene/odom",
                cls.odometry.append,
                reliable_qos,
            ),
            cls.node.create_subscription(
                HealthState,
                "/cave_scene/perception/health",
                cls.health.append,
                reliable_qos,
            ),
            cls.node.create_subscription(
                LocalMapState,
                "/cave_scene/local_map/state",
                cls.states.append,
                reliable_qos,
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
        for subscription in cls.subscriptions:
            cls.node.destroy_subscription(subscription)
        cls.node.destroy_node()
        rclpy.shutdown()

    @staticmethod
    def _stamp_ns(message):
        return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec

    @classmethod
    def _on_pose(cls, message):
        stamp_ns = cls._stamp_ns(message)
        cls.pose_watermark_ns = (
            stamp_ns
            if cls.pose_watermark_ns is None
            else max(cls.pose_watermark_ns, stamp_ns)
        )
        cls.poses.append(message)

    @classmethod
    def _on_observation(cls, message):
        cls.observations.append(message)
        cls.observation_watermarks.append(
            (cls._stamp_ns(message), cls.pose_watermark_ns)
        )

    def _spin_until(self, predicate, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if predicate():
                return True
        return predicate()

    def test_real_chain_matches_canonical_contract(self):
        self.assertTrue(
            self._spin_until(
                lambda: len(self.observations) >= 60
                and len(self.raw_scans) >= 60
                and len(self.released_scans) >= 60
                and any(state.revision >= 50 for state in self.states),
                25.0,
            ),
            "canonical chain did not produce the short equivalence window",
        )

        raw_by_stamp = {self._stamp_ns(message): message for message in self.raw_scans}
        released_by_stamp = {
            self._stamp_ns(message): message for message in self.released_scans
        }
        observation_by_stamp = {
            self._stamp_ns(message): message for message in self.observations
        }
        common_stamps = sorted(
            raw_by_stamp.keys() & released_by_stamp.keys() & observation_by_stamp.keys()
        )
        self.assertGreaterEqual(len(common_stamps), 50)
        sample_stamps = common_stamps[5:: max(1, len(common_stamps) // 5)][:5]
        for stamp_ns in sample_stamps:
            raw = raw_by_stamp[stamp_ns]
            released = released_by_stamp[stamp_ns]
            observation = observation_by_stamp[stamp_ns]
            self.assertEqual(list(raw.ranges), list(released.ranges))
            self.assertEqual(list(released.ranges), list(observation.ranges))
            self.assertEqual(observation.data_type, LidarObservation.DATA_TYPE_SCAN_2D)
            self.assertEqual(
                observation.ray_evidence,
                LidarObservation.RAY_EVIDENCE_FULL_RAY,
            )
            self.assertEqual(observation.header.frame_id, "scan_link")
            self.assertEqual(len(observation.ranges), 360)
            self.assertAlmostEqual(observation.range_min, 0.1, places=6)
            self.assertAlmostEqual(observation.range_max, 30.0, places=6)

        watermarked = [
            (stamp_ns, watermark_ns)
            for stamp_ns, watermark_ns in self.observation_watermarks
            if watermark_ns is not None
        ]
        self.assertGreaterEqual(len(watermarked), 50)
        self.assertTrue(
            all(watermark_ns >= stamp_ns + 50_000_000 for stamp_ns, watermark_ns in watermarked)
        )

        odometry_x = [message.pose.pose.position.x for message in self.odometry]
        self.assertGreaterEqual(len(odometry_x), 100)
        self.assertTrue(all(left <= right for left, right in zip(odometry_x, odometry_x[1:])))
        self.assertGreater(odometry_x[-1] - odometry_x[0], 2.0)

        committed = [state for state in self.states if state.revision > 0]
        self.assertTrue(committed)
        revisions = [state.revision for state in committed]
        self.assertTrue(all(left <= right for left, right in zip(revisions, revisions[1:])))
        self.assertEqual({state.map_epoch for state in committed}, {1})
        final_state = max(committed, key=lambda state: state.revision)
        self.assertGreaterEqual(final_state.revision, 50)
        self.assertTrue(final_state.has_known_bounds)
        self.assertGreater(
            final_state.known_bounds_max.x - final_state.known_bounds_min.x,
            2.0,
        )
        fingerprints = {
            message.mapper_contract_fingerprint
            for message in self.health
            if message.mapper_contract_fingerprint
        }
        self.assertEqual(fingerprints, {final_state.mapper_contract_fingerprint})
        self.assertEqual(
            final_state.maximum_active_ray_evidence,
            LocalMapState.RAY_EVIDENCE_FULL_RAY,
        )

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
        self.assertEqual([], [(status.level, status.message) for status in unexpected])

        node_names = set(self.node.get_node_names())
        self.assertTrue(
            {
                "cave_full_ray_truth",
                "cave_full_ray_odom",
                "cave_full_ray_scanner",
                "cave_full_ray_pose_gate",
                "cave_full_ray_perception_input",
                "cave_full_ray_local_map",
            }.issubset(node_names)
        )
        self.assertNotIn("scan_accumulator", node_names)
        self.assertNotIn("octomap_builder", node_names)
        topic_names = {name for name, _types in self.node.get_topic_names_and_types()}
        self.assertNotIn("/scan_returns", topic_names)
        self.assertFalse(any("cloud_map" in name for name in topic_names))
        self.assertEqual(
            len(
                self.node.get_publishers_info_by_topic(
                    "/cave_scene/local_map/state"
                )
            ),
            1,
        )


@launch_testing.post_shutdown_test()
class TestCanonicalEquivalenceShutdown(unittest.TestCase):
    def test_all_processes_exit_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
