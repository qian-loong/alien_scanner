import importlib.util
import os
from pathlib import Path
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
from launch_ros.actions import Node
from perception_interfaces.msg import HealthState, LidarObservation, LocalMapState
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan


TEST_DOMAIN_ID = str(200 + (os.getpid() % 20))
os.environ["ROS_DOMAIN_ID"] = TEST_DOMAIN_ID


def _package_share():
    return Path(get_package_share_directory("perception_local_map"))


def _load_scene_config():
    module_path = _package_share() / "launch" / "cave_full_ray_scene_config.py"
    spec = importlib.util.spec_from_file_location("cave_full_ray_scene_config", module_path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module.load_scene_config(
        _package_share() / "config" / "cave_full_ray_scene.yaml"
    )


@pytest.mark.launch_test
def generate_test_description():
    launch_file = _package_share() / "launch" / "cave_full_ray_scene.launch.py"
    system = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch_file)),
        launch_arguments={
            "show_rviz": "false",
            "map_update_enabled": "true",
        }.items(),
    )
    receiver = Node(
        package="perception_map_update",
        executable="perception_map_update_receiver_node",
        name="cave_full_ray_map_update_receiver",
        output="screen",
        parameters=[
            {
                "map_update_topic": "/cave_scene/local_map/updates",
                "local_map_state_topic": "/cave_scene/local_map/state",
                "map_resync_service": "/cave_scene/local_map/request_resync",
                "reconstructed_octomap_topic":
                    "/cave_scene/map_update_receiver/octomap",
                "expected_vehicle_id": "cave-scene-vehicle",
                "requester_id": "cave-full-ray-map-update-receiver",
                "resync_retry_period_s": 0.1,
            }
        ],
    )
    return launch.LaunchDescription(
        [
            launch.actions.SetEnvironmentVariable("ROS_DOMAIN_ID", TEST_DOMAIN_ID),
            system,
            receiver,
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestCaveFullRaySceneIntegration(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = _load_scene_config()
        cls.topics = cls.config["topics"]
        rclpy.init()
        cls.node = rclpy.create_node("cave_full_ray_scene_integration_test")
        cls.states = []
        cls.observations = []
        cls.released_scans = []
        cls.health = []
        cls.diagnostics = []
        cls.last_identity = None
        cls.last_identity_change_at = time.monotonic()

        state_qos = QoSProfile(depth=1000)
        state_qos.reliability = ReliabilityPolicy.RELIABLE
        state_qos.durability = DurabilityPolicy.VOLATILE
        sensor_qos = QoSProfile(depth=1000)
        sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        sensor_qos.durability = DurabilityPolicy.VOLATILE
        reliable_qos = QoSProfile(depth=1000)
        reliable_qos.reliability = ReliabilityPolicy.RELIABLE
        reliable_qos.durability = DurabilityPolicy.VOLATILE

        cls.subscriptions = [
            cls.node.create_subscription(
                LocalMapState,
                cls.topics["local_map_state"],
                cls._on_state,
                state_qos,
            ),
            cls.node.create_subscription(
                LidarObservation,
                cls.topics["observations"],
                cls.observations.append,
                sensor_qos,
            ),
            cls.node.create_subscription(
                LaserScan,
                cls.topics["released_scan"],
                cls.released_scans.append,
                sensor_qos,
            ),
            cls.node.create_subscription(
                HealthState,
                cls.topics["health"],
                cls.health.append,
                reliable_qos,
            ),
            cls.node.create_subscription(
                DiagnosticArray,
                "/diagnostics",
                cls.diagnostics.append,
                reliable_qos,
            ),
        ]

    @classmethod
    def tearDownClass(cls):
        for subscription in cls.subscriptions:
            cls.node.destroy_subscription(subscription)
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _on_state(cls, message):
        identity = (message.map_epoch, message.revision)
        if identity != cls.last_identity:
            cls.last_identity = identity
            cls.last_identity_change_at = time.monotonic()
        cls.states.append(message)

    def _spin_until(self, predicate, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if predicate():
                return True
        return False

    @staticmethod
    def _stamp_key(stamp):
        return stamp.sec, stamp.nanosec

    def _latest_committed_state(self):
        return next(
            (state for state in reversed(self.states) if state.revision > 0),
            None,
        )

    def _scene_has_covered_the_trajectory(self):
        state = self._latest_committed_state()
        if state is None or not state.has_known_bounds:
            return False
        unique_observations = {
            self._stamp_key(observation.header.stamp)
            for observation in self.observations
        }
        return (
            len(unique_observations) >= 100
            and state.revision >= 100
            and state.known_bounds_max.x - state.known_bounds_min.x >= 9.0
        )

    def _latest_diagnostic(self, name_fragment):
        for array in reversed(self.diagnostics):
            for status in array.status:
                if name_fragment in status.name:
                    return status, {entry.key: entry.value for entry in status.values}
        return None, {}

    def test_continuous_scene_advances_then_drains_without_stationary_refresh(self):
        self.assertTrue(
            self._spin_until(self._scene_has_covered_the_trajectory, timeout=30.0),
            "scene did not produce 100 unique observations and 9 m of X bounds",
        )

        drain_budget = (
            self.config["timing"]["pose_lead_delay_s"]
            + self.config["timing"]["transport_drain_s"]
            + 0.8
        )
        self.assertTrue(
            self._spin_until(
                lambda: time.monotonic() - self.last_identity_change_at >= drain_budget,
                timeout=8.0,
            ),
            "map revision did not settle after the trajectory ended",
        )

        committed_states = [state for state in self.states if state.revision > 0]
        self.assertGreater(len(committed_states), 0)
        epochs = {state.map_epoch for state in committed_states}
        self.assertEqual(len(epochs), 1)
        revisions = [state.revision for state in committed_states]
        self.assertTrue(
            all(left <= right for left, right in zip(revisions, revisions[1:]))
        )
        self.assertGreaterEqual(len(set(revisions)), 100)

        provenance_by_revision = {}
        for state in committed_states:
            provenance = (
                state.last_sensor_id,
                state.last_sensor_session_boot_time_ns,
                state.last_sensor_session_random_suffix,
                self._stamp_key(state.last_observation_stamp),
                state.last_observation_clock_domain,
            )
            if state.revision in provenance_by_revision:
                self.assertEqual(provenance_by_revision[state.revision], provenance)
            else:
                provenance_by_revision[state.revision] = provenance
        ordered_provenance = [
            provenance_by_revision[revision]
            for revision in sorted(provenance_by_revision)
        ]
        provenance_stamps = [entry[3] for entry in ordered_provenance]
        self.assertTrue(
            all(left < right for left, right in zip(provenance_stamps, provenance_stamps[1:]))
        )

        final_state = committed_states[-1]
        self.assertGreaterEqual(
            final_state.known_bounds_max.x - final_state.known_bounds_min.x,
            9.0,
        )
        self.assertEqual(final_state.last_sensor_id, self.config["sensor_contract"]["sensor_id"])
        self.assertEqual(
            final_state.last_observation_clock_domain,
            self.config["sensor_contract"]["clock_domain"],
        )
        self.assertTrue(
            any(
                message.mapper_contract_fingerprint
                == final_state.mapper_contract_fingerprint
                for message in self.health
            )
        )

        self.assertGreater(len(self.released_scans), 0)
        released_scan = self.released_scans[len(self.released_scans) // 2]
        scan_config = self.config["scan"]
        derived = self.config["derived"]
        self.assertEqual(released_scan.header.frame_id, self.config["frames"]["scan"])
        self.assertEqual(len(released_scan.ranges), scan_config["beam_count"])
        self.assertEqual(len(released_scan.intensities), 0)
        self.assertAlmostEqual(released_scan.angle_min, derived["angle_min_rad"], places=6)
        self.assertAlmostEqual(released_scan.angle_max, derived["angle_max_rad"], places=6)
        self.assertAlmostEqual(
            released_scan.angle_increment,
            derived["angle_increment_rad"],
            places=7,
        )
        self.assertAlmostEqual(released_scan.range_min, scan_config["range_min_m"], places=6)
        self.assertAlmostEqual(released_scan.range_max, scan_config["range_max_m"], places=6)
        self.assertEqual(released_scan.time_increment, 0.0)
        self.assertAlmostEqual(released_scan.scan_time, derived["scan_time_s"], places=6)

        self.assertGreater(len(self.observations), 0)
        observation = self.observations[len(self.observations) // 2]
        self.assertEqual(observation.data_type, LidarObservation.DATA_TYPE_SCAN_2D)
        self.assertEqual(
            observation.ray_evidence,
            LidarObservation.RAY_EVIDENCE_FULL_RAY,
        )
        self.assertEqual(observation.header.frame_id, released_scan.header.frame_id)
        self.assertEqual(len(observation.ranges), scan_config["beam_count"])
        self.assertEqual(len(observation.intensities), 0)
        self.assertAlmostEqual(observation.angle_min, released_scan.angle_min, places=6)
        self.assertAlmostEqual(observation.angle_max, released_scan.angle_max, places=6)
        self.assertAlmostEqual(
            observation.angle_increment,
            released_scan.angle_increment,
            places=7,
        )
        self.assertAlmostEqual(observation.range_min, released_scan.range_min, places=6)
        self.assertAlmostEqual(observation.range_max, released_scan.range_max, places=6)

        terminal_identity = (final_state.map_epoch, final_state.revision)
        terminal_provenance = provenance_by_revision[final_state.revision]
        terminal_sequence = self.states[-1].state_sequence
        self.assertTrue(
            self._spin_until(
                lambda: self.states[-1].state_sequence >= terminal_sequence + 3
                and not self.states[-1].map_fresh,
                timeout=3.0,
            ),
            "heartbeat did not continue through map freshness expiry",
        )
        after_drain = self._latest_committed_state()
        self.assertEqual(
            (after_drain.map_epoch, after_drain.revision), terminal_identity
        )
        self.assertEqual(
            (
                after_drain.last_sensor_id,
                after_drain.last_sensor_session_boot_time_ns,
                after_drain.last_sensor_session_random_suffix,
                self._stamp_key(after_drain.last_observation_stamp),
                after_drain.last_observation_clock_domain,
            ),
            terminal_provenance,
        )

        unique_observation_stamps = sorted(
            {
                observation.header.stamp.sec * 1_000_000_000
                + observation.header.stamp.nanosec
                for observation in self.observations
            }
        )
        self.assertGreaterEqual(len(unique_observation_stamps), 190)
        observation_duration_s = (
            unique_observation_stamps[-1] - unique_observation_stamps[0]
        ) / 1e9
        self.assertGreater(observation_duration_s, 0.0)
        self.assertGreaterEqual(
            (len(unique_observation_stamps) - 1) / observation_duration_s,
            9.5,
        )
        self.assertEqual(
            self._stamp_key(after_drain.last_observation_stamp),
            self._stamp_key(self.observations[-1].header.stamp),
        )

        def c3_drained():
            _, producer = self._latest_diagnostic(
                "cave_full_ray_local_map: map_update_producer"
            )
            _, receiver_values = self._latest_diagnostic(
                "cave_full_ray_map_update_receiver: map_update_receiver"
            )
            return (
                int(producer.get("published_revision", "0")) == terminal_identity[1]
                and producer.get("pending") == "0"
                and producer.get("in_flight") == "0"
                and int(producer.get("published_keyframes", "0")) >= 1
                and int(producer.get("published_deltas", "0")) >= 1
                and receiver_values.get("receiver_state") == "1"
                and int(receiver_values.get("map_epoch", "0")) == terminal_identity[0]
                and int(receiver_values.get("revision", "0")) == terminal_identity[1]
            )

        self.assertTrue(
            self._spin_until(c3_drained, timeout=3.0),
            "C3 producer/receiver did not drain to the authoritative terminal revision",
        )
        _, producer_timing = self._latest_diagnostic(
            "cave_full_ray_local_map: map_update_producer"
        )
        _, receiver_timing = self._latest_diagnostic(
            "cave_full_ray_map_update_receiver: map_update_receiver"
        )
        for key in (
            "traversal_duration_ns",
            "canonicalize_duration_ns",
            "content_hash_duration_ns",
            "validation_duration_ns",
            "diff_duration_ns",
            "encode_duration_ns",
            "update_hash_duration_ns",
        ):
            self.assertIn(key, producer_timing)
            self.assertGreaterEqual(int(producer_timing[key]), 0)
        self.assertIn("apply_duration_ns", receiver_timing)
        self.assertGreaterEqual(int(receiver_timing["apply_duration_ns"]), 0)

        mapper_diagnostics = [
            status.message.lower()
            for array in self.diagnostics
            for status in array.status
            if "cave_full_ray_local_map" in status.name
            and "mapper_input" in status.name
        ]
        forbidden_pose_rejections = (
            "no same-clock usable pose",
            "pose missing",
            "pose stale",
        )
        self.assertFalse(
            any(
                forbidden in message
                for message in mapper_diagnostics
                for forbidden in forbidden_pose_rejections
            ),
            mapper_diagnostics,
        )

        state_topic = self.topics["local_map_state"]
        map_topic = self.topics["local_map_octomap"]
        self.assertEqual(len(self.node.get_publishers_info_by_topic(state_topic)), 1)
        self.assertEqual(len(self.node.get_publishers_info_by_topic(map_topic)), 1)
        topic_names = {name for name, _ in self.node.get_topic_names_and_types()}
        self.assertNotIn("/scan_returns", topic_names)
        self.assertFalse(any("cloud_map" in name for name in topic_names))
        node_names = [name.lower() for name in self.node.get_node_names()]
        self.assertFalse(
            any(
                forbidden in name
                for name in node_names
                for forbidden in ("scan_accumulator", "octomap_builder", "fake_lidar")
            )
        )
        self.assertEqual(
            sum(name == "cave_full_ray_local_map" for name in node_names),
            1,
        )


@launch_testing.post_shutdown_test()
class TestCaveFullRaySceneExit(unittest.TestCase):
    def test_launch_completed_without_forced_shutdown(self):
        self.assertTrue(True)
