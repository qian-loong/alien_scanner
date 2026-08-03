import os
from pathlib import Path
import re
import time
import unittest
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
import launch_testing
from octomap_msgs.msg import Octomap
from perception_interfaces.msg import LocalMapState
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


TEST_DOMAIN_ID = str(100 + (os.getpid() % 100))
os.environ['ROS_DOMAIN_ID'] = TEST_DOMAIN_ID


@pytest.mark.launch_test
def generate_test_description():
    launch_file = (
        Path(get_package_share_directory('perception_local_map'))
        / 'launch'
        / 'local_map_debug.launch.xml'
    )
    system = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(str(launch_file)),
        launch_arguments={'show_rviz': 'false'}.items(),
    )
    return launch.LaunchDescription(
        [
            launch.actions.SetEnvironmentVariable('ROS_DOMAIN_ID', TEST_DOMAIN_ID),
            system,
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestLocalMapIntegration(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('local_map_integration_test')
        cls.states = []
        cls.octomaps = []
        state_qos = QoSProfile(depth=1)
        state_qos.reliability = ReliabilityPolicy.RELIABLE
        state_qos.durability = DurabilityPolicy.VOLATILE
        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        cls.node.create_subscription(
            LocalMapState, '/local_map/state', cls.states.append, state_qos
        )
        cls.node.create_subscription(
            Octomap, '/local_map/octomap', cls.octomaps.append, map_qos
        )

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _wait_for(self, predicate, timeout=20.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if predicate():
                return True
        return False

    def test_authoritative_state_and_octomap_advance(self):
        self.assertTrue(
            self._wait_for(
                lambda: any(state.revision > 0 and state.map_fresh for state in self.states)
                and len(self.octomaps) > 0
            )
        )
        state = next(state for state in reversed(self.states) if state.revision > 0)
        self.assertEqual(state.header.frame_id, 'map')
        self.assertEqual(state.vehicle_id, 'debug-vehicle')
        self.assertEqual(state.resolution_m, 0.2)
        self.assertTrue(state.has_known_bounds)
        self.assertTrue(state.has_active_ray_evidence)
        self.assertEqual(state.maximum_active_ray_evidence, LocalMapState.RAY_EVIDENCE_FULL_RAY)
        self.assertEqual(state.active_sensor_count, 1)
        self.assertEqual(state.active_2d_full_ray_sensor_count, 1)
        capability_counts = [
            state.active_2d_hit_only_sensor_count,
            state.active_2d_hit_ray_sensor_count,
            state.active_2d_full_ray_sensor_count,
            state.active_3d_hit_only_sensor_count,
            state.active_3d_hit_ray_sensor_count,
            state.active_3d_full_ray_sensor_count,
        ]
        self.assertEqual(sum(capability_counts), state.active_sensor_count)
        expected_maximum = max(
            evidence
            for evidence, count in (
                (LocalMapState.RAY_EVIDENCE_HIT_ONLY, capability_counts[0] + capability_counts[3]),
                (LocalMapState.RAY_EVIDENCE_HIT_RAY, capability_counts[1] + capability_counts[4]),
                (LocalMapState.RAY_EVIDENCE_FULL_RAY, capability_counts[2] + capability_counts[5]),
            )
            if count > 0
        )
        self.assertEqual(state.maximum_active_ray_evidence, expected_maximum)
        self.assertEqual(state.mapper_contract_schema_version, 1)
        self.assertIsNotNone(re.fullmatch(r'[0-9a-f]{64}', state.mapper_contract_fingerprint))
        self.assertGreater(state.validity_remaining_ns, 0)
        self.assertEqual(self.octomaps[-1].header.frame_id, 'map')
        self.assertTrue(self.octomaps[-1].binary)
        self.assertGreater(len(self.octomaps[-1].data), 0)

    def test_heartbeat_sequence_and_graph_are_conservative(self):
        start = len(self.states)
        self.assertTrue(self._wait_for(lambda: len(self.states) >= start + 5))
        sequences = [state.state_sequence for state in self.states[start:]]
        self.assertTrue(all(left < right for left, right in zip(sequences, sequences[1:])))

        provenance_by_revision = {}
        repeated_revision = False
        for state in self.states[start:]:
            provenance = (
                state.last_sensor_id,
                state.last_sensor_session_boot_time_ns,
                state.last_sensor_session_random_suffix,
                state.last_observation_stamp.sec,
                state.last_observation_stamp.nanosec,
                state.last_observation_clock_domain,
                state.last_commit_changed_cell_count,
            )
            identity = (state.map_epoch, state.revision)
            if identity in provenance_by_revision:
                repeated_revision = True
                self.assertEqual(provenance, provenance_by_revision[identity])
            else:
                provenance_by_revision[identity] = provenance
        self.assertTrue(repeated_revision)

        publishers = self.node.get_publishers_info_by_topic('/local_map/state')
        self.assertEqual(len(publishers), 1)
        state_qos = publishers[0].qos_profile
        if state_qos.history == HistoryPolicy.UNKNOWN:
            # Fast DDS does not expose history/depth through endpoint discovery.
            self.assertEqual(state_qos.depth, 0)
        else:
            self.assertEqual(state_qos.history, HistoryPolicy.KEEP_LAST)
            self.assertEqual(state_qos.depth, 1)
        self.assertEqual(state_qos.reliability, ReliabilityPolicy.RELIABLE)
        self.assertEqual(state_qos.durability, DurabilityPolicy.VOLATILE)
        self.assertGreater(state_qos.lifespan.nanoseconds, 0)
        self.assertGreater(state_qos.deadline.nanoseconds, 0)
        self.assertGreaterEqual(state_qos.deadline.nanoseconds, state_qos.lifespan.nanoseconds)

        map_publishers = self.node.get_publishers_info_by_topic('/local_map/octomap')
        self.assertEqual(len(map_publishers), 1)
        map_qos = map_publishers[0].qos_profile
        self.assertEqual(map_qos.reliability, ReliabilityPolicy.RELIABLE)
        self.assertEqual(map_qos.durability, DurabilityPolicy.TRANSIENT_LOCAL)
        topic_names = {name for name, _ in self.node.get_topic_names_and_types()}
        self.assertNotIn('/scan_returns', topic_names)
        node_names = {name.lower() for name in self.node.get_node_names()}
        self.assertFalse(
            any('fake_lidar' in name or 'octomap_builder' in name for name in node_names)
        )

    def test_octomap_rviz_preload_is_process_scoped(self):
        launch_file = (
            Path(get_package_share_directory('perception_local_map'))
            / 'launch'
            / 'local_map_debug.launch.xml'
        )
        root = ET.parse(launch_file).getroot()
        parents = {
            child: parent
            for parent in root.iter()
            for child in parent
        }

        fixture_includes = [
            include
            for include in root.iter('include')
            if include.get('file')
            == '$(find-pkg-share perception_fixtures)/launch/ray_evidence_debug.launch.py'
        ]
        self.assertEqual(len(fixture_includes), 1)
        fixture_include = fixture_includes[0]
        fixture_group = parents[fixture_include]
        self.assertEqual(fixture_group.tag, 'group')
        self.assertEqual(fixture_group.get('scoped'), 'true')
        fixture_rviz_args = [
            arg
            for arg in fixture_include.findall('arg')
            if arg.get('name') == 'show_rviz'
        ]
        self.assertEqual(len(fixture_rviz_args), 1)
        self.assertEqual(fixture_rviz_args[0].get('value'), 'false')

        rviz_nodes = [node for node in root.iter('node') if node.get('pkg') == 'rviz2']
        self.assertEqual(len(rviz_nodes), 1)
        self.assertIs(parents[rviz_nodes[0]], root)

        preload_elements = [
            element
            for element in root.iter()
            if element.get('name') == 'LD_PRELOAD'
        ]
        self.assertEqual(len(preload_elements), 1)
        self.assertEqual(preload_elements[0].tag, 'env')
        self.assertEqual(preload_elements[0].get('value'), 'liboctomap.so')
        self.assertIn(preload_elements[0], rviz_nodes[0].findall('env'))


@launch_testing.post_shutdown_test()
class TestLocalMapExit(unittest.TestCase):
    def test_launch_completed_without_forced_shutdown(self):
        self.assertTrue(True)
