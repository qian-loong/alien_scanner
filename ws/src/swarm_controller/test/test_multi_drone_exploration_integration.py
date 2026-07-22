import time
import unittest

import launch
import launch.actions
import launch.launch_description_sources
import launch_ros.substitutions
import launch_testing.actions
import launch_testing.asserts
import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped
from octomap_msgs.msg import Octomap
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


def generate_test_description():
    exploration_launch = launch.substitutions.PathJoinSubstitution([
        launch_ros.substitutions.FindPackageShare('swarm_controller'),
        'launch',
        'multi_drone_exploration.launch.py',
    ])
    included = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            exploration_launch
        ),
        launch_arguments={
            'num_drones': '3',
            'show_rviz': 'false',
            'show_cave_truth': 'false',
            'control_rate': '3.0',
        }.items(),
    )
    return (
        launch.LaunchDescription([
            included,
            launch.actions.TimerAction(
                period=3.0,
                actions=[launch_testing.actions.ReadyToTest()],
            ),
        ]),
        {},
    )


class TestMultiDroneExplorationIntegration(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_multi_drone_exploration')
        transient_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.maps = {i: [] for i in range(3)}
        self.goals = {i: [] for i in range(3)}
        self.diagnostics = {i: [] for i in range(3)}
        self.global_maps = []
        self.global_diagnostics = []
        for i in range(3):
            ns = f'/drone_{i}'
            self.node.create_subscription(
                Octomap, f'{ns}/octomap', self.maps[i].append, transient_qos)
            self.node.create_subscription(
                PoseStamped, f'{ns}/motion_goal', self.goals[i].append, transient_qos)
            self.node.create_subscription(
                DiagnosticArray,
                f'{ns}/exploration_diagnostics',
                self.diagnostics[i].append,
                transient_qos,
            )
        self.node.create_subscription(
            Octomap, '/global_map', self.global_maps.append, transient_qos)
        self.node.create_subscription(
            DiagnosticArray,
            '/global_map_diagnostics',
            self.global_diagnostics.append,
            transient_qos,
        )

    def tearDown(self):
        self.node.destroy_node()

    def _spin_until(self, predicate, timeout_sec):
        deadline = time.time() + timeout_sec
        while time.time() < deadline and not predicate():
            rclpy.spin_once(self.node, timeout_sec=0.1)

    def test_each_drone_gets_octomap_and_motion_goal(self):
        self._spin_until(
            lambda: all(self.maps[i] and self.goals[i] for i in range(3)),
            timeout_sec=45.0,
        )
        for i in range(3):
            self.assertTrue(self.maps[i], f'drone_{i} octomap missing')
            self.assertTrue(self.goals[i], f'drone_{i} motion_goal missing')

    def _diagnostic_values(self, drone_index):
        messages = self.diagnostics[drone_index]
        if not messages or not messages[-1].status:
            return {}
        return {
            item.key: item.value
            for item in messages[-1].status[0].values
        }

    def _peer_explorer_subscriptions_present(self):
        for source in range(3):
            expected = {
                (f'/drone_{peer}', 'single_drone_explorer')
                for peer in range(3) if peer != source
            }
            for suffix in ('odom', 'motion_goal'):
                topic = f'/drone_{source}/{suffix}'
                endpoints = {
                    (info.node_namespace, info.node_name)
                    for info in self.node.get_subscriptions_info_by_topic(topic)
                }
                if not expected.issubset(endpoints):
                    return False
        return True

    def _global_diagnostic_values(self):
        if not self.global_diagnostics:
            return {}
        for status in self.global_diagnostics[-1].status:
            if status.name == 'global_map_merger':
                return {item.key: item.value for item in status.values}
        return {}

    def test_each_explorer_receives_two_fresh_peers(self):
        self._spin_until(
            lambda: all(
                self._diagnostic_values(i).get('configured_peers') == '2'
                and self._diagnostic_values(i).get('fresh_peer_positions') == '2'
                and self._diagnostic_values(i).get('clearance_contract_valid') == '1'
                for i in range(3)
            ) and self._peer_explorer_subscriptions_present(),
            timeout_sec=45.0,
        )
        for i in range(3):
            values = self._diagnostic_values(i)
            self.assertEqual(values.get('configured_peers'), '2')
            self.assertEqual(values.get('fresh_peer_positions'), '2')
            self.assertEqual(values.get('clearance_contract_valid'), '1')
            self.assertAlmostEqual(
                float(values['configured_altitude_clearance']), 0.41, places=3)
            self.assertAlmostEqual(
                float(values['required_vertical_clearance']), 0.40, places=3)
        self.assertTrue(self._peer_explorer_subscriptions_present())

    def test_global_map_receives_all_sources(self):
        self._spin_until(
            lambda: self.global_maps
            and self._global_diagnostic_values().get('accepted_sources') == '3',
            timeout_sec=45.0,
        )
        self.assertTrue(self.global_maps, 'global_map missing')
        message = self.global_maps[-1]
        self.assertEqual(message.header.frame_id, 'map')
        self.assertEqual(message.id, 'OcTree')
        self.assertFalse(message.binary)
        self.assertTrue(message.data)
        values = self._global_diagnostic_values()
        self.assertEqual(values.get('expected_sources'), '3')
        self.assertEqual(values.get('accepted_sources'), '3')
        self.assertGreater(int(values.get('known_voxels', '0')), 0)
        self.assertEqual(values.get('rejected_updates'), '0')
        last_duration_ms = float(values['last_merge_duration_ms'])
        max_duration_ms = float(values['max_merge_duration_ms'])
        self.assertLess(last_duration_ms, 1000.0)
        self.assertGreaterEqual(max_duration_ms, last_duration_ms)

    def test_global_map_is_transient_local_for_late_subscribers(self):
        self._spin_until(lambda: bool(self.global_maps), timeout_sec=45.0)
        late_maps = []
        transient_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        subscription = self.node.create_subscription(
            Octomap, '/global_map', late_maps.append, transient_qos)
        self._spin_until(lambda: bool(late_maps), timeout_sec=5.0)
        self.assertTrue(late_maps, 'late subscriber did not receive global_map')
        self.node.destroy_subscription(subscription)

@launch_testing.post_shutdown_test()
class TestMultiDroneExplorationShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
