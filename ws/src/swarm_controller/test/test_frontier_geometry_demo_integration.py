import time
import unittest

import launch
import launch.actions
import launch_ros.actions
import launch_testing.actions
import launch_testing.asserts
import rclpy
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from visualization_msgs.msg import Marker, MarkerArray


STAGES = (
    'standard_tunnel_geometry',
    'single_ring',
    'bootstrap_yaw_sweep',
    'validated_observation_hop',
    'accumulated_frontier',
)


def _topic(stage):
    return f'/frontier_geometry_demo/stages/{stage}/markers'


def generate_test_description():
    demo = launch_ros.actions.Node(
        package='swarm_controller',
        executable='frontier_geometry_demo',
        name='frontier_geometry_demo',
        output='screen',
        parameters=[{
            'scene.compose_stages': True,
            'scan.ray_count': 36,
            'scan.yaw_steps': 36,
            'display.republish_rate_hz': 0.0,
        }],
    )
    return (
        launch.LaunchDescription([
            demo,
            launch.actions.TimerAction(
                period=1.0,
                actions=[launch_testing.actions.ReadyToTest()],
            ),
        ]),
        {'demo': demo},
    )


class TestFrontierGeometryDemoIntegration(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_frontier_geometry_demo')

    def tearDown(self):
        self.node.destroy_node()

    def _spin_until(self, predicate, timeout=10.0):
        deadline = time.time() + timeout
        while time.time() < deadline and not predicate():
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def test_composed_topics_are_latched_and_stage_scoped(self):
        self._spin_until(
            lambda: all(
                len(self.node.get_publishers_info_by_topic(_topic(stage))) == 1
                for stage in STAGES
            ),
            timeout=5.0,
        )

        for stage in STAGES:
            publishers = self.node.get_publishers_info_by_topic(_topic(stage))
            self.assertEqual(len(publishers), 1)
            self.assertEqual(
                publishers[0].qos_profile.reliability,
                ReliabilityPolicy.RELIABLE,
            )
            self.assertEqual(
                publishers[0].qos_profile.durability,
                DurabilityPolicy.TRANSIENT_LOCAL,
            )

        time.sleep(0.25)
        messages = {stage: [] for stage in STAGES}
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        subscriptions = []
        for stage in STAGES:
            subscriptions.append(self.node.create_subscription(
                MarkerArray,
                _topic(stage),
                lambda message, selected=stage: messages[selected].append(message),
                qos,
            ))

        self._spin_until(
            lambda: all(messages[stage] for stage in STAGES),
            timeout=5.0,
        )
        self.assertTrue(all(messages[stage] for stage in STAGES))

        namespaces = {}
        for stage in STAGES:
            markers = messages[stage][-1].markers
            self.assertGreater(len(markers), 1)
            self.assertEqual(markers[0].action, Marker.DELETEALL)
            self.assertTrue(all(
                marker.header.frame_id == 'map' for marker in markers
            ))
            namespaces[stage] = {marker.ns for marker in markers[1:]}

        self.assertIn('tunnel', namespaces['standard_tunnel_geometry'])
        self.assertIn(
            'normal_scan_plane', namespaces['standard_tunnel_geometry'])
        self.assertIn(
            'pitched_scan_ring', namespaces['standard_tunnel_geometry'])
        self.assertNotIn(
            'voxel_free', namespaces['standard_tunnel_geometry'])
        self.assertIn('scan_rays', namespaces['single_ring'])
        self.assertNotIn('voxel_free', namespaces['single_ring'])
        self.assertIn('voxel_free', namespaces['bootstrap_yaw_sweep'])
        self.assertNotIn(
            'known_free_hop', namespaces['bootstrap_yaw_sweep'])
        self.assertIn(
            'known_free_hop', namespaces['validated_observation_hop'])
        self.assertNotIn(
            'frontier_selected_endpoint',
            namespaces['validated_observation_hop'],
        )
        self.assertIn(
            'frontier_selected_endpoint', namespaces['accumulated_frontier'])

        for subscription in subscriptions:
            self.node.destroy_subscription(subscription)


@launch_testing.post_shutdown_test()
class TestFrontierGeometryDemoShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
