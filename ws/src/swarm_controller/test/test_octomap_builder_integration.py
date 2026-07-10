import time
import unittest

import launch
import launch.actions
import launch_ros.actions
import launch_testing.actions
import launch_testing.asserts
import rclpy
from octomap_msgs.msg import Octomap


def generate_test_description():
    return (
        launch.LaunchDescription([
            launch_ros.actions.Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='map_to_odom',
                arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            ),
            launch_ros.actions.Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='base_to_lidar',
                arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'lidar_link'],
            ),
            launch_ros.actions.Node(
                package='drone_scanner',
                executable='fake_odom',
                name='fake_odom',
                namespace='drone_0',
                output='screen',
                parameters=[{
                    'line.start_x': 0.0,
                    'line.start_y': 0.0,
                    'line.start_z': 1.5,
                    'line.end_x': 6.0,
                    'line.end_y': 0.0,
                    'line.end_z': 1.5,
                    'line.duration_seconds': 20.0,
                    'publish_rate': 20.0,
                    'odom_frame': 'odom',
                    'base_frame': 'base_link',
                }],
            ),
            launch_ros.actions.Node(
                package='drone_scanner',
                executable='fake_lidar',
                name='fake_lidar',
                namespace='drone_0',
                output='screen',
                parameters=[{
                    'cave_mode': 'tree',
                    'seed': 42,
                    'tree.loop_bulge': 12.0,
                    'tree.loop_direct_length': 16.0,
                    'density': 200,
                    'map_frame': 'map',
                    'lidar_frame': 'lidar_link',
                    'scan_rate': 10.0,
                    'num_beams': 90,
                    'max_range': 20.0,
                    'ring_pitch_rad': 0.35,
                    'range_noise_std': 0.0,
                    'noise_seed': 0,
                }],
            ),
            launch_ros.actions.Node(
                package='swarm_controller',
                executable='octomap_builder',
                name='octomap_builder',
                namespace='drone_0',
                output='screen',
                parameters=[{
                    'map_frame': 'map',
                    'input_topic': 'scan_returns',
                    'output_topic': 'octomap',
                    'resolution': 0.2,
                    'publish_rate': 2.0,
                    'max_range': 20.0,
                }],
            ),
            launch.actions.TimerAction(
                period=3.0,
                actions=[launch_testing.actions.ReadyToTest()],
            ),
        ]),
        {},
    )


class TestOctoMapBuilderIntegration(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_octomap_builder')

    def tearDown(self):
        self.node.destroy_node()

    def test_publishes_octomap(self, proc_output):
        msgs = []
        sub = self.node.create_subscription(
            Octomap,
            '/drone_0/octomap',
            lambda msg: msgs.append(msg),
            10,
        )
        try:
            deadline = time.time() + 6.0
            while time.time() < deadline and not msgs:
                rclpy.spin_once(self.node, timeout_sec=0.2)
        finally:
            self.node.destroy_subscription(sub)

        self.assertGreater(len(msgs), 0, 'expected /drone_0/octomap messages')
        msg = msgs[-1]
        self.assertEqual(msg.header.frame_id, 'map')
        self.assertIn('OcTree', msg.id)
        self.assertGreater(len(msg.data), 0)


@launch_testing.post_shutdown_test()
class TestOctoMapBuilderShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
