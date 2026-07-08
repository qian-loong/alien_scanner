import time
import unittest

import launch
import launch.actions
import launch_ros.actions
import launch_testing.actions
import launch_testing.asserts
import rclpy
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from tf2_ros import Buffer, TransformListener


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
                package='drone_scanner',
                executable='fake_odom',
                name='fake_odom',
                namespace='drone_0',
                output='screen',
                parameters=[{
                    'line.start_x': 0.0,
                    'line.start_y': 0.0,
                    'line.start_z': 1.5,
                    'line.end_x': 10.0,
                    'line.end_y': 0.0,
                    'line.end_z': 1.5,
                    'line.duration_seconds': 20.0,
                    'publish_rate': 20.0,
                    'odom_frame': 'odom',
                    'base_frame': 'base_link',
                }],
            ),
            launch.actions.TimerAction(
                period=1.0,
                actions=[launch_testing.actions.ReadyToTest()],
            ),
        ]),
        {},
    )


class TestFakeOdomIntegration(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_fake_odom')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)

    def tearDown(self):
        self.node.destroy_node()

    def _collect_odom(self, seconds):
        msgs = []
        sub = self.node.create_subscription(
            Odometry,
            '/drone_0/odom',
            lambda msg: msgs.append(msg),
            10,
        )
        try:
            end_time = time.time() + seconds
            while time.time() < end_time:
                rclpy.spin_once(self.node, timeout_sec=0.2)
        finally:
            self.node.destroy_subscription(sub)
        return msgs

    def _wait_for_tf(self, parent, child, timeout_sec=5.0):
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            try:
                return self.tf_buffer.lookup_transform(
                    parent,
                    child,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.5),
                )
            except Exception:
                rclpy.spin_once(self.node, timeout_sec=0.2)
        self.fail(f'TF {parent} -> {child} not available within {timeout_sec}s')

    def test_receives_odom(self, proc_output):
        msgs = self._collect_odom(3.0)
        self.assertGreater(len(msgs), 0, 'expected /drone_0/odom messages')
        odom = msgs[-1]
        self.assertEqual(odom.header.frame_id, 'odom')
        self.assertEqual(odom.child_frame_id, 'base_link')

    def test_odom_position_advances_along_x(self, proc_output):
        msgs = self._collect_odom(4.0)
        self.assertGreaterEqual(len(msgs), 2, 'need multiple odom samples')
        self.assertGreater(
            msgs[-1].pose.pose.position.x,
            msgs[0].pose.pose.position.x,
            'base_link x should increase along +X trajectory',
        )

    def test_tf_odom_to_base_link(self, proc_output):
        transform = self._wait_for_tf('odom', 'base_link')
        self.assertEqual(transform.header.frame_id, 'odom')
        self.assertEqual(transform.child_frame_id, 'base_link')

    def test_publish_rate(self, proc_output):
        msgs = self._collect_odom(2.0)
        # 20 Hz target; allow CI jitter
        self.assertGreaterEqual(len(msgs), 25, f'expected ~40 msgs in 2s, got {len(msgs)}')
        self.assertLessEqual(len(msgs), 55)


@launch_testing.post_shutdown_test()
class TestFakeOdomShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
