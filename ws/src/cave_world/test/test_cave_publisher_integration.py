import time
import unittest

import launch
import launch.actions
import launch_ros.actions
import launch_testing.actions
import launch_testing.asserts
import rclpy
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2


def _cave_points_qos():
    return QoSProfile(
        depth=1,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
    )


def generate_test_description():
    return (
        launch.LaunchDescription([
            launch_ros.actions.Node(
                package='cave_world',
                executable='cave_publisher',
                name='cave_publisher',
                output='screen',
                parameters=[{
                    'seed': 42,
                    'publish_rate': 5.0,
                    'topic': '/cave/points',
                    'frame_id': 'map',
                }],
            ),
            launch.actions.TimerAction(
                period=1.0,
                actions=[launch_testing.actions.ReadyToTest()],
            ),
        ]),
        {},
    )


class TestCavePublisherIntegration(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_cave_publisher')

    def tearDown(self):
        self.node.destroy_node()

    def _collect_messages(self, seconds):
        msgs = []
        sub = self.node.create_subscription(
            PointCloud2,
            '/cave/points',
            lambda msg: msgs.append(msg),
            _cave_points_qos(),
        )
        try:
            end_time = time.time() + seconds
            while time.time() < end_time:
                rclpy.spin_once(self.node, timeout_sec=0.2)
        finally:
            self.node.destroy_subscription(sub)
        return msgs

    def test_receives_cave_points(self, proc_output):
        msgs = self._collect_messages(3.0)
        self.assertGreater(len(msgs), 0, 'expected /cave/points messages')
        cloud = msgs[-1]
        self.assertEqual(cloud.header.frame_id, 'map')
        self.assertGreater(cloud.width * cloud.height, 0)

    def test_transient_local_reaches_late_subscriber(self, proc_output):
        time.sleep(0.5)
        msgs = self._collect_messages(2.0)
        self.assertGreater(len(msgs), 0, 'TRANSIENT_LOCAL cache should reach new subscriber')


@launch_testing.post_shutdown_test()
class TestCavePublisherShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
