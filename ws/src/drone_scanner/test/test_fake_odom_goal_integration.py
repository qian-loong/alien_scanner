import math
import time
import unittest

import launch
import launch.actions
import launch_ros.actions
import launch_testing.actions
import launch_testing.asserts
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


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
                namespace='goal_test',
                output='screen',
                parameters=[{
                    'line.start_x': 0.0,
                    'line.start_y': 0.0,
                    'line.start_z': 1.5,
                    'publish_rate': 20.0,
                    'odom_frame': 'odom',
                    'base_frame': 'base_link',
                    'motion.mode': 'goal',
                    'motion.goal_frame': 'map',
                    'motion.linear_speed': 0.5,
                    'motion.yaw_rate': 1.0,
                    'altitude_adapt.enable': False,
                }],
            ),
            launch.actions.TimerAction(
                period=1.0,
                actions=[launch_testing.actions.ReadyToTest()],
            ),
        ]),
        {},
    )


class TestFakeOdomGoalIntegration(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_fake_odom_goal')
        self.odom = []
        self.odom_sub = self.node.create_subscription(
            Odometry,
            '/goal_test/odom',
            self.odom.append,
            10,
        )
        goal_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.goal_pub = self.node.create_publisher(
            PoseStamped,
            '/goal_test/motion_goal',
            goal_qos,
        )

    def tearDown(self):
        self.node.destroy_node()

    def _spin_for(self, seconds):
        deadline = time.time() + seconds
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    @staticmethod
    def _goal(x, y, yaw):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 99.0  # goal mode intentionally ignores z
        msg.pose.orientation.z = math.sin(yaw * 0.5)
        msg.pose.orientation.w = math.cos(yaw * 0.5)
        return msg

    @staticmethod
    def _yaw(odom):
        q = odom.pose.pose.orientation
        return 2.0 * math.atan2(q.z, q.w)

    def test_goal_deduplication_and_hold(self, proc_output):
        deadline = time.time() + 5.0
        while time.time() < deadline and not self.odom:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.assertTrue(self.odom)

        target = self._goal(0.5, 0.0, 0.5)
        self.goal_pub.publish(target)
        self._spin_for(0.5)
        self.goal_pub.publish(target)  # must not restart the segment
        self._spin_for(0.7)

        reached = self.odom[-1]
        self.assertAlmostEqual(reached.pose.pose.position.x, 0.5, delta=0.08)
        self.assertAlmostEqual(self._yaw(reached), 0.5, delta=0.08)
        self.assertAlmostEqual(reached.pose.pose.position.z, 1.5, delta=0.01)

        self.goal_pub.publish(self._goal(2.0, 0.0, 0.5))
        self._spin_for(0.4)
        moving = self.odom[-1]
        hold_x = moving.pose.pose.position.x
        self.goal_pub.publish(self._goal(hold_x, 0.0, 0.5))
        self._spin_for(0.4)

        held = self.odom[-1]
        self.assertAlmostEqual(held.pose.pose.position.x, hold_x, delta=0.03)
        self.assertAlmostEqual(held.twist.twist.linear.x, 0.0, delta=0.01)
        self.assertAlmostEqual(held.twist.twist.angular.z, 0.0, delta=0.01)


@launch_testing.post_shutdown_test()
class TestFakeOdomGoalShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
