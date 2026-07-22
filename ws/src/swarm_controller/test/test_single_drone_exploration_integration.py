import math
import time
import unittest

import launch
import launch.actions
import launch.launch_description_sources
import launch_ros.substitutions
import launch_testing.actions
import launch_testing.asserts
import rclpy
from geometry_msgs.msg import PoseStamped
from octomap_msgs.msg import Octomap
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import MarkerArray


def generate_test_description():
    exploration_launch = launch.substitutions.PathJoinSubstitution([
        launch_ros.substitutions.FindPackageShare('swarm_controller'),
        'launch',
        'single_drone_exploration.launch.py',
    ])
    included = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            exploration_launch
        ),
        launch_arguments={
            'show_rviz': 'false',
            'show_cave_truth': 'false',
        }.items(),
    )
    return (
        launch.LaunchDescription([
            included,
            launch.actions.TimerAction(
                period=1.0,
                actions=[launch_testing.actions.ReadyToTest()],
            ),
        ]),
        {},
    )


class TestSingleDroneExplorationIntegration(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_single_drone_exploration')
        transient_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.goals = []
        self.maps = []
        self.markers = []
        self.node.create_subscription(
            PoseStamped,
            '/drone_0/motion_goal',
            self.goals.append,
            transient_qos,
        )
        self.node.create_subscription(
            Octomap,
            '/drone_0/octomap',
            self.maps.append,
            transient_qos,
        )
        self.node.create_subscription(
            MarkerArray,
            '/drone_0/exploration_markers',
            self.markers.append,
            10,
        )

    def tearDown(self):
        self.node.destroy_node()

    def test_observation_drives_yaw_rescan_command(self):
        deadline = time.time() + 12.0
        while time.time() < deadline and not (
            self.goals and self.maps and self.markers
        ):
            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.assertTrue(self.maps, 'OctoMap observation was not published')
        self.assertTrue(self.markers, 'exploration diagnostics were not published')
        self.assertTrue(self.goals, 'explorer did not issue a motion goal')

        goal = self.goals[-1]
        yaw = 2.0 * math.atan2(
            goal.pose.orientation.z,
            goal.pose.orientation.w,
        )
        self.assertEqual(goal.header.frame_id, 'map')
        self.assertAlmostEqual(goal.pose.position.x, 0.0, delta=0.1)
        self.assertAlmostEqual(goal.pose.position.y, 0.0, delta=0.1)
        yaw_step = math.pi / 4.0
        completed_steps = round(yaw / yaw_step)
        self.assertNotEqual(completed_steps, 0)
        self.assertAlmostEqual(yaw, completed_steps * yaw_step, delta=0.1)


@launch_testing.post_shutdown_test()
class TestSingleDroneExplorationShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
