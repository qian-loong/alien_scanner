import time
import unittest

import launch
import launch.actions
import launch_ros.actions
import launch_testing.actions
import launch_testing.asserts
import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from octomap_msgs.msg import Octomap
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from swarm_controller_interfaces.msg import ExplorationTask
from std_msgs.msg import Empty


def generate_test_description():
    probe = launch_ros.actions.Node(
        package='swarm_controller',
        executable='global_task_allocator_integration_probe',
        name='global_task_allocator_integration_probe',
        output='screen',
        parameters=[{
            'diagnostics_reject_after_seconds': 3.0,
            'controlled_global_updates': True,
        }],
    )
    allocator = launch_ros.actions.Node(
        package='swarm_controller',
        executable='global_task_allocator',
        name='global_task_allocator',
        output='screen',
        parameters=[{
            'map_frame': 'map',
            'drone_namespaces': ['drone_0', 'drone_1', 'drone_2'],
            'task_allocation.rate': 4.0,
            'task.lease': 1.0,
            'global_map.stale_timeout': 10.0,
            'global_map.diagnostics_timeout': 1.0,
            'drone.odom_timeout': 1.0,
            'drone.local_map_timeout': 1.0,
            'resolution': 0.1,
            'frontier.column_stride_voxels': 1,
            'frontier.min_z_layers': 5,
            'frontier.min_z_span': 0.4,
            'frontier.support_depth': 0.2,
            'frontier.min_columns': 4,
            'frontier.min_area': 0.05,
            'frontier.min_span': 0.3,
            'frontier.min_direction_consistency': 0.3,
            'frontier.min_persistence_updates': 1,
            'frontier.min_persistence_time': 0.0,
            'frontier.missed_update_grace': 1,
            'allocation.first_hop_distance': 0.5,
            'allocation.activation_updates': 1,
            'allocation.activation_time': 0.0,
            'allocation.deactivation_grace': 0.0,
            'body.robot_radius': 0.05,
            'body.robot_half_height': 0.05,
            'body.safety_margin': 0.05,
            'body.vertical_margin': 0.05,
        }],
    )
    return (
        launch.LaunchDescription([
            probe,
            allocator,
            launch.actions.TimerAction(
                period=1.0,
                actions=[launch_testing.actions.ReadyToTest()],
            ),
        ]),
        {'probe': probe, 'allocator': allocator},
    )


class TestGlobalTaskAllocatorIntegration(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_global_task_allocator')
        self.qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.tasks = {index: [] for index in range(3)}
        self.subscriptions = [
            self.node.create_subscription(
                ExplorationTask,
                f'/drone_{index}/exploration_task',
                self.tasks[index].append,
                self.qos,
            )
            for index in range(3)
        ]
        self.global_maps = []
        self.same_global_trigger = self.node.create_publisher(
            Empty, '/test/global_task_allocator/publish_same_stamp_global_map', 1,
        )
        self.older_global_trigger = self.node.create_publisher(
            Empty, '/test/global_task_allocator/publish_older_global_map', 1,
        )
        self.diagnostics = []
        self.subscriptions.append(self.node.create_subscription(
            DiagnosticArray,
            '/global_task_diagnostics',
            self.diagnostics.append,
            self.qos,
        ))
        self.subscriptions.append(self.node.create_subscription(
            Octomap,
            '/global_map',
            self.global_maps.append,
            self.qos,
        ))

    def tearDown(self):
        self.node.destroy_node()

    def _spin_until(self, predicate, timeout=12.0):
        deadline = time.time() + timeout
        while time.time() < deadline and not predicate():
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _coordinated(self):
        if not all(self.tasks[index] for index in range(3)):
            return False
        modes = [self.tasks[index][-1].mode for index in range(3)]
        return modes.count(ExplorationTask.MODE_ASSIGNED) == 2 \
            and modes.count(ExplorationTask.MODE_STANDBY) == 1

    def _latest_values(self):
        if not self.diagnostics:
            return {}
        statuses = [
            status for status in self.diagnostics[-1].status
            if status.name == 'global_task_allocator'
        ]
        if not statuses:
            return {}
        return {item.key: item.value for item in statuses[-1].values}


    def test_unique_assignment_renewal_and_late_subscriber_qos(self):
        self._spin_until(self._coordinated)
        self.assertTrue(self._coordinated())
        self._spin_until(
            lambda: self._latest_values().get('global_update_sequence') == '1',
            timeout=2.0,
        )
        self.assertEqual(
            self._latest_values().get('global_update_sequence'), '1',
            'initial controlled global map must be processed alone',
        )
        self._spin_until(lambda: bool(self.global_maps), timeout=1.0)
        initial_stamp = rclpy.time.Time.from_msg(
            self.global_maps[-1].header.stamp).nanoseconds
        initial_global_count = len(self.global_maps)
        self._spin_until(
            lambda: self.same_global_trigger.get_subscription_count() > 0,
            timeout=1.0,
        )
        self.same_global_trigger.publish(Empty())
        self._spin_until(
            lambda: len(self.global_maps) > initial_global_count
            and rclpy.time.Time.from_msg(
                self.global_maps[-1].header.stamp).nanoseconds == initial_stamp,
            timeout=2.0,
        )
        self.assertGreater(len(self.global_maps), initial_global_count)
        self._spin_until(
            lambda: self._latest_values().get('global_update_sequence') == '2',
            timeout=2.0,
        )
        self.assertEqual(
            self._latest_values().get('global_update_sequence'), '2',
            'same-stamp merged-tree update must advance allocator observation',
        )

        previous_global_count = len(self.global_maps)
        self._spin_until(
            lambda: self.older_global_trigger.get_subscription_count() > 0,
            timeout=1.0,
        )
        self.older_global_trigger.publish(Empty())
        self._spin_until(
            lambda: len(self.global_maps) > previous_global_count
            and rclpy.time.Time.from_msg(
                self.global_maps[-1].header.stamp).nanoseconds < initial_stamp,
            timeout=2.0,
        )
        self.assertGreater(len(self.global_maps), previous_global_count)
        self.assertLess(
            rclpy.time.Time.from_msg(
                self.global_maps[-1].header.stamp).nanoseconds,
            initial_stamp,
        )
        self._spin_until(
            lambda: self._latest_values().get(
                'global_map_regressed_stamp') == '1',
            timeout=2.0,
        )
        self.assertEqual(
            self._latest_values().get('global_map_regressed_stamp'), '1',
            'allocator must observe and reject the older-stamp map',
        )
        self.assertEqual(
            self._latest_values().get('global_update_sequence'), '2',
            'older-stamp merged-tree update must be ignored',
        )

        assigned = [
            self.tasks[index][-1]
            for index in range(3)
            if self.tasks[index][-1].mode == ExplorationTask.MODE_ASSIGNED
        ]
        self.assertEqual(len({task.task_id for task in assigned}), 2)
        self.assertTrue(all(task.allocator_epoch > 0 for task in assigned))

        watched_index = next(
            index for index in range(3)
            if self.tasks[index][-1].mode == ExplorationTask.MODE_ASSIGNED)
        previous = self.tasks[watched_index][-1]
        previous_count = len(self.tasks[watched_index])
        self._spin_until(
            lambda: len(self.tasks[watched_index]) > previous_count,
            timeout=2.0,
        )
        renewed = self.tasks[watched_index][-1]
        self.assertEqual(renewed.revision, previous.revision)
        self.assertEqual(renewed.task_id, previous.task_id)
        self.assertGreater(
            rclpy.time.Time.from_msg(renewed.header.stamp).nanoseconds,
            rclpy.time.Time.from_msg(previous.header.stamp).nanoseconds,
        )

        late_messages = []
        late = self.node.create_subscription(
            ExplorationTask,
            f'/drone_{watched_index}/exploration_task',
            late_messages.append,
            self.qos,
        )
        self._spin_until(lambda: bool(late_messages), timeout=2.0)
        self.assertTrue(late_messages)
        self.assertEqual(late_messages[-1].task_id, renewed.task_id)
        self.node.destroy_subscription(late)

        self._spin_until(lambda: bool(self.diagnostics), timeout=2.0)
        statuses = [
            status for message in self.diagnostics
            for status in message.status
            if status.name == 'global_task_allocator'
        ]
        self.assertTrue(statuses)
        values = {item.key: item.value for item in statuses[-1].values}
        self.assertGreaterEqual(int(values['matching_cardinality']), 2)

        self._spin_until(lambda: all(
            self.tasks[index]
            and self.tasks[index][-1].mode == ExplorationTask.MODE_LOCAL_FALLBACK
            for index in range(3)
        ), timeout=5.0)
        self.assertTrue(all(
            self.tasks[index][-1].mode == ExplorationTask.MODE_LOCAL_FALLBACK
            for index in range(3)
        ))
        self._spin_until(
            lambda: self.diagnostics and any(
                item.key == 'global_map_diagnostics_healthy' and item.value == '0'
                for item in self.diagnostics[-1].status[0].values
            ),
            timeout=2.0,
        )
        latest_values = {
            item.key: item.value
            for item in self.diagnostics[-1].status[0].values
        }
        self.assertEqual(latest_values['global_map_diagnostics_healthy'], '0')


@launch_testing.post_shutdown_test()
class TestGlobalTaskAllocatorShutdown(unittest.TestCase):

    def test_processes_exit_cleanly(self, proc_info, probe, allocator):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            process=probe,
            allowable_exit_codes=[0, -15],
        )
        launch_testing.asserts.assertExitCodes(
            proc_info,
            process=allocator,
            allowable_exit_codes=[0, -15],
        )
