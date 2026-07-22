import time
import unittest

import launch
import launch.actions
import launch_ros.actions
import launch_testing.actions
import launch_testing.asserts
import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Empty


def generate_test_description():
    common = {'use_sim_time': True}
    probe = launch_ros.actions.Node(
        package='swarm_controller',
        executable='global_task_allocator_integration_probe',
        name='global_task_allocator_clock_probe',
        output='screen',
        parameters=[common],
    )
    allocator = launch_ros.actions.Node(
        package='swarm_controller',
        executable='global_task_allocator',
        name='global_task_allocator_clock_reset',
        output='screen',
        parameters=[{
            **common,
            'map_frame': 'map',
            'drone_namespaces': ['drone_0', 'drone_1', 'drone_2'],
            'task_allocation.rate': 4.0,
            'task.lease': 1.0,
            'global_map.stale_timeout': 5.0,
            'global_map.diagnostics_timeout': 2.0,
            'drone.odom_timeout': 2.0,
            'drone.local_map_timeout': 5.0,
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


class TestGlobalTaskAllocatorClockReset(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_global_task_allocator_clock_reset')
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.diagnostics = []
        self.subscription = self.node.create_subscription(
            DiagnosticArray,
            '/global_task_diagnostics',
            self.diagnostics.append,
            qos,
        )
        clock_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.clock_publisher = self.node.create_publisher(
            Clock, '/clock', clock_qos)
        self.pause_inputs = self.node.create_publisher(
            Empty, '/test/global_task_allocator/pause_inputs', 1)
        self.resume_inputs = self.node.create_publisher(
            Empty, '/test/global_task_allocator/resume_inputs', 1)

    def tearDown(self):
        self.node.destroy_node()

    def _values(self):
        if not self.diagnostics:
            return {}
        statuses = [
            status for status in self.diagnostics[-1].status
            if status.name == 'global_task_allocator'
        ]
        if not statuses:
            return {}
        return {item.key: item.value for item in statuses[-1].values}

    def _publish_clock_until(self, seconds, predicate, timeout=10.0):
        message = Clock()
        message.clock.sec = seconds
        deadline = time.time() + timeout
        while time.time() < deadline:
            self.clock_publisher.publish(message)
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if predicate():
                return True
        return False

    def _publish_empty_when_connected(self, publisher):
        deadline = time.time() + 3.0
        while time.time() < deadline and publisher.get_subscription_count() == 0:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.assertGreater(publisher.get_subscription_count(), 0)
        for _ in range(3):
            publisher.publish(Empty())
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def test_lower_stamp_domain_recovers_after_clock_rollback(self):
        self.assertTrue(self._publish_clock_until(
            100,
            lambda: int(self._values().get(
                'global_map_applied_revision', '0')) > 0
            and all(int(self._values().get(
                f'drone_{index}.local_map_applied_revision', '0')) > 0
                for index in range(3)),
        ))
        first_sequence = int(self._values()['global_update_sequence'])
        first_consumed_revision = int(
            self._values()['global_map_consumed_revision'])

        self._publish_empty_when_connected(self.pause_inputs)
        self.assertTrue(self._publish_clock_until(
            50,
            lambda: int(self._values().get('ros_clock_resets', '0')) >= 1
            and self._values().get('global_map_applied_revision') == '0'
            and self._values().get('global_map_last_consumed_revision') == '0'
            and float(self._values().get(
                'global_map_applied_receive_age_seconds', '0')) == -1.0
            and float(self._values().get(
                'global_map_applied_header_age_seconds', '0')) == -1.0,
        ))

        self._publish_empty_when_connected(self.resume_inputs)
        self.assertTrue(self._publish_clock_until(
            50,
            lambda: int(self._values().get('ros_clock_resets', '0')) >= 1
            and int(self._values().get('global_update_sequence', '0'))
            > first_sequence
            and self._values().get('global_map_fresh') == '1'
            and self._values().get('global_map_diagnostics_healthy') == '1'
            and all(int(self._values().get(
                f'drone_{index}.local_map_applied_revision', '0')) > 0
                for index in range(3)),
        ))
        recovered = self._values()
        self.assertGreater(
            int(recovered['global_map_last_consumed_revision']),
            first_consumed_revision,
        )
        self.assertEqual(
            int(recovered['global_map_last_consumed_revision']),
            int(recovered['global_map_consumed_revision']),
        )
        self.assertEqual(
            recovered['global_map_last_consumed_applied'], '1')
        self.assertGreaterEqual(
            float(recovered[
                'global_map_last_consumed_receive_header_age_seconds']), 0.0)
        self.assertGreaterEqual(
            float(recovered[
                'global_map_last_consumed_consume_header_age_seconds']), 0.0)


@launch_testing.post_shutdown_test()
class TestGlobalTaskAllocatorClockResetShutdown(unittest.TestCase):

    def test_processes_exit_cleanly(self, proc_info, probe, allocator):
        launch_testing.asserts.assertExitCodes(
            proc_info, process=probe, allowable_exit_codes=[0, -15])
        launch_testing.asserts.assertExitCodes(proc_info, process=allocator)
