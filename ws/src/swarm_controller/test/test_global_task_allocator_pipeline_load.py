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


def generate_test_description():
    probe = launch_ros.actions.Node(
        package='swarm_controller',
        executable='global_task_allocator_integration_probe',
        name='global_task_allocator_load_probe',
        output='screen',
        parameters=[{
            'publish_period_ms': 1,
            'diagnostics_reject_after_seconds': -1.0,
        }],
    )
    allocator = launch_ros.actions.Node(
        package='swarm_controller',
        executable='global_task_allocator',
        name='global_task_allocator_load',
        output='screen',
        parameters=[{
            'map_frame': 'map',
            'drone_namespaces': ['drone_0', 'drone_1', 'drone_2'],
            'task_allocation.rate': 20.0,
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


class TestGlobalTaskAllocatorPipelineLoad(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_global_task_allocator_pipeline_load')
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

    def test_all_sources_progress_while_pending_inputs_coalesce(self):
        deadline = time.time() + 12.0
        backlog = None
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            values = self._values()
            if values and (
                    int(values.get('global_map_pending_coalesced', '0')) > 0
                    and int(values.get('global_map_applied_revision', '0')) > 0
                    and int(values.get('global_map_consumed_revision', '0'))
                    < int(values.get('global_map_latest_revision', '0'))
                    and any(int(values.get(
                        f'drone_{index}.local_map_pending_coalesced', '0')) > 0
                        for index in range(3))
                    and all(int(values.get(
                        f'drone_{index}.local_map_applied_revision', '0')) > 0
                        for index in range(3))):
                self.assertEqual(
                    int(values['global_map_last_consumed_revision']),
                    int(values['global_map_consumed_revision']),
                )
                self.assertLessEqual(
                    int(values['global_map_consumed_revision']),
                    int(values['global_map_latest_revision']),
                )
                self.assertLess(
                    int(values['global_map_last_consumed_revision']),
                    int(values['global_map_latest_revision']),
                )
                self.assertGreaterEqual(float(values[
                    'global_map_last_consumed_total_latency_seconds']), 0.0)
                for index in range(3):
                    prefix = f'drone_{index}.local_map_'
                    self.assertEqual(
                        int(values[prefix + 'last_consumed_revision']),
                        int(values[prefix + 'consumed_revision']),
                    )
                    self.assertLessEqual(
                        int(values[prefix + 'consumed_revision']),
                        int(values[prefix + 'latest_revision']),
                    )
                backlog = values
                break
        if backlog is None:
            self.fail('high-rate pipeline did not expose a coalesced backlog')

        consumed_revision = int(backlog['global_map_consumed_revision'])
        deadline = time.time() + 8.0
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            values = self._values()
            if int(values.get('global_map_consumed_revision', '0')) > consumed_revision:
                self.assertEqual(
                    int(values['global_map_last_consumed_revision']),
                    int(values['global_map_consumed_revision']),
                )
                return
        self.fail('global consumed timing did not advance after backlog snapshot')


@launch_testing.post_shutdown_test()
class TestGlobalTaskAllocatorPipelineLoadShutdown(unittest.TestCase):

    def test_processes_exit_cleanly(self, proc_info, probe, allocator):
        launch_testing.asserts.assertExitCodes(
            proc_info, process=probe, allowable_exit_codes=[0, -15])
        launch_testing.asserts.assertExitCodes(proc_info, process=allocator)
