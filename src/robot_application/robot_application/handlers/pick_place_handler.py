"""Pick-and-place mission handler for MissionExecutor."""

import time
from typing import Any, Dict, List, Optional, Tuple

import rclpy
from rclpy.duration import Duration
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParametersClient

from aruco_interfaces.msg import ClusterPickability
from aruco_interfaces.srv import AlignToCluster as AlignToClusterSrv


class PickPlaceHandler:
    """Executes MOVE_OBJECT tasks with pickability and alignment checks."""

    def __init__(self, executor_node):
        self.node = executor_node
        self.latest_pickability: Optional[ClusterPickability] = None

        self.node.declare_parameter('pickability_topic', '/cluster_pickability')
        self.node.declare_parameter('align_service_name', '/align_to_cluster')
        self.node.declare_parameter('align_timeout_sec', 12.0)
        self.node.declare_parameter('align_threshold', 0.02)
        self.node.declare_parameter('pickability_wait_sec', 2.0)
        self.node.declare_parameter('tag_manager_node_name', '/tag_manager_node')

        pickability_topic = self.node.get_parameter('pickability_topic').value
        align_service_name = self.node.get_parameter('align_service_name').value
        tag_manager_node_name = self.node.get_parameter('tag_manager_node_name').value

        self.align_client = self.node.create_client(AlignToClusterSrv, align_service_name)
        self.tag_manager_params = AsyncParametersClient(self.node, str(tag_manager_node_name))
        self.pickability_sub = self.node.create_subscription(
            ClusterPickability,
            pickability_topic,
            self.pickability_callback,
            10
        )

    def can_handle(self, task: dict) -> bool:
        task_type = str(task.get('task_type', '')).lower()
        return task_type in ('move_object', 'pick_place', 'pick_and_place')

    def pickability_callback(self, msg: ClusterPickability):
        self.latest_pickability = msg
        
    def execute(self, task: dict) -> Dict[str, Any]:
        carry_object = bool(task.get('carry_object', False))
        source_pick_id = str(task.get('source_pick_id', ''))
        pick_ref: Optional[dict] = None

        if not carry_object:
            self._set_sticky_assignment(False)
            try:
                approach_positions = self.extract_approach_positions(task)
                approached = False
                for approach_pose in approach_positions[:2]:
                    if self.node.navigate_to_pose(
                        float(approach_pose.get('x', 0.0)),
                        float(approach_pose.get('y', 0.0)),
                        float(approach_pose.get('theta', 0.0))
                    ):
                        approached = True
                        break

                if not approached:
                    self.lower_pick_priority(task, pick_ref, 'approach unreachable')
                    return self._build_outcome(task, 'FAILED', 'NAV_FAIL', False, source_pick_id=source_pick_id)

                # Keep arm->tag mapping stable only while robot is in the pick window.
                self._set_sticky_assignment(True)

                pickability = self.wait_for_pickability()
                if pickability is None or not bool(pickability.is_pickable):
                    self.lower_pick_priority(task, pick_ref, 'not pickable')
                    return self._build_outcome(task, 'FAILED', 'PICK_EMPTY', False, source_pick_id=source_pick_id)

                cluster_id = int(getattr(pickability, 'cluster_id', 0))
                if not self.align_to_cluster(cluster_id):
                    self.lower_pick_priority(task, pick_ref, 'alignment failed')
                    return self._build_outcome(task, 'FAILED', 'ALIGN_FAIL', False, source_pick_id=source_pick_id)

                if not self.lower_arm_and_enable_pump(task):
                    self.lower_pick_priority(task, pick_ref, 'actuation failed')
                    return self._build_outcome(task, 'FAILED', 'ACTUATION_FAIL', False, source_pick_id=source_pick_id)

                carry_object = True
            finally:
                self._set_sticky_assignment(False)

        excluded_ids = set(task.get('excluded_drop_ids', []))
        drop_info = self.select_drop_location(task, excluded_ids=excluded_ids)
        if drop_info is None:
            if pick_ref is not None:
                self.lower_pick_priority(task, pick_ref, 'no drop location')
            return self._build_outcome(
                task,
                'REPLAN_REQUIRED',
                'DROP_FULL',
                carry_object,
                source_pick_id=source_pick_id,
            )

        drop_location, drop_ref = drop_info
        drop_id = str(drop_ref.get('id', task.get('target_drop_id', '')))
        if self._is_drop_full(task, drop_ref):
            excluded_ids.add(drop_id)
            task['excluded_drop_ids'] = sorted(list(excluded_ids))
            return self._build_outcome(
                task,
                'REPLAN_REQUIRED',
                'DROP_FULL',
                carry_object,
                source_pick_id=source_pick_id,
                target_drop_id=drop_id
            )

        if not self.node.navigate_to_pose(
            float(drop_location.get('x', 0.0)),
            float(drop_location.get('y', 0.0)),
            float(drop_location.get('theta', 0.0))
        ):
            if pick_ref is not None:
                self.lower_pick_priority(task, pick_ref, 'drop unreachable')
            return self._build_outcome(
                task,
                'FAILED',
                'NAV_FAIL',
                carry_object,
                source_pick_id=source_pick_id,
                target_drop_id=drop_id
            )

        return self._build_outcome(
            task,
            'COMPLETED',
            'PLACED',
            False,
            source_pick_id=source_pick_id,
            target_drop_id=drop_id
        )


    def _build_outcome(
        self,
        task: dict,
        status: str,
        outcome_reason: str,
        carry_object: bool,
        source_pick_id: str = '',
        target_drop_id: str = '',
    ) -> Dict[str, Any]:
        return {
            'task_id': str(task.get('task_id', 'unknown')),
            'task_type': str(task.get('task_type', 'unknown')),
            'status': status,
            'outcome_reason': outcome_reason,
            'carry_object': bool(carry_object),
            'source_pick_id': source_pick_id,
            'target_drop_id': target_drop_id,
        }

    def wait_for_pickability(self) -> Optional[ClusterPickability]:
        timeout = float(self.node.get_parameter('pickability_wait_sec').value)
        deadline = self.node.get_clock().now() + Duration(seconds=timeout)

        while self.node.get_clock().now() < deadline and not self.node.stop_requested:
            if self.latest_pickability is not None:
                return self.latest_pickability
            time.sleep(0.05)

        return None

    def align_to_cluster(self, cluster_id: int) -> bool:
        if not self.align_client.wait_for_service(timeout_sec=2.0):
            self.node.get_logger().error('AlignToCluster service not available')
            return False

        request = AlignToClusterSrv.Request()
        request.cluster_id = cluster_id
        request.alignment_threshold = float(self.node.get_parameter('align_threshold').value)
        request.max_wait_time = float(self.node.get_parameter('align_timeout_sec').value)

        timeout = float(self.node.get_parameter('align_timeout_sec').value)
        future = self.align_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout + 1.0)
        
        if future.result() is None:
            self.node.get_logger().error('AlignToCluster service call timed out')
            return False

        response = future.result()
        if not response.success:
            self.node.get_logger().error(f'Alignment failed: {response.status_message}')
            return False

        self.node.get_logger().info(f'Alignment successful: {response.status_message}')
        return bool(response.success)

    def lower_arm_and_enable_pump(self, task: dict) -> bool:
        servo_id = int(self.node.get_parameter('arm_lower_servo_id').value)
        angle = float(self.node.get_parameter('arm_lower_angle_deg').value)
        speed = float(self.node.get_parameter('arm_lower_speed_deg_s').value)
        pump_id = int(self.node.get_parameter('pump_id').value)
        duty_cycle = float(self.node.get_parameter('pump_pick_duty_cycle').value)
        default_duration = float(self.node.get_parameter('pump_pick_duration_sec').value)
        duration = float(task.get('pickup_duration', default_duration))

        if not self.node.move_servo(servo_id, angle=angle, speed=speed):
            return False

        return self.node.control_pump(pump_id, enable=True, duty_cycle=duty_cycle, duration=duration)

    def lower_pick_priority(self, task: dict, pick_ref: Optional[dict], reason: str):
        penalty = int(self.node.get_parameter('priority_penalty').value)
        if isinstance(pick_ref, dict) and 'priority' in pick_ref:
            pick_ref['priority'] = max(0, int(pick_ref.get('priority', 0)) - penalty)
        elif 'priority' in task:
            task['priority'] = max(0, int(task.get('priority', 0)) - penalty)

        self.node.get_logger().warn(f'Pick priority lowered ({reason})')

    def _set_sticky_assignment(self, enabled: bool) -> None:
        if not self.tag_manager_params.wait_for_service(timeout_sec=0.2):
            return

        param = Parameter('sticky_assignment', Parameter.Type.BOOL, bool(enabled))
        future = self.tag_manager_params.set_parameters([param])
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=0.5)
