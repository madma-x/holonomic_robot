#!/usr/bin/env python3
"""Pick and place mission driven by planner-assigned tasks."""

import json
import os
import time
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory
import yaml

from aruco_interfaces.msg import ClusterPickability
from aruco_interfaces.action import AlignToCluster
from robot_application.mission_base import MissionBase, MissionState


class PickPlaceMission(MissionBase):
	"""Mission execution for prioritized pick-and-place tasks."""

	def __init__(self):
		super().__init__('pick_place')

		self.tasks: List[dict] = []
		self.current_task_idx = 0
		self.pickability_by_cluster: Dict[int, ClusterPickability] = {}

		self.declare_parameter('pickability_topic', '/cluster_pickability')
		self.declare_parameter('task_assignment_topic', '/planner/task_assignment')
		self.declare_parameter('align_action_name', '/align_to_cluster')
		self.declare_parameter('align_threshold', 0.03)
		self.declare_parameter('align_timeout_sec', 12.0)
		self.declare_parameter('pickability_wait_sec', 2.0)
		self.declare_parameter('priority_penalty', 10)
		self.declare_parameter('arm_lower_servo_id', 1)
		self.declare_parameter('arm_lower_angle_deg', 90.0)
		self.declare_parameter('arm_lower_speed_deg_s', 30.0)
		self.declare_parameter('pump_id', 0)
		self.declare_parameter('pump_pick_duty_cycle', 0.8)
		self.declare_parameter('pump_pick_duration_sec', 2.0)

		pickability_topic = self.get_parameter('pickability_topic').value
		assignment_topic = self.get_parameter('task_assignment_topic').value
		align_action_name = self.get_parameter('align_action_name').value

		self.align_client = ActionClient(self, AlignToCluster, align_action_name)
		self.pickability_sub = self.create_subscription(
			ClusterPickability,
			pickability_topic,
			self.pickability_callback,
			10
		)
		self.assignment_sub = self.create_subscription(
			String,
			assignment_topic,
			self.task_assignment_callback,
			10
		)

	def pickability_callback(self, msg: ClusterPickability):
		self.pickability_by_cluster[int(msg.cluster_id)] = msg

	def task_assignment_callback(self, msg: String):
		try:
			payload = json.loads(msg.data)
		except Exception as error:
			self.get_logger().warn(f'Failed to parse task assignment JSON: {error}')
			return

		assigned: List[dict] = []
		if isinstance(payload, dict) and isinstance(payload.get('tasks'), list):
			assigned = [task for task in payload.get('tasks', []) if isinstance(task, dict)]
		elif isinstance(payload, list):
			assigned = [task for task in payload if isinstance(task, dict)]
		elif isinstance(payload, dict):
			assigned = [payload]

		if assigned:
			self.tasks.extend(assigned)
			self.get_logger().info(f'Received {len(assigned)} planner task(s)')

	def load_mission_config(self):
		config_dir = get_package_share_directory('robot_application')
		config_file = os.path.join(config_dir, 'config', 'pick_place_tasks.yaml')

		try:
			with open(config_file, 'r', encoding='utf-8') as file:
				config = yaml.safe_load(file) or {}
			self.tasks = list(config.get('pick_place_tasks', []))
			self.get_logger().info(f'Loaded {len(self.tasks)} task(s) from config')
		except Exception as error:
			self.get_logger().error(f'Failed to load mission config: {error}')
			self.tasks = []

	def execute_mission(self):
		self.get_logger().info('Starting pick-place mission execution')

		if not self.tasks:
			self.get_logger().warn('No tasks loaded. Waiting for planner task assignments...')

		try:
			executed = 0
			while not self.stop_requested:
				if not self.tasks:
					time.sleep(0.2)
					continue

				task = self.tasks.pop(0)
				self.current_task_idx += 1

				result = self.execute_pick_place_task(task)
				if result:
					executed += 1
					self.get_logger().info(
						f'Task completed stage-1 flow: {task.get("name", "unnamed_task")}'
					)
				else:
					self.get_logger().warn(
						f'Task canceled/deferred: {task.get("name", "unnamed_task")}'
					)

				self.progress = min(100.0, float(executed) * 10.0)

			self.state = MissionState.IDLE

		except Exception as error:
			self.get_logger().error(f'Mission execution error: {error}')
			self.state = MissionState.FAILED
			return

		if self.state != MissionState.FAILED:
			self.state = MissionState.COMPLETED

	def execute_pick_place_task(self, task: dict) -> bool:
		pick_info = self.select_pick_location(task)
		if pick_info is None:
			self.get_logger().warn('No valid pick location in task')
			return False

		pick_location, pick_ref = pick_info
		approach_positions = self.extract_approach_positions(task, pick_ref)
		if not approach_positions:
			approach_positions = [pick_location]

		approached = False
		for approach_pose in approach_positions[:2]:
			if self.navigate_to_pose(
				float(approach_pose.get('x', 0.0)),
				float(approach_pose.get('y', 0.0)),
				float(approach_pose.get('theta', 0.0))
			):
				approached = True
				break

		if not approached:
			self.lower_pick_priority(task, pick_ref, 'approach unreachable')
			return False

		cluster_id = int(pick_ref.get('cluster_id', task.get('cluster_id', -1)))
		pickability = self.wait_for_pickability(cluster_id)
		if pickability is None or not bool(pickability.is_pickable):
			self.lower_pick_priority(task, pick_ref, 'not pickable')
			return False

		if not self.align_to_cluster(cluster_id):
			self.lower_pick_priority(task, pick_ref, 'alignment failed')
			return False

		if not self.lower_arm_and_enable_pump(task):
			self.lower_pick_priority(task, pick_ref, 'actuation failed')
			return False

		drop_location = self.select_drop_location(task)
		if drop_location is None:
			self.lower_pick_priority(task, pick_ref, 'no drop location')
			return False

		if not self.navigate_to_pose(
			float(drop_location.get('x', 0.0)),
			float(drop_location.get('y', 0.0)),
			float(drop_location.get('theta', 0.0))
		):
			self.lower_pick_priority(task, pick_ref, 'drop unreachable')
			return False

		return True

	def select_pick_location(self, task: dict) -> Optional[Tuple[dict, dict]]:
		pick_locations = list(task.get('pick_locations', []))
		if pick_locations:
			pick_locations.sort(key=lambda item: int(item.get('priority', 0)), reverse=True)
			selected = pick_locations[0]
			return selected.get('location', {}), selected

		pickup_location = task.get('pickup_location')
		if isinstance(pickup_location, dict):
			return pickup_location, task

		return None

	def extract_approach_positions(self, task: dict, pick_ref: dict) -> List[dict]:
		positions = list(pick_ref.get('approach_positions', []))
		if not positions:
			positions = list(task.get('approach_points', []))

		normalized = []
		for position in positions:
			if not isinstance(position, dict):
				continue
			pose = position.get('pose') if isinstance(position.get('pose'), dict) else position
			normalized.append({
				'x': pose.get('x', 0.0),
				'y': pose.get('y', 0.0),
				'theta': pose.get('theta', 0.0),
				'priority': int(position.get('priority', 1))
			})

		normalized.sort(key=lambda item: int(item.get('priority', 1)))
		return normalized

	def select_drop_location(self, task: dict) -> Optional[dict]:
		drop_positions = list(task.get('drop_positions', []))
		if drop_positions:
			drop_positions.sort(key=lambda drop: int(drop.get('priority', 999)))
			first_drop = drop_positions[0]
			location = first_drop.get('location', {})
			if isinstance(location, dict):
				return location

		place_location = task.get('place_location')
		if isinstance(place_location, dict):
			return place_location

		return None

	def wait_for_pickability(self, cluster_id: int) -> Optional[ClusterPickability]:
		timeout = float(self.get_parameter('pickability_wait_sec').value)
		deadline = self.get_clock().now() + Duration(seconds=timeout)

		while self.get_clock().now() < deadline and not self.stop_requested:
			if cluster_id in self.pickability_by_cluster:
				return self.pickability_by_cluster[cluster_id]
			time.sleep(0.05)

		if cluster_id == -1 and self.pickability_by_cluster:
			return next(iter(self.pickability_by_cluster.values()))

		return None

	def align_to_cluster(self, cluster_id: int) -> bool:
		if not self.align_client.wait_for_server(timeout_sec=2.0):
			self.get_logger().error('AlignToCluster action server not available')
			return False

		goal = AlignToCluster.Goal()
		goal.cluster_id = cluster_id
		goal.alignment_threshold = float(self.get_parameter('align_threshold').value)

		send_future = self.align_client.send_goal_async(goal)
		rclpy.spin_until_future_complete(self, send_future, timeout_sec=3.0)
		goal_handle = send_future.result()
		if goal_handle is None or not goal_handle.accepted:
			return False

		timeout = float(self.get_parameter('align_timeout_sec').value)
		result_future = goal_handle.get_result_async()
		rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout)
		if result_future.result() is None:
			goal_handle.cancel_goal_async()
			return False

		return bool(result_future.result().result.success)

	def lower_arm_and_enable_pump(self, task: dict) -> bool:
		servo_id = int(self.get_parameter('arm_lower_servo_id').value)
		angle = float(self.get_parameter('arm_lower_angle_deg').value)
		speed = float(self.get_parameter('arm_lower_speed_deg_s').value)
		pump_id = int(self.get_parameter('pump_id').value)
		duty_cycle = float(self.get_parameter('pump_pick_duty_cycle').value)
		duration = float(task.get('pickup_duration', self.get_parameter('pump_pick_duration_sec').value))

		if not self.move_servo(servo_id, angle=angle, speed=speed):
			return False

		return self.control_pump(pump_id, enable=True, duty_cycle=duty_cycle, duration=duration)

	def lower_pick_priority(self, task: dict, pick_ref: dict, reason: str):
		penalty = int(self.get_parameter('priority_penalty').value)
		if 'priority' in pick_ref:
			pick_ref['priority'] = max(0, int(pick_ref.get('priority', 0)) - penalty)
		elif 'priority' in task:
			task['priority'] = max(0, int(task.get('priority', 0)) - penalty)

		self.get_logger().warn(f'Pick priority lowered ({reason})')


def main(args=None):
	rclpy.init(args=args)
	mission = PickPlaceMission()
	mission.load_mission_config()

	try:
		rclpy.spin(mission)
	except KeyboardInterrupt:
		pass
	finally:
		mission.destroy_node()
		rclpy.shutdown()


if __name__ == '__main__':
	main()

