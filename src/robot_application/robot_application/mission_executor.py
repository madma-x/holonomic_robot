"""Single mission executor node with pluggable handlers."""

import json
import time
from typing import List, Optional

import rclpy
from std_msgs.msg import String

from robot_application.mission_base import MissionBase, MissionState
from robot_application.handlers.pick_place_handler import PickPlaceHandler


class MissionExecutor(MissionBase):
    """Centralized mission executor using in-process mission handlers."""

    def __init__(self):
        super().__init__('mission_executor')

        self.declare_parameter('task_assignment_topic', '/planner/task_assignment')
        assignment_topic = self.get_parameter('task_assignment_topic').value

        self.task_queue: List[dict] = []
        self.handlers = [PickPlaceHandler(self)]

        self.assignment_sub = self.create_subscription(
            String,
            assignment_topic,
            self.task_assignment_callback,
            10
        )

    def task_assignment_callback(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except Exception as error:
            self.get_logger().warn(f'Failed to parse task assignment JSON: {error}')
            return

        tasks: List[dict] = []
        if isinstance(payload, dict) and isinstance(payload.get('tasks'), list):
            tasks = [task for task in payload.get('tasks', []) if isinstance(task, dict)]
        elif isinstance(payload, list):
            tasks = [task for task in payload if isinstance(task, dict)]
        elif isinstance(payload, dict):
            tasks = [payload]

        if tasks:
            self.task_queue.extend(tasks)
            self.get_logger().info(f'Queued {len(tasks)} task(s) for execution')

    def load_mission_config(self):
        """MissionExecutor has no static mission file to load."""
        return

    def execute_mission(self):
        """Execute queued tasks and terminate when queue is drained."""
        self.get_logger().info('MissionExecutor started mission execution')

        started_at = time.time()
        task_wait_timeout = 3.0
        completed = 0

        while not self.stop_requested:
            if not self.task_queue:
                if time.time() - started_at > task_wait_timeout and completed == 0:
                    self.get_logger().warn('No task received after start request')
                    self.state = MissionState.FAILED
                    return

                if completed > 0:
                    self.state = MissionState.COMPLETED
                    self.progress = 100.0
                    return

                time.sleep(0.05)
                continue

            task = self.task_queue.pop(0)
            handler = self._find_handler(task)
            if handler is None:
                self.get_logger().error(f"No handler for task_type={task.get('task_type')}")
                self.state = MissionState.FAILED
                return

            self.get_logger().info(f"Executing task_id={task.get('task_id', 'unknown')}")
            success = handler.execute(task)
            if not success:
                self.state = MissionState.FAILED
                return

            completed += 1
            self.progress = min(99.0, float(completed) * 20.0)

        self.state = MissionState.IDLE

    def _find_handler(self, task: dict) -> Optional[PickPlaceHandler]:
        for handler in self.handlers:
            if handler.can_handle(task):
                return handler
        return None


def main(args=None):
    rclpy.init(args=args)
    executor = MissionExecutor()

    try:
        rclpy.spin(executor)
    except KeyboardInterrupt:
        pass
    finally:
        executor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
