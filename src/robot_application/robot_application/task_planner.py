"""Strategic task planner with dynamic prioritization.

Simplified architecture:
- Task prioritization based on game phase and utility
- Nav2 handles path-level replanning and obstacle avoidance
- Task planner only monitors goal-level failures (UNREACHABLE, TIMEOUT)
- Switch tasks on goal failure, but let Nav2 handle path decisions
"""

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import String, Float32, Int32
from std_srvs.srv import Trigger
from geometry_msgs.msg import Point, PoseStamped
import threading
import time
import json
from typing import List, Optional, Dict, Any

from robot_application.task_definitions import (
    Task, TaskType, TaskStatus,
    early_game_priority, mid_game_priority,
    late_game_priority, endgame_priority
)
from robot_application.season_task_composers import PickPlaceSeasonTaskComposer
from robot_application.task_adapters import PickPlaceTaskAdapter, ReturnBaseTaskAdapter
from robot_application.game_state_manager import GamePhase
from robot_application.world_state_manager import WorldStateManager


class TaskPlanner(Node):
    """
    Strategic task planner with dynamic prioritization.
    
    Re-evaluates task priorities based on:
    - Time remaining in match
    - Current score vs opponent
    - Robot position
    - Task completion history
    
    NOTE: Nav2 handles all path-level obstacle avoidance and replanning.
    TaskPlanner only monitors goal-level outcomes and switches tasks.
    """
    
    def __init__(self):
        super().__init__('task_planner')
        
        # Declare parameters
        self.declare_parameter('replan_interval_sec', 5.0)
        self.declare_parameter('enable_task_interruption', True)
        self.declare_parameter('min_utility_threshold', 0.5)
        self.declare_parameter('blue_base_location_x', 0.0)
        self.declare_parameter('blue_base_location_y', 0.0)
        self.declare_parameter('blue_base_location_theta', 0.0)
        self.declare_parameter('yellow_base_location_x', 0.0)
        self.declare_parameter('yellow_base_location_y', 0.0)
        self.declare_parameter('yellow_base_location_theta', 0.0)
        self.declare_parameter('mission_executor_assignment_topic', '/planner/task_assignment')
        self.declare_parameter('mission_executor_status_topic', '/mission_executor/mission_status')
        self.declare_parameter('mission_executor_start_service', '/mission_executor/start_mission')
        self.declare_parameter('team_color', 'blue')
        
        # Get parameters
        self.replan_interval = self.get_parameter('replan_interval_sec').value
        self.enable_interruption = self.get_parameter('enable_task_interruption').value
        self.min_utility_threshold = self.get_parameter('min_utility_threshold').value
        self.team_color = self._normalize_team_color(self.get_parameter('team_color').value)
        self.base_location = self._get_base_location_for_team(self.team_color)
        
        # Task management
        self.task_queue: List[Task] = []
        self.current_task: Optional[Task] = None
        self.completed_tasks: List[Task] = []
        self.failed_tasks: List[Task] = []
        self.world_state = WorldStateManager()
        self.task_composer = PickPlaceSeasonTaskComposer()
        self.recent_outcomes = set()
        self.last_mission_outcome: Optional[Dict[str, Any]] = None
        self.task_context_by_id: Dict[str, Dict[str, Any]] = {}
        self.task_adapters = {}
        
        # Game state tracking
        self.time_remaining = 180.0
        self.current_score = 0
        self.opponent_score = 0
        self.current_phase = GamePhase.SETUP
        self.match_active = False
        self.robot_position = Point()
        self.mission_executor_status = 'IDLE'
        
        # Subscribers for game state
        self.time_sub = self.create_subscription(
            Float32, '/game/time_remaining', self.time_callback, 10)
        self.score_sub = self.create_subscription(
            Int32, '/game/score', self.score_callback, 10)
        self.phase_sub = self.create_subscription(
            String, '/game/phase', self.phase_callback, 10)
        self.pose_sub = self.create_subscription(
            PoseStamped, '/odom', self.pose_callback, 10)
        
        # Publishers
        self.current_task_pub = self.create_publisher(String, '/planner/current_task', 10)
        self.queue_size_pub = self.create_publisher(Int32, '/planner/queue_size', 10)
        assignment_topic = self.get_parameter('mission_executor_assignment_topic').value
        status_topic = self.get_parameter('mission_executor_status_topic').value
        start_service = self.get_parameter('mission_executor_start_service').value

        self.mission_assignment_pub = self.create_publisher(String, assignment_topic, 10)
        self.mission_executor_status_sub = self.create_subscription(
            String,
            status_topic,
            self.mission_executor_status_callback,
            10
        )
        self.mission_executor_start_client = self.create_client(Trigger, start_service)

        # Task adapters: keep planner generic and delegate task-type specifics.
        self.task_adapters = {
            TaskType.MOVE_OBJECT: PickPlaceTaskAdapter(
                logger=self.get_logger(),
                mission_assignment_pub=self.mission_assignment_pub,
                world_state=self.world_state,
                task_context_by_id=self.task_context_by_id,
                task_queue=self.task_queue,
                task_pick_id_getter=self._task_pick_id,
                task_drop_candidates_getter=self._task_drop_candidates,
                start_mission_executor=self._start_mission_executor,
                wait_for_mission_executor_result=self._wait_for_mission_executor_result,
            ),
            TaskType.RETURN_BASE: ReturnBaseTaskAdapter(self.get_logger()),
        }
        
        # Services
        self.start_planning_srv = self.create_service(
            Trigger, '/planner/start', self.start_planning_callback)
        self.stop_planning_srv = self.create_service(
            Trigger, '/planner/stop', self.stop_planning_callback)
        self.reset_planning_srv = self.create_service(
            Trigger, '/planner/reset', self.reset_planning_callback)
        self.replan_srv = self.create_service(
            Trigger, '/planner/replan', self.replan_callback)
        
        # Planning control
        self.planning_active = False
        self.planning_thread = None
        self.stop_requested = False
        self._team_refresh_timer = None
        
        # Timers
        #self.replan_timer = self.create_timer(self.replan_interval, self.replan_tasks)
        self.status_timer = self.create_timer(1.0, self.publish_status)
        self.add_on_set_parameters_callback(self._on_parameters_changed)
        
        # Initialize default tasks
        self.initialize_default_tasks()
        
        self.get_logger().info('Task planner initialized (simplified, Nav2-integrated)')

    def _normalize_team_color(self, value: Any) -> str:
        normalized = str(value).strip().lower()
        return normalized if normalized in ('blue', 'yellow') else 'blue'

    def _get_base_location_for_team(self, team_color: str) -> Dict[str, float]:
        color = self._normalize_team_color(team_color)
        return {
            'x': float(self.get_parameter(f'{color}_base_location_x').value),
            'y': float(self.get_parameter(f'{color}_base_location_y').value),
            'theta': float(self.get_parameter(f'{color}_base_location_theta').value)
        }

    def _update_return_base_task(self):
        for task in self.task_queue:
            if task.task_type == TaskType.RETURN_BASE:
                task.parameters['target_location'] = dict(self.base_location)
                return
        if self.current_task and self.current_task.task_type == TaskType.RETURN_BASE:
            self.current_task.parameters['target_location'] = dict(self.base_location)

    def _refresh_team_dependent_state(self, team_color: str, publish_initial_pose: bool = False):
        self.team_color = self._normalize_team_color(team_color)
        self.base_location = self._get_base_location_for_team(self.team_color)
        self._update_return_base_task()
        self.get_logger().info(
            f'Team color set to {self.team_color}; base=({self.base_location["x"]:.3f}, '
            f'{self.base_location["y"]:.3f}, {self.base_location["theta"]:.3f})'
        )

    def _on_parameters_changed(self, params):
        relevant = {
            'team_color',
            'blue_base_location_x', 'blue_base_location_y', 'blue_base_location_theta',
            'yellow_base_location_x', 'yellow_base_location_y', 'yellow_base_location_theta',
        }
        changed = {param.name for param in params}
        if not (changed & relevant):
            return SetParametersResult(successful=True)

        next_team_color = self.team_color
        if 'team_color' in changed:
            if self.planning_active:
                return SetParametersResult(
                    successful=False,
                    reason='team_color cannot change while match/planning is active'
                )
            team_param = next(param for param in params if param.name == 'team_color')
            next_team_color = self._normalize_team_color(team_param.value)

        # Callback runs before node parameter values are updated; defer state refresh briefly.
        if self._team_refresh_timer is not None:
            self._team_refresh_timer.cancel()
            self._team_refresh_timer = None
        self._team_refresh_timer = self.create_timer(
            0.01,
            lambda: self._deferred_parameter_refresh(next_team_color)
        )
        return SetParametersResult(successful=True)

    def _deferred_parameter_refresh(self, team_color: str):
        # One-shot timer callback after parameter commit.
        if self._team_refresh_timer is not None:
            self._team_refresh_timer.cancel()
            self._team_refresh_timer = None
        self._refresh_team_dependent_state(team_color)
    
    
    def initialize_default_tasks(self):
        """Initialize the default task set via the current season composer."""
        pick_locations, drop_locations = self.world_state.reload_catalog_and_state()
        self.get_logger().info(
            f'Loaded pick/drop catalog: {len(pick_locations)} picks, {len(drop_locations)} drops'
        )

        for task in self.task_composer.compose_initial_tasks(
            world_state=self.world_state,
            base_location=self.base_location,
        ):
            self.add_task(task)

        self._log_initial_task_list()
        
        self.get_logger().info(f'Initialized {len(self.task_queue)} default tasks')

    def _log_initial_task_list(self):
        """Log the initial task queue with pose details for debugging planner setup."""
        if not self.task_queue:
            self.get_logger().info('Initial task list is empty')
            return

        lines = ['Initial task list:']
        for index, task in enumerate(self.task_queue, start=1):
            lines.append(
                f'  {index}. id={task.task_id} type={task.task_type.value} '
                f'name={task.name} priority={task.base_priority}'
            )

            pick_location = task.parameters.get('pick_location')
            if isinstance(pick_location, dict) and pick_location:
                lines.append(
                    f'     pick={self._format_location_summary(pick_location)} '
                    f'approaches={self._format_approach_positions(pick_location)}'
                )

            drop_location = task.parameters.get('drop_location')
            if isinstance(drop_location, dict) and drop_location:
                lines.append(
                    f'     drop={self._format_location_summary(drop_location)} '
                    f'approaches={self._format_approach_positions(drop_location)}'
                )

            target_location = task.parameters.get('target_location')
            if isinstance(target_location, dict) and target_location:
                lines.append(f'     target_pose={self._format_pose(target_location)}')

        self.get_logger().info('\n'.join(lines))

    def _format_location_summary(self, location: Dict[str, Any]) -> str:
        location_id = str(location.get('id', 'unknown'))
        location_name = str(location.get('name', location_id))
        return f'{location_name}({location_id})'

    def _format_approach_positions(self, location: Dict[str, Any]) -> str:
        approaches = location.get('approach_positions', [])
        if not isinstance(approaches, list) or not approaches:
            return '[]'

        formatted = []
        for approach in approaches:
            if not isinstance(approach, dict):
                continue
            approach_id = str(approach.get('id', 'approach'))
            formatted.append(f'{approach_id}:{self._format_pose(approach)}')
        return '[' + ', '.join(formatted) + ']'

    def _format_pose(self, pose: Dict[str, Any]) -> str:
        x = float(pose.get('x', 0.0))
        y = float(pose.get('y', 0.0))
        theta = float(pose.get('theta', 0.0))
        return f'(x={x:.3f}, y={y:.3f}, theta={theta:.3f})'

    def _task_pick_id(self, task: Task) -> str:
        return self.world_state.task_pick_id(task.parameters)

    def _task_drop_candidates(self, task: Task) -> List[Dict[str, Any]]:
        return self.world_state.task_drop_candidates(task.parameters)
    


    def add_task(self, task: Task):
        """Add a task to the queue."""
        self.task_queue.append(task)
        self.get_logger().debug(f'Added task: {task.name}')
    
    def remove_task(self, task_id: str):
        """Remove a task from the queue."""
        self.task_queue = [t for t in self.task_queue if t.task_id != task_id]
    
    def time_callback(self, msg: Float32):
        """Update time remaining."""
        self.time_remaining = msg.data
    
    def score_callback(self, msg: Int32):
        """Update current score."""
        self.current_score = msg.data
    
    def opponent_score_callback(self, msg: Int32):
        """Update opponent score."""
        self.opponent_score = msg.data
    
    def phase_callback(self, msg: String):
        """Update game phase."""
        try:
            self.current_phase = GamePhase[msg.data]
        except KeyError:
            pass

    def pose_callback(self, msg: PoseStamped):
        """Update robot pose estimate used by utility functions."""
        self.robot_position = msg.pose.position
    
    def replan_tasks(self):
        """Re-evaluate and re-prioritize task queue."""
        if not self.planning_active or len(self.task_queue) == 0:
            return
        
        # Calculate utility for all available tasks
        task_utilities = []
        for task in self.task_queue:
            if not task.is_available():
                continue
            
            if not task.can_execute(self.time_remaining):
                continue
            
            # Apply phase-specific priority function
            if self.current_phase == GamePhase.EARLY:
                task.priority_function = early_game_priority
            elif self.current_phase == GamePhase.MID:
                task.priority_function = mid_game_priority
            elif self.current_phase == GamePhase.LATE:
                task.priority_function = late_game_priority
            elif self.current_phase == GamePhase.ENDGAME:
                task.priority_function = endgame_priority
            
            utility = task.calculate_utility(self.time_remaining, self.current_score, self)
            
            if utility >= self.min_utility_threshold:
                task_utilities.append((task, utility))
        
        # Sort by utility (highest first)
        task_utilities.sort(key=lambda x: x[1], reverse=True)
        
        # Reorder task queue
        self.task_queue = [task for task, _ in task_utilities]
        
        # Check if current task should be interrupted
        if self.enable_interruption and self.current_task is not None:
            if len(self.task_queue) > 0:
                next_task, next_utility = task_utilities[0] if task_utilities else (None, 0)
                if next_task and next_task.task_id != self.current_task.task_id:
                    current_utility = self.current_task.calculate_utility(
                        self.time_remaining, self.current_score, self
                    )
                    
                    # Interrupt if new task is significantly better
                    if next_utility > current_utility * 1.5:
                        self.get_logger().warn(
                            f'Interrupting {self.current_task.name} for higher priority '
                            f'{next_task.name} (utility: {next_utility:.2f} vs {current_utility:.2f})'
                        )
                        self.interrupt_current_task()
    
    def start_planning_callback(self, request, response):
        """Start task planning and execution."""
        if self.planning_active:
            response.success = False
            response.message = 'Planning already active'
        else:
            self.planning_active = True
            self.stop_requested = False
            
            # Start planning thread
            self.planning_thread = threading.Thread(target=self.planning_loop)
            self.planning_thread.daemon = True
            self.planning_thread.start()
            
            response.success = True
            response.message = 'Strategic planning started'
            self.get_logger().info(response.message)
        
        return response
    
    def stop_planning_callback(self, request, response):
        """Stop task planning."""
        self.planning_active = False
        self.stop_requested = True
        
        if self.current_task:
            self.current_task.status = TaskStatus.CANCELED
        
        response.success = True
        response.message = 'Strategic planning stopped'
        self.get_logger().info(response.message)
        return response

    def reset_planning_callback(self, request, response):
        """Reset planner state and reinitialize default tasks."""
        self._reset_to_default_state()
        response.success = True
        response.message = f'Planner reset. Queue size: {len(self.task_queue)}'
        self.get_logger().info(response.message)
        return response

    def _reset_to_default_state(self):
        """Reset planner internals and load default tasks for a fresh match."""
        self.planning_active = False
        self.stop_requested = True
        self.current_task = None

        self.task_queue = []
        self.completed_tasks = []
        self.failed_tasks = []
        self.recent_outcomes = set()
        self.last_mission_outcome = None
        self.task_context_by_id = {}
        self.mission_executor_status = 'IDLE'

        self.base_location = self._get_base_location_for_team(self.team_color)
        self.initialize_default_tasks()
    
    def replan_callback(self, request, response):
        """Force immediate replanning."""
        self.replan_tasks()
        response.success = True
        response.message = f'Replanned. Queue size: {len(self.task_queue)}'
        return response
    
    def planning_loop(self):
        """Main planning loop - executes tasks in priority order."""
        self.get_logger().info('Strategic planning loop started')
        
        while self.planning_active and not self.stop_requested:
            # Get highest priority task
            if len(self.task_queue) == 0:
                self.get_logger().info('No tasks in queue, waiting...')
                time.sleep(2.0)
                continue
            
            # Select next task
            self.current_task = self.task_queue[0]
            self.current_task.status = TaskStatus.IN_PROGRESS
            self.current_task.start_time = time.time()
            self.current_task.attempts += 1
            
            self.get_logger().info(
                f'Executing task: {self.current_task.name} '
                f'(Priority: {self.current_task.base_priority}, '
                f'Points: {self.current_task.base_points}, '
                f'Time: {self.current_task.time_estimate}s)'
            )
            
            # Execute task
            success = self.execute_task(self.current_task)
            
            # Update task status
            if success:
                self.current_task.status = TaskStatus.COMPLETED
                self.completed_tasks.append(self.current_task)
                self.get_logger().info(f'Task completed: {self.current_task.name}')
            else:
                if self.current_task.attempts >= self.current_task.max_attempts:
                    self.current_task.status = TaskStatus.FAILED
                    self.failed_tasks.append(self.current_task)
                    self.get_logger().error(f'Task failed: {self.current_task.name}')
                else:
                    self.current_task.status = TaskStatus.PENDING
                    self.get_logger().warn(
                        f'Task attempt {self.current_task.attempts} failed, will retry'
                    )   
            
            # Remove from queue if completed or failed
            if self.current_task.status in [TaskStatus.COMPLETED, TaskStatus.FAILED]:
                self.task_queue.remove(self.current_task)
            
            self.current_task = None
            
            # Brief pause before next task
            time.sleep(0.5)
        
        self.get_logger().info('Strategic planning loop stopped')

    def interrupt_current_task(self):
        """Interrupt currently executing task."""
        if self.current_task:
            self.current_task.status = TaskStatus.CANCELED
            self.get_logger().warn(f'Task interrupted: {self.current_task.name}')
            # In real implementation, would cancel active action goals
    

    def execute_task(self, task: Task) -> bool:
        """Execute task by dispatching to mission implementation."""
        self.get_logger().info(f'Executing {task.task_type.value}: {task.name}')

        adapter = self.task_adapters.get(task.task_type)
        if adapter is not None:
            return adapter.execute(task)

        self.get_logger().warn(f'No task adapter configured for {task.task_type.value}; treating as completed')
        return True

    def _start_mission_executor(self) -> bool:
        """Call mission_executor start_mission service."""
        self.mission_executor_status = 'STARTING'

        if not self.mission_executor_start_client.wait_for_service(timeout_sec=2.0):
            return False

        future = self.mission_executor_start_client.call_async(Trigger.Request())
        if not self._wait_for_future(future, timeout_sec=3.0):
            return False
        response = future.result()
        if response is None:
            return False

        if response.success:
            return True

        # Mission might already be running and able to consume assignment.
        return 'already running' in response.message.lower()

    def _wait_for_future(self, future, timeout_sec: float, poll_interval: float = 0.05) -> bool:
        """Wait for an async future without re-entering the spinning executor."""
        deadline = time.time() + timeout_sec
        while time.time() < deadline and not self.stop_requested:
            if future.done():
                return True
            time.sleep(poll_interval)
        return future.done()

    def _wait_for_mission_executor_result(self, expected_task_id: str, timeout_sec: float) -> bool:
        """Wait for a terminal mission outcome for the dispatched task."""
        deadline = time.time() + timeout_sec
        baseline_outcome_seq = self._mission_outcome_sequence(self.last_mission_outcome)
        saw_running = (self.mission_executor_status == 'RUNNING')

        while time.time() < deadline and not self.stop_requested:
            status = self.mission_executor_status
            if status == 'RUNNING':
                saw_running = True

            outcome = self.last_mission_outcome or {}
            outcome_task_id = str(outcome.get('task_id', ''))
            outcome_seq = self._mission_outcome_sequence(outcome)
            if outcome_seq > baseline_outcome_seq and outcome_task_id == expected_task_id:
                status = str(outcome.get('status', status)).upper()
                self.mission_executor_status = status

            if outcome_seq > baseline_outcome_seq and outcome_task_id == expected_task_id and status == 'REPLAN_REQUIRED':
                adapter = self._adapter_for_outcome(outcome)
                if adapter and adapter.handle_replan_required(outcome):
                    self.mission_executor_status = 'RUNNING'
                    baseline_outcome_seq = outcome_seq
                    continue
                return False
            if outcome_seq > baseline_outcome_seq and outcome_task_id == expected_task_id and status == 'COMPLETED':
                return True
            if outcome_seq > baseline_outcome_seq and outcome_task_id == expected_task_id and status == 'FAILED':
                return False
            time.sleep(0.1)

        if saw_running:
            self.get_logger().warn('Mission executor timed out waiting for terminal outcome')
        return False

    def _mission_outcome_sequence(self, outcome: Optional[Dict[str, Any]]) -> int:
        if not isinstance(outcome, dict):
            return 0
        try:
            return int(outcome.get('outcome_seq', 0))
        except (TypeError, ValueError):
            return 0

    def mission_executor_status_callback(self, msg: String):
        """Track mission state from mission_executor node."""
        raw = msg.data.strip()
        if not raw:
            return

        if raw.startswith('{'):
            try:
                outcome = json.loads(raw)
            except json.JSONDecodeError:
                self.mission_executor_status = raw
                return

            dedupe_key = (
                str(outcome.get('task_id', '')),
                str(outcome.get('status', '')),
                str(outcome.get('outcome_seq', outcome.get('timestamp', '')))
            )
            if dedupe_key in self.recent_outcomes:
                return
            self.recent_outcomes.add(dedupe_key)

            self.last_mission_outcome = outcome
            status = str(outcome.get('status', 'UNKNOWN')).upper()
            self.mission_executor_status = status
            adapter = self._adapter_for_outcome(outcome)
            processing_result = adapter.process_outcome(outcome) if adapter else {}
            if processing_result.get('replan_tasks', False):
                self.replan_tasks()
            return

        self.mission_executor_status = raw

    def _adapter_for_outcome(self, outcome: Dict[str, Any]):
        task_type_value = str(outcome.get('task_type', '')).strip().lower()
        if not task_type_value:
            return self.task_adapters.get(TaskType.MOVE_OBJECT)
        for task_type, adapter in self.task_adapters.items():
            if task_type.value == task_type_value:
                return adapter
        return None
    
    def publish_status(self):
        """Publish planner status."""
        if self.current_task:
            task_msg = String()
            task_msg.data = f'{self.current_task.name} ({self.current_task.status.value})'
            self.current_task_pub.publish(task_msg)
        
        queue_msg = Int32()
        queue_msg.data = len(self.task_queue)
        self.queue_size_pub.publish(queue_msg)


def main(args=None):
    rclpy.init(args=args)
    planner = TaskPlanner()
    
    try:
        rclpy.spin(planner)
    except KeyboardInterrupt:
        pass
    finally:
        planner.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
