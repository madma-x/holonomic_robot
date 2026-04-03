"""Strategic task planner with dynamic prioritization.

Simplified architecture:
- Task prioritization based on game phase and utility
- Nav2 handles path-level replanning and obstacle avoidance
- Task planner only monitors goal-level failures (UNREACHABLE, TIMEOUT)
- Switch tasks on goal failure, but let Nav2 handle path decisions
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String, Float32, Int32
from std_srvs.srv import Trigger
from geometry_msgs.msg import Point, PoseStamped
from nav2_msgs.action import NavigateToPose
import threading
import time
import os
import yaml
import json
from typing import List, Optional, Dict, Any, Tuple
from ament_index_python.packages import get_package_share_directory

from robot_application.task_definitions import (
    Task, TaskType, TaskStatus,
    create_return_base_task, create_pick_place_task,
    early_game_priority, mid_game_priority,
    late_game_priority, endgame_priority
)
from robot_application.game_state_manager import GamePhase
from robot_application.mission_base import MissionBase


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
        self.declare_parameter('base_location_x', 0.0)
        self.declare_parameter('base_location_y', 0.0)
        self.declare_parameter('mission_executor_assignment_topic', '/planner/task_assignment')
        self.declare_parameter('mission_executor_status_topic', '/mission_executor/mission_status')
        self.declare_parameter('mission_executor_start_service', '/mission_executor/start_mission')
        self.declare_parameter('team_color', 'blue')
        self.declare_parameter('drop_full_cooldown_sec', 20.0)
        
        # Get parameters
        self.replan_interval = self.get_parameter('replan_interval_sec').value
        self.enable_interruption = self.get_parameter('enable_task_interruption').value
        self.min_utility_threshold = self.get_parameter('min_utility_threshold').value
        self.team_color = str(self.get_parameter('team_color').value)
        self.drop_full_cooldown_sec = float(self.get_parameter('drop_full_cooldown_sec').value)
        
        base_x = self.get_parameter('base_location_x').value
        base_y = self.get_parameter('base_location_y').value
        self.base_location = {'x': base_x, 'y': base_y, 'theta': 0.0}
        
        # Task management
        self.task_queue: List[Task] = []
        self.current_task: Optional[Task] = None
        self.completed_tasks: List[Task] = []
        self.failed_tasks: List[Task] = []
        self.empty_pick_locations = set()
        self.occupied_drop_locations = set()
        self.pick_locations_catalog: Dict[str, Dict[str, Any]] = {}
        self.drop_locations_catalog: Dict[str, Dict[str, Any]] = {}
        self.pick_state: Dict[str, Dict[str, Any]] = {}
        self.drop_state: Dict[str, Dict[str, Any]] = {}
        self.recent_outcomes = set()
        self.last_mission_outcome: Optional[Dict[str, Any]] = None
        self.task_context_by_id: Dict[str, Dict[str, Any]] = {}
        
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
        self.opponent_score_sub = self.create_subscription(
            Int32, '/game/opponent_score', self.opponent_score_callback, 10)
        self.phase_sub = self.create_subscription(
            String, '/game/phase', self.phase_callback, 10)
        self.pose_sub = self.create_subscription(
            PoseStamped, '/amcl_pose', self.pose_callback, 10)
        
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
        
        # Services
        self.start_planning_srv = self.create_service(
            Trigger, '/planner/start', self.start_planning_callback)
        self.stop_planning_srv = self.create_service(
            Trigger, '/planner/stop', self.stop_planning_callback)
        self.replan_srv = self.create_service(
            Trigger, '/planner/replan', self.replan_callback)
        
        # Planning control
        self.planning_active = False
        self.planning_thread = None
        self.stop_requested = False
        
        # Timers
        self.replan_timer = self.create_timer(self.replan_interval, self.replan_tasks)
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        # Initialize default tasks
        self.initialize_default_tasks()
        
        self.get_logger().info('Task planner initialized (simplified, Nav2-integrated)')
    
    
    def initialize_default_tasks(self):
        """Initialize default task library with pick-and-place tasks.

        - Picks are ordered by descending priority.
        - Each pick is linked to the closest drop location.
        """
        pick_locations, drop_locations = self._load_pick_drop_locations()

        self.pick_locations_catalog = {str(pick['id']): pick for pick in pick_locations}
        self.drop_locations_catalog = {str(drop['id']): drop for drop in drop_locations}
        self._initialize_world_state(pick_locations, drop_locations)

        sorted_picks = sorted(pick_locations, key=lambda pick: pick.get('priority', 0), reverse=True)

        for pick in sorted_picks:
            linked_drop = min(
                drop_locations,
                key=lambda drop: self._distance_between_locations(
                    pick.get('location', {}),
                    drop.get('location', {})
                )
            )

            pick_drop_task = create_pick_place_task(
                task_id=f"pick_place_{pick['id']}",
                pick_location=pick,
                drop_location=linked_drop
            )
            self.add_task(pick_drop_task)

        # Return to base (always available)
        self.add_task(create_return_base_task(self.base_location))
        
        self.get_logger().info(f'Initialized {len(self.task_queue)} default tasks')

    def _load_pick_drop_locations(self):
        """Load pick/drop location catalog from YAML and normalize schema."""
        config_path = None
        try:
            package_share = get_package_share_directory('robot_application')
            config_path = os.path.join(package_share, 'config', 'pick_drop_locations.yaml')
            with open(config_path, 'r', encoding='utf-8') as config_file:
                config = yaml.safe_load(config_file) or {}
        except Exception:
            config_path = os.path.abspath(
                os.path.join(os.path.dirname(__file__), '..', 'config', 'pick_drop_locations.yaml')
            )
            with open(config_path, 'r', encoding='utf-8') as config_file:
                config = yaml.safe_load(config_file) or {}

        pick_locations = self._normalize_locations(config.get('pick_locations', []))
        drop_locations = self._normalize_locations(config.get('drop_locations', []))

        if not pick_locations or not drop_locations:
            raise RuntimeError(f'pick_drop_locations.yaml is missing picks or drops: {config_path}')

        self.get_logger().info(
            f'Loaded pick/drop catalog: {len(pick_locations)} picks, {len(drop_locations)} drops'
        )
        return pick_locations, drop_locations

    def _normalize_locations(self, locations: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """Convert approach positions from {pose: {x,y,theta}} into flat {x,y,theta}."""
        normalized = []
        for location in locations:
            approach_positions = []
            for approach in location.get('approach_positions', []):
                pose = approach.get('pose', {})
                approach_positions.append({
                    'id': approach.get('id'),
                    'priority': approach.get('priority', 1),
                    'x': float(pose.get('x', 0.0)),
                    'y': float(pose.get('y', 0.0)),
                    'theta': float(pose.get('theta', 0.0))
                })

            normalized.append({
                'id': location.get('id'),
                'name': location.get('name', location.get('id', 'location')),
                'priority': int(location.get('priority', 0)),
                'capacity': int(location.get('capacity', 1)),
                'location': {
                    'x': float(location.get('location', {}).get('x', 0.0)),
                    'y': float(location.get('location', {}).get('y', 0.0)),
                    'theta': float(location.get('location', {}).get('theta', 0.0))
                },
                'approach_positions': sorted(
                    approach_positions,
                    key=lambda approach: int(approach.get('priority', 1))
                )
            })

        return normalized

    def _initialize_world_state(self, pick_locations: List[Dict[str, Any]], drop_locations: List[Dict[str, Any]]):
        self.pick_state = {
            str(pick['id']): {
                'empty': False,
                'last_update': time.time()
            }
            for pick in pick_locations
        }

        self.drop_state = {
            str(drop['id']): {
                'occupancy': 0,
                'capacity': int(drop.get('capacity', 1)),
                'blocked_until': 0.0,
                'last_known_color': 'unknown',
                'color_confidence': 0.0,
                'last_update': time.time()
            }
            for drop in drop_locations
        }

    def is_drop_available(self, drop_id: str) -> bool:
        drop = self.drop_state.get(drop_id)
        if drop is None:
            return False
        return int(drop.get('occupancy', 0)) < int(drop.get('capacity', 1))

    def mark_pick_empty(self, pick_id: str):
        if pick_id not in self.pick_state:
            return
        self.pick_state[pick_id]['empty'] = True
        self.pick_state[pick_id]['last_update'] = time.time()
        self.empty_pick_locations.add(pick_id)

    def mark_drop_full_temporarily(self, drop_id: str):
        drop = self.drop_state.get(drop_id)
        if drop is None:
            return
        drop['blocked_until'] = time.time() + self.drop_full_cooldown_sec
        drop['last_update'] = time.time()
        self.occupied_drop_locations.add(drop_id)

    def _distance_between_locations(self, from_location: Dict[str, float], to_location: Dict[str, float]) -> float:
        """Compute planar distance between two {x, y, theta} dict locations."""
        from_x = float(from_location.get('x', 0.0))
        from_y = float(from_location.get('y', 0.0))
        to_x = float(to_location.get('x', 0.0))
        to_y = float(to_location.get('y', 0.0))
        return ((from_x - to_x) ** 2 + (from_y - to_y) ** 2) ** 0.5
    


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

        if task.task_type == TaskType.MOVE_OBJECT:
            return self._execute_move_object_task(task)
        elif task.task_type == TaskType.RETURN_BASE:
            return self._execute_return_base_task(task)

        return True

    def _execute_move_object_task(self, task: Task) -> bool:
        """Send task to mission executor and wait for outcome."""
        full_drop_ids = [drop_id for drop_id in self.drop_state if not self.is_drop_available(drop_id)]

        payload = {
            'name': task.name,
            'task_id': task.task_id,
            'task_type': task.task_type.value,
            'pick_locations': task.parameters.get('pick_locations', []),
            'drop_positions': task.parameters.get('drop_locations', []),
            'pickup_method': task.parameters.get('pickup_method', 'pump'),
            'pickup_duration': task.parameters.get('pickup_duration', 2.0),
            'priority': task.base_priority,
            'carry_object': bool(task.parameters.get('carry_object', False)),
            'source_pick_id': str(task.parameters.get('source_pick_id', '')),
            'target_drop_id': str(task.parameters.get('target_drop_id', '')),
            'target_team_color': self.team_color,
            'object_color_before': str(task.parameters.get('object_color_before', 'unknown')),
            'full_drop_ids': full_drop_ids,
            'excluded_drop_ids': list(task.parameters.get('excluded_drop_ids', []))
        }

        self.task_context_by_id[task.task_id] = payload

        assignment_msg = String()
        assignment_msg.data = json.dumps(payload)
        self.mission_assignment_pub.publish(assignment_msg)
        self.get_logger().info(f'Dispatched task to mission_executor: {task.task_id}')

        start_ok = self._start_mission_executor()
        if not start_ok:
            self.get_logger().error('Failed to start mission_executor mission')
            return False

        timeout = max(15.0, float(task.time_estimate) + 25.0)
        return self._wait_for_mission_executor_result(timeout)

    def _start_mission_executor(self) -> bool:
        """Call mission_executor start_mission service."""
        if not self.mission_executor_start_client.wait_for_service(timeout_sec=2.0):
            return False

        future = self.mission_executor_start_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        response = future.result()
        if response is None:
            return False

        if response.success:
            return True

        # Mission might already be running and able to consume assignment.
        return 'already running' in response.message.lower()

    def _wait_for_mission_executor_result(self, timeout_sec: float) -> bool:
        """Wait for mission state transition to COMPLETED/FAILED."""
        deadline = time.time() + timeout_sec
        saw_running = (self.mission_executor_status == 'RUNNING')

        while time.time() < deadline and not self.stop_requested:
            status = self.mission_executor_status
            if status == 'RUNNING':
                saw_running = True
            if status == 'REPLAN_REQUIRED':
                outcome = self.last_mission_outcome or {}
                if self._handle_replan_required_outcome(outcome):
                    self.mission_executor_status = 'RUNNING'
                    continue
                return False
            if status == 'COMPLETED':
                return True
            if status == 'FAILED':
                return False
            time.sleep(0.1)

        if saw_running:
            self.get_logger().warn('Mission executor timed out waiting for terminal outcome')
        return False

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
            self._apply_outcome_to_world_state(outcome)
            return

        self.mission_executor_status = raw

    def _apply_outcome_to_world_state(self, outcome: Dict[str, Any]):
        status = str(outcome.get('status', '')).upper()
        reason = str(outcome.get('outcome_reason', '')).upper()
        source_pick_id = str(outcome.get('source_pick_id', ''))
        target_drop_id = str(outcome.get('target_drop_id', ''))

        if status == 'FAILED' and reason == 'PICK_EMPTY' and source_pick_id:
            self.mark_pick_empty(source_pick_id)
            self.task_queue = [
                task for task in self.task_queue
                if source_pick_id not in [str(pick.get('id', '')) for pick in task.parameters.get('pick_locations', [])]
            ]
            self.replan_tasks()
            return

        if status == 'REPLAN_REQUIRED' and reason == 'DROP_FULL' and target_drop_id:
            self.mark_drop_full_temporarily(target_drop_id)
            return

        if status == 'COMPLETED':
            if source_pick_id:
                self.mark_pick_empty(source_pick_id)
            if target_drop_id:
                self.commit_drop_color(target_drop_id, str(outcome.get('object_color_after', 'unknown')))
            task_id = str(outcome.get('task_id', ''))
            if task_id and task_id in self.task_context_by_id:
                del self.task_context_by_id[task_id]

    def _handle_replan_required_outcome(self, outcome: Dict[str, Any]) -> bool:
        reason = str(outcome.get('outcome_reason', '')).upper()
        carry_object = bool(outcome.get('carry_object', False))
        task_id = str(outcome.get('task_id', ''))

        if reason != 'DROP_FULL' or not carry_object or not task_id:
            return False

        original_payload = self.task_context_by_id.get(task_id)
        if not original_payload:
            return False

        blocked_drop_id = str(outcome.get('target_drop_id', ''))
        excluded_ids = set(original_payload.get('excluded_drop_ids', []))
        if blocked_drop_id:
            excluded_ids.add(blocked_drop_id)

        candidate_drops = list(original_payload.get('drop_positions', []))
        next_drop = None
        for drop in sorted(candidate_drops, key=lambda item: int(item.get('priority', 999))):
            drop_id = str(drop.get('id', ''))
            if drop_id in excluded_ids:
                continue
            if self.is_drop_available(drop_id):
                next_drop = drop
                break

        if next_drop is None:
            return False

        continuation_payload = dict(original_payload)
        continuation_payload['carry_object'] = True
        continuation_payload['source_pick_id'] = str(outcome.get('source_pick_id', continuation_payload.get('source_pick_id', '')))
        continuation_payload['target_drop_id'] = str(next_drop.get('id', ''))
        continuation_payload['excluded_drop_ids'] = sorted(list(excluded_ids))
        continuation_payload['full_drop_ids'] = [drop_id for drop_id in self.drop_state if not self.is_drop_available(drop_id)]
        continuation_payload['drop_positions'] = [next_drop] + [
            drop for drop in candidate_drops
            if str(drop.get('id', '')) != str(next_drop.get('id', ''))
        ]
        continuation_payload['object_color_before'] = str(outcome.get('object_color_before', continuation_payload.get('object_color_before', 'unknown')))

        self.task_context_by_id[task_id] = continuation_payload
        assignment_msg = String()
        assignment_msg.data = json.dumps(continuation_payload)
        self.mission_assignment_pub.publish(assignment_msg)
        self.get_logger().info(
            f'Replanned drop for {task_id}: blocked={blocked_drop_id}, next={continuation_payload["target_drop_id"]}'
        )
        return True
    
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
        rclpy.shutdown()


if __name__ == '__main__':
    main()
