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
from typing import List, Optional

from robot_application.task_definitions import (
    Task, TaskType, TaskStatus,
    create_pick_cube_task, create_place_cube_task,
    create_defend_task, create_steal_cube_task,
    create_return_base_task, create_move_object_task,
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
        
        # Get parameters
        self.replan_interval = self.get_parameter('replan_interval_sec').value
        self.enable_interruption = self.get_parameter('enable_task_interruption').value
        self.min_utility_threshold = self.get_parameter('min_utility_threshold').value
        
        base_x = self.get_parameter('base_location_x').value
        base_y = self.get_parameter('base_location_y').value
        self.base_location = {'x': base_x, 'y': base_y, 'theta': 0.0}
        
        # Task management
        self.task_queue: List[Task] = []
        self.current_task: Optional[Task] = None
        self.completed_tasks: List[Task] = []
        self.failed_tasks: List[Task] = []
        
        # Game state tracking
        self.time_remaining = 180.0
        self.current_score = 0
        self.opponent_score = 0
        self.current_phase = GamePhase.SETUP
        self.match_active = False
        self.robot_position = Point()
        
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
        """Initialize default task library."""
        # Example tasks - these would normally come from vision/perception
        
        # Pick and place cubes
        for i in range(3):
            cube_loc = {'x': 1.0 + i*0.3, 'y': 0.5, 'theta': 0.0}
            self.add_task(create_pick_cube_task(f"cube_{i}", cube_loc, points=10.0))
        
        # Scoring locations
        for i in range(3):
            score_loc = {'x': 2.0, 'y': 0.3 + i*0.3, 'theta': 1.57}
            self.add_task(create_place_cube_task(f"cube_{i}", score_loc, points=15.0))
        
        # Defend high-value area
        defend_loc = {'x': 1.5, 'y': 1.0, 'theta': 0.0}
        self.add_task(create_defend_task("scoring_zone", defend_loc, duration=30.0))
        
        # Steal from opponent (risky, high reward)
        steal_loc = {'x': 2.5, 'y': 1.5, 'theta': 3.14}
        self.add_task(create_steal_cube_task("opponent_cube", steal_loc, points=25.0))
        
        # Return to base (always available)
        self.add_task(create_return_base_task(self.base_location))
        
        self.get_logger().info(f'Initialized {len(self.task_queue)} default tasks')
    
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
    
    def execute_task(self, task: Task) -> bool:
        """Execute a task using appropriate mission."""
        # This is a simplified execution - in practice, you'd dispatch to
        # appropriate mission classes based on task type
        
        self.get_logger().info(f'Executing {task.task_type.value}: {task.name}')
        
        # Simulate task execution for now
        # In real implementation, this would:
        # 1. Load appropriate behavior tree or mission class
        # 2. Execute with task parameters
        # 3. Monitor progress
        # 4. Handle interruptions
        
        # For demonstration, just sleep for estimated time
        sleep_time = min(task.time_estimate, self.time_remaining * 0.8)
        
        start_time = time.time()
        while time.time() - start_time < sleep_time:
            if self.stop_requested or not self.planning_active:
                return False
            time.sleep(0.1)
        
        # Simulate success/failure based on probability
        import random
        return random.random() < task.success_probability
    
    def interrupt_current_task(self):
        """Interrupt currently executing task."""
        if self.current_task:
            self.current_task.status = TaskStatus.CANCELED
            self.get_logger().warn(f'Task interrupted: {self.current_task.name}')
            # In real implementation, would cancel active action goals
    
    def pose_callback(self, msg: PoseStamped):
        """Update robot position."""
        self.robot_position = msg.pose.position
    
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
