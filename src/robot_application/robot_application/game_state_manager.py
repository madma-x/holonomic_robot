"""Game state management for competitive robotics."""

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rcl_interfaces.msg import SetParametersResult
from std_srvs.srv import Trigger, SetBool
from std_msgs.msg import Float32, Int32, String, Bool
from geometry_msgs.msg import PoseStamped, Point, PoseWithCovarianceStamped
import math
import time
from enum import Enum


class GamePhase(Enum):
    """Game phases based on remaining time."""
    SETUP = 0       # Pre-match setup
    EARLY = 1       # >60s remaining - strategic positioning, high-value tasks
    MID = 2         # 30-60s - balanced scoring and defense
    LATE = 3        # 10-30s - quick wins, secure position
    ENDGAME = 4     # <10s - return to base, finalize score
    FINISHED = 5    # Match over


class GameStateManager(Node):
    """Manages game state for strategic decision making."""
    
    def __init__(self):
        super().__init__('game_state_manager')
        
        # Declare parameters
        self.declare_parameter('match_duration_sec', 180.0)  # 3 minutes
        self.declare_parameter('early_phase_threshold', 60.0)
        self.declare_parameter('mid_phase_threshold', 30.0)
        self.declare_parameter('late_phase_threshold', 10.0)
        self.declare_parameter('auto_start', False)
        self.declare_parameter('planner_start_service', '/planner/start')
        self.declare_parameter('planner_stop_service', '/planner/stop')
        self.declare_parameter('planner_reset_service', '/planner/reset')
        self.declare_parameter('team_color', 'blue')
        self.declare_parameter('blue_initial_pose_x', 0.0)
        self.declare_parameter('blue_initial_pose_y', 0.0)
        self.declare_parameter('blue_initial_pose_theta', 0.0)
        self.declare_parameter('yellow_initial_pose_x', 0.0)
        self.declare_parameter('yellow_initial_pose_y', 0.0)
        self.declare_parameter('yellow_initial_pose_theta', 0.0)
        
        # Get parameters
        self.match_duration = self.get_parameter('match_duration_sec').value
        self.early_threshold = self.get_parameter('early_phase_threshold').value
        self.mid_threshold = self.get_parameter('mid_phase_threshold').value
        self.late_threshold = self.get_parameter('late_phase_threshold').value
        
        # Game state
        self.match_started = False
        self.match_start_time = None
        self.current_score = 0
        self.opponent_score = 0
        self.current_phase = GamePhase.SETUP
        self.base_position = Point()  # Home position
        self.objectives_completed = set()
        self._match_end_processed = False
        
        # Publishers
        self.time_remaining_pub = self.create_publisher(Float32, '/game/time_remaining', 10)
        self.score_pub = self.create_publisher(Int32, '/game/score', 10)
        self.phase_pub = self.create_publisher(String, '/game/phase', 10)
        self.match_active_pub = self.create_publisher(Bool, '/game/match_active', 10)
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

        self.planner_client_cb_group = ReentrantCallbackGroup()
        planner_start_service = self.get_parameter('planner_start_service').value
        planner_stop_service = self.get_parameter('planner_stop_service').value
        planner_reset_service = self.get_parameter('planner_reset_service').value
        self.planner_start_client = self.create_client(
            Trigger,
            str(planner_start_service),
            callback_group=self.planner_client_cb_group,
        )
        self.planner_stop_client = self.create_client(
            Trigger,
            str(planner_stop_service),
            callback_group=self.planner_client_cb_group,
        )
        self.planner_reset_client = self.create_client(
            Trigger,
            str(planner_reset_service),
            callback_group=self.planner_client_cb_group,
        )
        
        
        # Services
        self.start_match_srv = self.create_service(
            Trigger, '/game/start_match', self.start_match_callback)
        self.stop_match_srv = self.create_service(
            Trigger, '/game/stop_match', self.stop_match_callback)
        self.reset_match_srv = self.create_service(
            Trigger, '/game/reset_match', self.reset_match_callback)
        self.reset_pose_srv = self.create_service(
            Trigger, '/game/reset_pose', self.reset_pose_callback)
        self.add_score_srv = self.create_service(
            SetBool, '/game/add_score', self.add_score_callback)
        
        # Timers
        self.state_timer = self.create_timer(0.1, self.update_game_state)
        self.publish_timer = self.create_timer(0.5, self.publish_state)
        self.add_on_set_parameters_callback(self._on_parameters_changed)
        
        self.get_logger().info('Game state manager initialized')
        
        # Auto-start if enabled
        if self.get_parameter('auto_start').value:
            self.start_match()
    
    def start_match_callback(self, request, response):
        """Start match timer."""
        planner_started, planner_message = self.start_match()
        response.success = planner_started
        response.message = (
            f'Match started - {self.match_duration}s duration. {planner_message}'
        )
        return response
    
    def stop_match_callback(self, request, response):
        """Stop match timer."""
        self._stop_match_state(mark_finished=True)
        self._stop_and_reset_planner()
        response.success = True
        response.message = f'Match stopped - Final score: {self.current_score}'
        self.get_logger().info(response.message)
        return response

    def reset_match_callback(self, request, response):
        """Fully reset match state and planner default tasks."""
        self._stop_match_state(mark_finished=False)
        self.current_score = 0
        self.opponent_score = 0
        self.objectives_completed.clear()
        self.current_phase = GamePhase.SETUP
        self._stop_and_reset_planner()
        response.success = True
        response.message = 'Match state reset to SETUP and planner reset to default tasks'
        self.get_logger().info(response.message)
        return response
    
    def add_score_callback(self, request, response):
        """Add points to score."""
        points = 10 if request.data else 5  # Simple scoring
        self.current_score += points
        response.success = True
        response.message = f'Added {points} points - Total: {self.current_score}'
        self.get_logger().info(response.message)
        return response

    def _normalize_team_color(self, value) -> str:
        normalized = str(value).strip().lower()
        return normalized if normalized in ('blue', 'yellow') else 'blue'

    def _get_initial_pose_for_team(self) -> dict:
        team_color = self._normalize_team_color(self.get_parameter('team_color').value)
        return {
            'team_color': team_color,
            'x': float(self.get_parameter(f'{team_color}_initial_pose_x').value),
            'y': float(self.get_parameter(f'{team_color}_initial_pose_y').value),
            'theta': float(self.get_parameter(f'{team_color}_initial_pose_theta').value)
        }

    def _publish_initial_pose(self) -> dict:
        pose = self._get_initial_pose_for_team()

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = pose['x']
        msg.pose.pose.position.y = pose['y']
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.z = math.sin(pose['theta'] / 2.0)
        msg.pose.pose.orientation.w = math.cos(pose['theta'] / 2.0)
        msg.pose.covariance[0] = 0.000025
        msg.pose.covariance[7] = 0.000025
        msg.pose.covariance[35] = 0.0003
        self.initial_pose_pub.publish(msg)

        self.get_logger().info(
            f'Published initial pose for {pose["team_color"]}: '
            f'x={pose["x"]:.3f}, y={pose["y"]:.3f}, theta={pose["theta"]:.3f}'
        )
        return pose

    def reset_pose_callback(self, request, response):
        """Publish the team-color-specific initial pose without affecting match state."""
        pose = self._publish_initial_pose()
        response.success = True
        response.message = (
            f'Reset pose published for {pose["team_color"]}: '
            f'x={pose["x"]:.3f}, y={pose["y"]:.3f}, theta={pose["theta"]:.3f}'
        )
        return response
    
    def start_match(self):
        """Start the match and request planner execution."""
        self.match_started = True
        self._match_end_processed = False
        self.match_start_time = time.time()
        self.current_score = 0
        self.opponent_score = 0
        self.objectives_completed.clear()
        self.current_phase = GamePhase.EARLY
        self.get_logger().info(f'Match started! Duration: {self.match_duration}s')
        return self.start_task_planner()

    def start_task_planner(self):
        """Request task planner startup through the planner start service."""
        service_name = str(self.get_parameter('planner_start_service').value)
        if not self.planner_start_client.wait_for_service(timeout_sec=2.0):
            message = f'Task planner start service unavailable: {service_name}'
            self.get_logger().error(message)
            return False, message

        future = self.planner_start_client.call_async(Trigger.Request())
        if not self._wait_for_future(future, timeout_sec=3.0):
            message = 'Task planner start request timed out'
            self.get_logger().error(message)
            return False, message

        result = future.result()
        if result is None:
            message = 'Task planner start request timed out'
            self.get_logger().error(message)
            return False, message

        if result.success:
            self.get_logger().info(f'Task planner started: {result.message}')
            return True, result.message

        self.get_logger().warn(f'Task planner did not start cleanly: {result.message}')
        return False, result.message

    def _call_trigger_client(self, client, service_name: str, timeout_sec: float = 3.0) -> bool:
        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(f'Service unavailable: {service_name}')
            return False

        future = client.call_async(Trigger.Request())
        if not self._wait_for_future(future, timeout_sec=timeout_sec):
            self.get_logger().warn(f'Service timeout: {service_name}')
            return False

        result = future.result()
        if result is None:
            self.get_logger().warn(f'Service returned no response: {service_name}')
            return False
        if not result.success:
            self.get_logger().warn(f'Service reported failure ({service_name}): {result.message}')
            return False
        return True

    def _stop_and_reset_planner(self):
        stop_service = str(self.get_parameter('planner_stop_service').value)
        reset_service = str(self.get_parameter('planner_reset_service').value)
        self._call_trigger_client(self.planner_stop_client, stop_service)
        self._call_trigger_client(self.planner_reset_client, reset_service)

    def _stop_match_state(self, mark_finished: bool):
        self.match_started = False
        self.match_start_time = None
        self._match_end_processed = True
        self.current_phase = GamePhase.FINISHED if mark_finished else GamePhase.SETUP

    def _on_parameters_changed(self, params):
        for param in params:
            if param.name == 'team_color' and self.match_started:
                return SetParametersResult(
                    successful=False,
                    reason='team_color cannot change while match is active'
                )
        return SetParametersResult(successful=True)

    def _wait_for_future(self, future, timeout_sec: float, poll_interval: float = 0.05) -> bool:
        """Wait for an async future without re-entering the spinning executor."""
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            if future.done():
                return True
            time.sleep(poll_interval)
        return future.done()

    def update_game_state(self):
        """Update game state based on time."""
        if not self.match_started:
            return
        
        time_remaining = self.get_time_remaining()
        
        # Update phase
        old_phase = self.current_phase
        if time_remaining <= 0:
            if not self._match_end_processed:
                self._stop_match_state(mark_finished=True)
                self._stop_and_reset_planner()
                self.get_logger().info(f'Match finished! Final score: {self.current_score}')
        elif time_remaining < self.late_threshold:
            self.current_phase = GamePhase.ENDGAME
        elif time_remaining < self.mid_threshold:
            self.current_phase = GamePhase.LATE
        elif time_remaining < self.early_threshold:
            self.current_phase = GamePhase.MID
        else:
            self.current_phase = GamePhase.EARLY
        
        # Log phase transitions
        if old_phase != self.current_phase:
            self.get_logger().info(
                f'Phase transition: {old_phase.name} -> {self.current_phase.name} '
                f'({time_remaining:.1f}s remaining)'
            )
    
    def get_time_remaining(self) -> float:
        """Get remaining match time."""
        if not self.match_started or self.match_start_time is None:
            return self.match_duration
        
        elapsed = time.time() - self.match_start_time
        remaining = self.match_duration - elapsed
        return max(0.0, remaining)
    
    def get_time_elapsed(self) -> float:
        """Get elapsed match time."""
        if not self.match_started or self.match_start_time is None:
            return 0.0
        return time.time() - self.match_start_time
    
    def publish_state(self):
        """Publish game state."""
        # Time remaining
        time_msg = Float32()
        time_msg.data = self.get_time_remaining()
        self.time_remaining_pub.publish(time_msg)
        
        # Score
        score_msg = Int32()
        score_msg.data = self.current_score
        self.score_pub.publish(score_msg)
                
        # Phase
        phase_msg = String()
        phase_msg.data = self.current_phase.name
        self.phase_pub.publish(phase_msg)
        
        # Match active
        active_msg = Bool()
        active_msg.data = self.match_started
        self.match_active_pub.publish(active_msg)
    


def main(args=None):
    rclpy.init(args=args)
    node = GameStateManager()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
