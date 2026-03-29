"""Game state management for competitive robotics."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32, String, Bool
from geometry_msgs.msg import PoseStamped, Point
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
        self.robot_position = Point()
        self.base_position = Point()  # Home position
        self.objectives_completed = set()
        
        # Publishers
        self.time_remaining_pub = self.create_publisher(Float32, '/game/time_remaining', 10)
        self.score_pub = self.create_publisher(Int32, '/game/score', 10)
        self.opponent_score_pub = self.create_publisher(Int32, '/game/opponent_score', 10)
        self.phase_pub = self.create_publisher(String, '/game/phase', 10)
        self.match_active_pub = self.create_publisher(Bool, '/game/match_active', 10)
        
        # Subscribers
        self.pose_sub = self.create_subscription(
            PoseStamped, '/amcl_pose', self.pose_callback, 10)
        
        # Services
        from std_srvs.srv import Trigger, SetBool
        self.start_match_srv = self.create_service(
            Trigger, '/game/start_match', self.start_match_callback)
        self.stop_match_srv = self.create_service(
            Trigger, '/game/stop_match', self.stop_match_callback)
        self.add_score_srv = self.create_service(
            SetBool, '/game/add_score', self.add_score_callback)
        
        # Timers
        self.state_timer = self.create_timer(0.1, self.update_game_state)
        self.publish_timer = self.create_timer(0.5, self.publish_state)
        
        self.get_logger().info('Game state manager initialized')
        
        # Auto-start if enabled
        if self.get_parameter('auto_start').value:
            self.start_match()
    
    def start_match_callback(self, request, response):
        """Start match timer."""
        self.start_match()
        response.success = True
        response.message = f'Match started - {self.match_duration}s duration'
        return response
    
    def stop_match_callback(self, request, response):
        """Stop match timer."""
        self.match_started = False
        self.current_phase = GamePhase.FINISHED
        response.success = True
        response.message = f'Match stopped - Final score: {self.current_score}'
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
    
    def start_match(self):
        """Start the match."""
        self.match_started = True
        self.match_start_time = time.time()
        self.current_score = 0
        self.opponent_score = 0
        self.objectives_completed.clear()
        self.current_phase = GamePhase.EARLY
        self.get_logger().info(f'Match started! Duration: {self.match_duration}s')
    
    def pose_callback(self, msg: PoseStamped):
        """Update robot position."""
        self.robot_position = msg.pose.position
    
    def update_game_state(self):
        """Update game state based on time."""
        if not self.match_started:
            return
        
        time_remaining = self.get_time_remaining()
        
        # Update phase
        old_phase = self.current_phase
        if time_remaining <= 0:
            self.current_phase = GamePhase.FINISHED
            self.match_started = False
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
        
        opp_score_msg = Int32()
        opp_score_msg.data = self.opponent_score
        self.opponent_score_pub.publish(opp_score_msg)
        
        # Phase
        phase_msg = String()
        phase_msg.data = self.current_phase.name
        self.phase_pub.publish(phase_msg)
        
        # Match active
        active_msg = Bool()
        active_msg.data = self.match_started
        self.match_active_pub.publish(active_msg)
    
    def get_distance_to_base(self) -> float:
        """Calculate distance from robot to base."""
        import math
        dx = self.robot_position.x - self.base_position.x
        dy = self.robot_position.y - self.base_position.y
        return math.sqrt(dx*dx + dy*dy)


def main(args=None):
    rclpy.init(args=args)
    node = GameStateManager()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
