"""Base class for mission implementations."""

import math
import time
from typing import Tuple

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from action_msgs.msg import GoalStatus
from std_srvs.srv import Trigger, SetBool
from std_msgs.msg import String, Float32
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav2_msgs.action import ComputePathToPose
import threading
from enum import Enum

try:
    from aruco_interfaces.srv import MoveRelative
    HAS_MOVE_RELATIVE = True
except ImportError:
    MoveRelative = None
    HAS_MOVE_RELATIVE = False

try:
    from robot_actuators.action import MoveServo, ControlPump, ExecuteSequence
    from robot_actuators.msg import ActuatorStep
    HAS_ROBOT_ACTUATORS = True
except ImportError:
    MoveServo = None
    ControlPump = None
    ExecuteSequence = None
    ActuatorStep = None
    HAS_ROBOT_ACTUATORS = False


class MissionState(Enum):
    """Mission execution states."""
    IDLE = 0
    RUNNING = 1
    PAUSED = 2
    COMPLETED = 3
    FAILED = 4
    RECOVERING = 5


class MissionBase(Node):
    """Base class for all mission implementations."""
    
    def __init__(self, mission_name: str):
        super().__init__(f'{mission_name}_mission')
        
        self.mission_name = mission_name
        self.state = MissionState.IDLE
        self.progress = 0.0
        self.mission_thread = None
        self.stop_requested = False
        self.pause_requested = False
        
        # Declare parameters
        self.declare_parameter('nav_timeout_sec', 120.0)
        self.declare_parameter('servo_timeout_sec', 10.0)
        self.declare_parameter('pump_timeout_sec', 30.0)
        self.declare_parameter('enable_recovery', True)
        self.declare_parameter('max_recovery_attempts', 3)
        self.declare_parameter('mock_navigation', False)
        self.declare_parameter('mock_actuators', not HAS_ROBOT_ACTUATORS)
        self.declare_parameter('planner_timeout_sec', 8.0)
        self.declare_parameter('post_drop_back_distance_m', 0.1)
        self.declare_parameter('post_drop_back_timeout_s', 4.0)
        self.declare_parameter('post_drop_back_max_linear_speed_mps', 0.15)
        
        # Get parameters
        self.nav_timeout = self.get_parameter('nav_timeout_sec').value
        self.servo_timeout = self.get_parameter('servo_timeout_sec').value
        self.pump_timeout = self.get_parameter('pump_timeout_sec').value
        self.enable_recovery = self.get_parameter('enable_recovery').value
        self.max_recovery_attempts = self.get_parameter('max_recovery_attempts').value
        self.mock_navigation = self.get_parameter('mock_navigation').value
        self.mock_actuators = self.get_parameter('mock_actuators').value
        self.planner_timeout = self.get_parameter('planner_timeout_sec').value
        self.post_drop_back_distance_m = self.get_parameter('post_drop_back_distance_m').value
        self.post_drop_back_timeout_s = self.get_parameter('post_drop_back_timeout_s').value
        self.post_drop_back_max_linear_speed_mps = self.get_parameter('post_drop_back_max_linear_speed_mps').value
        
        # Action clients
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.compute_path_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        if HAS_ROBOT_ACTUATORS and not self.mock_actuators:
            self.servo_client = ActionClient(self, MoveServo, 'move_servo')
            self.pump_client = ActionClient(self, ControlPump, 'control_pump')
            self.sequence_client = ActionClient(self, ExecuteSequence, 'execute_sequence')
        else:
            self.servo_client = None
            self.pump_client = None
            self.sequence_client = None
            if self.mock_actuators:
                self.get_logger().warn('MissionBase running with mock actuators enabled')
            else:
                self.get_logger().warn('robot_actuators package unavailable; actuator actions disabled')

        if HAS_MOVE_RELATIVE:
            self.move_relative_client = self.create_client(MoveRelative, 'move_relative')
        else:
            self.move_relative_client = None
        
        # Services
        self.load_srv = self.create_service(
            SetBool, f'/{mission_name}/load_mission', self.load_mission_callback)
        self.start_srv = self.create_service(
            Trigger, f'/{mission_name}/start_mission', self.start_mission_callback)
        self.pause_srv = self.create_service(
            Trigger, f'/{mission_name}/pause_mission', self.pause_mission_callback)
        self.stop_srv = self.create_service(
            Trigger, f'/{mission_name}/stop_mission', self.stop_mission_callback)
        self.emergency_stop_srv = self.create_service(
            Trigger, f'/{mission_name}/emergency_stop', self.emergency_stop_callback)
        
        # Publishers
        self.status_pub = self.create_publisher(String, f'/{mission_name}/mission_status', 10)
        self.progress_pub = self.create_publisher(Float32, f'/{mission_name}/mission_progress', 10)
        
        # Status timer
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        self.get_logger().info(f'{mission_name} mission initialized')
    
    def load_mission_callback(self, request, response):
        """Load mission configuration."""
        try:
            self.load_mission_config()
            response.success = True
            response.message = f'{self.mission_name} mission loaded'
            self.get_logger().info(response.message)
        except Exception as e:
            response.success = False
            response.message = f'Failed to load mission: {str(e)}'
            self.get_logger().error(response.message)
        return response
    
    def start_mission_callback(self, request, response):
        """Start mission execution."""
        if self.state == MissionState.RUNNING:
            response.success = False
            response.message = 'Mission already running'
        elif self.mission_thread is not None and self.mission_thread.is_alive():
            response.success = False
            response.message = 'Mission thread still active'
        else:
            self.stop_requested = False
            self.pause_requested = False
            self.state = MissionState.RUNNING
            self.progress = 0.0
            
            # Start mission in separate thread
            self.mission_thread = threading.Thread(target=self.execute_mission)
            self.mission_thread.daemon = True
            self.mission_thread.start()
            
            response.success = True
            response.message = f'{self.mission_name} mission started'
            self.get_logger().info(response.message)
        
        return response
    
    def pause_mission_callback(self, request, response):
        """Pause mission execution."""
        if self.state == MissionState.RUNNING:
            self.pause_requested = True
            self.state = MissionState.PAUSED
            response.success = True
            response.message = 'Mission paused'
        else:
            response.success = False
            response.message = f'Cannot pause mission in state: {self.state.name}'
        
        self.get_logger().info(response.message)
        return response
    
    def stop_mission_callback(self, request, response):
        """Stop mission execution."""
        self.stop_requested = True
        self.pause_requested = False
        self.state = MissionState.IDLE
        self.progress = 0.0
        
        response.success = True
        response.message = 'Mission stopped'
        self.get_logger().info(response.message)
        return response
    
    def emergency_stop_callback(self, request, response):
        """Emergency stop - halt all actions immediately."""
        self.get_logger().warn('EMERGENCY STOP TRIGGERED')
        
        # Cancel all active actions
        if self.nav_client._goal_handle is not None:
            self.nav_client._goal_handle.cancel_goal_async()
        
        # Stop all actuators via their emergency stop services
        self.call_service_async('/emergency_stop_servos', Trigger)
        self.call_service_async('/emergency_stop_pumps', Trigger)
        
        # Stop mission
        self.stop_requested = True
        self.state = MissionState.FAILED
        
        response.success = True
        response.message = 'Emergency stop executed'
        return response
    
    def publish_status(self):
        """Publish mission status."""
        status_msg = String()
        status_msg.data = self.state.name
        self.status_pub.publish(status_msg)
        
        progress_msg = Float32()
        progress_msg.data = self.progress
        self.progress_pub.publish(progress_msg)

    def _wait_for_future(self, future, timeout_sec: float, poll_interval: float = 0.05) -> bool:
        """Wait for an async future without re-entering the spinning executor."""
        deadline = time.time() + timeout_sec
        while time.time() < deadline and not self.stop_requested:
            if future.done():
                return True
            time.sleep(poll_interval)
        return future.done()
    
    def navigate_to_pose(self, x: float, y: float, theta: float) -> bool:
        """Navigate to a pose using Nav2."""
        if self.mock_navigation:
            self.get_logger().info(
                f'[MOCK] Navigating to: x={x:.2f}, y={y:.2f}, theta={theta:.2f}'
            )
            return True

        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 action server not available')
            return False
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        
        # Convert theta to quaternion (simplified for z-axis rotation)
        goal_msg.pose.pose.orientation.z = math.sin(theta / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(theta / 2.0)
        
        self.get_logger().info(f'Navigating to: x={x:.2f}, y={y:.2f}, theta={theta:.2f}')
        
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        if not self._wait_for_future(send_goal_future, timeout_sec=5.0):
            self.get_logger().error('Navigation goal request timed out')
            return False
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Navigation goal rejected')
            return False
        
        result_future = goal_handle.get_result_async()
        if not self._wait_for_future(result_future, timeout_sec=self.nav_timeout):
            self.get_logger().error('Navigation timeout')
            return False
        
        wrapped_result = result_future.result()
        if wrapped_result is None:
            self.get_logger().error('Navigation failed without result')
            return False

        status = wrapped_result.status
        if status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(f'Navigation failed with status={status}')
            return False

        nav_result = wrapped_result.result
        if hasattr(nav_result, 'error_code') and int(nav_result.error_code) != 0:
            self.get_logger().error(
                f'Navigation failed with error_code={int(nav_result.error_code)}'
            )
            return False

        self.get_logger().info('Navigation succeeded')
        return True

    def compute_path_to_pose(
        self,
        x: float,
        y: float,
        theta: float,
        planner_id: str = ''
    ) -> Tuple[bool, float, str]:
        """Compute a path to pose and return (ok, path_length_m, reason)."""
        if self.mock_navigation:
            return True, 0.0, 'mock_navigation'

        if not self.compute_path_client.wait_for_server(timeout_sec=2.0):
            return False, float('inf'), 'planner_server_unavailable'

        goal = ComputePathToPose.Goal()
        goal.goal = PoseStamped()
        goal.goal.header.frame_id = 'map'
        goal.goal.header.stamp = self.get_clock().now().to_msg()
        goal.goal.pose.position.x = float(x)
        goal.goal.pose.position.y = float(y)
        goal.goal.pose.orientation.z = math.sin(float(theta) / 2.0)
        goal.goal.pose.orientation.w = math.cos(float(theta) / 2.0)
        goal.planner_id = str(planner_id)
        goal.use_start = False

        send_goal_future = self.compute_path_client.send_goal_async(goal)
        if not self._wait_for_future(send_goal_future, timeout_sec=3.0):
            return False, float('inf'), 'planner_goal_timeout'

        goal_handle = send_goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return False, float('inf'), 'planner_goal_rejected'

        result_future = goal_handle.get_result_async()
        if not self._wait_for_future(result_future, timeout_sec=float(self.planner_timeout)):
            return False, float('inf'), 'planner_result_timeout'

        wrapped_result = result_future.result()
        if wrapped_result is None:
            return False, float('inf'), 'planner_no_result'

        if wrapped_result.status != GoalStatus.STATUS_SUCCEEDED:
            return False, float('inf'), f'planner_status_{int(wrapped_result.status)}'

        action_result = wrapped_result.result
        if hasattr(action_result, 'error_code') and int(action_result.error_code) != 0:
            return False, float('inf'), f'planner_error_{int(action_result.error_code)}'

        path = getattr(action_result, 'path', None)
        poses = list(getattr(path, 'poses', [])) if path is not None else []
        if len(poses) < 2:
            return False, float('inf'), 'planner_empty_path'

        length = 0.0
        previous = poses[0].pose.position
        for pose_stamped in poses[1:]:
            current = pose_stamped.pose.position
            dx = float(current.x) - float(previous.x)
            dy = float(current.y) - float(previous.y)
            length += math.hypot(dx, dy)
            previous = current

        return True, float(length), 'ok'
    
    def move_servo(self, servo_id: int, angle: float, speed: float = 30.0) -> bool:
        """Move servo to target angle."""
        if self.mock_actuators:
            self.get_logger().info(f'[MOCK] Moving servo {servo_id} to {angle}° at {speed}°/s')
            return True

        if self.servo_client is None:
            self.get_logger().error('Servo action client unavailable')
            return False

        if not self.servo_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('Servo action server not available')
            return False
        
        goal_msg = MoveServo.Goal()
        goal_msg.servo_id = servo_id
        goal_msg.target_deg = angle
        goal_msg.speed_deg_s = speed
        
        self.get_logger().info(f'Moving servo {servo_id} to {angle}°')
        
        send_goal_future = self.servo_client.send_goal_async(goal_msg)
        if not self._wait_for_future(send_goal_future, timeout_sec=2.0):
            self.get_logger().error('Servo goal request timed out')
            return False
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Servo goal rejected')
            return False
        
        result_future = goal_handle.get_result_async()
        if not self._wait_for_future(result_future, timeout_sec=self.servo_timeout):
            self.get_logger().error('Servo action timeout')
            return False
        
        if result_future.result() is not None:
            return result_future.result().result.success
        self.get_logger().error('Servo action failed without result')
        return False
    
    def control_pump(self, pump_id: int, enable: bool, duty_cycle: float = 1.0, 
                     duration: float = 0.0) -> bool:
        """Control pump on/off with optional duration."""
        if self.mock_actuators:
            action = 'ON' if enable else 'OFF'
            self.get_logger().info(
                f'[MOCK] Turning pump {pump_id} {action} duty={duty_cycle:.2f} duration={duration:.2f}s'
            )
            return True

        if self.pump_client is None:
            self.get_logger().error('Pump action client unavailable')
            return False

        if not self.pump_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('Pump action server not available')
            return False
        
        goal_msg = ControlPump.Goal()
        goal_msg.pump_id = pump_id
        goal_msg.enable = enable
        goal_msg.duty_cycle = duty_cycle
        goal_msg.duration_sec = duration
        
        action = 'ON' if enable else 'OFF'
        self.get_logger().info(f'Turning pump {pump_id} {action}')
        
        send_goal_future = self.pump_client.send_goal_async(goal_msg)
        if not self._wait_for_future(send_goal_future, timeout_sec=2.0):
            self.get_logger().error('Pump goal request timed out')
            return False
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Pump goal rejected')
            return False
        
        # If duration > 0, wait for completion
        if duration > 0:
            result_future = goal_handle.get_result_async()
            if not self._wait_for_future(result_future, timeout_sec=duration + self.pump_timeout):
                self.get_logger().error('Pump action timeout')
                return False
            if result_future.result() is not None:
                return result_future.result().result.success
            self.get_logger().error('Pump action failed without result')
            return False
        
        return True
    
    def execute_sequence(self, steps: list) -> bool:
        """Execute an ordered list of ActuatorStep via the actuator_sequencer."""
        if self.mock_actuators:
            for i, step in enumerate(steps):
                if step.step_type == ActuatorStep.MOVE_SERVO:
                    self.get_logger().info(
                        f'[MOCK] Step {i}: MOVE_SERVO servo={step.servo_id} → {step.target_deg:.1f}°'
                    )
                else:
                    action = 'ON' if step.pump_enable else 'OFF'
                    self.get_logger().info(
                        f'[MOCK] Step {i}: CONTROL_PUMP pump={step.pump_id} {action}'
                    )
            return True

        if self.sequence_client is None:
            self.get_logger().error('Sequence action client unavailable')
            return False

        if not self.sequence_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('execute_sequence action server not available')
            return False

        goal_msg = ExecuteSequence.Goal()
        goal_msg.steps = steps

        send_goal_future = self.sequence_client.send_goal_async(goal_msg)
        if not self._wait_for_future(send_goal_future, timeout_sec=5.0):
            self.get_logger().error('Sequence goal request timed out')
            return False

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Sequence goal rejected')
            return False

        total_timeout = self.servo_timeout * len(steps) + self.pump_timeout
        result_future = goal_handle.get_result_async()
        if not self._wait_for_future(result_future, timeout_sec=total_timeout):
            self.get_logger().error('Sequence action timed out')
            return False

        res = result_future.result()
        if res is None:
            self.get_logger().error('Sequence action returned no result')
            return False
        if not res.result.success:
            self.get_logger().error(
                f'Sequence failed at step {res.result.failed_step}: {res.result.message}'
            )
        return bool(res.result.success)

    def move_relative(
        self,
        target_x_m: float,
        target_y_m: float,
        target_yaw_rad: float,
        pos_tolerance_m: float = 0.02,
        yaw_tolerance_rad: float = 0.05,
        timeout_s: float = 6.0,
        max_linear_speed_mps: float = 0.2,
        max_angular_speed_rps: float = 0.5,
    ) -> bool:
        """Move robot relative to current frame using motion_controller service."""
        if self.mock_navigation:
            self.get_logger().info(
                f'[MOCK] move_relative x={target_x_m:.3f} y={target_y_m:.3f} yaw={target_yaw_rad:.3f}'
            )
            return True

        if self.move_relative_client is None:
            self.get_logger().error('MoveRelative service client unavailable')
            return False

        if not self.move_relative_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('MoveRelative service not available')
            return False

        request = MoveRelative.Request()
        request.target_x_m = float(target_x_m)
        request.target_y_m = float(target_y_m)
        request.target_yaw_rad = float(target_yaw_rad)
        request.pos_tolerance_m = float(pos_tolerance_m)
        request.yaw_tolerance_rad = float(yaw_tolerance_rad)
        request.timeout_s = float(timeout_s)
        request.max_linear_speed_mps = float(max_linear_speed_mps)
        request.max_angular_speed_rps = float(max_angular_speed_rps)

        future = self.move_relative_client.call_async(request)
        if not self._wait_for_future(future, timeout_sec=max(2.0, timeout_s + 1.0)):
            self.get_logger().error('MoveRelative service call timed out')
            return False

        response = future.result()
        if response is None:
            self.get_logger().error('MoveRelative returned no response')
            return False

        if not response.success:
            self.get_logger().warn(f'MoveRelative failed: {response.status_message}')
        else:
            self.get_logger().info(f'MoveRelative succeeded: {response.status_message}')
        return bool(response.success)

    def move_back_straight_after_drop(
        self,
        distance_m: float | None = None,
        timeout_s: float | None = None,
        max_linear_speed_mps: float | None = None,
    ) -> bool:
        """Move straight backward after a drop using the relative move service."""
        retreat_distance_m = abs(
            float(distance_m if distance_m is not None else self.post_drop_back_distance_m)
        )
        retreat_timeout_s = float(
            timeout_s if timeout_s is not None else self.post_drop_back_timeout_s
        )
        retreat_max_linear_speed_mps = abs(float(
            max_linear_speed_mps
            if max_linear_speed_mps is not None
            else self.post_drop_back_max_linear_speed_mps
        ))

        if retreat_distance_m == 0.0:
            self.get_logger().info('Post-drop move back skipped: distance is zero')
            return True

        self.get_logger().info(
            'Post-drop move back: distance=%.3fm timeout=%.2fs max_linear_speed=%.3fm/s'
            % (retreat_distance_m, retreat_timeout_s, retreat_max_linear_speed_mps)
        )
        return self.move_relative(
            -retreat_distance_m,
            0.0,
            0.0,
            timeout_s=retreat_timeout_s,
            max_linear_speed_mps=retreat_max_linear_speed_mps,
        )

    def call_service_async(self, service_name: str, srv_type):
        """Helper to call a service asynchronously."""
        client = self.create_client(srv_type, service_name)
        if client.wait_for_service(timeout_sec=1.0):
            future = client.call_async(srv_type.Request())
            return future
        return None
    
    # Abstract methods to be implemented by subclasses
    def load_mission_config(self):
        """Load mission-specific configuration. Override in subclass."""
        pass
    
    def execute_mission(self):
        """Execute mission logic. Override in subclass."""
        self.get_logger().error('execute_mission() not implemented in subclass')
        self.state = MissionState.FAILED
