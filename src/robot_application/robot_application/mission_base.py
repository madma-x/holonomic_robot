"""Base class for mission implementations."""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Trigger, SetBool
from std_msgs.msg import String, Float32
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from robot_actuators.action import MoveServo, ControlPump, ControlGripper
import threading
from enum import Enum


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
        
        # Get parameters
        self.nav_timeout = self.get_parameter('nav_timeout_sec').value
        self.servo_timeout = self.get_parameter('servo_timeout_sec').value
        self.pump_timeout = self.get_parameter('pump_timeout_sec').value
        self.enable_recovery = self.get_parameter('enable_recovery').value
        self.max_recovery_attempts = self.get_parameter('max_recovery_attempts').value
        
        # Action clients
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.servo_client = ActionClient(self, MoveServo, 'move_servo')
        self.pump_client = ActionClient(self, ControlPump, 'control_pump')
        self.gripper_client = ActionClient(self, ControlGripper, 'control_gripper')
        
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
    
    def navigate_to_pose(self, x: float, y: float, theta: float) -> bool:
        """Navigate to a pose using Nav2."""
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 action server not available')
            return False
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        
        # Convert theta to quaternion (simplified for z-axis rotation)
        import math
        goal_msg.pose.pose.orientation.z = math.sin(theta / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(theta / 2.0)
        
        self.get_logger().info(f'Navigating to: x={x:.2f}, y={y:.2f}, theta={theta:.2f}')
        
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=5.0)
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Navigation goal rejected')
            return False
        
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=self.nav_timeout)
        
        if result_future.result() is not None:
            self.get_logger().info('Navigation succeeded')
            return True
        else:
            self.get_logger().error('Navigation timeout')
            return False
    
    def move_servo(self, servo_id: int, angle: float, speed: float = 30.0) -> bool:
        """Move servo to target angle."""
        if not self.servo_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('Servo action server not available')
            return False
        
        goal_msg = MoveServo.Goal()
        goal_msg.servo_id = servo_id
        goal_msg.angle_deg = angle
        goal_msg.max_speed_deg_per_sec = speed
        
        self.get_logger().info(f'Moving servo {servo_id} to {angle}°')
        
        send_goal_future = self.servo_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=2.0)
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Servo goal rejected')
            return False
        
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=self.servo_timeout)
        
        if result_future.result() is not None:
            return result_future.result().result.success
        else:
            self.get_logger().error('Servo action timeout')
            return False
    
    def control_pump(self, pump_id: int, enable: bool, duty_cycle: float = 1.0, 
                     duration: float = 0.0) -> bool:
        """Control pump on/off with optional duration."""
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
        rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=2.0)
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Pump goal rejected')
            return False
        
        # If duration > 0, wait for completion
        if duration > 0:
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future, 
                                           timeout_sec=duration + self.pump_timeout)
            if result_future.result() is not None:
                return result_future.result().result.success
            else:
                self.get_logger().error('Pump action timeout')
                return False
        
        return True
    
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
