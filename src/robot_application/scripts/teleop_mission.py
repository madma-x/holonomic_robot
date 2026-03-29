#!/usr/bin/env python3
"""Teleop mission - manual control with safety features."""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_srvs.srv import Trigger
from std_msgs.msg import String
import time


class TeleopMission(Node):
    """Teleop mission: manual control with velocity limits and safety."""
    
    def __init__(self):
        super().__init__('teleop_mission')
        
        # Declare parameters
        self.declare_parameter('max_linear_vel', 0.5)  # m/s
        self.declare_parameter('max_angular_vel', 1.0)  # rad/s
        self.declare_parameter('linear_scale', 1.0)
        self.declare_parameter('angular_scale', 1.0)
        self.declare_parameter('deadman_button', 4)  # L1 button
        self.declare_parameter('turbo_button', 5)  # R1 button
        self.declare_parameter('enable_timeout', True)
        self.declare_parameter('timeout_duration', 5.0)  # seconds
        
        # Get parameters
        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        self.max_angular_vel = self.get_parameter('max_angular_vel').value
        self.linear_scale = self.get_parameter('linear_scale').value
        self.angular_scale = self.get_parameter('angular_scale').value
        self.deadman_button = self.get_parameter('deadman_button').value
        self.turbo_button = self.get_parameter('turbo_button').value
        self.enable_timeout = self.get_parameter('enable_timeout').value
        self.timeout_duration = self.get_parameter('timeout_duration').value
        
        # State
        self.last_joy_time = time.time()
        self.deadman_pressed = False
        self.turbo_mode = False
        self.enabled = True
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/teleop/status', 10)
        
        # Subscribers
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        
        # Services
        self.enable_srv = self.create_service(Trigger, '/teleop/enable', self.enable_callback)
        self.disable_srv = self.create_service(Trigger, '/teleop/disable', self.disable_callback)
        
        # Timers
        self.timeout_timer = self.create_timer(0.1, self.check_timeout)
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        self.get_logger().info('Teleop mission initialized')
        self.get_logger().info(f'Max linear vel: {self.max_linear_vel} m/s')
        self.get_logger().info(f'Max angular vel: {self.max_angular_vel} rad/s')
        self.get_logger().info(f'Deadman button: {self.deadman_button}')
    
    def joy_callback(self, msg: Joy):
        """Process joystick input."""
        self.last_joy_time = time.time()
        
        # Check deadman button
        if len(msg.buttons) > self.deadman_button:
            self.deadman_pressed = msg.buttons[self.deadman_button] == 1
        else:
            self.deadman_pressed = False
        
        # Check turbo button
        if len(msg.buttons) > self.turbo_button:
            self.turbo_mode = msg.buttons[self.turbo_button] == 1
        else:
            self.turbo_mode = False
        
        # Only send commands if deadman pressed and teleop enabled
        if not self.deadman_pressed or not self.enabled:
            self.send_zero_velocity()
            return
        
        # Extract velocity commands from joystick
        # Typical joystick layout: axes[1] = forward/back, axes[0] = left/right strafe, axes[3] = rotation
        twist = Twist()
        
        if len(msg.axes) >= 4:
            # Linear velocity (forward/back)
            linear_x = msg.axes[1] * self.linear_scale * self.max_linear_vel
            # Strafe velocity (left/right) - for holonomic robot
            linear_y = msg.axes[0] * self.linear_scale * self.max_linear_vel
            # Angular velocity (rotation)
            angular_z = msg.axes[3] * self.angular_scale * self.max_angular_vel
            
            # Apply turbo mode (2x speed)
            if self.turbo_mode:
                linear_x *= 2.0
                linear_y *= 2.0
                angular_z *= 2.0
            
            # Clamp velocities
            twist.linear.x = self.clamp(linear_x, -self.max_linear_vel, self.max_linear_vel)
            twist.linear.y = self.clamp(linear_y, -self.max_linear_vel, self.max_linear_vel)
            twist.angular.z = self.clamp(angular_z, -self.max_angular_vel, self.max_angular_vel)
        
        self.cmd_vel_pub.publish(twist)
    
    def check_timeout(self):
        """Check for joystick timeout."""
        if not self.enable_timeout:
            return
        
        time_since_joy = time.time() - self.last_joy_time
        
        if time_since_joy > self.timeout_duration and self.enabled:
            self.get_logger().warn(f'Joystick timeout ({time_since_joy:.1f}s), stopping robot')
            self.send_zero_velocity()
    
    def send_zero_velocity(self):
        """Send zero velocity command."""
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
    
    def enable_callback(self, request, response):
        """Enable teleop control."""
        self.enabled = True
        response.success = True
        response.message = 'Teleop enabled'
        self.get_logger().info(response.message)
        return response
    
    def disable_callback(self, request, response):
        """Disable teleop control."""
        self.enabled = False
        self.send_zero_velocity()
        response.success = True
        response.message = 'Teleop disabled'
        self.get_logger().info(response.message)
        return response
    
    def publish_status(self):
        """Publish teleop status."""
        status = String()
        if self.enabled:
            if self.deadman_pressed:
                status.data = 'ACTIVE' if not self.turbo_mode else 'ACTIVE_TURBO'
            else:
                status.data = 'ENABLED_IDLE'
        else:
            status.data = 'DISABLED'
        
        self.status_pub.publish(status)
    
    @staticmethod
    def clamp(value, min_val, max_val):
        """Clamp value between min and max."""
        return max(min_val, min(max_val, value))


def main(args=None):
    rclpy.init(args=args)
    
    node = TeleopMission()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
