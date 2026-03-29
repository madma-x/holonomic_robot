#!/usr/bin/env python3
"""
Fake optical flow sensor that provides odometry with increasing drift over time.
Simulates visual odometry from a downward-facing camera.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Quaternion, TransformStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import math
import random


class FakeOpticalFlow(Node):
    def __init__(self):
        super().__init__('fake_optical_flow')
        
        # Robot pose in optical flow frame (with drift)
        self.x = 0.3
        self.y = 0.2
        self.theta = 0.0
        
        # Robot velocities
        self.vx = 0.0
        self.vy = 0.0
        self.vth = 0.0
        
        # Drift parameters
        self.linear_drift_rate = 0.0  # m/s drift rate
        self.angular_drift_rate = 0.001 # rad/s drift rate
        self.noise_std = 0.001  # measurement noise
        
        # Accumulated drift
        self.drift_x = 0.0
        self.drift_y = 0.0
        self.drift_theta = 0.0
        
        # Time tracking
        self.last_time = self.get_clock().now()
        self.start_time = self.get_clock().now()
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Subscribe to RViz's "Set Initial Pose" (2D Pose Estimate)
        self.initialpose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            'initialpose',
            self.initialpose_callback,
            10
        )
        
        # Timer for odometry updates (30 Hz)
        self.timer = self.create_timer(1.0/30.0, self.update_odometry)
        
        self.get_logger().info('Fake optical flow sensor started (with drift)')
    
    def cmd_vel_callback(self, msg):
        """Update velocity commands from cmd_vel."""
        self.vx = msg.linear.x
        self.vy = msg.linear.y
        self.vth = msg.angular.z
    
    def initialpose_callback(self, msg):
        """Update odometry when RViz publishes Set Initial Pose (2D Pose Estimate)."""
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        # Extract yaw from quaternion
        quat = msg.pose.pose.orientation
        self.theta = math.atan2(2.0 * (quat.w * quat.z + quat.x * quat.y),
                                 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z))
        self.get_logger().info(f'Initial pose set from RViz: x={self.x:.2f}, y={self.y:.2f}, yaw={self.theta:.2f} rad')
    
    def update_odometry(self):
        """Update robot pose with drift and noise."""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        # Gyro drift accumulates over time (IMU integration error)
        # Angular drift continues even when stationary
        self.drift_theta += random.gauss(0, self.angular_drift_rate) * dt
        
        # Optical flow: accurate linear measurements, no drift when stationary
        # Only add noise to velocity measurements (not accumulated drift)
        noise_vx = random.gauss(0, self.noise_std) if abs(self.vx) > 0.001 or abs(self.vy) > 0.001 else 0
        noise_vy = random.gauss(0, self.noise_std) if abs(self.vx) > 0.001 or abs(self.vy) > 0.001 else 0
        
        # Gyro noise always present
        noise_vth = random.gauss(0, self.noise_std * 0.5)
        
        # Measured velocities with noise
        measured_vx = self.vx + noise_vx
        measured_vy = self.vy + noise_vy
        measured_vth = self.vth + noise_vth
        
        # Transform linear velocities to world frame using DRIFTED orientation
        # This is key: linear measurement is accurate, but rotation error accumulates
        delta_x = (measured_vx * math.cos(self.theta) - measured_vy * math.sin(self.theta)) * dt
        delta_y = (measured_vx * math.sin(self.theta) + measured_vy * math.cos(self.theta)) * dt
        delta_theta = measured_vth * dt
        
        # Update pose (linear drift comes from wrong rotation, not position measurement)
        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta + self.drift_theta * dt
        
        # Normalize theta
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # Create quaternion from yaw
        quat = self.quaternion_from_euler(0, 0, self.theta)
        
        # Publish odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        
        # Position (with drift)
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = quat
        
        # Set covariance to indicate uncertainty (increases over time)
        runtime = (current_time - self.start_time).nanoseconds / 1e9
        position_var = 0.01 + runtime * 0.001  # Variance increases with time
        orientation_var = 0.01 + runtime * 0.0005
        
        odom.pose.covariance = [
            position_var, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, position_var, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, orientation_var
        ]
        
        # Velocity
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = self.vth
        
        # Broadcast TF transform
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = quat
        self.tf_broadcaster.sendTransform(t)
        
        self.odom_pub.publish(odom)
    
    def quaternion_from_euler(self, roll, pitch, yaw):
        """Convert Euler angles to quaternion."""
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        
        quat = Quaternion()
        quat.x = qx
        quat.y = qy
        quat.z = qz
        quat.w = qw
        return quat


def main(args=None):
    rclpy.init(args=args)
    node = FakeOpticalFlow()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
