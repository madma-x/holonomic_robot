#!/usr/bin/env python3
"""
Fake lidar publisher that simulates 2D laser scan data.
Performs ray-casting against a 3m x 2m rectangular room to simulate lidar readings.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import math


class FakeLidarPublisher(Node):
    def __init__(self):
        super().__init__('fake_lidar_publisher')
        
        # Define 8-point polygon (same as environment markers)
        self.polygon_points = [
            (0.0, 0.0),      # Point 0: Bottom left corner
            (0.6, 0.0),      # Point 1: Bottom right (long wall)
            (0.6, 0.5),      # Point 2: Right wall goes up
            (2.4, 0.5),      # Point 3: Alcove entrance right
            (2.4, 0.0),      # Point 4: Alcove back right (unique feature!)
            (3.0, 0.0),      # Point 5: Alcove back left
            (3.0, 2.0),      # Point 6: Alcove entrance left
            (0.0, 2.0),      # Point 7: Left wall back to start
        ]
        
        # Lidar parameters
        self.angle_min = 0.0
        self.angle_max = 2 * math.pi
        self.angle_increment = 0.225*math.pi / 180.0  # 0.225 degree resolution
        self.num_readings = int((self.angle_max - self.angle_min) / self.angle_increment)
        self.range_min = 0.12
        self.range_max = 12.0
        
        # TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Publisher
        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)
        
        # Timer for lidar updates (10 Hz)
        self.timer = self.create_timer(0.1, self.publish_scan)
        
        self.get_logger().info('Fake lidar publisher started')
    
    def get_robot_pose(self):
        """Get robot pose from TF tree."""
        try:
            # Get transform from odom to base_scan (lidar frame)
            transform = self.tf_buffer.lookup_transform(
                'odom',
                'base_scan',
                rclpy.time.Time()
            )
            
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            
            # Extract yaw from quaternion
            quat = transform.transform.rotation
            siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
            cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
            theta = math.atan2(siny_cosp, cosy_cosp)
            
            return x, y, theta
        except TransformException as ex:
            self.get_logger().debug(f'Could not transform: {ex}')
            return 0.0, 0.0, 0.0
    
    def ray_cast(self, x, y, theta, angle):
        """
        Cast a ray from robot position at given angle and find intersection with polygon walls.
        Returns distance to nearest wall.
        """
        # Ray angle in world frame
        ray_angle = theta + angle
        
        # Ray direction
        dx = math.cos(ray_angle)
        dy = math.sin(ray_angle)
        
        min_distance = self.range_max
        
        # Check intersection with each edge of the polygon
        num_points = len(self.polygon_points)
        for i in range(num_points):
            # Wall segment from point i to point i+1
            x1, y1 = self.polygon_points[i]
            x2, y2 = self.polygon_points[(i + 1) % num_points]
            
            # Line-line intersection using parametric equations
            # Ray: (x, y) + t * (dx, dy)
            # Wall: (x1, y1) + s * (x2-x1, y2-y1), where 0 <= s <= 1
            
            wall_dx = x2 - x1
            wall_dy = y2 - y1
            
            denominator = dx * wall_dy - dy * wall_dx
            
            if abs(denominator) > 1e-6:  # Not parallel
                # Solve for t and s
                t = ((x1 - x) * wall_dy - (y1 - y) * wall_dx) / denominator
                s = ((x1 - x) * dy - (y1 - y) * dx) / denominator
                
                # Check if intersection is valid (t > 0, 0 <= s <= 1)
                if t > 0 and 0 <= s <= 1:
                    # Calculate distance
                    intersect_x = x + t * dx
                    intersect_y = y + t * dy
                    distance = math.sqrt((intersect_x - x)**2 + (intersect_y - y)**2)
                    min_distance = min(min_distance, distance)
        
        return min_distance
    
    def publish_scan(self):
        """Publish simulated laser scan."""
        # Get robot pose
        x, y, theta = self.get_robot_pose()
        
        # Create LaserScan message
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'base_scan'
        
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_increment
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        
        # Perform ray-casting for each angle
        ranges = []
        for i in range(self.num_readings):
            angle = self.angle_min + i * self.angle_increment
            distance = self.ray_cast(x, y, theta, angle)
            
            # Clamp to valid range
            if distance < self.range_min:
                distance = self.range_min
            elif distance > self.range_max:
                distance = float('inf')  # No detection
            
            ranges.append(distance)
        
        scan.ranges = ranges
        scan.intensities = []
        
        self.scan_pub.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = FakeLidarPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
