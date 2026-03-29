#!/usr/bin/env python3
"""
Floor image publisher - converts image to colored point cloud on floor plane.
This actually displays the image ON THE FLOOR in RVIZ.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import struct
import cv2
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory


class FloorImagePublisher(Node):
    def __init__(self):
        super().__init__('floor_image_publisher')
        
        # Publisher
        self.cloud_pub = self.create_publisher(PointCloud2, 'floor_point_cloud', 10)
        
        # Load image
        try:
            pkg_dir = get_package_share_directory('holonomic_robot_bringup')
            image_path = os.path.join(pkg_dir, 'images', 'floor_marking.png')
        except:
            image_path = 'images/floor_marking.png'
        
        if os.path.exists(image_path):
            self.image = cv2.imread(image_path)
            self.get_logger().info(f'✓ Loaded floor image: {image_path}')
            self.get_logger().info(f'  Image shape: {self.image.shape}')
        else:
            self.get_logger().error(f'✗ Image not found: {image_path}')
            self.get_logger().info('Creating dummy checkerboard pattern...')
            # Create a simple checkerboard pattern as fallback
            self.image = self.create_checkerboard(400, 400, 50)
        
        # Floor dimensions (adjust to your room size)
        self.floor_width = 3.0   # meters
        self.floor_height = 2.0  # meters
        self.floor_x_offset = 0.0  # Start position
        self.floor_y_offset = 0.0
        
        # Timer
        self.timer = self.create_timer(1.0, self.publish_floor_cloud)
        
        self.get_logger().info('Floor image publisher started')
        self.get_logger().info('In RVIZ: Add -> PointCloud2 -> Topic: /floor_point_cloud')
        self.get_logger().info('         Style: Flat Squares, Size: 0.03, Color: RGB8')
    
    def create_checkerboard(self, width, height, square_size):
        """Create a checkerboard pattern as fallback."""
        img = np.zeros((height, width, 3), dtype=np.uint8)
        for i in range(0, height, square_size):
            for j in range(0, width, square_size):
                if ((i // square_size) + (j // square_size)) % 2 == 0:
                    img[i:i+square_size, j:j+square_size] = [200, 200, 200]  # Light gray
                else:
                    img[i:i+square_size, j:j+square_size] = [100, 100, 100]  # Dark gray
        return img
    
    def publish_floor_cloud(self):
        """Convert image to colored point cloud on floor."""
        if self.image is None:
            return
        
        # Resize image for reasonable point count (100x80 = 8000 points)
        img_small = cv2.resize(self.image, (300, 200))
        height, width = img_small.shape[:2]
        
        # Create point cloud points
        points = []
        
        for y in range(height):
            for x in range(width):
                # Map pixel position to floor coordinates
                px = self.floor_x_offset + (x / width) * self.floor_width
                py = self.floor_y_offset + (y / height) * self.floor_height
                pz = 0.002  # Slightly above floor (2mm)
                
                # Get pixel color (BGR -> RGB)
                b, g, r = img_small[y, x]
                
                # Pack RGB into float for PointCloud2
                rgb = struct.unpack('f', struct.pack('I', (int(r) << 16) | (int(g) << 8) | int(b)))[0]
                
                points.append([px, py, pz, rgb])
        
        # Create PointCloud2 message
        header = Header()
        header.frame_id = 'odom'
        header.stamp = self.get_clock().now().to_msg()
        
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        
        cloud_data = []
        for p in points:
            cloud_data.append(struct.pack('ffff', p[0], p[1], p[2], p[3]))
        
        cloud_msg = PointCloud2()
        cloud_msg.header = header
        cloud_msg.height = 1
        cloud_msg.width = len(points)
        cloud_msg.fields = fields
        cloud_msg.is_bigendian = False
        cloud_msg.point_step = 16
        cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width
        cloud_msg.is_dense = True
        cloud_msg.data = b''.join(cloud_data)
        
        self.cloud_pub.publish(cloud_msg)


def main(args=None):
    rclpy.init(args=args)
    node = FloorImagePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()