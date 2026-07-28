#!/usr/bin/env python3
"""
Environment markers publisher for visualizing the room walls in RVIZ2.
Publishes MarkerArray representing an asymmetric L-shaped room.
"""

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA


class EnvironmentMarkers(Node):
    def __init__(self):
        super().__init__('environment_markers')
        
        # Publisher
        self.marker_pub = self.create_publisher(MarkerArray, 'environment_markers', 10)
        
        # Define asymmetric L-shaped room with alcove (8 points)
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
        
        # Timer to publish markers
        self.timer = self.create_timer(1.0, self.publish_markers)
        
        self.get_logger().info('Environment markers publisher started (asymmetric L-shape)')
    
    def publish_markers(self):
        """Publish wall markers for the polygon."""
        marker_array = MarkerArray()
        
        # Create wall markers between consecutive points
        num_points = len(self.polygon_points)
        for i in range(num_points):
            p1 = self.polygon_points[i]
            p2 = self.polygon_points[(i + 1) % num_points]
            marker = self.create_wall_marker(i, p1[0], p1[1], p2[0], p2[1])
            marker_array.markers.append(marker)
        
        self.marker_pub.publish(marker_array)
    
    def create_wall_marker(self, marker_id, x1, y1, x2, y2):
        """Create a line marker representing a wall."""
        marker = Marker()
        marker.header.frame_id = 'odom'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'walls'
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        
        # Wall points
        p1 = Point()
        p1.x = x1
        p1.y = y1
        p1.z = 0.0
        
        p2 = Point()
        p2.x = x2
        p2.y = y2
        p2.z = 0.0
        
        marker.points = [p1, p2]
        
        # Wall appearance
        marker.scale.x = 0.1  # Line width
        marker.color.r = 0.5
        marker.color.g = 0.5
        marker.color.b = 0.5
        marker.color.a = 1.0
        
        marker.lifetime.sec = 0  # Forever
        
        return marker


def main(args=None):
    rclpy.init(args=args)
    node = EnvironmentMarkers()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()