#!/usr/bin/env python3
"""Publish initial custom object markers for Nav2 custom_objects_layer."""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from visualization_msgs.msg import Marker, MarkerArray


class CustomObjectsInitializer(Node):
    def __init__(self):
        super().__init__('custom_objects_initializer')

        self.declare_parameter('topic', '/custom_objects')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('marker_ns', 'custom_objects')

        # One center per pick location (8 total, IDs 0..7)
        self.declare_parameter('group_centers_x', [0.0])
        self.declare_parameter('group_centers_y', [0.0])
        self.declare_parameter('group_centers_theta', [0.0])

        self.declare_parameter('object_size_x', 0.15)
        self.declare_parameter('object_size_y', 0.20)
        self.declare_parameter('object_height', 0.02)

        topic = str(self.get_parameter('topic').value)
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pub = self.create_publisher(MarkerArray, topic, qos)

        self._publish_timer = self.create_timer(0.5, self._publish_tick)
        self._published = False

    def _publish_tick(self):
        if self._published:
            return

        marker_array = self._build_marker_array()
        if marker_array is None:
            return

        self.pub.publish(marker_array)
        self._published = True
        self.get_logger().info(
            f'Published {len(marker_array.markers)} custom object markers '
            f'on {self.pub.topic_name}'
        )

    def _publish_once(self):  # kept for API compat
        pass  # noqa

    def _build_marker_array(self):
        frame_id = str(self.get_parameter('frame_id').value)
        marker_ns = str(self.get_parameter('marker_ns').value)
        group_centers_x = list(self.get_parameter('group_centers_x').value)
        group_centers_y = list(self.get_parameter('group_centers_y').value)
        group_centers_theta = list(self.get_parameter('group_centers_theta').value)
        size_x = float(self.get_parameter('object_size_x').value)
        size_y = float(self.get_parameter('object_size_y').value)
        size_z = float(self.get_parameter('object_height').value)

        if not group_centers_x:
            self.get_logger().error('group_centers_x is empty; no objects published')
            return None

        if len(group_centers_x) != len(group_centers_y):
            self.get_logger().error(
                'group_centers_x and group_centers_y must have the same length; no objects published'
            )
            return None

        marker_array = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        theta_values = group_centers_theta if len(group_centers_theta) == len(group_centers_x) else [0.0] * len(group_centers_x)

        for marker_id, (cx, cy, theta) in enumerate(zip(group_centers_x, group_centers_y, theta_values)):
            half_theta = float(theta) * 0.5
            marker = Marker()
            marker.header.frame_id = frame_id
            marker.header.stamp = stamp
            marker.ns = marker_ns
            marker.id = marker_id
            marker.type = Marker.CUBE
            marker.action = Marker.ADD

            marker.pose.position.x = float(cx)
            marker.pose.position.y = float(cy)
            marker.pose.position.z = 0.0
            marker.pose.orientation.z = math.sin(half_theta)
            marker.pose.orientation.w = math.cos(half_theta)

            marker.scale.x = size_x
            marker.scale.y = size_y
            marker.scale.z = size_z

            marker.color.r = 1.0
            marker.color.g = 0.45
            marker.color.b = 0.0
            marker.color.a = 0.85

            marker.lifetime.sec = 0

            marker_array.markers.append(marker)

        return marker_array


def main(args=None):
    rclpy.init(args=args)
    node = CustomObjectsInitializer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
