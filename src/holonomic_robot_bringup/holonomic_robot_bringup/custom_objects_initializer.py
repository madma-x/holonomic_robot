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

        self.declare_parameter('intra_spacing_x', 0.0)
        self.declare_parameter('intra_spacing_y', 0.0)
        # Explicit group centers in map frame.
        self.declare_parameter('group_centers_x', [0.0])
        self.declare_parameter('group_centers_y', [0.0])
        self.declare_parameter('group_centers_theta', [0.0])

        self.declare_parameter('object_size_x', 0.15)
        self.declare_parameter('object_size_y', 0.20)
        self.declare_parameter('object_height', 0.0)

        topic = str(self.get_parameter('topic').value)
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pub = self.create_publisher(MarkerArray, topic, qos)

        self._publish_once_timer = self.create_timer(0.5, self._publish_once)
        self._published = False

    def _publish_once(self):
        if self._published:
            return

        frame_id = str(self.get_parameter('frame_id').value)
        marker_ns = str(self.get_parameter('marker_ns').value)

        intra_spacing_x = float(self.get_parameter('intra_spacing_x').value)
        intra_spacing_y = float(self.get_parameter('intra_spacing_y').value)
        group_centers_x = list(self.get_parameter('group_centers_x').value)
        group_centers_y = list(self.get_parameter('group_centers_y').value)
        group_centers_theta = list(self.get_parameter('group_centers_theta').value)

        size_x = float(self.get_parameter('object_size_x').value)
        size_y = float(self.get_parameter('object_size_y').value)
        size_z = float(self.get_parameter('object_height').value)

        marker_array = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        offsets = [
            (0.0,-0.75),
            (0.0, -0.25),
            (0.0, 0.25),
            (0.0,0.75),
        ]

        if not group_centers_x and not group_centers_y:
            self.get_logger().error(
                'group_centers_x/group_centers_y are empty; no objects published'
            )
            return

        if len(group_centers_x) != len(group_centers_y):
            self.get_logger().error(
                'group_centers_x and group_centers_y must have the same length; no objects published'
            )
            return

        theta_values = [0.0] * len(group_centers_x)
        if group_centers_theta:
            if len(group_centers_theta) != len(group_centers_x):
                self.get_logger().error(
                    'group_centers_theta must match group_centers_x/group_centers_y length; '
                    'using 0.0 rad for all group orientations'
                )
            else:
                theta_values = [float(theta) for theta in group_centers_theta]

        group_centers = [
            (float(x), float(y), theta)
            for x, y, theta in zip(group_centers_x, group_centers_y, theta_values)
        ]

        object_id = 0
        total_groups = len(group_centers)
        for cx, cy, theta in group_centers:
            half_theta = theta * 0.5
            qz = math.sin(half_theta)
            qw = math.cos(half_theta)
            cos_t = math.cos(theta)
            sin_t = math.sin(theta)

            for ox, oy in offsets:
                # Rotate local offset by group theta into map frame
                rx = cos_t * ox * intra_spacing_x - sin_t * oy * intra_spacing_y
                ry = sin_t * ox * intra_spacing_x + cos_t * oy * intra_spacing_y

                marker = Marker()
                marker.header.frame_id = frame_id
                marker.header.stamp = stamp
                marker.ns = marker_ns
                marker.id = object_id
                marker.type = Marker.CUBE
                marker.action = Marker.ADD

                marker.pose.position.x = cx + rx
                marker.pose.position.y = cy + ry
                marker.pose.position.z = 0.0
                marker.pose.orientation.z = qz
                marker.pose.orientation.w = qw

                marker.scale.x = size_x
                marker.scale.y = size_y
                marker.scale.z = size_z

                marker.color.r = 1.0
                marker.color.g = 0.45
                marker.color.b = 0.0
                marker.color.a = 0.85

                marker.lifetime.sec = 0

                marker_array.markers.append(marker)
                object_id += 1

        self.pub.publish(marker_array)
        self._published = True
        self.get_logger().info(
            f'Published {len(marker_array.markers)} custom object markers '
            f'({total_groups} groups x 4) on {self.pub.topic_name}'
        )


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
