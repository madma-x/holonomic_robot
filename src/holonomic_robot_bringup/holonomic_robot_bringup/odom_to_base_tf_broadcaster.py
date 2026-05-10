#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.time import Time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomToBaseTfBroadcaster(Node):
    def __init__(self) -> None:
        super().__init__('odom_to_base_tf_broadcaster')

        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('parent_frame', 'odom')
        self.declare_parameter('child_frame', 'base_footprint')
        self.declare_parameter('use_odom_stamp', False)
        self.declare_parameter('max_odom_lag_sec', 0.5)
        self.declare_parameter('max_odom_lead_sec', 0.1)

        odom_topic = self.get_parameter('odom_topic').value
        self.parent_frame = self.get_parameter('parent_frame').value
        self.child_frame = self.get_parameter('child_frame').value
        self.use_odom_stamp = bool(self.get_parameter('use_odom_stamp').value)
        self.max_odom_lag_sec = float(self.get_parameter('max_odom_lag_sec').value)
        self.max_odom_lead_sec = float(self.get_parameter('max_odom_lead_sec').value)
        self._last_skew_warn_ns = 0

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_sub = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            qos,
        )

        self.get_logger().info(
            'Broadcasting TF from odometry: '
            f'parent={self.parent_frame}, child={self.child_frame}, topic={odom_topic}, '
            f'use_odom_stamp={self.use_odom_stamp}'
        )

    def odom_callback(self, msg: Odometry) -> None:
        t = TransformStamped()
        now = self.get_clock().now()
        if self.use_odom_stamp:
            odom_time = Time.from_msg(msg.header.stamp)
            skew_sec = (now - odom_time).nanoseconds / 1e9
            if -self.max_odom_lead_sec <= skew_sec <= self.max_odom_lag_sec:
                t.header.stamp = msg.header.stamp
            else:
                # Guard against stale/future timestamps from external odometry clocks.
                now_ns = now.nanoseconds
                if now_ns - self._last_skew_warn_ns > 2_000_000_000:
                    self.get_logger().warning(
                        'Ignoring odom stamp due to clock skew '
                        f'({skew_sec:.3f}s); using local ROS time for TF.'
                    )
                    self._last_skew_warn_ns = now_ns
                t.header.stamp = now.to_msg()
        else:
            t.header.stamp = now.to_msg()
        t.header.frame_id = self.parent_frame
        t.child_frame_id = self.child_frame

        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OdomToBaseTfBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
