#!/usr/bin/env python3
"""Odometry + localization bridge: republishes ground truth as /odom and
map -> odom, perfect (no noise).

Noise simulation was tried and pulled back out (kept causing more confusion
than value while the rest of the nav stack was being validated) - this just
copies /sim/true_odom through to /odom and odom -> base_footprint, exactly,
frame-renamed. Kept as a separate node (rather than publishing /odom directly
from sim_node) so the topic/TF interface matches the real robot's shape and a
real noisy odometry source can be dropped in later without touching anything
downstream.

map -> odom is identity: since /odom is a perfect copy of ground truth, map
and odom always coincide exactly. Publishing it here (instead of a separate
localization_sim node) keeps the TF chain to one extra hop instead of two,
which is what was dropping scan-synced RViz messages under load.

sub  /sim/true_odom   nav_msgs/Odometry   ground truth (map -> base_footprint)
pub  /odom            nav_msgs/Odometry   identical pose/twist, frame renamed
tf   odom -> base_footprint
tf   map -> odom = identity
"""

import math

import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


def _add_sec(stamp, extra_sec):
    t = stamp.sec + stamp.nanosec * 1e-9 + extra_sec
    sec = int(math.floor(t))
    return TimeMsg(sec=sec, nanosec=int((t - sec) * 1e9))


class OdomSimNode(Node):
    def __init__(self):
        super().__init__('odom_sim')
        dp = self.declare_parameter
        # sim_file kept as a parameter (unused) for launch-interface parity
        # with the rest of table_sim, in case noise ever comes back.
        dp('sim_file', '')
        dp('map_frame', 'map')
        dp('odom_frame', 'odom')
        dp('base_frame', 'base_footprint')
        dp('transform_tolerance_sec', 0.2)
        g = lambda n: self.get_parameter(n).value  # noqa: E731

        self.map_frame = g('map_frame')
        self.odom_frame = g('odom_frame')
        self.base_frame = g('base_frame')
        self.tf_tolerance = float(g('transform_tolerance_sec'))

        self.pub_odom = self.create_publisher(Odometry, '/odom', 10)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(Odometry, '/sim/true_odom', self._on_true, 10)
        self.get_logger().info('odom_sim ready: perfect odometry (no noise), map->odom identity')

    def _on_true(self, msg: Odometry):
        m = Odometry()
        m.header.stamp = msg.header.stamp
        m.header.frame_id = self.odom_frame
        m.child_frame_id = self.base_frame
        m.pose = msg.pose
        m.twist = msg.twist
        self.pub_odom.publish(m)

        # Future-date both TFs (but not the /odom message itself) so Nav2 can
        # request them at "now" without ever hitting extrapolation into the
        # future against our last broadcast.
        future = _add_sec(msg.header.stamp, self.tf_tolerance)

        t = TransformStamped()
        t.header.stamp = future
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        p = msg.pose.pose.position
        t.transform.translation.x, t.transform.translation.y = p.x, p.y
        t.transform.rotation = msg.pose.pose.orientation

        tm = TransformStamped()
        tm.header.stamp = future
        tm.header.frame_id = self.map_frame
        tm.child_frame_id = self.odom_frame
        tm.transform.rotation.w = 1.0

        self.tf.sendTransform([t, tm])


def main(args=None):
    rclpy.init(args=args)
    node = OdomSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
