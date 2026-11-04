#!/usr/bin/env python3
"""Click-to-drive helper for validating the simulator.

Click "2D Goal Pose" in RViz: the robot drives there in a straight line
(holonomic, with speed/accel limits) and turns to the clicked heading.
Publishes /cmd_vel only while a goal is active, so it can stay running
next to any other controller.

Current pose comes from a map -> base_footprint TF lookup, not /odom: /odom
is the noisy, drifting dead-reckoning estimate (odom_sim) and lives in its
own odom frame, which does not coincide with map - comparing it directly
against a goal clicked in RViz's map-frame "2D Goal Pose" would compute a
bogus error vector (frame mismatch), occasionally pointing straight at a
wall instead of the clicked point.
"""

import math

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from std_msgs.msg import Int32
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


def _yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class GoalDriver(Node):
    def __init__(self):
        super().__init__('goal_driver')
        self.declare_parameter('max_vel', 0.6)
        self.declare_parameter('max_acc', 1.0)
        self.declare_parameter('max_ang_vel', 2.5)
        self.declare_parameter('pos_tol', 0.01)
        self.declare_parameter('yaw_tol', 0.02)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.pose = None
        self.goal = None
        self.v = (0.0, 0.0)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(PoseStamped, '/goal_pose', self._goal, 10)
        self.create_subscription(Int32, '/sim/reset_event', self._on_reset, 10)
        self.dt = 0.1
        self.create_timer(self.dt, self._tick)
        self.get_logger().info('goal_driver: click "2D Goal Pose" in RViz to move the robot')

    def _on_reset(self, _msg):
        # The robot teleports back to its start pose on /sim/reset; drop any
        # in-progress goal and velocity ramp rather than trying to chase a
        # stale goal from a stale assumed velocity.
        self.goal = None
        self.v = (0.0, 0.0)

    def _update_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.get_parameter('map_frame').value,
                self.get_parameter('base_frame').value,
                rclpy.time.Time())
        except TransformException:
            return
        t = tf.transform.translation
        self.pose = (t.x, t.y, _yaw(tf.transform.rotation))

    def _goal(self, m):
        if m.header.frame_id not in ('map', 'odom', ''):
            self.get_logger().warn(f'goal in frame {m.header.frame_id}, expected map')
        self.goal = (m.pose.position.x, m.pose.position.y, _yaw(m.pose.orientation))
        # Start the accel ramp fresh for every new goal: self.v is an
        # internal model with no feedback from the robot's actual velocity,
        # so carrying it over from a previous (possibly unfinished, possibly
        # pre-/sim/reset) goal would apply acceleration limiting around a
        # stale assumption and make the robot oscillate/overshoot for real.
        self.v = (0.0, 0.0)
        self.get_logger().info('goal x=%.3f y=%.3f yaw=%.1f deg' % (self.goal[0], self.goal[1],
                                                                     math.degrees(self.goal[2])))

    def _tick(self):
        self._update_pose()
        if self.goal is None or self.pose is None:
            return
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        x, y, th = self.pose
        gx, gy, gth = self.goal
        dx, dy = gx - x, gy - y
        d = math.hypot(dx, dy)
        eth = _wrap(gth - th)
        if d < g('pos_tol') and abs(eth) < g('yaw_tol'):
            self.pub.publish(Twist())
            self.goal = None
            self.v = (0.0, 0.0)
            self.get_logger().info('goal reached')
            return
        v = min(g('max_vel'), math.sqrt(2 * g('max_acc') * 0.8 * d), 2.0 * d + 0.02)
        tvx, tvy = (dx / d * v, dy / d * v) if d > 1e-6 else (0.0, 0.0)
        # acceleration limit
        ax, ay = tvx - self.v[0], tvy - self.v[1]
        a = math.hypot(ax, ay)
        lim = g('max_acc') * self.dt
        if a > lim:
            ax, ay = ax * lim / a, ay * lim / a
        self.v = (self.v[0] + ax, self.v[1] + ay)
        c, s = math.cos(th), math.sin(th)
        cmd = Twist()
        cmd.linear.x = c * self.v[0] + s * self.v[1]       # world -> body frame
        cmd.linear.y = -s * self.v[0] + c * self.v[1]
        cmd.angular.z = max(-g('max_ang_vel'), min(g('max_ang_vel'), 3.0 * eth))
        self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    n = GoalDriver()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
