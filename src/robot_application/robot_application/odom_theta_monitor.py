import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32


class OdomThetaMonitor(Node):
    def __init__(self) -> None:
        super().__init__('odom_theta_monitor')

        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('theta_deg_topic', '/odom_theta_deg')
        self.declare_parameter('theta_rad_topic', '/odom_theta_rad')
        self.declare_parameter('log_hz', 5.0)

        odom_topic = str(self.get_parameter('odom_topic').value)
        theta_deg_topic = str(self.get_parameter('theta_deg_topic').value)
        theta_rad_topic = str(self.get_parameter('theta_rad_topic').value)
        log_hz = float(self.get_parameter('log_hz').value)

        self._last_theta_rad = None
        self._last_theta_deg = None
        self._last_quaternion = None

        self._theta_deg_pub = self.create_publisher(Float32, theta_deg_topic, 10)
        self._theta_rad_pub = self.create_publisher(Float32, theta_rad_topic, 10)
        self._odom_sub = self.create_subscription(Odometry, odom_topic, self._odom_cb, 10)

        if log_hz > 0.0:
            self._log_timer = self.create_timer(1.0 / log_hz, self._log_latest_theta)
        else:
            self._log_timer = None

        self.get_logger().info(
            f'Monitoring yaw from {odom_topic}; publishing deg on {theta_deg_topic} and rad on {theta_rad_topic}'
        )

    def _odom_cb(self, msg: Odometry) -> None:
        orientation = msg.pose.pose.orientation
        theta_rad = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )
        theta_deg = math.degrees(theta_rad)

        self._last_theta_rad = theta_rad
        self._last_theta_deg = theta_deg
        self._last_quaternion = orientation


        theta_deg_msg = Float32()
        theta_deg_msg.data = float(theta_deg)
        self._theta_deg_pub.publish(theta_deg_msg)

    def _log_latest_theta(self) -> None:
        if self._last_theta_rad is None or self._last_quaternion is None:
            return

        orientation = self._last_quaternion
        self.get_logger().info(
            'theta=%.3f rad (%.1f deg) q=[x=%.4f y=%.4f z=%.4f w=%.4f]'
            % (
                self._last_theta_rad,
                self._last_theta_deg,
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w,
            )
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OdomThetaMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()