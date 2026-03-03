#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from tf_transformations import quaternion_from_euler


class WaypointTester(Node):
    def __init__(self):
        super().__init__('waypoint_tester')

        # list of waypoints (x, y, yaw in radians) in the "map" frame
        self.waypoints = [
            {'x': 1.0, 'y': 0.0, 'yaw': 0.0},
            {'x': 1.0, 'y': 1.0, 'yaw': 1.57},
            {'x': 0.0, 'y': 1.0, 'yaw': 3.14},
        ]
        self.current = 0

        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._client.wait_for_server()

        # start immediately
        self.send_next_goal()

    def send_next_goal(self):
        if self.current >= len(self.waypoints):
            self.get_logger().info('All waypoints completed, shutting down')
            rclpy.shutdown()
            return

        wp = self.waypoints[self.current]
        goal_msg = NavigateToPose.Goal()

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = wp['x']
        pose.pose.position.y = wp['y']
        q = quaternion_from_euler(0.0, 0.0, wp['yaw'])
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        goal_msg.pose = pose
        self.get_logger().info(f'sending waypoint {self.current}: {wp}')
        fut = self._client.send_goal_async(goal_msg)
        fut.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('goal rejected')
            self.current += 1
            self.send_next_goal()
            return
        self.get_logger().info('goal accepted')
        get_res = goal_handle.get_result_async()
        get_res.add_done_callback(self.result_callback)

    def result_callback(self, future):
        status = future.result().status
        self.get_logger().info(f'waypoint {self.current} finished with status {status}')
        self.current += 1
        self.send_next_goal()


def main(args=None):
    rclpy.init(args=args)
    node = WaypointTester()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
