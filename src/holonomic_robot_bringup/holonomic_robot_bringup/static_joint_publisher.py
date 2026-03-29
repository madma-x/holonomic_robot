#!/usr/bin/env python3
"""
Static joint state publisher for wheel joints.
Publishes zero positions for all wheel joints.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class StaticJointPublisher(Node):
    def __init__(self):
        super().__init__('static_joint_publisher')
        
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.1, self.publish_joint_states)
        
        self.get_logger().info('Static joint publisher started')
    
    def publish_joint_states(self):
        """Publish static joint states for wheels."""
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = [
            'front_wheel_joint',
            'left_rear_wheel_joint',
            'right_rear_wheel_joint'
        ]
        joint_state.position = [0.0, 0.0, 0.0]
        joint_state.velocity = []
        joint_state.effort = []
        
        self.joint_pub.publish(joint_state)


def main(args=None):
    rclpy.init(args=args)
    node = StaticJointPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
