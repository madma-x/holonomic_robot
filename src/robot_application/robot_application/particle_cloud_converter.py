#!/usr/bin/env python3
"""
Converts nav2_msgs/ParticleCloud to geometry_msgs/PoseArray for RViz visualization.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav2_msgs.msg import ParticleCloud
from geometry_msgs.msg import PoseArray, Pose
import math


class ParticleCloudConverter(Node):
    def __init__(self):
        super().__init__('particle_cloud_converter')
        
        # QoS profile matching AMCL's particle cloud publisher
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribe to ParticleCloud
        self.particle_sub = self.create_subscription(
            ParticleCloud,
            '/particle_cloud',
            self.particle_callback,
            qos_profile
        )
        
        # Publish as PoseArray
        self.pose_array_pub = self.create_publisher(
            PoseArray,
            '/particle_cloud_poses',
            10
        )
        
        self.get_logger().info('ParticleCloud converter started')
    
    def particle_callback(self, msg):
        """Convert ParticleCloud to PoseArray."""
        pose_array = PoseArray()
        pose_array.header = msg.header
        
        for particle in msg.particles:
            pose = Pose()
            # ParticleCloud already has full Pose structure
            pose.position = particle.pose.position
            pose.orientation = particle.pose.orientation
            
            pose_array.poses.append(pose)
        
        self.pose_array_pub.publish(pose_array)


def main(args=None):
    rclpy.init(args=args)
    node = ParticleCloudConverter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
