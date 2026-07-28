"""Test utility to delete custom object markers by pick id."""

import argparse
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from visualization_msgs.msg import Marker, MarkerArray


class CustomObjectDeleteTest(Node):
    """Publishes a DELETE marker for a single pick-id."""

    def __init__(self, topic: str, marker_ns: str, frame_id: str):
        super().__init__('custom_object_delete_test')
        self._marker_ns = marker_ns
        self._frame_id = frame_id

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._pub = self.create_publisher(MarkerArray, topic, qos)

    @staticmethod
    def pick_id_to_marker_id(pick_id: int) -> int:
        """Map pick id (1..8) to its marker id (0..7)."""
        if pick_id < 1 or pick_id > 8:
            raise ValueError('pick_id must be in range [1, 8]')
        return pick_id - 1

    def publish_delete(self, marker_id: int):
        marker = Marker()
        marker.header.frame_id = self._frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = self._marker_ns
        marker.id = marker_id
        marker.action = Marker.DELETE

        marker_array = MarkerArray()
        marker_array.markers.append(marker)
        self._pub.publish(marker_array)


def parse_args():
    parser = argparse.ArgumentParser(
        description='Delete custom object marker by pick id (1..8).'
    )
    parser.add_argument('--pick-id', type=int, required=True, help='Pick id in [1..8]')
    parser.add_argument(
        '--delay-sec',
        type=float,
        default=0.2,
        help='Delay between republishes',
    )
    parser.add_argument(
        '--republish',
        type=int,
        default=1,
        help='Number of times to republish the DELETE message',
    )
    parser.add_argument('--topic', default='/custom_objects', help='Marker topic')
    parser.add_argument('--ns', default='custom_objects', help='Marker namespace')
    parser.add_argument('--frame-id', default='map', help='Marker frame id')
    return parser.parse_args()


def main(args=None):
    cli = parse_args()

    rclpy.init(args=args)
    node = CustomObjectDeleteTest(cli.topic, cli.ns, cli.frame_id)

    try:
        marker_id = node.pick_id_to_marker_id(int(cli.pick_id))
        node.get_logger().info(f'pick_id={cli.pick_id} -> marker_id={marker_id}')

        for i in range(max(1, cli.republish)):
            node.publish_delete(marker_id)
            node.get_logger().info(f'DELETE marker id={marker_id} (publish {i + 1}/{cli.republish})')
            time.sleep(0.5)
            rclpy.spin_once(node, timeout_sec=0.05)
            if i < cli.republish - 1:
                time.sleep(max(0.0, float(cli.delay_sec)))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
