import rclpy
from rclpy.node import Node


class MarkerTrackerNode(Node):
    def __init__(self):
        super().__init__('marker_tracker_node')
        self.get_logger().info('marker_tracker_node started')


def main(args=None):
    rclpy.init(args=args)
    node = MarkerTrackerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
