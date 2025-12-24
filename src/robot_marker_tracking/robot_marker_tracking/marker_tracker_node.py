from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from . import config
from .transform import AffinePixelToWorld
from .vision import find_marker_center_pixel


class MarkerTrackerNode(Node):
    def __init__(self):
        super().__init__(config.NODE_NAME)

        self.publisher = self.create_publisher(Path, config.PATH_TOPIC, 10)  # publish path
        self.latest_position = None
        self.timer = self.create_timer(config.PUBLISH_PERIOD_SEC, self.publish_path)  # publish every 1 sec

        self.path = Path()  # create path object for rviz
        self.path.header.frame_id = config.FRAME_ID

        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            config.IMAGE_TOPIC,
            self.image_callback,
            qos_profile_sensor_data,
        )

        # pixel -> world transform
        self.pix_to_world = AffinePixelToWorld(config.PIX_PTS, config.WORLD_PTS)

        self.get_logger().info("MarkerTrackerNode started")

    def publish_path(self):
        if self.latest_position is None:
            return

        X, Y = self.latest_position.x, self.latest_position.y

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = config.FRAME_ID
        pose.pose.position.x = X
        pose.pose.position.y = Y
        pose.pose.position.z = 0.0

        self.path.poses.append(pose)

        if len(self.path.poses) > config.MAX_PATH_LEN:
            self.path.poses.pop(0)

        self.path.header.stamp = pose.header.stamp
        self.publisher.publish(self.path)

    def image_callback(self, msg: Image):
        self.get_logger().info("Got image in callback")

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")  # OpenCV uses BGR, ROS does not, we need to convert
        h, w, _ = frame.shape
        self.get_logger().info(f"Picture size: {w}x{h}")

        center = find_marker_center_pixel(
            frame_bgr=frame,
            hsv_range=config.HSV_RANGE,
            margin_of_error_area=config.MARGIN_OF_ERROR_AREA,
        )
        if center is None:
            self.get_logger().info("No marker was found")
            return

        world = self.pix_to_world.pixel_to_world(center.u, center.v)

        # create pose for current path
        self.latest_position = world
        if len(self.path.poses) > config.MAX_PATH_LEN:  # last 100000 poses
            self.path.poses.pop(0)

        self.get_logger().info(
            f"Pixel(u,v)=({center.u:.1f},{center.v:.1f}) -> World(X,Y)=({world.x:.3f},{world.y:.3f})"
        )


def main(args=None):
    rclpy.init(args=args)
    node = MarkerTrackerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
