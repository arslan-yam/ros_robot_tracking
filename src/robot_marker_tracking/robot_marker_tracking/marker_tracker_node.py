from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped

from . import config
from .transform import AffinePixelToWorld
from .vision import MarkerDetector
from .path_recorder import PathRecorder
from .logger import AppLogger
from .timer import PeriodicTimer


class MarkerTrackerNode(Node):
    def __init__(self):
        super().__init__(config.NODE_NAME)

        self.log = AppLogger(self, component="MARKER_TRACKER")  # logger wrapper

        self.publisher = self.create_publisher(
            type=__import__("nav_msgs.msg", fromlist=["Path"]).Path,  # avoids extra import line noise
            topic=config.PATH_TOPIC,
            qos_profile=10,
        )  # publish path

        self.latest_position = None

        self.timer = PeriodicTimer(self, config.PUBLISH_PERIOD_SEC, self.publish_path)  # publish every 1 sec

        self.path_recorder = PathRecorder(frame_id=config.FRAME_ID, max_len=config.MAX_PATH_LEN)

        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            config.IMAGE_TOPIC,
            self.image_callback,
            qos_profile_sensor_data,
        )

        # pixel -> world transform
        self.pix_to_world = AffinePixelToWorld(config.PIX_PTS, config.WORLD_PTS)

        # marker detector
        self.detector = MarkerDetector(config.HSV_RANGE, config.MARGIN_OF_ERROR_AREA)

        self.log.info("MarkerTrackerNode started")

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

        self.path_recorder.add_pose(pose)
        self.path_recorder.set_stamp_from_pose(pose)
        self.publisher.publish(self.path_recorder.path)

    def image_callback(self, msg: Image):
        self.log.info("Got image in callback")

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")  # OpenCV uses BGR, ROS does not, we need to convert
        h, w, _ = frame.shape
        self.log.info(f"Picture size: {w}x{h}")

        center = self.detector.detect(frame)
        if center is None:
            self.log.info("No marker was found")
            return

        world = self.pix_to_world.pixel_to_world(center.u, center.v)

        # create pose for current path
        self.latest_position = world

        self.log.info(
            f"Pixel(u,v)=({center.u:.1f},{center.v:.1f}) -> World(X,Y)=({world.x:.3f},{world.y:.3f})"
        )


def main(args=None):
    rclpy.init(args=args)
    node = MarkerTrackerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
