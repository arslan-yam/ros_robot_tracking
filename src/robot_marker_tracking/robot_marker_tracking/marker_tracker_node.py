import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

import cv2
import numpy as np

class MarkerTrackerNode(Node):
    def __init__(self):
        super().__init__('marker_tracker_node')
        self.publisher = self.create_publisher(Path, '/robot_path', 10)  # publish path
        self.timer = self.create_timer(0.1, self.publish_path)  # publish every 0.1 sec
        self.path = Path()  # create path object for rviz
        self.path.header.frame_id = "map"
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/overhead_camera/image',
            self.image_callback,
            qos_profile_sensor_data
        )

        #find right parameters
        self.margin_of_error = 50
        self.lower_bound = np.array([0, 0, 0])
        self.upper_bound = np.array([180, 255, 70])

        pix_pts = np.array([
            [478.0, 399.0],  # (-2,-2)
            [160.0,  80.0],  # ( 2, 2)
            [160.0, 399.0],  # (-2, 2)
        ], dtype=np.float32)
        world_pts = np.array([
            [-2.0, -2.0],
            [ 2.0,  2.0],
            [-2.0,  2.0],
        ], dtype=np.float32)
        self.A_pix_to_world = cv2.getAffineTransform(pix_pts, world_pts)

        self.get_logger().info('MarkerTrackerNode started')

    def pixel_to_world(self, u: float, v: float) -> tuple[float, float]:
        A = self.A_pix_to_world
        X = float(A[0, 0] * u + A[0, 1] * v + A[0, 2])
        Y = float(A[1, 0] * u + A[1, 1] * v + A[1, 2])
        return X, Y
    
    def publish_path(self):
        self.publisher.publish(self.path)
        self.get_logger().info(f"Publishing path with {len(self.path.poses)} poses")

    def image_callback(self, msg: Image):
        self.get_logger().info("Got image in callback")
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8') # OpenCV uses BGR, ROS does not, we need to convert
        h, w, _ = frame.shape
        self.get_logger().info(f"Picture size: {w}x{h}")
        hsv_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  # convert picture BGR → HSV

        mask = cv2.inRange(hsv_image, self.lower_bound, self.upper_bound)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)  #RETR_EXTERNAL for external contours
        if not contours:
            self.get_logger().info("No marker was found")
            return
        
        marker = max(contours, key=cv2.contourArea)
        marker_area = cv2.contourArea(marker)
        if marker_area < self.margin_of_error:
            self.get_logger().info("Noise")
            return
        
        M = cv2.moments(marker)
        if M["m00"] == 0:
            return
        
        center_x = int(M["m10"] / M["m00"])
        center_y = int(M["m01"] / M["m00"])
        X, Y = self.pixel_to_world(center_x, center_y)

        # create pose for current path
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "map"
        pose.pose.position.x = X
        pose.pose.position.y = Y
        pose.pose.position.z = 0.0  # we flatten
        self.path.poses.append(pose)
        if len(self.path.poses) > 100000:  # last 100000 poses
            self.path.poses.pop(0)

        self.get_logger().info(
            f"Pixel(u,v)=({center_x:.1f},{center_y:.1f}) -> World(X,Y)=({X:.3f},{Y:.3f})"
        )

def main(args=None):
    rclpy.init(args=args)
    node = MarkerTrackerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
