import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import numpy as np

class MarkerTrackerNode(Node):
    def __init__(self):
        super().__init__('marker_tracker_node')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/overhead_camera/image',
            self.image_callback,
            qos_profile_sensor_data
        )

        #find right parameters
        self.margin_of_error = 50
        self.lower_bound = np.array([0, 120, 70])
        self.upper_bound = np.array([10, 255, 255])
        self.get_logger().info('MarkerTrackerNode started') 


    def image_callback(self, msg: Image):
        self.get_logger().info("Got image in callback")
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8') # OpenCV uses BGR, ROS does not, we need to convert
        h, w, _ = frame.shape
        self.get_logger().info(f"Picture size: {w}x{h}")
        hsv_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) # convert picture BGR → HSV

        mask = cv2.inRange(hsv_image, self.lower_bound, self.upper_bound)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) #RETR_EXTERNAL for external contours
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
        self.get_logger().info(f"Marker coordinates: x={center_x}, y={center_y}, area={marker_area:.2f}")


def main(args=None):
    rclpy.init(args=args)
    node = MarkerTrackerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
