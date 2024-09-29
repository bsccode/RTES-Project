#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import numpy as np

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',  # Adjust if your camera topic is different
            self.image_callback,
            10)
        self.publisher_ = self.create_publisher(Bool, '/object_detected', 10)
        self.bridge = CvBridge()
        self.get_logger().info('Object Detection Node has been started.')

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # Convert to HSV color space
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Define color ranges for red and blue
        # Red color ranges
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([180, 255, 255])
        # Blue color range
        lower_blue = np.array([100, 150, 0])
        upper_blue = np.array([140, 255, 255])

        # Create masks
        mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)
        mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

        # Combine masks
        mask = cv2.bitwise_or(mask_red, mask_blue)

        # Check if the object is detected
        if cv2.countNonZero(mask) > 1000:  # Adjust threshold as needed
            self.publisher_.publish(Bool(data=True))
            self.get_logger().info('Target object detected.')
        else:
            self.publisher_.publish(Bool(data=False))

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Object Detection Node stopped cleanly.')
    except Exception as e:
        node.get_logger().error(f'Exception in Object Detection Node: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
