#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from cv_bridge import CvBridge
import cv2
import numpy as np

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')
        
        # Subscription to the camera image topic
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',  # Adjust if your camera topic is different
            self.image_callback,
            10)
        
        # Publisher to indicate if any object is detected
        self.publisher_ = self.create_publisher(Bool, '/object_detected', 10)
        self.color_publisher_ = self.create_publisher(String, '/color_detected', 10)
        
        # Initialize CvBridge
        self.bridge = CvBridge()
        
        self.get_logger().info('Object Detection Node has been started.')

        # Parameters for circularity and contour area thresholds
        self.declare_parameter('circularity_threshold', 0.7)  # Adjust as needed
        self.declare_parameter('min_contour_area', 500)       # Adjust as needed

        # Retrieve parameter values
        self.circularity_threshold = self.get_parameter('circularity_threshold').value
        self.min_contour_area = self.get_parameter('min_contour_area').value

        # Track the last detected color and detection status
        self.last_detected_color = None

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')
            return

        # Convert to HSV color space
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Define color ranges for red, blue, and green
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([180, 255, 255])
        lower_blue = np.array([100, 150, 0])
        upper_blue = np.array([140, 255, 255])
        lower_green = np.array([35, 100, 100])
        upper_green = np.array([85, 255, 255])

        # Create masks for red, blue, and green
        mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)
        mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
        mask_green = cv2.inRange(hsv, lower_green, upper_green)

        # Apply morphological operations to reduce noise
        kernel = np.ones((5, 5), np.uint8)
        mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)
        mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_DILATE, kernel)

        mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_OPEN, kernel)
        mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_DILATE, kernel)

        mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_OPEN, kernel)
        mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_DILATE, kernel)

        # Combine all masks to detect any target color
        combined_mask = cv2.bitwise_or(cv2.bitwise_or(mask_red, mask_blue), mask_green)

        # Find contours in the combined mask
        contours, _ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Initialize detection flag
        detected = False
        color_found = None

        # Iterate through each contour to check for circularity and color
        for contour in contours:
            area = cv2.contourArea(contour)
            perimeter = cv2.arcLength(contour, True)
            if perimeter == 0:
                continue  # Avoid division by zero

            # Calculate circularity
            circularity = 4 * np.pi * (area / (perimeter * perimeter))

            # Debugging: Log circularity and area at DEBUG level
            self.get_logger().debug(f'Contour Area: {area}, Perimeter: {perimeter}, Circularity: {circularity}')

            # Check if contour meets both circularity and area thresholds
            if circularity > self.circularity_threshold and area > self.min_contour_area:
                # Create a mask for the current contour
                mask_contour = np.zeros_like(combined_mask)
                cv2.drawContours(mask_contour, [contour], -1, 255, -1)

                # Check which color mask has the most overlap with the contour
                overlap_red = cv2.countNonZero(cv2.bitwise_and(mask_red, mask_contour))
                overlap_blue = cv2.countNonZero(cv2.bitwise_and(mask_blue, mask_contour))
                overlap_green = cv2.countNonZero(cv2.bitwise_and(mask_green, mask_contour))

                # Determine which color has the maximum overlap
                max_overlap = max(overlap_red, overlap_blue, overlap_green)

                if max_overlap == overlap_red and overlap_red > 0:
                    color_found = 'red'
                elif max_overlap == overlap_blue and overlap_blue > 0:
                    color_found = 'blue'
                elif max_overlap == overlap_green and overlap_green > 0:
                    color_found = 'green'

                if color_found:
                    detected = True
                    self.get_logger().info(f'{color_found} ball found.')
                
                break  # Detect only the first valid spherical object

        # Publish detection result
        if detected and color_found != self.last_detected_color:
            self.publisher_.publish(Bool(data=True))
            self.color_publisher_.publish(String(data=color_found))
            self.last_detected_color = color_found  # Update last detected color

        elif not detected:
            self.publisher_.publish(Bool(data=False))
            self.last_detected_color = None  # Reset color when nothing is detected

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
