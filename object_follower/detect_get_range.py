#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class DetectAndRange(Node):
    def __init__(self):
        super().__init__('detect_and_range')

        # Publishers
        self.publisher_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.publisher_image = self.create_publisher(Image, '/camera/image_raw', 10)

        # Subscribers
        self.scan_subscriber = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # Camera setup
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().info("Error: Could not open camera.")
            exit()

        self.bridge = CvBridge()

        # Color threshold (red object)
        self.lower_red = np.array([0, 120, 70])
        self.upper_red = np.array([10, 255, 255])

        # Initialize object distance
        self.object_distance = None

        # Timer for processing
        self.timer = self.create_timer(0.1, self.timer_callback)

    def scan_callback(self, msg):
        # Find the minimum distance in the LIDAR scan range (this is your object range)
        min_distance = min(msg.ranges)
        if min_distance < 35.0:  # Stopping distance is now set to 35 cm
            self.object_distance = min_distance
        else:
            self.object_distance = None

    def move_robot(self, centroid_x, frame_width):
        twist = Twist()

        # Control parameters
        angular_speed = 0.5
        center_threshold = 50

        # If object is to the left, rotate left
        if centroid_x < frame_width // 2 - center_threshold:
            twist.angular.z = angular_speed
        # If object is to the right, rotate right
        elif centroid_x > frame_width // 2 + center_threshold:
            twist.angular.z = -angular_speed
        else:
            twist.angular.z = 0.0  # Stop rotating when centered

        # Publish the twist message
        self.publisher_cmd_vel.publish(twist)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().info("Failed to capture frame")
            return

        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv_frame, self.lower_red, self.upper_red)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            contour = max(contours, key=cv2.contourArea)
            if cv2.contourArea(contour) > 500:
                (x, y), radius = cv2.minEnclosingCircle(contour)
                cx, cy = self.find_centroid(contour)

                cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 0), 2)
                cv2.circle(frame, (cx, cy), 5, (255, 0, 0), -1)

                self.move_robot(cx, frame.shape[1])

        # Convert to ROS2 image message and publish
        image_message = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.publisher_image.publish(image_message)

    def find_centroid(self, contour):
        M = cv2.moments(contour)
        if M['m00'] != 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
        else:
            cx, cy = 0, 0
        return cx, cy


def main(args=None):
    rclpy.init(args=args)
    detect_and_range = DetectAndRange()

    try:
        rclpy.spin(detect_and_range)
    except KeyboardInterrupt:
        pass
    finally:
        detect_and_range.cap.release()
        detect_and_range.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
