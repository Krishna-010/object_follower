#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ObjectTracker(Node):
    def __init__(self):
        super().__init__('object_tracker')

        # Create a publisher for the '/cmd_vel' topic and for the image stream
        self.publisher_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.publisher_image = self.create_publisher(Image, '/camera/image_raw', 10)

        # Define camera capture
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().info("Error: Could not open camera.")
            exit()

        # Define the color range for the object (e.g., red color in HSV)
        self.lower_red = np.array([0, 120, 70])
        self.upper_red = np.array([10, 255, 255])

        # Set up OpenCV to ROS2 image bridge
        self.bridge = CvBridge()

        # Timer to periodically process the camera frames
        self.timer = self.create_timer(0.1, self.timer_callback)

    def find_centroid(self, contour):
        M = cv2.moments(contour)
        if M['m00'] != 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
        else:
            cx, cy = 0, 0
        return cx, cy

    def move_robot(self, centroid_x, frame_width):
        twist = Twist()

        # Control parameters (adjustable)
        linear_speed = 0.2
        angular_speed = 0.5
        center_threshold = 50  # Pixels tolerance for being "centered"

        # Object is to the left, rotate left
        if centroid_x < frame_width // 2 - center_threshold:
            twist.angular.z = angular_speed
        # Object is to the right, rotate right
        elif centroid_x > frame_width // 2 + center_threshold:
            twist.angular.z = -angular_speed
        # Object is centered, move forward
        else:
            twist.linear.x = linear_speed

        # Publish movement command
        self.publisher_cmd_vel.publish(twist)

    def timer_callback(self):
        # Capture frame-by-frame
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().info("Failed to grab frame")
            return

        # Convert the frame to HSV color space
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Create a mask for red color
        mask = cv2.inRange(hsv_frame, self.lower_red, self.upper_red)

        # Perform morphological operations to remove small noises in the mask
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # If any object is detected
        if contours:
            # Find the largest contour (assuming it's the object we want to track)
            contour = max(contours, key=cv2.contourArea)

            # Ignore small contours (set a minimum size threshold)
            if cv2.contourArea(contour) > 500:
                # Draw a bounding circle around the detected object
                ((x, y), radius) = cv2.minEnclosingCircle(contour)

                # Calculate the centroid of the object
                cx, cy = self.find_centroid(contour)

                # Highlight the object and its centroid
                cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 0), 2)
                cv2.circle(frame, (cx, cy), 5, (255, 0, 0), -1)

                # Move the robot towards the object
                self.move_robot(cx, frame.shape[1])

        # Convert OpenCV frame to ROS2 Image and publish
        image_message = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.publisher_image.publish(image_message)

def main(args=None):
    rclpy.init(args=args)
    object_tracker = ObjectTracker()

    try:
        rclpy.spin(object_tracker)
    except KeyboardInterrupt:
        pass
    finally:
        object_tracker.cap.release()
        cv2.destroyAllWindows()
        object_tracker.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
