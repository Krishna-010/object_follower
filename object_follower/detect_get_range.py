import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class DetectAndRange(Node):
    def __init__(self):
        super().__init__('detect_and_range')
        self.publisher_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription_scan = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
        self.subscription_image = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.bridge = CvBridge()
        self.laser_data = None
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Object detection params
        self.lower_red = np.array([0, 120, 70])
        self.upper_red = np.array([10, 255, 255])
        self.lidar_distance = None
        self.get_logger().info("Detect and Range Node Initialized")

    def image_callback(self, data):
        frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv_frame, self.lower_red, self.upper_red)

        # Detect the object
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest_contour) > 500:
                ((x, y), radius) = cv2.minEnclosingCircle(largest_contour)
                if radius > 10:
                    cx, cy = int(x), int(y)
                    cv2.circle(frame, (cx, cy), int(radius), (0, 255, 0), 2)
                    self.get_logger().info(f"Object Detected at Coordinates: ({cx}, {cy})")
        else:
            self.get_logger().info("No object detected")
    
    def laser_callback(self, msg):
        self.laser_data = msg.ranges
        if len(self.laser_data) > 0:
            self.lidar_distance = min(self.laser_data)  # Closest object distance
            self.get_logger().info(f"LIDAR Distance: {self.lidar_distance:.2f} meters")

    def timer_callback(self):
        if self.lidar_distance and self.lidar_distance <= 0.35:
            self.stop_robot()

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher_cmd_vel.publish(twist)
        self.get_logger().info("Robot stopped")

def main(args=None):
    rclpy.init(args=args)
    node = DetectAndRange()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
