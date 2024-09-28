import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point, Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')

        self.publisher_centroid = self.create_publisher(Point, '/object_centroid', 10)
        self.publisher_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.camera_callback, 10)

    def camera_callback(self, data):
        frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower_red = np.array([0, 120, 70])
        upper_red = np.array([10, 255, 255])
        mask = cv2.inRange(hsv_frame, lower_red, upper_red)

        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest_contour) > 500:
                cx, cy = self.find_centroid(largest_contour)
                self.get_logger().info(f'Centroid at: ({cx}, {cy})')

                point = Point()
                point.x = cx
                point.y = cy
                point.z = 0  # Not using z, but it's needed for Point message
                self.publisher_centroid.publish(point)

    def find_centroid(self, contour):
        M = cv2.moments(contour)
        if M['m00'] != 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
        else:
            cx, cy = 0, 0
        return cx, cy

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = 0
        self.publisher_cmd_vel.publish(twist)
        self.get_logger().info('Emergency stop initiated: Robot stopped.')

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_robot()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
