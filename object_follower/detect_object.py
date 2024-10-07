import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from rclpy.qos import QoSProfile

class DetectObject(Node):
    def __init__(self):
        super().__init__('detect_object')

        # QoS profile
        qos_profile = QoSProfile(depth=10)

        # Publisher for the detected object image
        self.image_publisher = self.create_publisher(Image, '/detected_object_image', qos_profile)
        self.cap = cv2.VideoCapture(0)  # Open the camera

        self.get_logger().info("Detect Object node initialized")

    def detect_object(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Failed to capture image")
            return

        # Convert to HSV and apply color thresholding (change as needed)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_color = np.array([0, 120, 70])  # Red
        upper_color = np.array([10, 255, 255])
        mask = cv2.inRange(hsv, lower_color, upper_color)
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(largest_contour)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

            # Publish the image with the bounding box
            self.publish_image(frame)

            self.get_logger().info("Object found, sending image.")
        else:
            self.get_logger().info("No object detected.")

    def publish_image(self, frame):
        bridge = CvBridge()
        msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.image_publisher.publish(msg)

    def stop(self):
        self.cap.release()
        cv2.destroyAllWindows()

def main():
    rclpy.init()
    node = DetectObject()
    try:
        while rclpy.ok():
            rclpy.spin_once(node)
            node.detect_object()
    except KeyboardInterrupt:
        node.stop()
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
