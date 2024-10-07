import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from rclpy.qos import QoSProfile
import cv2
import numpy as np
from cv_bridge import CvBridge

class DetectObjectNode(Node):
    def __init__(self):
        super().__init__('detect_object')
        self.bridge = CvBridge()
        
        qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=rclpy.qos.QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=10
        )
        
        self.image_subscription = self.create_subscription(
            CompressedImage,
            '/camera/image/compressed',
            self.image_callback,
            qos_profile
        )
        self.image_publisher = self.create_publisher(CompressedImage, '/object_image', qos_profile)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.detected_object = False
        self.get_logger().info("Detect Object Node Initialized")

    def image_callback(self, msg):
        self.get_logger().info("Image received from camera")
        cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        
        # Simple object detection based on color (change as needed)
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        lower_red = np.array([0, 100, 100])
        upper_red = np.array([10, 255, 255])
        mask = cv2.inRange(hsv, lower_red, upper_red)
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            self.detected_object = True
            largest_contour = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(largest_contour)
            cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            self.get_logger().info(f"Object detected at coordinates: {x}, {y}")
        else:
            self.detected_object = False
            self.get_logger().info("No object detected")

        # Publish the image with the detected object outline
        compressed_image = self.bridge.cv2_to_compressed_imgmsg(cv_image)
        self.image_publisher.publish(compressed_image)

    def timer_callback(self):
        if not self.detected_object:
            self.get_logger().info("No object detected, rotating to find object...")

def main(args=None):
    rclpy.init(args=args)
    node = DetectObjectNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
