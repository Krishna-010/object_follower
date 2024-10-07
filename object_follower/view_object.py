import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ViewObject(Node):
    def __init__(self):
        super().__init__('view_object')

        # Create a subscriber for the detected object image
        self.image_subscriber = self.create_subscription(Image, '/detected_object_image', self.image_callback, 10)

        # Initialize CV bridge
        self.bridge = CvBridge()

        self.get_logger().info("View Object node initialized")

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imshow("Detected Object", frame)
        cv2.waitKey(1)

    def stop(self):
        cv2.destroyAllWindows()

def main():
    rclpy.init()
    node = ViewObject()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
