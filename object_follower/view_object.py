import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from rclpy.qos import QoSProfile
import cv2
from cv_bridge import CvBridge

class ViewObjectNode(Node):
    def __init__(self):
        super().__init__('view_object')

        qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=rclpy.qos.QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=10
        )

        self.image_subscription = self.create_subscription(
            CompressedImage,
            '/object_image',
            self.image_callback,
            qos_profile
        )
        self.bridge = CvBridge()
        self.get_logger().info("View Object Node Initialized")

    def image_callback(self, msg):
        self.get_logger().info("Image received from TurtleBot")

        # Convert ROS CompressedImage message to OpenCV format
        cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")

        # Display the image
        cv2.imshow("TurtleBot Camera View", cv_image)

        # Wait for keypress (1 ms delay)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ViewObjectNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
