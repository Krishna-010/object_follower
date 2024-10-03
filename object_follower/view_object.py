import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class ViewObject(Node):
    def __init__(self):
        super().__init__('view_object')
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.bridge = CvBridge()
        self.get_logger().info("View Object Node Initialized")
    
    def image_callback(self, data):
        frame = self.bridge.imgmsg_to_cv2(data, "bgr8")

        # You can add object detection here if needed
        cv2.imshow("Robot Camera", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ViewObject()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
