import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ViewObject(Node):
    def __init__(self):
        super().__init__('view_object')
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image,
            'object_detection_image',
            self.image_callback,
            10)
        self.get_logger().info('ViewObject node initialized')

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Display the image
        cv2.imshow("Camera Feed", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = ViewObject()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down ViewObject node.')
    finally:
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
