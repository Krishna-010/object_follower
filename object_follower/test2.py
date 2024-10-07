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
            10
        )
        self.frame_rate = 10  # Reduce the frame rate to limit lag
        self.timer = self.create_timer(1 / self.frame_rate, self.process_image)
        self.image = None
        self.get_logger().info('ViewObject node initialized')

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        self.image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def process_image(self):
        if self.image is not None:
            # Display the image in an OpenCV window
            cv2.imshow('Object Detection', self.image)

            # Add a delay and check for quit events
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.get_logger().info('Quitting...')
                rclpy.shutdown()
    
    def stop(self):
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = ViewObject()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down ViewObject node.')
    finally:
        node.stop()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
