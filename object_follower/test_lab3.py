import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class DetectObject(Node):
    def __init__(self):
        super().__init__('detect_object')
        self.cap = cv2.VideoCapture(0)  # Initialize camera
        self.bridge = CvBridge()  # Convert between ROS and OpenCV
        self.image_pub = self.create_publisher(Image, 'object_detection_image', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('DetectObject node initialized')

    def timer_callback(self):
        ret, frame = self.cap.read()

        if not ret:
            self.get_logger().error('Error: Could not read from camera.')
            return

        # Placeholder object detection logic (to be replaced)
        detected_object = False

        if detected_object:
            cv2.rectangle(frame, (100, 100), (200, 200), (0, 255, 0), 2)
            self.get_logger().info('Object detected, publishing image...')
        else:
            self.get_logger().info('No object detected, publishing camera feed...')

        # Convert OpenCV image to ROS Image message
        image_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        self.image_pub.publish(image_msg)

    def stop(self):
        self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    node = DetectObject()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down DetectObject node.')
    finally:
        node.stop()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
