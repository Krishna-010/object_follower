import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

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

        # Convert BGR image to HSV
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define the range for the orange color in HSV
        lower_orange = np.array([10, 100, 20])
        upper_orange = np.array([25, 255, 255])

        # Threshold the HSV image to get only orange colors
        mask = cv2.inRange(hsv_frame, lower_orange, upper_orange)

        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            # Find the largest contour (which we assume is the ball)
            largest_contour = max(contours, key=cv2.contourArea)

            # Get bounding box coordinates around the object
            x, y, w, h = cv2.boundingRect(largest_contour)

            # Draw the bounding box
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

            # Optionally, you can find the center of the object
            object_center = (int(x + w / 2), int(y + h / 2))

            # Log that the object has been detected
            self.get_logger().info(f'Object detected at {object_center}')
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
