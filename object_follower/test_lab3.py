import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

class DetectObject(Node):
    def __init__(self):
        super().__init__('detect_object')
        self.cap = cv2.VideoCapture(0)  # Initialize camera (0 is default camera)
        self.bridge = CvBridge()  # Used to convert ROS image messages to OpenCV images
        self.image_pub = self.create_publisher(Image, 'object_detection_image', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # Timer to periodically check for the object
        self.get_logger().info('DetectObject node initialized')

    def timer_callback(self):
        # Capture frame from the camera
        ret, frame = self.cap.read()

        if not ret:
            self.get_logger().error('Error: Could not read from camera.')
            return

        # Check if object detection is successful (replace with actual detection logic)
        # For testing, we are just displaying the image
        detected_object = False  # Placeholder, set to True if object is detected
        if detected_object:
            # Draw rectangle around detected object (for example purposes)
            cv2.rectangle(frame, (100, 100), (200, 200), (0, 255, 0), 2)
            self.get_logger().info('Object detected, publishing image...')
        else:
            self.get_logger().info('No object detected, displaying camera feed...')

        # Display the camera feed (for testing purposes)
        cv2.imshow("Camera Feed", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.cap.release()
            cv2.destroyAllWindows()

        # Convert OpenCV image to ROS Image message and publish
        image_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        self.image_pub.publish(image_msg)

    def stop(self):
        # Release the camera when stopping the node
        self.cap.release()
        cv2.destroyAllWindows()

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
