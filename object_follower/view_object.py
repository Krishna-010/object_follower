import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import logging

class CameraViewer(Node):
    def __init__(self):
        super().__init__('camera_viewer')
        self.get_logger().info('Camera Viewer node initialized')

        # Initialize the CvBridge
        self.bridge = CvBridge()

        # Set up subscriber for the compressed image
        self.image_subscriber = self.create_subscription(
            CompressedImage,
            '/camera/image/compressed',
            self.image_callback,
            10
        )

        self.get_logger().info('Subscribed to camera topic')

    def image_callback(self, msg):
        try:
            # Convert the compressed image to a cv2 format
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")

            # Display the image
            cv2.imshow("Camera Feed", cv_image)
            cv2.waitKey(1)  # Update the display window

            self.get_logger().debug('Image received and displayed')

        except Exception as e:
            self.get_logger().error(f"Error in image callback: {e}")

def main(args=None):
    rclpy.init(args=args)
    camera_viewer = CameraViewer()

    try:
        rclpy.spin(camera_viewer)
    except Exception as e:
        camera_viewer.get_logger().error(f"Error while running node: {e}")
    finally:
        camera_viewer.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()  # Close OpenCV windows

if __name__ == '__main__':
    main()
