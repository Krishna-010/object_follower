import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2

class ObjectViewer(Node):
    def __init__(self):
        super().__init__('object_viewer')

        # Create subscribers for camera feed and object centroid with appropriate QoS settings
        qos_profile = rclpy.qos.QoSProfile(depth=10)
        qos_profile.reliability = rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT

        self.subscription_image = self.create_subscription(
            Image, '/camera/image_raw', self.camera_callback, qos_profile)
        self.subscription_centroid = self.create_subscription(
            Point, '/object_centroid', self.centroid_callback, qos_profile)

        self.bridge = CvBridge()
        self.centroid = None  # To store the received centroid coordinates

    def camera_callback(self, data):
        # Convert the ROS Image message to an OpenCV image
        frame = self.bridge.imgmsg_to_cv2(data, "bgr8")

        # If a centroid has been received, draw it on the frame
        if self.centroid:
            cx, cy = int(self.centroid.x), int(self.centroid.y)
            cv2.circle(frame, (cx, cy), 10, (0, 255, 0), -1)  # Draw a green circle at the centroid
            cv2.putText(frame, 'Object', (cx - 20, cy - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # Display the frame with a smaller delay (1 ms)
        cv2.imshow('Object Viewer', frame)
        cv2.waitKey(1)  # Minimal delay to ensure frame updates

    def centroid_callback(self, data):
        # Receive the centroid coordinates from the detect_object node
        self.centroid = data

    def stop_viewing(self):
        # Properly close all OpenCV windows when stopping
        cv2.destroyAllWindows()
        self.get_logger().info('Stopping Object Viewer.')

def main(args=None):
    rclpy.init(args=args)
    viewer = ObjectViewer()

    try:
        rclpy.spin(viewer)
    except KeyboardInterrupt:
        viewer.stop_viewing()
    finally:
        viewer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
