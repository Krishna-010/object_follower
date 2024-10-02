import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, LaserScan
from geometry_msgs.msg import Twist
import numpy as np
import cv2
from cv_bridge import CvBridge
import time

class DetectObjectAndGetRange(Node):
    def __init__(self):
        super().__init__('detect_and_range')

        self.get_logger().info("Node initialized: detect_and_range")

        # Set parameters
        self.declare_parameter('show_image_bool', True)
        self.declare_parameter('window_name', "Raw Image")
        self.target_distance = 0.35  # Desired distance (35 cm)

        self.bridge = CvBridge()

        # Publishers and subscribers
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)

        self.sub_img = self.create_subscription(
            CompressedImage,
            '/camera/image/compressed',
            self.image_callback,
            10
        )

        self.sub_scan = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.latest_scan = None
        self.get_logger().info("Waiting for scan data...")

    def image_callback(self, data):
        try:
            self.get_logger().info("Image received")
            np_arr = np.frombuffer(data.data, np.uint8)
            image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            # Object detection logic here (simplified for brevity)
            detected_object = self.detect_object(image)

            if detected_object:
                centroid_x = detected_object['centroid_x']
                self.get_logger().info(f"Object detected at {centroid_x} px in the image.")
                
                self.move_robot_to_face_object(centroid_x, image.shape[1])

            if self.get_parameter('show_image_bool').value:
                cv2.imshow(self.get_parameter('window_name').value, image)
                cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Error in image callback: {e}")

    def detect_object(self, image):
        # Simulated object detection logic (replace with actual detection)
        object_detected = {'centroid_x': image.shape[1] // 2}  # Simulate object in the center
        return object_detected

    def scan_callback(self, scan_data):
        try:
            self.get_logger().info("Laser scan data received")
            self.latest_scan = scan_data
        except Exception as e:
            self.get_logger().error(f"Error in scan callback: {e}")

    def move_robot_to_face_object(self, centroid_x, frame_width):
        try:
            cmd = Twist()
            turn_threshold = 20  # Pixels

            if abs(centroid_x - frame_width / 2) > turn_threshold:
                # If the object is not centered, rotate the robot
                cmd.angular.z = -0.2 if centroid_x > frame_width / 2 else 0.2
                self.get_logger().info(f"Rotating {'right' if cmd.angular.z < 0 else 'left'}")
            else:
                cmd.angular.z = 0.0
                self.get_logger().info("Object is centered.")

            self.pub_cmd.publish(cmd)
        except Exception as e:
            self.get_logger().error(f"Error in move_robot_to_face_object: {e}")

    def stop_robot(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.pub_cmd.publish(cmd)
        self.get_logger().info("Robot stopped.")

def main(args=None):
    rclpy.init(args=args)
    node = DetectObjectAndGetRange()

    try:
        rclpy.spin(node)
    except Exception as e:
        node.get_logger().error(f"Error: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
