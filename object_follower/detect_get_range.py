import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, LaserScan
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
import logging

class DetectAndRange(Node):
    def __init__(self):
        super().__init__('detect_and_range')

        # Logging for debugging
        logging.basicConfig(level=logging.DEBUG)

        # Declare and initialize the variables
        self.bridge = CvBridge()
        self.laser_ranges = None
        self.laser_angle_increment = None
        self.object_detected = False

        # Set up the subscribers
        self.image_subscriber = self.create_subscription(
            CompressedImage,
            '/camera/image/compressed',
            self.image_callback,
            10)

        # Set up QoS for LIDAR to handle reliability issue
        qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
            history=rclpy.qos.QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=10
        )
        self.laser_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            qos_profile)

        # Create a publisher for stopping the robot in case of emergency
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer to control how often to process the image and range
        self.timer = self.create_timer(0.5, self.process_data)

    def image_callback(self, msg):
        """Handles incoming image data from the camera."""
        try:
            logging.debug("Received an image")
            np_arr = np.frombuffer(msg.data, np.uint8)
            image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            self.detect_object(image)
        except Exception as e:
            logging.error(f"Error processing image: {e}")

    def laser_callback(self, msg):
        """Handles incoming LIDAR data."""
        try:
            logging.debug("Received LIDAR data")
            self.laser_ranges = np.array(msg.ranges)
            self.laser_angle_increment = msg.angle_increment
        except Exception as e:
            logging.error(f"Error processing LIDAR: {e}")

    def detect_object(self, image):
        """Detects the object from the image and draws a rectangle around it."""
        try:
            # Convert the image to grayscale
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            # Use a simple threshold to detect the object (or replace with actual model)
            _, thresh = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY)

            # Find contours to detect the object
            contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            # Assuming the largest contour is the object (this is a placeholder)
            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                x, y, w, h = cv2.boundingRect(largest_contour)

                # Draw a rectangle around the detected object
                cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                self.object_detected = True
                self.object_x_center = x + w // 2
            else:
                self.object_detected = False

            # Display the image with detection (for debugging)
            cv2.imshow("Object Detection", image)
            cv2.waitKey(1)
        except Exception as e:
            logging.error(f"Error detecting object: {e}")

    def process_data(self):
        """Processes the camera and LIDAR data to get the range to the detected object."""
        if self.object_detected and self.laser_ranges is not None:
            try:
                # Convert the object's X position to an angle
                frame_width = 640  # Assuming the camera frame width is 640 pixels
                fov = np.pi / 3  # 60 degree FOV for example
                object_angle = (self.object_x_center / frame_width) * fov - fov / 2

                # Use the angle to find the corresponding LIDAR data
                laser_index = int((object_angle + (fov / 2)) / self.laser_angle_increment)

                # Check the range from the LIDAR
                object_distance = self.laser_ranges[laser_index]
                logging.info(f"Object detected at {object_distance:.2f} meters")

                # Check if within stopping distance
                if object_distance < 0.5:  # 50 cm threshold
                    self.stop_robot()
            except Exception as e:
                logging.error(f"Error calculating distance to object: {e}")
        else:
            logging.debug("No object detected or no LIDAR data available")

    def stop_robot(self):
        """Publishes a zero-velocity Twist message to stop the robot."""
        try:
            logging.info("Stopping the robot")
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_publisher.publish(twist)
        except Exception as e:
            logging.error(f"Error stopping robot: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = DetectAndRange()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        logging.info("Node stopped by user")
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
