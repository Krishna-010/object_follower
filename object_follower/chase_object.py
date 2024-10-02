import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import logging

class ChaseObject(Node):
    def __init__(self):
        super().__init__('chase_object')
        self.get_logger().info('Chase node initialized')

        # Setup QoS for LaserScan subscriber
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT  # Change to BEST_EFFORT to match typical LIDAR publishers

        # Set up subscriber and publisher
        self.laser_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            qos_profile)

        self.cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Control parameters
        self.linear_speed = 0.2
        self.angular_speed = 0.5
        self.stop_distance = 0.35  # Stop when within 35 cm of object

        self.get_logger().info('Initialized laser subscriber and cmd_vel publisher')

    def laser_callback(self, msg):
        try:
            self.get_logger().debug("Laser callback triggered")
            # Extract the distance data from the LIDAR message
            ranges = msg.ranges
            if len(ranges) == 0:
                self.get_logger().warn("LIDAR data is empty!")
                return

            # Find the distance directly in front of the robot
            front_distance = ranges[len(ranges) // 2]

            # Log the detected distance for debugging
            self.get_logger().debug(f"Front distance: {front_distance}")

            # Define Twist message to move the robot
            twist = Twist()

            # If the object is further than the stop distance, move forward
            if front_distance > self.stop_distance:
                twist.linear.x = self.linear_speed
                self.get_logger().info(f"Moving forward, distance: {front_distance}")
            else:
                # Stop the robot
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.get_logger().info(f"Stopping as object is within {self.stop_distance}m")

            # Publish the Twist command to control the robot
            self.cmd_publisher.publish(twist)
            self.get_logger().debug("Published Twist message")

        except Exception as e:
            self.get_logger().error(f"Error in laser callback: {e}")

def main(args=None):
    rclpy.init(args=args)
    chase_node = ChaseObject()

    try:
        rclpy.spin(chase_node)
    except Exception as e:
        chase_node.get_logger().error(f"Error while running node: {e}")
    finally:
        chase_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
