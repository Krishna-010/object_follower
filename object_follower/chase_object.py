import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np
import logging

class ChaseObject(Node):
    def __init__(self):
        super().__init__('chase_object')
        self.get_logger().info('Chase node initialized')

        # Initialize the publisher to cmd_vel
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('Initialized cmd_vel publisher')

        # Adjust the QoS settings for the LaserScan subscriber
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT  # Ensure reliable communication

        # Initialize the subscriber to LaserScan
        self.laser_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            qos_profile
        )
        self.get_logger().info('Initialized laser subscriber')

        # PID parameters
        self.kp = 0.5  # Proportional gain
        self.ki = 0.1  # Integral gain
        self.kd = 0.05  # Derivative gain
        self.prev_error = 0.0
        self.integral = 0.0

        # Target distance from the object (in meters)
        self.target_distance = 0.5  # 35 cm
        self.max_speed = 0.5  # Maximum linear speed

    def laser_callback(self, msg):
        # Get the distance to the object
        distances = np.array(msg.ranges)
        min_distance = np.min(distances)  # Minimum distance detected

        # Calculate PID control variables
        error = self.target_distance - min_distance  # Calculate error
        self.integral += error  # Update integral
        derivative = error - self.prev_error  # Calculate derivative
        self.prev_error = error  # Update previous error

        # Calculate control output
        linear_speed = self.kp * error + self.ki * self.integral + self.kd * derivative

        # Ensure linear speed does not exceed max speed
        linear_speed = max(min(linear_speed, self.max_speed), 0.0)

        # Create and publish the Twist message
        twist = Twist()
        twist.linear.x = linear_speed
        twist.angular.z = 0.0  # No rotation for this example

        self.cmd_vel_publisher.publish(twist)
        self.get_logger().info(f'Publishing: Linear speed = {linear_speed}, Distance to object = {min_distance:.2f} m')

def main(args=None):
    rclpy.init(args=args)
    chase_object = ChaseObject()

    try:
        rclpy.spin(chase_object)
    except Exception as e:
        chase_object.get_logger().error(f'Error while running node: {e}')
    finally:
        chase_object.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
