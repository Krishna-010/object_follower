import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile
from std_msgs.msg import Float32
import time

class ChaseObject(Node):
    def __init__(self):
        super().__init__('chase_object')

        # QoS profile
        qos_profile = QoSProfile(depth=10)

        # Publisher for velocity commands
        self.publisher = self.create_publisher(Twist, '/cmd_vel', qos_profile)

        # Subscribers for distance and angle
        self.distance_subscriber = self.create_subscription(Float32, '/distance', self.distance_callback, qos_profile)
        self.angle_subscriber = self.create_subscription(Float32, '/angle', self.angle_callback, qos_profile)

        # PID controller parameters
        self.Kp_linear = 0.5
        self.Ki_linear = 0.1
        self.Kd_linear = 0.1
        self.Kp_angular = 1.0
        self.Ki_angular = 0.1
        self.Kd_angular = 0.1

        # PID control variables
        self.prev_error_distance = 0.0
        self.integral_distance = 0.0
        self.prev_error_angle = 0.0
        self.integral_angle = 0.0

        self.get_logger().info("Chase Object node initialized")

        # Initialize distance and angle
        self.distance = 0.0
        self.angle = 0.0

    def distance_callback(self, msg):
        self.distance = msg.data

    def angle_callback(self, msg):
        self.angle = msg.data

    def chase(self):
        # Calculate linear and angular velocities using PID control
        error_distance = self.distance - 0.5  # Target distance is 50 cm
        self.integral_distance += error_distance
        derivative_distance = error_distance - self.prev_error_distance
        linear_velocity = (self.Kp_linear * error_distance) + (self.Ki_linear * self.integral_distance) + (self.Kd_linear * derivative_distance)
        self.prev_error_distance = error_distance

        error_angle = self.angle
        self.integral_angle += error_angle
        derivative_angle = error_angle - self.prev_error_angle
        angular_velocity = (self.Kp_angular * error_angle) + (self.Ki_angular * self.integral_angle) + (self.Kd_angular * derivative_angle)
        self.prev_error_angle = error_angle

        # Create the command message
        cmd = Twist()
        cmd.linear.x = max(0.0, linear_velocity)  # Prevent negative velocity
        cmd.angular.z = angular_velocity

        self.publisher.publish(cmd)
        self.get_logger().info(f"Moving towards object: Linear Velocity: {cmd.linear.x:.2f}, Angular Velocity: {cmd.angular.z:.2f}")

    def stop(self):
        # Emergency stop
        cmd = Twist()
        self.publisher.publish(cmd)
        self.get_logger().info("Emergency stop")

def main():
    rclpy.init()
    node = ChaseObject()
    try:
        while rclpy.ok():
            rclpy.spin_once(node)
            node.chase()
            time.sleep(0.1)
    except KeyboardInterrupt:
        node.stop()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
