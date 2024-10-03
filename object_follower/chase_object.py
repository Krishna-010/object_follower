import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.previous_error = 0
        self.integral = 0

    def calculate(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt
        self.previous_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative

class ChaseObject(Node):
    def __init__(self):
        super().__init__('chase_object')
        self.publisher_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
        self.distance = None

        self.linear_pid = PIDController(kp=1.0, ki=0.01, kd=0.1)
        self.angular_pid = PIDController(kp=0.5, ki=0.01, kd=0.1)
        self.get_logger().info("Chase node initialized with PID controllers")

    def laser_callback(self, msg):
        if len(msg.ranges) > 0:
            self.distance = min(msg.ranges)
            self.get_logger().info(f"Object is {self.distance:.2f} meters in front of the robot")

            twist = Twist()
            error = self.distance - 0.5  # Target distance is 50 cm

            # PID for linear velocity
            dt = 0.1  # time interval
            twist.linear.x = self.linear_pid.calculate(error, dt)
            if twist.linear.x > 0.5:
                twist.linear.x = 0.5  # Limit maximum speed

            # Add logic for angular velocity here if necessary
            twist.angular.z = 0.0

            self.publisher_cmd_vel.publish(twist)
            self.get_logger().info(f"Linear Velocity: {twist.linear.x:.2f}, Angular Velocity: {twist.angular.z:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = ChaseObject()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
