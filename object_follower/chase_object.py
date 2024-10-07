import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile

class ChaseObjectNode(Node):
    def __init__(self):
        super().__init__('chase_object')
        
        qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=rclpy.qos.QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=10
        )

        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', qos_profile)
        self.timer = self.create_timer(0.1, self.timer_callback)

        # PID Controller variables
        self.kp = 1.0
        self.ki = 0.0
        self.kd = 0.5
        self.previous_error = 0.0
        self.integral = 0.0
        self.setpoint = 0.5  # Desired distance (in meters)

        self.current_distance = None
        self.get_logger().info("Chase Object Node Initialized")

    def update_pid(self, distance):
        error = self.setpoint - distance
        self.integral += error
        derivative = error - self.previous_error
        self.previous_error = error

        # PID output
        linear_velocity = self.kp * error + self.ki * self.integral + self.kd * derivative
        return linear_velocity

    def timer_callback(self):
        if self.current_distance:
            velocity = Twist()

            # Calculate velocities using PID control
            linear_velocity = self.update_pid(self.current_distance)

            if self.current_distance < 0.05:
                self.get_logger().info("Too close to object, stopping")
                velocity.linear.x = 0.0
                velocity.angular.z = 0.0
            else:
                velocity.linear.x = linear_velocity
                velocity.angular.z = 0.0  # Keep moving straight

            self.cmd_vel_publisher.publish(velocity)
            self.get_logger().info(f"Moving towards object, Distance: {self.current_distance}, Velocity: {velocity.linear.x}")
        else:
            self.get_logger().info("Waiting for distance data...")

def main(args=None):
    rclpy.init(args=args)
    node = ChaseObjectNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
