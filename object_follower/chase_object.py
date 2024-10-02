import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class ChaseObject(Node):
    def __init__(self):
        super().__init__('chase_object')
        self.get_logger().info("Chase object node initialized")

        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub_object_info = self.create_subscription(
            Twist, '/object_info', self.object_info_callback, 10)

        # PID Controller Parameters
        self.kp = 0.5  # Proportional gain
        self.ki = 0.0  # Integral gain (not used here)
        self.kd = 0.1  # Derivative gain
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()

        self.target_distance = 0.35  # Desired distance in meters

    def object_info_callback(self, msg):
        try:
            self.get_logger().info("Object information received")
            distance_error = msg.linear.x - self.target_distance

            # PID control logic
            current_time = time.time()
            delta_time = current_time - self.last_time
            self.last_time = current_time

            p_term = self.kp * distance_error
            d_term = self.kd * (distance_error - self.prev_error) / delta_time
            self.prev_error = distance_error

            velocity = p_term + d_term

            cmd = Twist()
            cmd.linear.x = max(0.0, min(velocity, 0.22))  # Limiting max speed

            if abs(distance_error) < 0.05:
                cmd.linear.x = 0.0  # Stop when within 5 cm of target

            self.pub_cmd.publish(cmd)
            self.get_logger().info(f"Moving towards object with speed {cmd.linear.x}")

        except Exception as e:
            self.get_logger().error(f"Error in object_info_callback: {e}")

    def stop_robot(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.pub_cmd.publish(cmd)
        self.get_logger().info("Robot stopped.")

def main(args=None):
    rclpy.init(args=args)
    node = ChaseObject()

    try:
        rclpy.spin(node)
    except Exception as e:
        node.get_logger().error(f"Error: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
