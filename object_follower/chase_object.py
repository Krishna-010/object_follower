import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import time

class ChaseObject(Node):
    def __init__(self):
        super().__init__('chase_object')

        # Publisher for velocity commands
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber for object detection information (centroid)
        self.centroid_subscriber = self.create_subscription(
            Float32,
            '/object_centroid',
            self.centroid_callback,
            10
        )

        # PID control parameters
        self.kp_angular = 0.002   # Proportional gain
        self.ki_angular = 0.0001  # Integral gain
        self.kd_angular = 0.001   # Derivative gain

        self.prev_error_x = 0.0   # Previous error
        self.integral_x = 0.0     # Integral of error
        self.prev_time = time.time()

        self.target_distance = 0.2  # Target stopping distance (in meters)

        # Subscriber for distance to object
        self.distance_subscriber = self.create_subscription(
            Float32,
            '/object_distance',
            self.distance_callback,
            10
        )

        self.current_distance = None
        self.stop_flag = False

    def centroid_callback(self, msg):
        if self.stop_flag:
            return

        centroid_x = msg.data  # The X coordinate of the object centroid
        frame_width = 640  # Assuming a frame width of 640 pixels

        # PID control for angular velocity
        error_x = centroid_x - frame_width // 2  # The error in X-axis (object centering)
        current_time = time.time()
        dt = current_time - self.prev_time

        # Proportional term
        p_term = self.kp_angular * error_x

        # Integral term
        self.integral_x += error_x * dt
        i_term = self.ki_angular * self.integral_x

        # Derivative term
        derivative_x = (error_x - self.prev_error_x) / dt
        d_term = self.kd_angular * derivative_x

        # Total angular velocity (PID output)
        angular_z = -(p_term + i_term + d_term)

        # Update previous error and time
        self.prev_error_x = error_x
        self.prev_time = current_time

        # Create the Twist message to control the robot
        twist = Twist()

        # Apply only angular velocity (for rotation)
        twist.angular.z = angular_z

        # Publish the velocity command
        self.cmd_vel_publisher.publish(twist)

    def distance_callback(self, msg):
        self.current_distance = msg.data

        # Stop if the object is within the target distance
        if self.current_distance <= self.target_distance:
            self.stop_robot()

    def stop_robot(self):
        self.stop_flag = True

        # Publish zero velocities to stop the robot
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)

        self.get_logger().info('Robot stopped within target distance.')

def main(args=None):
    rclpy.init(args=args)
    chase_object = ChaseObject()

    try:
        rclpy.spin(chase_object)
    except KeyboardInterrupt:
        pass
    finally:
        chase_object.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
