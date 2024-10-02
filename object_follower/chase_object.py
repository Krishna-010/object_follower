import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from time import time

class ChaseObject(Node):
    def __init__(self):
        super().__init__('chase_object')

        # Create publisher for /cmd_vel topic
        self.publisher_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)

        # Initialize PID variables for angular velocity (rotation)
        self.prev_error_angular = 0.0
        self.integral_angular = 0.0
        self.prev_time = time()

        # PID gains (these values will need to be tuned)
        self.Kp_angular = 0.4
        self.Ki_angular = 0.01
        self.Kd_angular = 0.2

        # PID variables for linear velocity (moving forward)
        self.prev_error_linear = 0.0
        self.integral_linear = 0.0

        self.Kp_linear = 0.5
        self.Ki_linear = 0.01
        self.Kd_linear = 0.2

        # Desired stopping distance
        self.desired_distance = 0.35  # 35cm

    def move_robot(self, centroid_x, frame_width, object_distance):
        twist = Twist()

        # Calculate error (object's x position relative to the center of the frame)
        error_angular = (frame_width // 2) - centroid_x
        current_time = time()
        dt = current_time - self.prev_time

        # Calculate proportional term
        p_term_angular = self.Kp_angular * error_angular

        # Calculate integral term
        self.integral_angular += error_angular * dt
        i_term_angular = self.Ki_angular * self.integral_angular

        # Calculate derivative term
        derivative_angular = (error_angular - self.prev_error_angular) / dt if dt > 0 else 0.0
        d_term_angular = self.Kd_angular * derivative_angular

        # Calculate total angular control output (rotation)
        angular_z = p_term_angular + i_term_angular + d_term_angular

        # Update previous error and time
        self.prev_error_angular = error_angular
        self.prev_time = current_time

        # Apply angular velocity (rotate the robot to center the object)
        twist.angular.z = angular_z

        # PID control for linear velocity (moving towards the object)
        error_linear = object_distance - self.desired_distance

        # Calculate proportional, integral, and derivative terms for linear movement
        p_term_linear = self.Kp_linear * error_linear
        self.integral_linear += error_linear * dt
        i_term_linear = self.Ki_linear * self.integral_linear
        derivative_linear = (error_linear - self.prev_error_linear) / dt if dt > 0 else 0.0
        d_term_linear = self.Kd_linear * derivative_linear

        # Calculate total linear control output
        linear_x = p_term_linear + i_term_linear + d_term_linear

        # Update previous error for linear velocity
        self.prev_error_linear = error_linear

        # If the object is within the desired distance, stop linear movement
        if object_distance <= self.desired_distance:
            linear_x = 0.0

        # Apply linear velocity
        twist.linear.x = linear_x

        # Publish the Twist message to move the robot
        self.publisher_cmd_vel.publish(twist)
