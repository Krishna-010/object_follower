import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import logging
import time

class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0
        self.integral = 0

    def calculate(self, setpoint, measured_value):
        # Calculate error
        error = setpoint - measured_value
        
        # Proportional term
        P_out = self.Kp * error
        
        # Integral term
        self.integral += error
        I_out = self.Ki * self.integral
        
        # Derivative term
        derivative = error - self.prev_error
        D_out = self.Kd * derivative
        
        # Total output
        output = P_out + I_out + D_out
        
        # Save error for next loop
        self.prev_error = error
        
        return output

class ChaseObject(Node):
    def __init__(self):
        super().__init__('chase_object')

        # Logging for debugging
        logging.basicConfig(level=logging.DEBUG)

        # Declare and initialize variables
        self.object_distance = None

        # Set QoS to BEST_EFFORT to handle the LIDAR compatibility issue
        qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=rclpy.qos.QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=10
        )

        # Subscribe to the LaserScan data from LIDAR
        self.laser_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            qos_profile)

        # Create a publisher to control the robot's velocity
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # PID controller for linear velocity (distance to object)
        self.linear_pid = PIDController(Kp=0.5, Ki=0.0, Kd=0.1)  # Tune the PID gains as needed
        # PID controller for angular velocity (centering the object)
        self.angular_pid = PIDController(Kp=0.5, Ki=0.0, Kd=0.1)  # Tune these as needed

        # Set desired distance (50 cm)
        self.desired_distance = 0.5

        # Timer for control loop
        self.timer = self.create_timer(0.1, self.chase_object)

        logging.info("Chase node initialized")

    def laser_callback(self, msg):
        """Handles incoming LIDAR data."""
        try:
            logging.debug("Received LIDAR data")
            # Get the distance directly in front of the robot (assuming index 0 corresponds to the front)
            self.object_distance = msg.ranges[len(msg.ranges) // 2]
        except Exception as e:
            logging.error(f"Error processing LIDAR data: {e}")

    def chase_object(self):
        """Moves the robot toward the object using a PID controller."""
        try:
            if self.object_distance is not None:
                logging.info(f"Object distance: {self.object_distance:.2f} meters")

                # Create Twist message for movement
                twist = Twist()

                # Use PID to calculate linear velocity (based on distance)
                linear_velocity = self.linear_pid.calculate(self.desired_distance, self.object_distance)
                
                # Bound linear velocity to a maximum value
                twist.linear.x = min(max(linear_velocity, 0.0), 0.2)  # Move forward at calculated speed
                twist.angular.z = 0.0  # Default to no rotation
                
                logging.debug(f"Linear Velocity (PID controlled): {twist.linear.x}")

                if self.object_distance <= self.desired_distance:
                    # If the object is within 35 cm, stop the robot
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    logging.info("Object within stopping distance, stopping robot.")
                
                self.cmd_vel_publisher.publish(twist)
            else:
                logging.debug("No LIDAR data available to chase object")
        except Exception as e:
            logging.error(f"Error chasing object: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ChaseObject()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        logging.info("Node stopped by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
