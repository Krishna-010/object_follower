import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
import time

class ObjectChaser(Node):
    def __init__(self):
        super().__init__('object_chaser')

        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.centroid_subscription = self.create_subscription(
            Point, '/object_centroid', self.centroid_callback, 10)
        self.distance_subscription = self.create_subscription(
            Point, '/object_distance', self.distance_callback, 10)

        self.object_centroid = None
        self.object_distance = float('inf')
        self.target_distance = 0.2  # 20 cm
        self.start_time = time.time()
        self.time_limit = 5  # 5 second time limit

    def centroid_callback(self, centroid):
        self.object_centroid = (centroid.x, centroid.y)
        self.get_logger().info(f'Object centroid received: {self.object_centroid}')

    def distance_callback(self, distance):
        self.object_distance = distance.x  # We are using the x field to represent distance
        self.get_logger().info(f'Object distance received: {self.object_distance:.2f} meters')

        if self.object_centroid and self.object_distance:
            self.move_towards_object()

        if time.time() - self.start_time > self.time_limit:
            self.get_logger().info("Time limit reached, stopping.")
            self.stop_robot()

        if self.object_distance <= self.target_distance:
            self.get_logger().info("Reached target distance, stopping.")
            self.stop_robot()

    def move_towards_object(self):
        twist = Twist()

        frame_width = 640  # Assume a fixed frame width
        center_threshold = 50  # Pixel threshold for centering

        # Rotate to center on the object
        if self.object_centroid[0] < frame_width // 2 - center_threshold:
            twist.angular.z = 0.5
        elif self.object_centroid[0] > frame_width // 2 + center_threshold:
            twist.angular.z = -0.5
        else:
            twist.angular.z = 0  # Stop rotating when centered

        # Move forward if the object is far
        if self.object_distance > self.target_distance:
            twist.linear.x = 0.2
        else:
            twist.linear.x = 0

        # Publish the movement command
        self.cmd_vel_publisher.publish(twist)

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = 0
        self.cmd_vel_publisher.publish(twist)
        self.get_logger().info('Emergency stop initiated: Robot stopped.')

def main(args=None):
    rclpy.init(args=args)
    node = ObjectChaser()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_robot()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
