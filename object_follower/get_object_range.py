import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, Twist

class ObjectRange(Node):
    def __init__(self):
        super().__init__('object_range')
        self.subscription = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.publisher_distance = self.create_publisher(Point, '/object_distance', 10)
        self.publisher_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)

    def lidar_callback(self, scan):
        distance = min(scan.ranges)  # Get the minimum range from LIDAR scan
        self.get_logger().info(f'Object distance: {distance:.2f} meters')

        # Publish the distance
        point = Point()
        point.x = distance  # Using x to represent distance
        point.y = 0
        point.z = 0
        self.publisher_distance.publish(point)

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = 0
        self.publisher_cmd_vel.publish(twist)
        self.get_logger().info('Emergency stop initiated: Robot stopped.')

def main(args=None):
    rclpy.init(args=args)
    node = ObjectRange()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_robot()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
