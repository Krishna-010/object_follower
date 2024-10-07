import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from rclpy.qos import QoSProfile

class GetRange(Node):
    def __init__(self):
        super().__init__('get_range')

        # QoS profile
        qos_profile = QoSProfile(depth=10)

        # Publisher for distance and angle
        self.distance_publisher = self.create_publisher(Float32, '/distance', qos_profile)
        self.angle_publisher = self.create_publisher(Float32, '/angle', qos_profile)

        # Subscriber for LIDAR data
        self.lidar_subscriber = self.create_subscription(LaserScan, '/scan', self.lidar_callback, qos_profile)

        self.get_logger().info("Get Range node initialized")

    def lidar_callback(self, msg):
        min_distance = min(msg.ranges)
        angle_index = msg.ranges.index(min_distance)
        angle = msg.angle_min + angle_index * msg.angle_increment

        # Publish distance and angle
        self.publish_distance(min_distance)
        self.publish_angle(angle)

    def publish_distance(self, distance):
        distance_msg = Float32()
        distance_msg.data = distance
        self.distance_publisher.publish(distance_msg)

    def publish_angle(self, angle):
        angle_msg = Float32()
        angle_msg.data = angle
        self.angle_publisher.publish(angle_msg)

def main():
    rclpy.init()
    node = GetRange()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
