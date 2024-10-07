import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile

class GetRangeNode(Node):
    def __init__(self):
        super().__init__('get_range')
        
        qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=rclpy.qos.QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=10
        )
        
        self.laser_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            qos_profile
        )
        self.distance = None
        self.get_logger().info("Get Range Node Initialized")

    def laser_callback(self, msg):
        # Assuming the object is at the center of the robot's view (angle = 0)
        mid_index = len(msg.ranges) // 2
        self.distance = msg.ranges[mid_index]
        
        if self.distance and not float('inf') in [self.distance]:
            self.get_logger().info(f"Distance to object: {self.distance:.2f} meters")

def main(args=None):
    rclpy.init(args=args)
    node = GetRangeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
