import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
import cv2
from cv_bridge import CvBridge

class MinimalVideoSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_video_subscriber')

        # Set Parameters
        self.declare_parameter('show_image_bool', True)
        self.declare_parameter('window_name', "Raw Image")

        self._display_image = bool(self.get_parameter('show_image_bool').value)
        self._titleOriginal = self.get_parameter('window_name').value

        if self._display_image:
            cv2.namedWindow(self._titleOriginal, cv2.WINDOW_AUTOSIZE)
            cv2.moveWindow(self._titleOriginal, 50, 50)

        image_qos_profile = QoSProfile(depth=5)
        image_qos_profile.history = QoSHistoryPolicy.KEEP_LAST
        image_qos_profile.durability = QoSDurabilityPolicy.VOLATILE
        image_qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT

        self._video_subscriber = self.create_subscription(
            CompressedImage,
            '/camera/image/compressed',
            self._image_callback,
            image_qos_profile)
        self._video_subscriber

    def _image_callback(self, compressed_image):
        self.get_logger().info("Received image")  # Debug log
        try:
            self._imgBGR = CvBridge().compressed_imgmsg_to_cv2(compressed_image, "bgr8")
            if self._display_image:
                self.show_image(self._imgBGR)
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def show_image(self, img):
        cv2.imshow(self._titleOriginal, img)
        cv2.waitKey(1)

def main():
    rclpy.init()
    video_subscriber = MinimalVideoSubscriber()

    try:
        rclpy.spin(video_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        if video_subscriber._display_image:
            cv2.destroyAllWindows()
        video_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
