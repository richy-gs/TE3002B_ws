import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class ImageSubscriberNode(Node):
    def __init__(self):
        super().__init__("image_subscriber_node")
        self.subscription = self.create_subscription(
            Image, "image", self.listener_callback, 10
        )
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            cv_image_rotada = cv2.rotate(cv_image, cv2.ROTATE_180)
            cv2.imshow("Image from publisher topic", cv_image_rotada)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriberNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
