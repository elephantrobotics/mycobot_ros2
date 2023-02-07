import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class image_Subscription(Node):
    def __init__(self):
        super().__init__("image_subscription")

        self.bridge = CvBridge()

        self.image_sub = self.create_subscription(
            msg_type=Image,
            topic="camera/image",
            callback=self.img_callback,
            qos_profile=1
        )

    def img_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        cv2.imshow("sub window", cv_image)
        
        cv2.waitKey(10)


def main(args=None):
    rclpy.init(args=args)

    img_sub = image_Subscription()
    rclpy.spin(img_sub)

    img_sub.destroy_node()

    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
