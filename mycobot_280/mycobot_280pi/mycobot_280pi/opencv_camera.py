import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class Image_Publisher(Node):
    def __init__(self):
        super().__init__("image_publisher")

        # Declaring launch parameters
        self.declare_parameter('num', "0")

        # Get the parameters in the launch file
        self.cap_num = self.get_parameter(
            'num').get_parameter_value().integer_value

        self.bridge = CvBridge()

        self.image_pub = self.create_publisher(
            msg_type=Image,
            topic="camera/image",
            qos_profile=1
        )
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.i = 1

    def timer_callback(self):
        # open camera
        cap = cv2.VideoCapture(self.cap_num)
        if not cap.isOpened():
            cap.open(1)
        if self.i == 1:
            self.get_logger().info("{} The camera was successfully turned on!".format(self.cap_num))

        _, frame = cap.read()

        self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, 'bgr8'))
        self.get_logger().info("The {}th data release...".format(self.i))
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    img_pub = Image_Publisher()
    rclpy.spin(img_pub)

    img_pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
