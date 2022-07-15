import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class Listener(Node):
    def __init__(self):
        super().__init__("real_listener_1")

        self.sub = self.create_subscription(
            msg_type=JointState,
            topic="joint_states",
            callback=self.callback,
            qos_profile=10
        )

    def callback(self, msg):
        angles_data = [int(i/math.pi*180) for i in list(msg.position)]
        self.get_logger().info(
            '\n\t angles: {}\n'.format(
                angles_data
            )
        )


def main(args=None):
    rclpy.init(args=args)

    listener = Listener()

    rclpy.spin(listener)

    listener.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
