#!/usr/bin/env python2
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from mycobot_interfaces.msg import MycobotAngles


class Listener(Node):
    def __init__(self):
        super().__init__("real_listener_1")
        self.get_logger().info("start...")
        # init publisher.
        self.pub = self.create_publisher(
            msg_typ=JointState,
            topic="joint_states",
            qos_profile=10,
        )
        # init subscriber.
        self.sub = self.create_subscription(
            msg_type=MycobotAngles,
            topic="mycobot/angles_real",
            callback=self.callback,
        )

    def callback(self, data):
        """`mycobot/angles_real` subscriber callback method.

        Args:
            data (MycobotAngles): callback argument.
        """
        # ini publisher object.
        joint_state_send = JointState()
        joint_state_send.header = Header()

        joint_state_send.name = [
            "joint2_to_joint1",
            "joint3_to_joint2",
            "joint4_to_joint3",
            "joint5_to_joint4",
            "joint6_to_joint5",
            "joint6output_to_joint6",
        ]
        joint_state_send.velocity = [0.0,]
        joint_state_send.effort = []
        joint_state_send.header.stamp = self.get_clock().now().to_msg()

        # process callback data.
        radians_list = [
            data.joint_1 * (math.pi / 180),
            data.joint_2 * (math.pi / 180),
            data.joint_3 * (math.pi / 180),
            data.joint_4 * (math.pi / 180),
            data.joint_5 * (math.pi / 180),
            data.joint_6 * (math.pi / 180),
        ]
        self.get_logger().info("res: {}".format(radians_list))

        joint_state_send.position = radians_list
        self.pub.publish(joint_state_send)


def main(args=None):
    rclpy.init(args=args)
    listener = Listener()
    rclpy.spin(listener)
    listener.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
