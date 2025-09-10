#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS2 node for synchronizing RViz joint states with a MyCobot Pro450 robot.

This module subscribes to the "joint_states" topic from RViz, converts
the received joint positions from radians to degrees, and sends them
to the MyCobot Pro450 robotic arm. It ensures that the pymycobot
library version meets the minimum requirement.

Author: weijian.wang
Date: 2025-09-10
"""
import time
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import pymycobot
from packaging import version

# Minimum required pymycobot library version
MIN_REQUIRE_VERSION = '4.0.0'

CURRENT_VERSION = pymycobot.__version__
print(f'current pymycobot library version: {CURRENT_VERSION}')
if version.parse(CURRENT_VERSION) < version.parse(MIN_REQUIRE_VERSION):
    raise RuntimeError(
        f'The version of pymycobot library must be greater than {MIN_REQUIRE_VERSION} or higher. '
        'The current version is {CURRENT_VERSION}. Please upgrade the library version.')
else:
    print('pymycobot library version meets the requirements!')
    from pymycobot import Pro450Client


class SliderSubscriber(Node):
    """ROS2 node for subscribing to joint states and controlling MyCobot Pro450.

    This node listens to the "joint_states" topic, processes the data into
    degrees according to RViz order, and sends the joint angles to the robot.

    Attributes:
        subscription (rclpy.subscription.Subscription): Subscription to the
            "joint_states" topic.
        mycobot_450 (Pro450Client): Client interface to control the Pro450 robot.
        rviz_order (list[str]): List of joint names in the expected RViz order.
    """

    def __init__(self):
        """Initialize the Slider_Subscriber node and set up connections."""
        super().__init__("control_sync_plan")
        self.subscription = self.create_subscription(
            JointState,
            "joint_states",
            self.listener_callback,
            10
        )

        # Declare robot connection parameters
        self.declare_parameter('ip', '192.168.0.32')
        self.declare_parameter('port', 450)

        ip = self.get_parameter("ip").get_parameter_value().string_value
        port = self.get_parameter("port").get_parameter_value().integer_value

        self.get_logger().info(f"ip:{ip}, port:{port}")
        self.mycobot_450 = Pro450Client(ip, port)
        time.sleep(0.05)
        if self.mycobot_450.get_fresh_mode() != 1:
            self.mycobot_450.set_fresh_mode(1)
        time.sleep(0.05)

        # Joint order in RViz
        self.rviz_order = ['joint1', 'joint2',
                           'joint3', 'joint4', 'joint5', 'joint6']

    def listener_callback(self, msg):
        """Callback to process received joint states.

        Converts joint positions from radians to degrees, rearranges them
        according to the RViz order, and sends them to the robot.

        Args:
            msg (JointState): The message containing joint names and positions.
        """
        # Create a mapping of joint names to their position values
        joint_state_dict = {name: msg.position[i]
                            for i, name in enumerate(msg.name)}

        # Rearrange joint angles according to RViz order
        data_list = []
        for joint in self.rviz_order:
            if joint in joint_state_dict:
                radians_to_angles = round(
                    math.degrees(joint_state_dict[joint]), 2)
                data_list.append(radians_to_angles)

        self.get_logger().info(f'joint_angles: {data_list}')
        self.mycobot_450.send_angles(data_list, 25)


def main(args=None):
    """Entry point for the ROS2 node.

    Initializes the ROS2 system, creates the SliderSubscriber node,
    and keeps it running until shutdown.

    Args:
        args (list[str], optional): Command-line arguments for ROS2.
            Defaults to None.
    """
    rclpy.init(args=args)
    slider_subscriber = SliderSubscriber()

    rclpy.spin(slider_subscriber)

    slider_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
