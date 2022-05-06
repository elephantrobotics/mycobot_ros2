#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros.transform_listener import TransformListener
from visualization_msgs.msg import Marker


class Talker(Node):
    def __init__(self):
        super().__init__("following_marker")
        self.pub_marker = self.create_publisher(
            msg_type=Marker,
            topic="visualization_marker",
            qos_profile=10,
        )
        self.rate = self.create_rate(20)

    def start(self):

        listener = TransformListener()

        marker_ = Marker()
        marker_.header.frame_id = "/joint1"
        marker_.ns = "basic_cube"

        print("publishing ...")
        while rclpy.ok():
            rclpy.spin_once(self)
            now = self.get_clock().now() - 0.1
            try:
                trans, rot = listener.lookupTransform("joint1", "basic_shapes", now)
            except Exception as e:
                print(e)
                continue

            print(type(trans), trans)
            print(type(rot), rot)

            # marker
            marker_.header.stamp = now.to_msg()
            marker_.type = marker_.CUBE
            marker_.action = marker_.ADD
            marker_.scale.x = 0.04
            marker_.scale.y = 0.04
            marker_.scale.z = 0.04

            # marker position initial
            marker_.pose.position.x = trans[0]
            marker_.pose.position.y = trans[1]
            marker_.pose.position.z = trans[2]
            marker_.pose.orientation.x = rot[0]
            marker_.pose.orientation.y = rot[1]
            marker_.pose.orientation.z = rot[2]
            marker_.pose.orientation.w = rot[3]

            marker_.color.a = 1.0
            marker_.color.g = 1.0
            self.pub_marker.publish(marker_)

            self.rate.sleep()

def main(args=None):
    rclpy.init(args=args)
    talker = Talker()
    talker.start()
    rclpy.spin(talker)
    talker.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
