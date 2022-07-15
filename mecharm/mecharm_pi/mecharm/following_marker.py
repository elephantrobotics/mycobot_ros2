import rclpy
from rclpy.node import Node
from tf2_ros.transform_listener import TransformListener
from visualization_msgs.msg import Marker
from tf2_ros.buffer import Buffer


class Talker(Node):
    def __init__(self):
        super().__init__("following_marker")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pub_marker = self.create_publisher(
            msg_type=Marker,
            topic="visualization_marker",
            qos_profile=10,
        )

        self.timer = self.create_timer(0.05, self.timer_callback)

    def timer_callback(self):
        marker_ = Marker()
        marker_.header.frame_id = "/joint1"
        marker_.ns = "basic_cube"

        print("publishing ...")
        while rclpy.ok():
            rclpy.spin_once(self)
            now = self.get_clock().now()
            try:
                trans = self.tf_buffer.lookup_transform(
                    "joint1",
                    "basic_shapes",
                    now
                )
                print("trans")
            except Exception as e:
                print(e)
                continue

            print(type(trans), trans)

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
            marker_.pose.orientation.x = trans[3]
            marker_.pose.orientation.y = trans[4]
            marker_.pose.orientation.z = trans[5]
            marker_.pose.orientation.w = trans[6]

            marker_.color.a = 1.0
            marker_.color.g = 1.0
            self.pub_marker.publish(marker_)
            print(marker_)


def main():
    rclpy.init()
    talker = Talker()

    rclpy.spin(talker)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
