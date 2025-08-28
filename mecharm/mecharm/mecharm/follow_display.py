import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from visualization_msgs.msg import Marker
import pymycobot
from packaging import version

# Minimum required pymycobot version
MIN_REQUIRE_VERSION = '3.6.1'

current_verison = pymycobot.__version__
print('current pymycobot library version: {}'.format(current_verison))

if version.parse(current_verison) < version.parse(MIN_REQUIRE_VERSION):
    raise RuntimeError(
        'The version of pymycobot library must be greater than {} or higher. '
        'Current version is {}. Please upgrade the library version.'.format(
            MIN_REQUIRE_VERSION, current_verison
        )
    )
else:
    print('pymycobot library version meets the requirements!')
    from pymycobot import MechArm270


class Talker(Node):
    """ROS2 node to publish joint states and visualize end-effector position."""

    def __init__(self):
        """Initialize the Talker node and connect to MyCobot320."""
        super().__init__("follow_display")
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)

        port = self.get_parameter("port").get_parameter_value().string_value
        baud = self.get_parameter("baud").get_parameter_value().integer_value

        self.get_logger().info("port:%s, baud:%d" % (port, baud))
        self.mc = MechArm270(port, str(baud))
        self.mc.release_all_servos()

    def start(self):
        """Start publishing joint states and visualization markers.

        Publishes:
            JointState messages to 'joint_states' topic.
            Marker messages to 'visualization_marker' topic.
        """
        pub = self.create_publisher(
            msg_type=JointState,
            topic="joint_states",
            qos_profile=10
        )
        pub_marker = self.create_publisher(
            msg_type=Marker,
            topic="visualization_marker",
            qos_profile=10
        )
        rate = self.create_rate(30)

        # Initialize joint state message
        joint_state_send = JointState()
        joint_state_send.header = Header()
        joint_state_send.name = [
            "joint1_to_base",
            "joint2_to_joint1",
            "joint3_to_joint2",
            "joint4_to_joint3",
            "joint5_to_joint4",
            "joint6_to_joint5",
        ]
        joint_state_send.velocity = [0.0]
        joint_state_send.effort = []

        # Initialize marker
        marker_ = Marker()
        marker_.header.frame_id = "/base"
        marker_.ns = "my_namespace"

        while rclpy.ok():
            rclpy.spin_once(self)
            joint_state_send.header.stamp = self.get_clock().now().to_msg()
            try:
                # Get joint angles in radians
                angles = self.mc.get_radians()
                data_list = []
                for _, value in enumerate(angles):
                    data_list.append(value)

                joint_state_send.position = data_list
                pub.publish(joint_state_send)

                # Get end-effector coordinates
                coords = self.mc.get_coords()

                # Configure marker
                marker_.header.stamp = self.get_clock().now().to_msg()
                marker_.type = marker_.SPHERE
                marker_.action = marker_.ADD
                marker_.scale.x = 0.04
                marker_.scale.y = 0.04
                marker_.scale.z = 0.04

                # Default coordinates if retrieval fails
                if not coords:
                    coords = [0, 0, 0, 0, 0, 0]

                # Set marker position
                marker_.pose.position.x = coords[1] / 1000 * -1
                marker_.pose.position.y = coords[0] / 1000
                marker_.pose.position.z = coords[2] / 1000

                marker_.color.a = 1.0
                marker_.color.g = 1.0
                pub_marker.publish(marker_)

                rate.sleep()
            except Exception as e:
                print(e)


def main(args=None):
    """Main function to run the Talker node.

    Args:
        args (list, optional): Command-line arguments for ROS2. Defaults to None.
    """
    rclpy.init(args=args)

    talker = Talker()
    talker.start()
    rclpy.spin(talker)

    talker.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
