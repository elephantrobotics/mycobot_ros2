import time
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import pymycobot
from packaging import version

# Minimum required pymycobot version
MIN_REQUIRE_VERSION = '3.6.1'

current_verison = pymycobot.__version__
print('current pymycobot library version: {}'.format(current_verison))
if version.parse(current_verison) < version.parse(MIN_REQUIRE_VERSION):
    raise RuntimeError(
        'The version of pymycobot library must be greater than {} or higher. '
        'The current version is {}. Please upgrade the library version.'.format(
            MIN_REQUIRE_VERSION, current_verison
        )
    )
else:
    print('pymycobot library version meets the requirements!')
    from pymycobot import MechArm270


class Slider_Subscriber(Node):
    """ROS2 node that subscribes to joint states and sends commands to MechArm270."""

    def __init__(self):
        """Initialize the subscriber node and connect to MechArm270."""
        super().__init__("control_slider")
        self.subscription = self.create_subscription(
            JointState,
            "joint_states",
            self.listener_callback,
            10
        )
        self.subscription

        # Declare robot connection parameters
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value
        self.get_logger().info("port:%s, baud:%d" % (port, baud))

        # Connect to MechArm270
        self.mc = MechArm270(port, baud)
        time.sleep(0.05)
        self.mc.set_fresh_mode(1)
        time.sleep(0.05)

    def listener_callback(self, msg: JointState):
        """Handle received joint state messages and send angles to the robot.

        Args:
            msg (JointState): ROS2 JointState message containing joint positions
                in radians.

        Returns:
            None
        """
        data_list = []
        for _, value in enumerate(msg.position):
            radians_to_angles = round(math.degrees(value), 2)
            data_list.append(radians_to_angles)

        self.get_logger().info('joint_angles: {}'.format(data_list))
        self.mc.send_angles(data_list, 25)


def main(args=None):
    """Main entry point for the Slider_Subscriber node.

    Args:
        args (list, optional): Command-line arguments for ROS2. Defaults to None.

    Returns:
        None
    """
    rclpy.init(args=args)
    slider_subscriber = Slider_Subscriber()

    rclpy.spin(slider_subscriber)

    slider_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
