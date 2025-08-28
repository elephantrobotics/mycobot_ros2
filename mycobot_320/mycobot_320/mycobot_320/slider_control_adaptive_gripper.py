import time
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import pymycobot
from packaging import version

# Minimum required pymycobot version
MIN_REQUIRE_VERSION = '3.6.0'

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
    from pymycobot.mycobot320 import MyCobot320


class Slider_Subscriber(Node):
    """ROS2 node that subscribes to joint states and controls MyCobot320, including gripper."""

    def __init__(self):
        """Initialize the subscriber node and connect to MyCobot320."""
        super().__init__("control_slider")

        # Subscribe to joint states
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
        self.get_logger().info(f"port: {port}, baud: {baud}")

        # Connect to MyCobot320
        self.mc = MyCobot320(port, baud)
        time.sleep(0.05)
        self.mc.set_fresh_mode(1)
        time.sleep(0.05)

    def listener_callback(self, msg: JointState):
        """Process received joint states and control robot joints and gripper.

        Args:
            msg (JointState): ROS2 message containing joint positions in radians
                and gripper position as additional elements.

        Returns:
            None
        """
        data_list = []
        gripper_value = 0

        for i, value in enumerate(msg.position):
            if i < 6:
                # Convert radians to degrees for the robot joints
                radians_to_angles = round(math.degrees(value), 2)
                data_list.append(radians_to_angles)
            else:
                # Map gripper value to 0-100 scale
                min_val = -1.11
                max_val = 0
                mapped_value = (value - min_val) / (max_val - min_val) * 100
                gripper_value = int(round(mapped_value, 2))

        self.get_logger().info(
            f'joint_list: {data_list}, gripper_value: {gripper_value}')
        self.mc.send_angles(data_list, 25)
        self.mc.set_gripper_value(gripper_value, 80, 1)


def main(args=None):
    """Main function to run the Slider_Subscriber node.

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
