import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import time
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
    from pymycobot import MyPalletizer260


class Slider_Subscriber(Node):
    """ROS2 node that subscribes to joint_states and controls MyPalletizer260 in real-time."""

    def __init__(self):
        """Initialize the Slider_Subscriber node and set up ROS2 subscription and robot connection."""
        super().__init__("control_slider_gripper")

        self.subscription = self.create_subscription(
            msg_type=JointState,
            topic="joint_states",
            callback=self.listener_callback,
            qos_profile=10
        )
        self.subscription

        self.declare_parameter('port', '/dev/ttyAMA0')
        self.declare_parameter('baud', 1000000)
        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value
        self.get_logger().info("port:%s, baud:%d" % (port, baud))

        self.mc = MyPalletizer260(port, baud)
        time.sleep(0.05)

        self.angles_queue = None
        self.gripper_value = None

        threading.Thread(target=self._send_loop, daemon=True).start()

    def listener_callback(self, msg):
        """Callback for joint_states subscription.

        Converts joint positions from radians to degrees and maps the gripper position.

        Args:
            msg (JointState): The incoming joint state message.
        """
        data_list = []
        gripper_value = 0
        for i, value in enumerate(msg.position):
            if i < 4:
                radians_to_angles = round(math.degrees(value), 2)
                data_list.append(radians_to_angles)
            else:
                min_val = -0.74
                max_val = 0.15
                mapped_value = (value - min_val) / (max_val - min_val) * 100
                gripper_value = int(round(mapped_value, 2))

        self.get_logger().info('data_list: {} gripper_value: {} '.format(data_list, gripper_value))
        self.angles_queue = data_list
        self.gripper_value = gripper_value

    def _send_loop(self):
        """Background loop to send angles and gripper commands to the robotic arm."""
        while True:
            time.sleep(0.01)
            if self.angles_queue:
                self.mc.send_angles(self.angles_queue, 25)
                self.angles_queue = None
            if self.gripper_value is not None:
                self.mc.set_gripper_value(self.gripper_value, 80, gripper_type=1)
                self.gripper_value = None


def main(args=None):
    """Entry point for running the Slider_Subscriber node."""
    rclpy.init(args=args)
    slider_subscriber = Slider_Subscriber()

    rclpy.spin(slider_subscriber)

    slider_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
