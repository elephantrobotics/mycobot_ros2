import math
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import pymycobot
from packaging import version

# Minimum required pymycobot version
MIN_REQUIRE_VERSION = '4.0.5'

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
    from pymycobot import UltraArmP1
    from pymycobot.robot_info import RobotLimit

ROBOT_LIMIT = RobotLimit.robot_limit.get("UltraArmP1", {})
JOINT_LIMITS = list(zip(
    ROBOT_LIMIT.get("angles_min", [-165, -18, 89, -179]),
    ROBOT_LIMIT.get("angles_max", [165, 85, 200, 179]),
))


def valid_angles(angles):
    return (
        isinstance(angles, list)
        and len(angles) == 4
        and all(low <= angle <= high for angle, (low, high) in zip(angles, JOINT_LIMITS))
    )


def angles_to_joint_positions(angles):
    display_angles = list(angles)
    display_angles[2] -= 90
    return [math.radians(value) for value in display_angles]

class Talker(Node):
    """ROS2 node to publish joint states and visualize end-effector position."""

    def __init__(self):
        """Initialize the Talker node and connect to ultraArm P1."""
        super().__init__("follow_display")
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 1000000)
        self.declare_parameter('publish_rate', 30.0)

        port = self.get_parameter("port").get_parameter_value().string_value
        baud = self.get_parameter("baud").get_parameter_value().integer_value
        self.publish_rate = max(self.get_parameter("publish_rate").get_parameter_value().double_value,0.1)

        self.get_logger().info("port:%s, baud:%d" % (port, baud))
        self.ua = UltraArmP1(port, baud)
        time.sleep(0.02)
        self.ua.set_end_button_enable()
        self.last_invalid_log_time = 0.0
        
        self.get_logger().info("Please press the LED button at the end of the machine to drag the joint.\n请按下机器末端LED按钮进行关节拖拽运动\n")

    def start(self):
        """Start publishing joint states and visualization markers.

        Publishes:
            JointState messages to 'joint_states' topic.
        """
        pub = self.create_publisher(
            msg_type=JointState,
            topic="joint_states",
            qos_profile=10
        )
        
        rate = self.create_rate(self.publish_rate)

        # Initialize joint state message
        joint_state_send = JointState()
        joint_state_send.header = Header()
        joint_state_send.name = ["J1", "J2", "J3", "J4"]
        joint_state_send.velocity = [0.0]
        joint_state_send.effort = []

        self.get_logger().info("Publishing ...")
        while rclpy.ok():
            rclpy.spin_once(self)
            joint_state_send.header.stamp = self.get_clock().now().to_msg()
            try:
                # Get robot joint angles
                angles = self.ua.get_angles_info()
                if valid_angles(angles):
                    last_valid_positions = angles_to_joint_positions(angles)
                else:
                    now = self.get_clock().now().nanoseconds / 1e9
                    if now - self.last_invalid_log_time >= 5.0:
                        # self.get_logger().warn("Failed to get valid angles: {}".format(angles))
                        self.last_invalid_log_time = now
                    if last_valid_positions is None:
                        rate.sleep()
                        continue

                joint_state_send.position = list(last_valid_positions)
                pub.publish(joint_state_send)

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
    try:
        talker.start()
    except KeyboardInterrupt:
        pass
    finally:
        talker.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    # talker = Talker()
    # talker.start()
    # rclpy.spin(talker)

    # talker.destroy_node()
    # rclpy.shutdown()


if __name__ == "__main__":
    main()
