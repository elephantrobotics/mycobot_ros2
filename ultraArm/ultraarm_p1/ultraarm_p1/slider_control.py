import rclpy
import time
from sensor_msgs.msg import JointState
from rclpy.node import Node
import math
import pymycobot
from packaging import version
# min low version require
MIN_REQUIRE_VERSION = '4.0.5'

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
    from pymycobot import UltraArmP1


class Slider_Subscriber(Node):
    """ROS2 node that subscribes to joint states and sends commands to ultraArm P1."""

    def __init__(self):
        super().__init__("control_slider")
        self.subscription = self.create_subscription(
            JointState,
            "joint_states",
            self.listener_callback,
            10
        )
        # self.subscription
        # Declare robot connection parameters
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 1000000)
        self.declare_parameter('speed', 25)
        self.declare_parameter('command_rate', 5.0)
        self.declare_parameter('queue_limit', 0)
        self.declare_parameter('min_angle_delta', 0.2)
        self.declare_parameter('use_stop_before_send', True)
        self.declare_parameter('stop_queue_threshold', 10)
        self.declare_parameter('stop_settle_time', 0.02)

        port = self.get_parameter("port").get_parameter_value().string_value
        baud = self.get_parameter("baud").get_parameter_value().integer_value
        self.speed = self.get_parameter("speed").get_parameter_value().integer_value
        self.command_rate = self.get_parameter("command_rate").get_parameter_value().double_value
        self.command_rate = max(self.command_rate, 0.1)
        self.queue_limit = self.get_parameter("queue_limit").get_parameter_value().integer_value
        self.min_angle_delta = self.get_parameter("min_angle_delta").get_parameter_value().double_value
        self.use_stop_before_send = self.get_parameter("use_stop_before_send").get_parameter_value().bool_value
        self.stop_queue_threshold = self.get_parameter("stop_queue_threshold").get_parameter_value().integer_value
        self.stop_settle_time = self.get_parameter("stop_settle_time").get_parameter_value().double_value
        self.latest_angles = None
        self.last_sent_angles = None
        self.target_dirty = False

        self.get_logger().info("port:%s, baud:%d" % (port, baud))
        self.ua = UltraArmP1(port, baud)
        self.ua.set_joint_enable(0)
        self.command_timer = self.create_timer(1.0 / self.command_rate, self.command_worker)

    def joint_state_to_angles(self, msg):
        """Convert JointState radians into ultraArm P1 joint angles in degrees."""
        data_list = []
        for value in msg.position:
            radians_to_angles = round(math.degrees(value), 2)
            data_list.append(radians_to_angles)
        joint1 = data_list[0]
        joint2 = data_list[1]
        joint3 = data_list[5] + 90
        joint4 = data_list[-1]
        return [joint1, joint2, joint3, joint4]

    def angles_changed(self, new_angles):
        """Return True when the target changed enough to justify a new command."""
        if self.last_sent_angles is None:
            return True
        return any(
            abs(new - old) >= self.min_angle_delta
            for new, old in zip(new_angles, self.last_sent_angles)
        )

    def get_robot_queue_size(self):
        """Read firmware queue size when supported by the pymycobot driver."""
        if not hasattr(self.ua, "get_queue_size"):
            return None
        try:
            size = self.ua.get_queue_size()
        except Exception as exc:
            self.get_logger().warn("Failed to read robot queue size: %s" % exc)
            return None
        if isinstance(size, int) and size >= 0:
            return size
        return None

    def stop_robot_queue(self):
        """Stop current motion and clear queued firmware targets when supported."""
        if not hasattr(self.ua, "stop"):
            self.get_logger().warn("Robot driver does not provide stop(); cannot clear queued targets.")
            return False
        try:
            self.ua.stop()
            if self.stop_settle_time > 0:
                time.sleep(self.stop_settle_time)
            return True
        except Exception as exc:
            self.get_logger().warn("Failed to stop robot before sending latest target: %s" % exc)
            return False

    def command_worker(self):
        """Send only the newest slider target at a bounded rate."""
        target = list(self.latest_angles) if self.latest_angles is not None else None
        should_send = self.target_dirty and target is not None and self.angles_changed(target)
        if not should_send:
            return

        queue_size = self.get_robot_queue_size()
        if self.use_stop_before_send:
            if queue_size is None or queue_size > self.stop_queue_threshold:
                if queue_size is not None:
                    self.get_logger().info("clear robot queue before latest target, queue_size: %s" % queue_size)
                self.stop_robot_queue()
        elif queue_size is not None and queue_size > self.queue_limit:
            self.get_logger().warn(
                "Robot command queue is high (%s > %s); holding newest target."
                % (queue_size, self.queue_limit)
            )
            return

        self.get_logger().info("send angles: %s" % target)
        try:
            self.ua.set_angles(target, self.speed, _async=False)
            self.last_sent_angles = target
            if self.latest_angles == target:
                self.target_dirty = False
        except Exception as exc:
            self.get_logger().error("Failed to send angles: %s" % exc)

    def listener_callback(self, msg):
        """Handle received joint state messages and send angles to the robot.

        Args:
            msg (JointState): ROS2 JointState message containing joint positions
                in radians.

        Returns:
            None
        """

        try:
            self.latest_angles = self.joint_state_to_angles(msg)
            self.target_dirty = True
        except IndexError:
            self.get_logger().warn("Invalid JointState position length: %s" % len(msg.position))


def main(args=None):
    rclpy.init(args=args)
    slider_subscriber = Slider_Subscriber()

    rclpy.spin(slider_subscriber)

    slider_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
