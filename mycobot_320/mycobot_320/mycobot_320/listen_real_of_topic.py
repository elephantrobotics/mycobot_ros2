import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class Listener(Node):
    """ROS2 node that subscribes to joint_states and logs joint angles in degrees."""

    def __init__(self):
        """Initialize the Listener node and subscribe to the joint_states topic."""
        super().__init__("real_listener_1")

        self.sub = self.create_subscription(
            msg_type=JointState,
            topic="joint_states",
            callback=self.callback,
            qos_profile=10
        )

    def callback(self, msg):
        """Handle incoming JointState messages.

        Converts joint positions from radians to degrees and logs them.

        Args:
            msg (JointState): The incoming joint states message.
        """
        angles_data = [int(i / math.pi * 180) for i in list(msg.position)]
        self.get_logger().info(
            '\n\t angles: {}\n'.format(
                angles_data
            )
        )


def main(args=None):
    """Entry point for running the Listener node.

    Initializes ROS2, creates the Listener node, and spins it.

    Args:
        args (list, optional): Command-line arguments for ROS2. Defaults to None.
    """
    rclpy.init(args=args)

    listener = Listener()

    rclpy.spin(listener)

    listener.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
