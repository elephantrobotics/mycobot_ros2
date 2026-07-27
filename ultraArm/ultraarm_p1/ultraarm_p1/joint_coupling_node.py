#!/usr/bin/env python
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
"""_summary_
The J2–J3 joint coupling node has the following overall structure:

joint_state_publisher_gui
│
▼
/joint_states_raw
│
▼
joint_coupling_node
(J2-J3 constraints)
│
▼
/joint_states
│
├── robot_state_publisher
│
├── RViz
│
└── slider_control.py
(Controls the real robot)
"""
J2_RANGE = (-18.0, 85.0)
J3_RANGE = (-1.0, 110.0)
ZERO_EPS_DEG = 0.1
INVALID_WARN_DELTA_DEG = 0.5


def snap_zero(angle_deg):
    return 0.0 if abs(angle_deg) < ZERO_EPS_DEG else angle_deg


class JointCouplingNode(Node):
    """Filter JointState messages using the ultraArm P1 J2-J3 coupling limits."""

    def __init__(self):
        super().__init__("joint_coupling_node")
        self.pub = self.create_publisher(JointState, "/joint_states", 10)
        self.sub = self.create_subscription(
            JointState,
            "/joint_states_raw",
            self.callback,
            10
        )
        self.last_valid_msg = None
        self.last_invalid_pair = None
        self.was_invalid = False
        self.get_logger().info("Joint coupling node started")

    def joint_angle_deg(self, msg, joint_name):
        try:
            index = msg.name.index(joint_name)
        except ValueError:
            raise KeyError(joint_name)

        return math.degrees(msg.position[index])

    def clone_joint_state(self, msg):
        clone = JointState()
        clone.header.stamp = msg.header.stamp
        clone.header.frame_id = msg.header.frame_id
        clone.name = list(msg.name)
        clone.position = list(msg.position)
        clone.velocity = list(msg.velocity)
        clone.effort = list(msg.effort)
        return clone

    def valid_region(self, a, b):
        """
        a: J2 model angle (deg)
        b: J3 model angle (deg)
        """
        if not (J2_RANGE[0] <= a <= J2_RANGE[1] and J3_RANGE[0] <= b <= J3_RANGE[1]):
            return False

        if -18 <= a < 0:
            # J2 < 0: reject J3 >= 42° (blocks abs(cos) second lobe / mesh penetration).
            if b >= 42.0:
                return False
            cond1 = math.cos(math.radians(-a + b)) - math.sin(math.radians(45 + a)) <= 7 / 30
            cond2 = abs(math.cos(math.radians(-a + b))) >= 15.4 / 30
            return cond1 and cond2

        if 0 <= a <= 50.87:
            return math.cos(math.radians(a - b)) >= 15.4 / 30

        if 50.87 < a < 76.72:
            return True

        if 76.72 <= a <= 85:
            return abs(math.cos(math.radians(a - b))) >= 6.89 / 30

        return False

    def callback(self, msg):
        try:
            j2 = round(snap_zero(self.joint_angle_deg(msg, "J2")), 2)
            j3 = round(snap_zero(self.joint_angle_deg(msg, "J3")), 2)
        except (KeyError, IndexError):
            self.get_logger().warn("JointState missing J2 or J3; message ignored")
            return

        if not self.valid_region(j2, j3):
            # self.get_logger().warn(f"Invalid J2-J3 combination: {j2:.2f} {j3:.2f}")
            should_warn = not self.was_invalid
            if self.last_invalid_pair is not None:
                should_warn = should_warn or abs(j2 - self.last_invalid_pair[0]) >= INVALID_WARN_DELTA_DEG
                should_warn = should_warn or abs(j3 - self.last_invalid_pair[1]) >= INVALID_WARN_DELTA_DEG
            else:
                should_warn = True

            if should_warn:
                self.get_logger().warn(f"Invalid J2-J3 combination: {j2:.2f} {j3:.2f}")
                self.last_invalid_pair = (j2, j3)
            self.was_invalid = True

            if self.last_valid_msg is not None:
                safe_msg = self.clone_joint_state(self.last_valid_msg)
                safe_msg.header.stamp = self.get_clock().now().to_msg()
                self.pub.publish(safe_msg)
            return

        self.was_invalid = False
        self.last_invalid_pair = None
        self.last_valid_msg = self.clone_joint_state(msg)

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = JointCouplingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()