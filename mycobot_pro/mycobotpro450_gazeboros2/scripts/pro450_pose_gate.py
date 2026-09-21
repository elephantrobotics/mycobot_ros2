#!/usr/bin/env python3
"""Wait for a stable real-Pro450 snapshot and persist it for Gazebo startup."""

import math
import os
import tempfile
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState


ARM_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
GRIPPER_JOINT = "gripper_controller"
COMMAND_JOINTS = ARM_JOINTS + [GRIPPER_JOINT]
JOINT_LIMITS_RAD = [
    (math.radians(-162), math.radians(162)),
    (math.radians(-125), math.radians(125)),
    (math.radians(-154), math.radians(154)),
    (math.radians(-162), math.radians(162)),
    (math.radians(-162), math.radians(162)),
    (math.radians(-165), math.radians(165)),
    (0.0, 1.0),
]


class Pro450PoseGate(Node):
    def __init__(self):
        super().__init__("pro450_pose_gate")
        self.declare_parameter(
            "output_file", "/tmp/mycobotpro450_real_initial_positions.yaml"
        )
        self.declare_parameter("timeout_sec", 600.0)
        self.declare_parameter("max_snapshot_age_sec", 2.5)
        self.output_file = str(self.get_parameter("output_file").value)
        self.max_snapshot_age_sec = max(
            0.1, float(self.get_parameter("max_snapshot_age_sec").value)
        )
        self.deadline = time.monotonic() + max(
            1.0, float(self.get_parameter("timeout_sec").value)
        )
        self.success = False
        self.failure_reason = ""

        snapshot_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            JointState,
            "/pro450/real_state_snapshot",
            self._snapshot_cb,
            snapshot_qos,
        )
        self.create_timer(0.2, self._timeout_cb)
        self.get_logger().info(
            "Waiting for a stable, read-only Pro450 pose snapshot; "
            "Gazebo startup is gated."
        )

    def _snapshot_cb(self, msg):
        if self.success or self.failure_reason:
            return
        stamp_sec = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) / 1e9
        now_sec = self.get_clock().now().nanoseconds / 1e9
        if stamp_sec <= 0.0 or abs(now_sec - stamp_sec) > self.max_snapshot_age_sec:
            self.get_logger().warning("Ignoring stale Pro450 pose snapshot.")
            return
        values = dict(zip(msg.name, msg.position))
        if not all(name in values for name in COMMAND_JOINTS):
            self.get_logger().warning("Ignoring incomplete Pro450 pose snapshot.")
            return

        positions = [float(values[name]) for name in COMMAND_JOINTS]
        if not all(math.isfinite(value) for value in positions):
            self.get_logger().warning("Ignoring non-finite Pro450 pose snapshot.")
            return
        for name, value, limits in zip(COMMAND_JOINTS, positions, JOINT_LIMITS_RAD):
            if not limits[0] <= value <= limits[1]:
                self.get_logger().warning(
                    f"Ignoring snapshot: {name}={value:.6f} rad is outside limits."
                )
                return

        try:
            self._write_yaml_atomically(positions)
        except OSError as exc:
            self.failure_reason = f"Unable to persist real pose snapshot: {exc}"
            self.get_logger().error(self.failure_reason)
            return

        self.success = True
        self.get_logger().info(
            f"Stable Pro450 pose snapshot saved to {self.output_file}; "
            "Gazebo startup may continue."
        )

    def _write_yaml_atomically(self, positions):
        output_dir = os.path.dirname(self.output_file) or "."
        os.makedirs(output_dir, exist_ok=True)
        lines = ["initial_positions:"]
        for name, value in zip(COMMAND_JOINTS, positions):
            lines.append(f"  {name}: {value:.12f}")
        payload = "\n".join(lines) + "\n"

        fd, temporary_path = tempfile.mkstemp(
            prefix=".pro450_initial_", suffix=".yaml", dir=output_dir, text=True
        )
        try:
            with os.fdopen(fd, "w", encoding="utf-8", newline="\n") as handle:
                handle.write(payload)
                handle.flush()
                os.fsync(handle.fileno())
            os.replace(temporary_path, self.output_file)
        except Exception:
            try:
                os.unlink(temporary_path)
            except OSError:
                pass
            raise

    def _timeout_cb(self):
        if self.success or self.failure_reason:
            return
        if time.monotonic() >= self.deadline:
            self.failure_reason = "Timed out waiting for a stable Pro450 pose snapshot."
            self.get_logger().error(self.failure_reason)


def main(args=None):
    rclpy.init(args=args)
    node = Pro450PoseGate()
    try:
        while rclpy.ok() and not node.success and not node.failure_reason:
            rclpy.spin_once(node, timeout_sec=0.2)
        return_code = 0 if node.success else 1
    except KeyboardInterrupt:
        return_code = 130
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    raise SystemExit(return_code)


if __name__ == "__main__":
    main()
