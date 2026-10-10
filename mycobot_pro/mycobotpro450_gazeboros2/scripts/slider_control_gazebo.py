#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Validated Pro450 slider-command bridge for Gazebo and optional real hardware."""

import math
import queue
import threading
import time

import rclpy
from builtin_interfaces.msg import Duration, Time
from geometry_msgs.msg import Pose
from moveit_msgs.msg import (
    AllowedCollisionEntry,
    CollisionObject,
    PlanningScene,
    PlanningSceneComponents,
)
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene, GetStateValidity
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty, String, Float64MultiArray, Bool
from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from pro450_real_keyboard import RealPoseReader, sdk_speed_for_rad
from pro450_real_mirror import RealPoseBuffer

from pro450_sdk_adapter import Pro450Client
from pro450_feedback import attach_feedback, publish_feedback, publish_age
from pro450_gripper_profile import gripper_speed_for_duration, measured_gripper_rate, GripperMotionEstimate


DEFAULT_PRO450_IP = "192.168.0.232"
DEFAULT_PRO450_PORT = 4500

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
MAX_VELOCITY_RAD_S = [1.0] * 7
MIN_TRAJECTORY_DURATION = 0.5
MAX_TRAJECTORY_DURATION = 600.0
SPLINE_PEAK_VELOCITY_FACTOR = 1.5
COLLISION_SAMPLE_STEP = math.radians(1.0)
MAX_COLLISION_SAMPLES = 400
DEFAULT_COLLISION_DEPTH_TOLERANCE_M = 0.0001
FORCE_EXECUTE_MAX_SPEED_SCALE = 0.10
FLOOR_OBJECT_ID = "pro450_ground_safety"
REAL_POSE_TOLERANCE_RAD = 0.05
WAYPOINT_ARM_TOLERANCE_RAD = math.radians(0.5)
WAYPOINT_GRIPPER_TOLERANCE = 0.02
# Distance from the MoveIt-checked joint-space line that stops the real arm.
LINE_DEVIATION_LIMIT_RAD = math.radians(1.0)
GRIPPER_FOLLOW_STEP = 5
REAL_ARRIVAL_MARGIN_S = 10.0
# Collision validation still computes the first collision point and penetration
# depth. These flags only control how much diagnostic detail is exposed in the
# operator-facing status text, so the previous wording can be restored easily.
SHOW_COLLISION_PATH_PERCENT = False
SHOW_COLLISION_DEPTH = False
SHOW_COLLISION_BODIES = False


def sdk_motion_accepted(result):
    """_async send returns 1. A blocking send returns 0 on arrival, -1 with no reply."""
    return result in (0, 1)


def gripper_write_accepted(result):
    """Accept the speed/angle echo bug. pymycobot returns -1 unless the echo is 1."""
    if result in (1, -1, None):
        return True
    return not isinstance(result, str) and result != 0


class SliderControl(Node):
    def __init__(self):
        super().__init__("slider_control_gazebo")

        self.declare_parameter("mode", "simulation")
        self.declare_parameter("pro450_ip", DEFAULT_PRO450_IP)
        self.declare_parameter("pro450_port", DEFAULT_PRO450_PORT)
        self.declare_parameter("startup_read_only", True)
        self.declare_parameter("feedback_hz", 10.0)
        self.declare_parameter("startup_stable_samples", 5)
        self.declare_parameter("startup_sample_interval_sec", 0.2)
        self.declare_parameter("startup_stable_tolerance_deg", 0.2)
        # Keep the MoveIt ground surface coincident with Gazebo's z=0 plane.
        # A positive value is an optional safety margin, not model geometry.
        self.declare_parameter("floor_clearance_m", 0.0)
        self.declare_parameter(
            "collision_depth_tolerance_m",
            DEFAULT_COLLISION_DEPTH_TOLERANCE_M,
        )
        self.declare_parameter("floor_size_m", 20.0)
        self.declare_parameter("floor_thickness_m", 0.10)
        self.declare_parameter("floor_frame", "world")
        mode_name = str(self.get_parameter("mode").value).strip().lower()
        if mode_name not in ("simulation", "real"):
            raise ValueError("mode must be either 'simulation' or 'real'.")
        self.mode = 2 if mode_name == "real" else 1
        self.startup_read_only = bool(
            self.get_parameter("startup_read_only").value
        )
        self.startup_stable_samples = max(
            2, int(self.get_parameter("startup_stable_samples").value)
        )
        self.startup_sample_interval_sec = max(
            0.05,
            float(self.get_parameter("startup_sample_interval_sec").value),
        )
        self.startup_stable_tolerance_deg = max(
            0.0,
            float(self.get_parameter("startup_stable_tolerance_deg").value),
        )
        self.pro450_ip = self.get_parameter("pro450_ip").value
        self.pro450_port = int(self.get_parameter("pro450_port").value)
        self.floor_clearance_m = max(
            0.0, float(self.get_parameter("floor_clearance_m").value)
        )
        self.collision_depth_tolerance_m = max(
            0.0,
            float(self.get_parameter("collision_depth_tolerance_m").value),
        )
        self.floor_size_m = max(1.0, float(self.get_parameter("floor_size_m").value))
        self.floor_thickness_m = max(
            0.01, float(self.get_parameter("floor_thickness_m").value)
        )
        self.floor_frame = str(self.get_parameter("floor_frame").value)

        self.pub_arm = self.create_publisher(
            JointTrajectory, "/arm_controller/joint_trajectory", 10
        )
        self.pub_gripper = self.create_publisher(
            JointTrajectory, "/pro_gripper_controller/joint_trajectory", 10
        )
        status_topic = "/pro450/startup_status" if self.mode == 2 and self.startup_read_only else "/pro450/slider_status"
        self.status_pub = self.create_publisher(String, status_topic, 10)
        self._handoff_timer = None
        self._handoff_ack = self.create_publisher(String, "/pro450/snapshot_released", 10)
        self.create_subscription(String, "/pro450/snapshot_consumed", self._snapshot_consumed_cb, 10)
        snapshot_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.real_snapshot_pub = self.create_publisher(
            JointState, "/pro450/real_state_snapshot", snapshot_qos
        )

        self.create_subscription(JointState, "/joint_states", self._feedback_cb, 10)
        self.create_subscription(
            JointState, "/pro450/slider_targets", self._target_cb, 10
        )
        self.create_subscription(
            JointState, "/pro450/slider_force_targets", self._force_target_cb, 10
        )
        self.create_subscription(Empty, "/pro450/slider_stop", self._stop_cb, 10)

        self.validity_client = self.create_client(
            GetStateValidity, "/check_state_validity"
        )
        self.get_scene_client = self.create_client(
            GetPlanningScene, "/get_planning_scene"
        )
        self.apply_scene_client = self.create_client(
            ApplyPlanningScene, "/apply_planning_scene"
        )

        self._state_lock = threading.RLock()
        self._current_positions = None
        self._feedback_time = 0.0
        self._feedback_valid = False
        self._command_active = False
        self._stop_requested = False
        self._stop_event = threading.Event()
        # The read-only startup snapshot must not advertise idle while the
        # active controller is validating/executing a command.
        self.busy_pub = None
        if not (self.mode == 2 and self.startup_read_only):
            self.busy_pub = self.create_publisher(Bool, "/pro450/slider_busy", 10)
            self.create_timer(0.1, self._publish_command_busy)

        self.mc = None
        self._motion_sent = False
        self._real_thread = None
        self._mirror_thread = None
        self._mirror_stop = threading.Event()
        self._real_mirror = None
        self._pose_reader = None
        self._arm_moving = False
        self._sample_velocity = [0.0] * 7
        self._sample_publish_lock = threading.Lock()
        self._gripper_model = GripperMotionEstimate()
        self._last_read_error = 0.0
        self._real_motion_enabled = self.mode == 2 and not self.startup_read_only
        self.command_queue = queue.Queue(maxsize=1)
        if self.mode == 2:
            self._setup_real_mirror()
            self._initialize_pro450()
            self._pose_reader = RealPoseReader(
                self.mc, JOINT_LIMITS_RAD, report=self.get_logger().debug)
            self._pose_reader.seed_gripper(self._startup_pose[6])
            stamp = self.mc.sample_time('arm')
            if stamp is not None:
                self._pose_reader.observe_arm(
                    [math.degrees(v) for v in self._startup_pose[:6]], *stamp)
            self._on_real_sample(self._startup_pose)
            if self._real_motion_enabled:
                attach_feedback(self, self._pose_reader)
                self._mirror_thread = threading.Thread(
                    target=self._mirror_loop, daemon=True)
                self._mirror_thread.start()
                self._real_thread = threading.Thread(
                    target=self._real_robot_worker, daemon=True)
                self._real_thread.start()
                self._publish_status("Ready: Real Robot + Gazebo motion mode.")
            else:
                self.create_timer(1.0, self._refresh_read_only_snapshot)
                self._publish_status(
                    "Ready: real robot startup is READ ONLY; stable pose snapshot published."
                )
        else:
            self._publish_status("Ready: Gazebo simulation mode.")

    def _publish_status(self, message):
        self.get_logger().info(message)
        msg = String()
        msg.data = message
        self.status_pub.publish(msg)
        self._publish_command_busy()

    def _publish_command_busy(self):
        if self.busy_pub is not None:
            msg = Bool()
            # Also called while _state_lock is held on duplicate rejection.
            with self._state_lock:
                msg.data = self._command_active
                self.busy_pub.publish(msg)

    def _setup_real_mirror(self):
        self._real_mirror = RealPoseBuffer()
        self.real_joint_pub = self.create_publisher(
            JointState, "/pro450/real_joint_states", 1)
        self.real_age_pub = self.create_publisher(Float64MultiArray, "/pro450/real_feedback_age", 1)
        self.gripper_estimate_pub = self.create_publisher(Float64MultiArray, "/pro450/gripper_estimate", 1)

    def _on_real_sample(self, pose):
        with self._pose_reader.cache.lock:
            value = self._pose_reader.gripper
            measured = self._pose_reader.gripper_time
            valid = self._pose_reader.gripper_valid
        if valid and measured is not None:
            self._gripper_model.observe(value, measured)
        publish_feedback(self, pose, self._pose_reader, COMMAND_JOINTS)

    def _mirror_loop(self):
        next_tick = time.monotonic()
        while rclpy.ok() and not self._mirror_stop.is_set():
            next_tick += 0.02
            if int(next_tick * 50) % 25 == 0:
                publish_age(self, self._pose_reader)
            try:
                rendered = self._real_mirror.render()
                gripper = self._gripper_model.render()
                msg = Float64MultiArray()
                msg.data = list(gripper) if gripper is not None else [0.0, 0.0, 0.0]
                self.gripper_estimate_pub.publish(msg)
                if rendered is not None and gripper is not None:
                    pose, velocity = rendered
                    # Gripper readings have their own timestamps. Never return
                    # to the arm buffer's old opening or interpolated slope.
                    pose[6], velocity[6] = gripper[:2]
                    self._publish_trajectory(pose, 0.04, velocity)
                elif gripper is not None:
                    self._publish_gripper_trajectory(gripper[0], 0.04, gripper[1])
            except Exception:
                self.get_logger().debug("Could not render the measured pose.")
            delay = next_tick - time.monotonic()
            if delay > 0.0:
                self._mirror_stop.wait(delay)
            else:
                next_tick = time.monotonic()

    def _read_startup_gripper(self, timeout=10.0):
        """Acquire a real initial opening, with a deadline even for a blocked SDK read."""
        sdk = self.mc
        finished = threading.Event()
        cancelled = threading.Event()
        result = {'attempts': 0, 'last': None}
        deadline = time.monotonic() + timeout

        def read_until_valid():
            try:
                while not cancelled.is_set() and time.monotonic() < deadline:
                    result['attempts'] += 1
                    try:
                        value = sdk.get_pro_gripper_angle()
                    except Exception as exc:
                        value = f"{type(exc).__name__}: {exc}"
                    result['last'] = value
                    if cancelled.is_set() or time.monotonic() >= deadline:
                        return
                    if (not isinstance(value, bool) and isinstance(value, (int, float))
                            and math.isfinite(value) and 0.0 <= value <= 100.0):
                        result['value'] = float(value)
                        return
                    cancelled.wait(min(0.1, max(0.0, deadline - time.monotonic())))
            finally:
                finished.set()

        self.get_logger().info(f"Reading initial gripper angle; timeout {timeout:.1f}s...")
        worker = threading.Thread(target=read_until_valid, daemon=True)
        worker.start()
        finished.wait(max(0.0, deadline - time.monotonic()))
        cancelled.set()
        if 'value' not in result:
            # Close the read-only connection to unblock a pending SDK query.
            # No pose, speed or motion commands are issued during this retry.
            sdk.close()
            worker.join(timeout=1.0)
            raise RuntimeError(
                f"initial gripper angle read timed out after {timeout:.1f}s "
                f"({result['attempts']} attempts, last={result['last']!r})")
        worker.join()
        self.get_logger().info(
            f"Initial gripper angle acquired: {result['value']:.1f} "
            f"after {result['attempts']} read(s).")
        return result['value']

    def _initialize_pro450(self):
        if Pro450Client is None:
            raise RuntimeError("pymycobot is required only for real Pro450 mode")
        try:
            self.get_logger().info(
                f"Connecting to Pro450 @ {self.pro450_ip}:{self.pro450_port}"
            )
            self.mc = Pro450Client(self.pro450_ip, self.pro450_port,
                                   feedback_hz=float(self.get_parameter("feedback_hz").value))
            power_state = self.mc.is_power_on()
            if power_state != 1:
                raise RuntimeError(
                    f"robot is not powered and will not be powered automatically "
                    f"(is_power_on={power_state})"
                )

            error_code = self.mc.get_error_information()
            if error_code not in (0, None):
                raise RuntimeError(f"robot reports error code {error_code}")

            samples = []
            for index in range(self.startup_stable_samples):
                moving = self.mc.is_moving()
                if moving != 0:
                    raise RuntimeError(
                        f"robot must be stationary for pose synchronization "
                        f"(is_moving={moving})"
                    )
                angles = self.mc.get_angles()
                samples.append(self._validate_real_angles(angles))
                if index + 1 < self.startup_stable_samples:
                    time.sleep(self.startup_sample_interval_sec)

            for joint_index in range(len(ARM_JOINTS)):
                values = [sample[joint_index] for sample in samples]
                if max(values) - min(values) > self.startup_stable_tolerance_deg:
                    raise RuntimeError(
                        f"{ARM_JOINTS[joint_index]} did not remain stable during "
                        "startup sampling"
                    )

            gripper_value = self._read_startup_gripper(timeout=10.0)

            averaged_angles = [
                sum(sample[index] for sample in samples) / len(samples)
                for index in range(len(ARM_JOINTS))
            ]
            self._startup_pose = [math.radians(value) for value in averaged_angles] + [
                gripper_value / 100.0
            ]
            self._publish_real_snapshot(averaged_angles, gripper_value)
            self.get_logger().info(
                "Pro450 connected in read-only startup phase. Stable angles: "
                f"{averaged_angles}; gripper={gripper_value:.1f}."
            )
        except Exception as exc:
            if self.mc is not None:
                self.mc.close()
            self.mc = None
            raise RuntimeError(f"Unable to initialize Pro450: {exc}") from exc

    def _publish_real_snapshot(self, angles_deg, gripper_value):
        snapshot = JointState()
        snapshot.header.stamp = self.get_clock().now().to_msg()
        stamps = [self.mc.sample_time(kind) for kind in ('arm', 'gripper')]
        if all(stamp is not None for stamp in stamps):
            wall = min(stamp[1] for stamp in stamps)
            seconds = int(wall)
            snapshot.header.stamp = Time(sec=seconds, nanosec=int((wall - seconds) * 1e9))
        token = getattr(self.mc, "owner_token", "")
        snapshot.header.frame_id = ("pro450:handoff:" if self.startup_read_only else "pro450:retain:") + token
        snapshot.name = list(COMMAND_JOINTS)
        snapshot.position = [math.radians(value) for value in angles_deg] + [
            float(gripper_value) / 100.0
        ]
        self.real_snapshot_pub.publish(snapshot)

    def _snapshot_consumed_cb(self, msg):
        if self.mode != 2 or not self.startup_read_only or self.mc is None:
            return
        if msg.data != getattr(self.mc, "owner_token", ""):
            return
        self.mc.close()
        ack = String()
        ack.data = msg.data
        self._handoff_ack.publish(ack)
        if self._handoff_timer is None:
            self.get_logger().info("Startup snapshot consumed; robot connection released.")
            self._handoff_timer = self.create_timer(1.0, lambda: rclpy.try_shutdown())

    def _refresh_read_only_snapshot(self):
        """Refresh the latched pose without issuing any hardware write command."""
        if self.mc is None or self._real_motion_enabled or self._handoff_timer is not None:
            return
        try:
            moving = self.mc.is_moving()
            if moving != 0:
                self.get_logger().warning(
                    "Real pose snapshot not refreshed because the Pro450 is moving."
                )
                return
            angles = self._validate_real_angles(self.mc.get_angles())
            gripper_value = self.mc.get_pro_gripper_angle()
            if isinstance(gripper_value, bool) or not isinstance(
                gripper_value, (int, float)
            ):
                raise RuntimeError(f"invalid force-gripper angle: {gripper_value!r}")
            gripper_value = float(gripper_value)
            if gripper_value < 0:
                raise RuntimeError(
                    f"force-gripper read failed: SDK returned {gripper_value}; "
                    "no valid startup snapshot will be published")
            if not math.isfinite(gripper_value) or not 0.0 <= gripper_value <= 100.0:
                raise RuntimeError(f"invalid force-gripper angle: {gripper_value!r}")
            self._publish_real_snapshot(angles, gripper_value)
        except Exception as exc:
            self.get_logger().error(f"Read-only Pro450 snapshot refresh failed: {exc}")

    @staticmethod
    def _validate_real_angles(angles):
        if not isinstance(angles, (list, tuple)) or len(angles) != len(ARM_JOINTS):
            raise RuntimeError(f"invalid joint angle response: {angles!r}")
        result = []
        for name, raw_value, limits in zip(ARM_JOINTS, angles, JOINT_LIMITS_RAD):
            if isinstance(raw_value, bool) or not isinstance(raw_value, (int, float)):
                raise RuntimeError(f"invalid {name} angle: {raw_value!r}")
            value = float(raw_value)
            if not math.isfinite(value):
                raise RuntimeError(f"invalid {name} angle: {raw_value!r}")
            lower_deg = math.degrees(limits[0])
            upper_deg = math.degrees(limits[1])
            if not lower_deg <= value <= upper_deg:
                raise RuntimeError(
                    f"{name}={value:.4f} deg is outside [{lower_deg}, {upper_deg}]"
                )
            result.append(value)
        return result

    def _feedback_cb(self, msg):
        values = dict(zip(msg.name, msg.position))
        if not all(name in values for name in COMMAND_JOINTS):
            return
        positions = [float(values[name]) for name in COMMAND_JOINTS]
        velocities = dict(zip(msg.name, msg.velocity)) if len(msg.velocity) == len(msg.name) else {}
        feedback_valid = all(math.isfinite(value) for value in positions) and all(
            name in velocities and math.isfinite(float(velocities[name]))
            for name in COMMAND_JOINTS
        )
        with self._state_lock:
            self._current_positions = positions
            self._feedback_time = time.monotonic()
            self._feedback_valid = feedback_valid

    def _target_cb(self, msg):
        if self.mode == 2 and not self._real_motion_enabled:
            return
        self._handle_target(msg, force_collision=False)

    def _force_target_cb(self, msg):
        if self.mode != 1:
            self._publish_status(
                "Rejected: collision override is disabled in Real Robot + Gazebo mode."
            )
            return
        self._handle_target(msg, force_collision=True)

    def _handle_target(self, msg, force_collision):
        if "teleop_keyboard_gazebo" in self.get_node_names():
            self._publish_status("Rejected: keyboard controller is running.")
            return
        values = dict(zip(msg.name, msg.position))
        if not all(name in values for name in COMMAND_JOINTS):
            self._publish_status("Rejected: target message is missing one or more joints.")
            return

        target = [float(values[name]) for name in COMMAND_JOINTS]
        if not all(math.isfinite(value) for value in target):
            self._publish_status("Rejected: target contains NaN or infinity.")
            return

        for name, value, limits in zip(COMMAND_JOINTS, target, JOINT_LIMITS_RAD):
            if not limits[0] <= value <= limits[1]:
                self._publish_status(
                    f"Rejected: {name}={value:.4f} rad is outside its limits."
                )
                return

        speed_scale = 0.2
        if msg.velocity:
            finite_speeds = [abs(value) for value in msg.velocity if math.isfinite(value)]
            if finite_speeds:
                speed_scale = max(finite_speeds)
        speed_scale = max(0.01, min(1.0, speed_scale))
        if force_collision:
            speed_scale = min(speed_scale, FORCE_EXECUTE_MAX_SPEED_SCALE)

        if self.mode == 2:
            self._queue_real_motion(target, speed_scale)
            return

        with self._state_lock:
            if self._command_active:
                self._publish_status("Rejected: a command is already being validated/executed.")
                return
            current = (
                list(self._current_positions)
                if self._current_positions is not None
                else None
            )
            feedback_age = time.monotonic() - self._feedback_time
            feedback_valid = self._feedback_valid
            self._command_active = True
            self._stop_requested = False
            self._stop_event.clear()

        self._publish_command_busy()
        if (
            current is None
            or feedback_age > 1.0
            or not feedback_valid
        ):
            with self._state_lock:
                self._command_active = False
            self._publish_status(
                "Rejected: Gazebo position/velocity feedback is missing, stale, or contains NaN. Restart/reset simulation."
            )
            return

        threading.Thread(
            target=self._validate_and_execute,
            args=(current, target, speed_scale, force_collision),
            daemon=True,
        ).start()

    def _validate_and_execute(self, current, target, speed_scale, force_collision):
        try:
            if force_collision:
                self._publish_status(
                    "WARNING: simulation-only collision override active; "
                    "MoveIt collision validation is being skipped."
                )
            else:
                self._publish_status("Validating interpolated path for collisions...")
                valid, reason = self._path_is_valid(current, target)
                if not valid:
                    self._publish_status(f"Rejected: {reason}")
                    return

            duration = self._trajectory_duration(current, target, speed_scale)
            if self._stop_requested:
                self._publish_status("Cancelled before execution.")
                return

            self._publish_trajectory(target, duration)

            execution_kind = "FORCE SIMULATION" if force_collision else "Executing"
            self._publish_status(
                f"{execution_kind} at {speed_scale * 100:.0f}% speed; "
                f"planned duration {duration:.2f} s."
            )
            if self._stop_event.wait(duration):
                self._publish_status("Execution stopped.")
            else:
                self._publish_status("Execution complete.")
        except Exception as exc:
            self.get_logger().error(f"Slider command failed: {exc}")
            self._publish_status(f"Execution failed: {exc}")
        finally:
            with self._state_lock:
                self._command_active = False
            self._publish_command_busy()

    def _path_is_valid(self, current, target):
        floor_ready, floor_reason = self._ensure_floor_collision_scene()
        if not floor_ready:
            return False, floor_reason

        if not self.validity_client.wait_for_service(timeout_sec=2.0):
            return False, "MoveIt /check_state_validity service is unavailable."

        max_delta = max(abs(b - a) for a, b in zip(current, target))
        sample_count = max(
            2, min(MAX_COLLISION_SAMPLES, math.ceil(max_delta / COLLISION_SAMPLE_STEP))
        )

        current_valid, previous_contacts, reason = self._state_is_valid(current)
        if reason:
            return False, reason
        escaping_collision = not current_valid
        initial_contacts = dict(previous_contacts)

        for index in range(1, sample_count + 1):
            if self._stop_requested:
                return False, "STOP requested."
            ratio = index / sample_count
            sample = [a + (b - a) * ratio for a, b in zip(current, target)]

            valid, contacts, reason = self._state_is_valid(sample)
            if reason:
                return False, reason
            if valid:
                escaping_collision = False
                previous_contacts = {}
                continue

            if escaping_collision and self._collision_is_decreasing(
                initial_contacts,
                previous_contacts,
                contacts,
            ):
                previous_contacts = contacts
                continue

            return False, self._format_collision_reason(ratio, contacts)

        if escaping_collision:
            return False, (
                "target remains in collision; move farther along a path that "
                "fully exits the existing contact."
            )

        return True, "valid"

    def _state_is_valid(self, positions):
        """Return validity and maximum penetration depth for every body pair."""

        request = GetStateValidity.Request()
        request.robot_state.is_diff = True
        request.robot_state.joint_state.name = list(COMMAND_JOINTS)
        request.robot_state.joint_state.position = positions
        # Empty group checks the entire robot, including the separate gripper.
        request.group_name = ""

        future = self.validity_client.call_async(request)
        deadline = time.monotonic() + 2.0
        while not future.done() and time.monotonic() < deadline:
            if self._stop_requested:
                return False, {}, "STOP requested."
            time.sleep(0.01)
        if not future.done():
            return False, {}, "MoveIt state-validity check timed out."
        response = future.result()
        if response is None:
            return False, {}, "MoveIt state-validity check failed."
        if response.valid:
            return True, {}, ""

        contacts = {}
        for contact in response.contacts:
            if contact.depth <= self.collision_depth_tolerance_m:
                continue
            pair = tuple(sorted((contact.contact_body_1, contact.contact_body_2)))
            contacts[pair] = max(contacts.get(pair, 0.0), contact.depth)

        # FCL can report tiny contacts at coincident triangle surfaces. Ignore
        # only the configured numerical tolerance. An invalid response without
        # contacts can be a constraint failure and must remain a hard failure.
        if response.contacts and not contacts:
            return True, {}, ""
        if not contacts:
            return False, {}, "MoveIt reported an invalid state without contacts."
        return False, contacts, ""

    def _collision_is_decreasing(self, initial, previous, current):
        if not set(current).issubset(initial):
            return False
        tolerance = self.collision_depth_tolerance_m
        for pair, depth in current.items():
            if depth > initial[pair] + tolerance:
                return False
            if depth > previous.get(pair, 0.0) + tolerance:
                return False
        return True

    @staticmethod
    def _format_collision_reason(ratio, contacts):
        if not contacts:
            if SHOW_COLLISION_PATH_PERCENT:
                return f"invalid state at {ratio * 100:.0f}% of path."
            return "invalid state."

        pair, depth = max(contacts.items(), key=lambda item: item[1])
        summary = "collision detected"
        if SHOW_COLLISION_PATH_PERCENT:
            summary += f" at {ratio * 100:.0f}% of path"

        details = []
        if SHOW_COLLISION_BODIES:
            details.append(f"{pair[0]} vs {pair[1]}")
        if SHOW_COLLISION_DEPTH:
            details.append(f"depth {depth * 1000.0:.3f} mm")
        return f"{summary} ({', '.join(details)})." if details else f"{summary}."

    def _wait_for_future(self, future, timeout_sec, operation):
        deadline = time.monotonic() + timeout_sec
        while not future.done() and time.monotonic() < deadline:
            if self._stop_requested:
                return None, "STOP requested."
            time.sleep(0.01)
        if not future.done():
            return None, f"{operation} timed out."
        try:
            response = future.result()
        except Exception as exc:
            return None, f"{operation} failed: {exc}"
        if response is None:
            return None, f"{operation} failed."
        return response, ""

    @staticmethod
    def _ensure_acm_name(acm, name):
        """Add a symmetric row/column to an AllowedCollisionMatrix."""
        if name in acm.entry_names:
            return
        old_size = len(acm.entry_names)
        while len(acm.entry_values) < old_size:
            acm.entry_values.append(AllowedCollisionEntry())
        for row in acm.entry_values[:old_size]:
            if len(row.enabled) < old_size:
                row.enabled.extend([False] * (old_size - len(row.enabled)))
            elif len(row.enabled) > old_size:
                del row.enabled[old_size:]
            row.enabled.append(False)
        new_row = AllowedCollisionEntry()
        new_row.enabled = [False] * (old_size + 1)
        acm.entry_names.append(name)
        acm.entry_values.append(new_row)

    def _ensure_floor_collision_scene(self):
        if not self.get_scene_client.wait_for_service(timeout_sec=3.0):
            return False, "MoveIt /get_planning_scene service is unavailable."
        if not self.apply_scene_client.wait_for_service(timeout_sec=3.0):
            return False, "MoveIt /apply_planning_scene service is unavailable."

        get_request = GetPlanningScene.Request()
        get_request.components.components = (
            PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        )
        get_future = self.get_scene_client.call_async(get_request)
        get_response, reason = self._wait_for_future(
            get_future, 3.0, "Reading MoveIt allowed-collision matrix"
        )
        if get_response is None:
            return False, reason

        scene = PlanningScene()
        scene.is_diff = True
        scene.allowed_collision_matrix = get_response.scene.allowed_collision_matrix
        acm = scene.allowed_collision_matrix
        # Historical 'Never' exclusions hid gripper-to-wrist contacts. Keep
        # mechanical attachment pairs; enable the movable fingers vs wrist.
        fingers = {f"gripper_{side}{i}" for side in ('left', 'right') for i in (1, 2, 3)}
        for i, left in enumerate(acm.entry_names):
            for j, right in enumerate(acm.entry_names):
                if ((left in fingers and right in ('link5', 'link6')) or
                        (right in fingers and left in ('link5', 'link6'))):
                    acm.entry_values[i].enabled[j] = False
        self._ensure_acm_name(acm, "base")
        self._ensure_acm_name(acm, FLOOR_OBJECT_ID)

        base_index = acm.entry_names.index("base")
        floor_index = acm.entry_names.index(FLOOR_OBJECT_ID)
        acm.entry_values[base_index].enabled[floor_index] = True
        acm.entry_values[floor_index].enabled[base_index] = True
        acm.entry_values[floor_index].enabled[floor_index] = True

        floor = CollisionObject()
        floor.header.frame_id = self.floor_frame
        floor.id = FLOOR_OBJECT_ID
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [
            self.floor_size_m,
            self.floor_size_m,
            self.floor_thickness_m,
        ]
        pose = Pose()
        pose.orientation.w = 1.0
        # The default collision surface matches Gazebo z=0.  A configured
        # positive clearance deliberately turns it into a safety margin.
        pose.position.z = self.floor_clearance_m - self.floor_thickness_m / 2.0
        floor.primitives = [primitive]
        floor.primitive_poses = [pose]
        floor.operation = CollisionObject.ADD
        scene.world.collision_objects = [floor]

        apply_request = ApplyPlanningScene.Request()
        apply_request.scene = scene
        apply_future = self.apply_scene_client.call_async(apply_request)
        apply_response, reason = self._wait_for_future(
            apply_future, 3.0, "Applying MoveIt ground collision object"
        )
        if apply_response is None:
            return False, reason
        if not apply_response.success:
            return False, "MoveIt rejected the ground collision object."

        self.get_logger().info(
            f"Installed MoveIt ground safety object in '{self.floor_frame}' "
            f"with {self.floor_clearance_m * 1000.0:.0f} mm clearance; "
            "only link 'base' may contact it."
        )
        return True, "ready"

    @staticmethod
    def _trajectory_duration(current, target, speed_scale):
        durations = []
        for start, end, maximum in zip(current, target, MAX_VELOCITY_RAD_S):
            # joint_trajectory_controller uses spline interpolation.  With
            # zero endpoint velocity, peak speed is about 1.5x the average,
            # so size the duration for the peak rather than the average.
            durations.append(
                SPLINE_PEAK_VELOCITY_FACTOR
                * abs(end - start)
                / max(0.01, maximum * speed_scale)
            )
        return max(MIN_TRAJECTORY_DURATION, min(MAX_TRAJECTORY_DURATION, max(durations)))

    def _publish_trajectory(self, target, duration, velocities=None):
        seconds = int(duration)
        nanoseconds = int((duration - seconds) * 1_000_000_000)

        arm = JointTrajectory()
        # Keep the header stamp at zero: trajectory controllers interpret this
        # as "start now".  This avoids mixing wall time from this node with
        # Gazebo simulation time used by controller_manager.
        arm.joint_names = list(ARM_JOINTS)
        arm_point = JointTrajectoryPoint()
        arm_point.positions = list(target[:6])
        if velocities is not None:
            arm_point.velocities = list(velocities[:6])
        arm_point.time_from_start = Duration(sec=seconds, nanosec=nanoseconds)
        arm.points = [arm_point]
        self.pub_arm.publish(arm)

        self._publish_gripper_trajectory(target[6], duration, None if velocities is None else velocities[6])

    def _publish_gripper_trajectory(self, position, duration, velocity=None):
        seconds = int(duration)
        nanoseconds = int((duration - seconds) * 1_000_000_000)
        gripper = JointTrajectory()
        gripper.joint_names = [GRIPPER_JOINT]
        gripper_point = JointTrajectoryPoint()
        gripper_point.positions = [position]
        if velocity is not None:
            gripper_point.velocities = [velocity]
        gripper_point.time_from_start = Duration(sec=seconds, nanosec=nanoseconds)
        gripper.points = [gripper_point]
        self.pub_gripper.publish(gripper)

    def _queue_real_motion(self, target, speed_scale):
        with self._state_lock:
            if self._command_active:
                self._publish_status("Rejected: a command is already being validated/executed.")
                return
            self._command_active = True
            self._stop_requested = False
            self._stop_event.clear()
        self._publish_command_busy()
        try:
            self.command_queue.put_nowait((list(target), speed_scale))
        except queue.Full:
            with self._state_lock:
                self._command_active = False
            self._publish_status("Rejected: a command is already being validated/executed.")

    def _read_real_positions(self, moving=None):
        pose = self._pose_reader(moving=self._arm_moving if moving is None else moving)
        self._on_real_sample(pose)
        return pose

    def _gazebo_matches(self, positions):
        """True when Gazebo is showing this measured pose within the tolerance."""
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            if self._stop_requested:
                return False
            with self._state_lock:
                gazebo = (
                    list(self._current_positions)
                    if self._current_positions is not None else None
                )
                age = time.monotonic() - self._feedback_time
                valid = self._feedback_valid
            if (gazebo is not None and valid and age <= 1.0 and
                    max(abs(left - right) for left, right in zip(gazebo, positions))
                    <= REAL_POSE_TOLERANCE_RAD):
                return True
            self._read_real_positions(moving=True)
            self._stop_event.wait(0.1)
        return False

    def _stop_real_motion(self):
        """SDK stop on the owner thread, then show the measured pose."""
        self._gripper_model.hold()
        if self.mc is not None and self._motion_sent:
            self.mc.stop()
            # Arm STOP does not stop the independent gripper motor.
            try:
                opening = self._pose_reader.read_fresh_gripper()
                self._send_gripper(int(round(opening * 100.0)))
            except Exception as exc:
                self.get_logger().error(f"Could not hold gripper at measured opening: {exc}")
        self._motion_sent = False
        try:
            self._read_real_positions()
        except Exception as exc:
            self.get_logger().error(f"Could not mirror the stopped pose: {exc}")

    def _read_real_arm(self):
        return self._read_real_positions()[:6]

    def _send_gripper(self, opening):
        result = self.mc.set_pro_gripper_angle(opening)
        if not gripper_write_accepted(result):
            raise RuntimeError(f"gripper returned {result!r}")

    def _follow_real_line(self, current, target, timeout):
        """Watch one firmware move and stop it if it leaves the checked line.

        Returns True on arrival, False on STOP. Raises on deviation, bad
        feedback, or timeout; the caller stops the arm.
        """
        start_arm, goal_arm = current[:6], target[:6]
        delta = [goal - start for start, goal in zip(start_arm, goal_arm)]
        length_sq = sum(value * value for value in delta)
        arm_moves = length_sq > WAYPOINT_ARM_TOLERANCE_RAD ** 2
        grip_goal = target[6]
        deadline = time.monotonic() + timeout
        bad_reads = 0
        self._arm_moving = True
        try:
            while rclpy.ok():
                if self._stop_requested:
                    return False
                try:
                    arm = self._read_real_positions(moving=True)[:6]
                except Exception:
                    bad_reads += 1
                    if bad_reads >= 3:
                        raise
                    time.sleep(0.05)
                    continue
                bad_reads = 0
                if arm_moves:
                    progress = sum(
                        (value - start) * step
                        for value, start, step in zip(arm, start_arm, delta)) / length_sq
                    progress = max(0.0, min(1.0, progress))
                    deviation = max(
                        abs(value - (start + step * progress))
                        for value, start, step in zip(arm, start_arm, delta))
                    if deviation > LINE_DEVIATION_LIMIT_RAD:
                        raise RuntimeError(
                            f"left the checked path by {math.degrees(deviation):.2f} deg")
                else:
                    progress = 1.0
                arm_done = all(
                    abs(value - goal) <= WAYPOINT_ARM_TOLERANCE_RAD
                    for value, goal in zip(arm, goal_arm))
                estimate = self._gripper_model.sample()
                gripper_done = estimate is None or estimate[2]
                if arm_done and gripper_done and (not arm_moves or self.mc.is_moving() == 0):
                    try:
                        self._pose_reader.read_fresh_gripper()
                        measured = self._read_real_positions(moving=False)
                    except Exception:
                        measured = None
                    if (measured is not None and
                            abs(measured[6] - grip_goal) <= WAYPOINT_GRIPPER_TOLERANCE):
                        return True
                if time.monotonic() >= deadline:
                    raise RuntimeError("timed out waiting for the real robot to arrive")
        finally:
            self._arm_moving = False
        return False

    def _validate_real_motion(self, current, target):
        """ROS collision calls run separately; only this caller touches SDK.

        The owner continues six-axis polling with cached gripper values while
        the validation thread waits for MoveIt. No writes occur in that thread.
        """
        finished = threading.Event()
        result = {}

        def validate():
            try:
                self._publish_status("Validating interpolated path for collisions...")
                valid, reason = self._path_is_valid(current, target)
                result['value'] = valid, reason
            except Exception as exc:
                result['error'] = exc
            finally:
                finished.set()

        worker = threading.Thread(target=validate, daemon=True)
        worker.start()
        failure = None
        next_read = time.monotonic()
        while not finished.wait(0.02):
            if not rclpy.ok():
                self._stop_requested = True
                self._stop_event.set()
            if not self._stop_requested and failure is None and time.monotonic() >= next_read:
                try:
                    self._read_real_positions(moving=True)
                except Exception as exc:
                    failure = exc
                    # Cancel pending ROS scans before returning control to the
                    # owner loop; an old scan must never overlap a new command.
                    self._stop_requested = True
                    self._stop_event.set()
                next_read = time.monotonic() + 0.1
        worker.join()
        if failure is not None:
            raise failure
        if 'error' in result:
            raise result['error']
        if self._stop_requested:
            return False, "STOP requested."
        return result['value']

    def _confirm_gripper_speed(self, speed):
        """Retry transient speed reads before sending any movement command."""
        if self._stop_requested:
            return False
        write_result = self.mc.set_pro_gripper_speed(speed)
        readings = []
        for attempt in range(3):
            if self._stop_requested:
                return False
            try:
                value = self.mc.get_pro_gripper_speed()
            except Exception as exc:
                value = f"{type(exc).__name__}: {exc}"
            readings.append(value)
            if self._stop_requested:
                return False
            valid = (not isinstance(value, bool) and isinstance(value, (int, float))
                     and math.isfinite(value) and 1 <= value <= 100 and value == int(value))
            if valid and value == speed:
                return True
            if attempt < 2:
                self._publish_status(
                    f"Confirming gripper speed {speed}: retry {attempt + 1}/2...")
                # Keep arm feedback current; this uses the cached gripper angle.
                self._read_real_positions(moving=True)
                if self._stop_event.wait(0.1):
                    return False
        raise RuntimeError(
            f"gripper speed could not be confirmed after 3 reads: "
            f"requested={speed}, set_result={write_result!r}, readbacks={readings!r}")

    def _run_real_motion(self, target, speed_scale):
        if self._stop_requested:
            self._publish_status("Cancelled before execution.")
            return
        if "teleop_keyboard_gazebo" in self.get_node_names():
            self._publish_status("Rejected: keyboard controller is running.")
            return
        self._pose_reader.cached_gripper()
        current = self._read_real_positions(moving=True)
        if not self._gazebo_matches(current):
            if self._stop_requested:
                self._publish_status("Cancelled before execution.")
            else:
                self._publish_status(
                    f"Rejected: Gazebo differs from the real robot by more than {REAL_POSE_TOLERANCE_RAD:.2f} rad."
                )
            return
        while not self._stop_requested:
            valid, reason = self._validate_real_motion(current, target)
            if not valid:
                self._publish_status(f"Rejected: {reason}")
                return
            confirmed = self._read_real_positions(moving=True)
            changed = max(abs(a-b) for a,b in zip(confirmed,current)) > 0.01
            current = confirmed
            if not changed:
                break
            self._publish_status("Start pose changed; revalidating before motion...")
        if self._stop_requested:
            self._publish_status("Cancelled before execution.")
            return
        duration = self._trajectory_duration(current, target, speed_scale)
        grip_start = int(round(current[6] * 100.0))
        grip_goal = int(round(target[6] * 100.0))
        grip_speed = gripper_speed_for_duration(grip_start, grip_goal, duration)
        mean_arm_speed = max(abs(b-a) for a,b in zip(current[:6], target[:6])) / duration
        speed = max(1, sdk_speed_for_rad(mean_arm_speed))
        if self._stop_requested:
            self._publish_status("Cancelled before execution.")
            return
        self._publish_status(
            f"Executing at {speed_scale * 100:.0f}% speed; "
            f"planned duration {duration:.2f} s."
        )
        if grip_speed is not None:
            if not self._confirm_gripper_speed(grip_speed):
                self._publish_status("Cancelled before execution.")
                return
            rate = measured_gripper_rate(grip_speed, 'opening' if grip_goal > grip_start else 'closing')
            self._motion_sent = True
            self._gripper_model.start(current[6], grip_goal / 100.0, rate)
            self._send_gripper(grip_goal)
        if self._stop_requested:
            if self._motion_sent:
                self._stop_real_motion()
            self._publish_status("Cancelled before execution.")
            return
        arm_deg = [math.degrees(value) for value in target[:6]]
        arm_moves = max(abs(a-b) for a,b in zip(current[:6], target[:6])) > WAYPOINT_ARM_TOLERANCE_RAD
        self._motion_sent = True
        result = self.mc.send_angles(arm_deg, speed, _async=True) if arm_moves else 1
        if not sdk_motion_accepted(result):
            self._stop_real_motion()
            self._publish_status(
                f"Real robot command failed: send_angles returned {result!r}")
            return
        if not self._follow_real_line(
                current, target, duration * 2.0 + REAL_ARRIVAL_MARGIN_S):
            self._stop_real_motion()
            self._publish_status("Execution stopped.")
            return
        self._motion_sent = False
        self._publish_status("Execution complete.")

    def _real_robot_worker(self):
        next_mirror = 0.0
        while rclpy.ok():
            if self._stop_requested and self._motion_sent:
                try:
                    self._stop_real_motion()
                    self._publish_status("Execution stopped.")
                except Exception as exc:
                    self._publish_status(f"Real robot STOP failed: {exc}")
                continue
            try:
                target, speed_scale = self.command_queue.get(timeout=0.1)
            except queue.Empty:
                now = time.monotonic()
                if now >= next_mirror:
                    try:
                        self._read_real_positions()
                    except Exception as exc:
                        if now - self._last_read_error >= 2.0:
                            self.get_logger().warning(f"Real feedback unavailable: {exc}")
                            self._last_read_error = now
                    next_mirror = now + 0.25
                continue
            try:
                self._run_real_motion(target, speed_scale)
            except Exception as exc:
                self._publish_status(f"Real robot command failed: {exc}")
                try:
                    self._stop_real_motion()
                except Exception:
                    self._motion_sent = False
            finally:
                with self._state_lock:
                    self._command_active = False
                self._publish_command_busy()

    def _stop_cb(self, _msg):
        self._stop_requested = True
        self._stop_event.set()
        if self.mode != 2:
            with self._state_lock:
                current = (
                    list(self._current_positions)
                    if self._current_positions is not None
                    else None
                )
                feedback_valid = self._feedback_valid
            if (
                feedback_valid
                and current is not None
                and all(math.isfinite(value) for value in current)
            ):
                self._publish_trajectory(current, MIN_TRAJECTORY_DURATION)
            self._publish_status(
                "STOP applied; controller commanded to hold current position."
            )
            return
        if not self._real_motion_enabled:
            self._publish_status(
                "Simulation STOP applied; real robot remains READ ONLY and received no command."
            )
            return
        self._publish_status(
            "STOP applied; controller commanded to hold current position."
        )


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SliderControl()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f"Failed to start slider controller: {exc}")
    finally:
        if node is not None:
            if node._real_thread is not None:
                node._stop_requested = True
                node._stop_event.set()
                node._real_thread.join(timeout=2.0)
            if node._mirror_thread is not None:
                node._mirror_stop.set()
                node._mirror_thread.join(timeout=0.5)
            if node.mc is not None and node._motion_sent:
                try:
                    # Stop motion but keep servo torque enabled; releasing all
                    # servos on process exit could let a loaded arm fall.
                    node.mc.stop()
                except Exception:
                    pass
            if node.mc is not None and hasattr(node.mc, "close"):
                node.mc.close()
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
