#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Validated Pro450 slider-command bridge for Gazebo and optional real hardware."""

import math
import queue
import threading
import time

import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point, Pose
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
from std_msgs.msg import Empty, String
from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from pymycobot import Pro450Client


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
# Collision validation still computes the first collision point and penetration
# depth. These flags only control how much diagnostic detail is exposed in the
# operator-facing status text, so the previous wording can be restored easily.
SHOW_COLLISION_PATH_PERCENT = False
SHOW_COLLISION_DEPTH = False


def estimate_end_effector_height(j2_deg, j3_deg, j4_deg):
    j2 = math.radians(j2_deg)
    j3 = math.radians(j3_deg)
    j4 = math.radians(j4_deg)
    angle3 = j2 + j3
    angle4 = angle3 + j4
    return (
        0.155
        + 0.048
        + 0.18 * math.cos(j2)
        + 0.1735 * math.cos(angle3)
        + 0.08 * math.cos(angle4)
        + 0.17 * math.cos(angle4)
    )


class SliderControl(Node):
    def __init__(self):
        super().__init__("slider_control_gazebo")

        self.declare_parameter("mode", "simulation")
        self.declare_parameter("pro450_ip", DEFAULT_PRO450_IP)
        self.declare_parameter("pro450_port", DEFAULT_PRO450_PORT)
        self.declare_parameter("startup_read_only", True)
        self.declare_parameter("startup_stable_samples", 5)
        self.declare_parameter("startup_sample_interval_sec", 0.2)
        self.declare_parameter("startup_stable_tolerance_deg", 0.2)
        self.declare_parameter("minimum_real_height_mm", 170.0)
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
        self.minimum_real_height_mm = float(
            self.get_parameter("minimum_real_height_mm").value
        )
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
        self.status_pub = self.create_publisher(String, "/pro450/slider_status", 10)
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
        self.create_subscription(Point, "/pro450/end_effector_coords", self._coords_cb, 10)

        self.validity_client = self.create_client(
            GetStateValidity, "/check_state_validity"
        )
        self.get_scene_client = self.create_client(
            GetPlanningScene, "/get_planning_scene"
        )
        self.apply_scene_client = self.create_client(
            ApplyPlanningScene, "/apply_planning_scene"
        )

        self._state_lock = threading.Lock()
        self._current_positions = None
        self._feedback_time = 0.0
        self._feedback_valid = False
        self._coords = None
        self._command_active = False
        self._stop_requested = False
        self._stop_event = threading.Event()

        self.mc = None
        self._real_motion_enabled = self.mode == 2 and not self.startup_read_only
        self.command_queue = queue.Queue(maxsize=1)
        if self.mode == 2:
            self._initialize_pro450()
            if self._real_motion_enabled:
                threading.Thread(target=self._real_robot_worker, daemon=True).start()
                threading.Thread(target=self._height_monitor, daemon=True).start()
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

    def _initialize_pro450(self):
        try:
            self.get_logger().info(
                f"Connecting to Pro450 @ {self.pro450_ip}:{self.pro450_port}"
            )
            self.mc = Pro450Client(self.pro450_ip, self.pro450_port)
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

            gripper_value = self.mc.get_pro_gripper_angle()
            if isinstance(gripper_value, bool) or not isinstance(
                gripper_value, (int, float)
            ):
                raise RuntimeError(f"invalid force-gripper angle: {gripper_value!r}")
            gripper_value = float(gripper_value)
            if not math.isfinite(gripper_value) or not 0.0 <= gripper_value <= 100.0:
                raise RuntimeError(f"invalid force-gripper angle: {gripper_value!r}")

            averaged_angles = [
                sum(sample[index] for sample in samples) / len(samples)
                for index in range(len(ARM_JOINTS))
            ]
            self._publish_real_snapshot(averaged_angles, gripper_value)
            self.get_logger().info(
                "Pro450 connected in read-only startup phase. Stable angles: "
                f"{averaged_angles}; gripper={gripper_value:.1f}."
            )
        except Exception as exc:
            self.mc = None
            raise RuntimeError(f"Unable to initialize Pro450: {exc}") from exc

    def _publish_real_snapshot(self, angles_deg, gripper_value):
        snapshot = JointState()
        snapshot.header.stamp = self.get_clock().now().to_msg()
        snapshot.name = list(COMMAND_JOINTS)
        snapshot.position = [math.radians(value) for value in angles_deg] + [
            float(gripper_value) / 100.0
        ]
        self.real_snapshot_pub.publish(snapshot)

    def _refresh_read_only_snapshot(self):
        """Refresh the latched pose without issuing any hardware write command."""
        if self.mc is None or self._real_motion_enabled:
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

    def _coords_cb(self, msg):
        with self._state_lock:
            self._coords = (float(msg.x), float(msg.y), float(msg.z))

    def _target_cb(self, msg):
        if self.mode == 2 and not self._real_motion_enabled:
            self._publish_status(
                "Rejected: real robot startup is READ ONLY; motion implementation is deferred."
            )
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

            if self.mode == 2:
                target_deg = [math.degrees(value) for value in target[:6]]
                target_height_mm = 1000.0 * estimate_end_effector_height(
                    target_deg[1], target_deg[2], target_deg[3]
                )
                if target_height_mm < self.minimum_real_height_mm:
                    self._publish_status(
                        f"Rejected: estimated real end height {target_height_mm:.1f} mm "
                        f"is below {self.minimum_real_height_mm:.1f} mm."
                    )
                    return

            duration = self._trajectory_duration(current, target, speed_scale)
            if self._stop_requested:
                self._publish_status("Cancelled before execution.")
                return

            self._publish_trajectory(target, duration)

            if self.mode == 2:
                robot_speed = max(1, min(100, round(speed_scale * 100.0)))
                self._replace_real_command(target, robot_speed)

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
        request.group_name = "arm"

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

        details = [f"{pair[0]} vs {pair[1]}"]
        if SHOW_COLLISION_DEPTH:
            details.append(f"depth {depth * 1000.0:.3f} mm")
        return f"{summary} ({', '.join(details)})."

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

    def _publish_trajectory(self, target, duration):
        seconds = int(duration)
        nanoseconds = int((duration - seconds) * 1_000_000_000)

        arm = JointTrajectory()
        # Keep the header stamp at zero: trajectory controllers interpret this
        # as "start now".  This avoids mixing wall time from this node with
        # Gazebo simulation time used by controller_manager.
        arm.joint_names = list(ARM_JOINTS)
        arm_point = JointTrajectoryPoint()
        arm_point.positions = list(target[:6])
        arm_point.time_from_start = Duration(sec=seconds, nanosec=nanoseconds)
        arm.points = [arm_point]
        self.pub_arm.publish(arm)

        gripper = JointTrajectory()
        gripper.joint_names = [GRIPPER_JOINT]
        gripper_point = JointTrajectoryPoint()
        gripper_point.positions = [target[6]]
        gripper_point.time_from_start = Duration(sec=seconds, nanosec=nanoseconds)
        gripper.points = [gripper_point]
        self.pub_gripper.publish(gripper)

    def _replace_real_command(self, target, speed):
        command = ([math.degrees(value) for value in target[:6]], target[6], speed)
        try:
            self.command_queue.get_nowait()
        except queue.Empty:
            pass
        self.command_queue.put_nowait(command)

    def _real_robot_worker(self):
        while rclpy.ok():
            try:
                arm_deg, gripper_rad, speed = self.command_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            if self.mc is None or self._stop_requested:
                continue
            try:
                self.mc.send_angles(arm_deg, speed)
                gripper_value = max(0, min(100, round(gripper_rad * 100.0)))
                # Pro450Client's force-gripper API takes the requested 0..100
                # opening value; unlike the serial gripper API it does not
                # require a Modbus gripper id here.
                self.mc.set_pro_gripper_angle(gripper_value)
            except Exception as exc:
                self._publish_status(f"Real robot command failed: {exc}")

    def _stop_cb(self, _msg):
        self._stop_requested = True
        self._stop_event.set()
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
        if self.mc is not None and self._real_motion_enabled:
            try:
                self.mc.stop()
            except Exception as exc:
                self.get_logger().error(f"Real robot STOP failed: {exc}")
        if self.mode == 2 and not self._real_motion_enabled:
            self._publish_status(
                "Simulation STOP applied; real robot remains READ ONLY and received no command."
            )
        else:
            self._publish_status(
                "STOP applied; controller commanded to hold current position."
            )

    def _height_monitor(self):
        while rclpy.ok():
            with self._state_lock:
                coords = self._coords
            if coords is not None and coords[2] < self.minimum_real_height_mm:
                self._stop_requested = True
                self._stop_event.set()
                if self.mc is not None:
                    try:
                        self.mc.stop()
                    except Exception as exc:
                        self.get_logger().error(f"Automatic height STOP failed: {exc}")
            time.sleep(0.05)


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
            if node.mc is not None and node._real_motion_enabled:
                try:
                    # Stop motion but keep servo torque enabled; releasing all
                    # servos on process exit could let a loaded arm fall.
                    node.mc.stop()
                except Exception:
                    pass
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
