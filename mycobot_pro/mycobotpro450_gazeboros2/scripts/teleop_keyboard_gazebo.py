#!/usr/bin/env python3
"""Feedback-based, collision-checked Pro450 keyboard control."""

import math
import os
import select
import signal
import sys
import termios
import threading
import time
import tty

import rclpy
from builtin_interfaces.msg import Duration, Time
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene, GetStateValidity
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from pro450_feedback import attach_feedback, publish_feedback, publish_age
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from pro450_hold_profile import (
    PositionVelocityEstimator, collision_stop_angle, hold_setpoint, trapezoid_samples,
)
from pro450_real_keyboard import (
    RealKeyboardTransport, RealPoseReader, SDK_RAD_PER_SPEED, valid_gripper_position,
)
from pro450_real_mirror import RealPoseBuffer

# Borrow the existing Pro450 slider's collision implementation, not its node
# constructor. This keeps the same floor, contact tolerance and path sampling.
from slider_control_gazebo import (
    ARM_JOINTS, COMMAND_JOINTS, DEFAULT_COLLISION_DEPTH_TOLERANCE_M,
    DEFAULT_PRO450_IP, DEFAULT_PRO450_PORT, GRIPPER_JOINT,
    JOINT_LIMITS_RAD, SliderControl,
)

NORMAL_STEP = math.radians(1.0)
FAST_STEP = math.radians(5.0)
GRIPPER_STEP = 0.05
SPEED_SCALE = 0.10
MAX_VELOCITY = 1.0 * SPEED_SCALE
MAX_ACCELERATION = 0.2
# The real-robot step path still uses its separately tested integer speed.
# Do not infer a continuous-hold speed from the single 100 ~= 150 deg/s sample.
REAL_SPEED = 4
# Kept for the retired step path. A real gripper hold uses the same gear
# integers as the arm: 4, 8, 12, 16, 20.
REAL_GRIPPER_SPEED = 30
MIN_DURATION = 0.6
MAX_DURATION = 30.0
MAX_FEEDBACK_AGE = 1.0
SETTLE_TOLERANCE = math.radians(1.0)
MIRROR_TOLERANCE = math.radians(3.0)
HOLD_VALIDATION_ADVANCE = 0.55
# First check is short so motion starts quickly; later checks extend it.
HOLD_FIRST_VALIDATION_ADVANCE = 0.15
HOLD_VALIDATION_TRIGGER = 0.35
HOLD_COLLISION_MARGIN = 0.01
HOLD_MAX_ACCELERATION = 1.40
# One degree before the collision boundary. The gripper's 0..1 opening treats
# one degree as about 1.75 units, rounded to 2 units (0.02).
HOLD_STOP_MARGIN = math.radians(1.0)
HOLD_SCAN_GRID = math.radians(1.0)
HOLD_SCAN_FINE = math.radians(0.05)
HOLD_GRIPPER_STOP_MARGIN = 0.02
HOLD_GRIPPER_SCAN_GRID = 0.02
HOLD_GRIPPER_SCAN_FINE = 0.001
# Joint 4 calibration, median of SDK speeds 4, 8, 12, 16 and 20.
# Simulation uses this acceleration for the trapezoid to the scanned stop.
HOLD_TRACKED_ACCELERATION = 0.40
HOLD_GRIPPER_SPEED_GEARS = (0.20, 0.30, 0.40, 0.50, 0.60)
HOLD_GRIPPER_URDF_VELOCITY_LIMIT = 1.0
# Pro450 URDF limits J1-J6 to 1 rad/s. Keep the simulation hold gears below it.
HOLD_ARM_URDF_VELOCITY_LIMIT = 1.0
HOLD_SPEED_GEARS = tuple(SDK_RAD_PER_SPEED * value for value in (4, 8, 12, 16, 20))
DEFAULT_HOLD_SPEED_GEAR = 2
if max(HOLD_SPEED_GEARS) >= HOLD_ARM_URDF_VELOCITY_LIMIT:
    raise ValueError("Pro450 hold speed gears must stay below the URDF velocity limit")
if max(HOLD_GRIPPER_SPEED_GEARS) >= HOLD_GRIPPER_URDF_VELOCITY_LIMIT:
    raise ValueError("Pro450 gripper gears must stay below the URDF velocity limit")


class TeleopKeyboard(Node):
    _path_is_valid = SliderControl._path_is_valid
    _state_is_valid = SliderControl._state_is_valid
    _collision_is_decreasing = SliderControl._collision_is_decreasing
    _format_collision_reason = staticmethod(SliderControl._format_collision_reason)
    _wait_for_future = SliderControl._wait_for_future
    _ensure_acm_name = staticmethod(SliderControl._ensure_acm_name)
    _ensure_floor_collision_scene = SliderControl._ensure_floor_collision_scene

    def __init__(self):
        super().__init__("teleop_keyboard_gazebo")
        self.declare_parameter("mode", "simulation")
        self.declare_parameter("real_hold_enabled", True)
        self.declare_parameter("real_gripper_hold_enabled", True)
        self.real_hold_enabled = bool(self.get_parameter("real_hold_enabled").value)
        self.real_gripper_hold_enabled = bool(
            self.get_parameter("real_gripper_hold_enabled").value)
        self.real_transport = None
        self._real_thread = None
        self._real_mirror = None
        self._mirror_thread = None
        self._mirror_stop = threading.Event()
        self._sample_velocity = [0.0] * 7
        self.real_joint_pub = None
        self._mirror_positions = None
        self._mirror_time = 0.0
        self.mode = str(self.get_parameter("mode").value).strip().lower()
        if self.mode not in ("simulation", "real"):
            raise ValueError("mode must be simulation or real")

        self.pub_arm = self.create_publisher(
            JointTrajectory, "/arm_controller/joint_trajectory", 10)
        self.pub_gripper = self.create_publisher(
            JointTrajectory, "/pro_gripper_controller/joint_trajectory", 10)
        self.create_subscription(JointState, "/joint_states", self._feedback_cb, 10)
        self.validity_client = self.create_client(
            GetStateValidity, "/check_state_validity")
        self.get_scene_client = self.create_client(
            GetPlanningScene, "/get_planning_scene")
        self.apply_scene_client = self.create_client(
            ApplyPlanningScene, "/apply_planning_scene")
        self.floor_clearance_m = 0.0
        self.collision_depth_tolerance_m = DEFAULT_COLLISION_DEPTH_TOLERANCE_M
        self.floor_size_m = 20.0
        self.floor_thickness_m = 0.10
        self.floor_frame = "world"
        self._state_lock = threading.Lock()
        self._robot_lock = threading.Lock()
        self._positions = None
        self._velocities = None
        self._feedback_time = 0.0
        self._sim_velocity_estimator = PositionVelocityEstimator()
        self._sim_velocity_warning_emitted = False
        self._active = False
        self._worker = None
        self._armed = False
        self._stop_requested = False
        self.mc = None
        self._hold_lock = threading.Lock()
        self._hold_axis = None
        self._hold_pending = None
        self._hold_direction = 0
        self._hold_pressed = False
        self._hold_velocity = 0.0
        self._hold_validated_limit = None
        self._hold_validation_pending = False
        self._hold_first_validation = False
        self._hold_validation_worker = None
        self._hold_generation = 0
        self._hold_last_tick = time.monotonic()
        self._hold_speed_gear = DEFAULT_HOLD_SPEED_GEAR
        self._real_braking = False
        self._releasing_axis = None
        self._brake_since = 0.0
        self._hold_final_target = None
        self._hold_goal_sent = False

        if self.mode == "real":
            qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST, depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.snapshot_pub = self.create_publisher(
                JointState, "/pro450/real_state_snapshot", qos)
            self._connect_read_only()
            self._real_pose_reader = RealPoseReader(
                self.mc, JOINT_LIMITS_RAD, report=self.get_logger().debug)
            self._real_pose_reader.seed_gripper(self._startup_pose[6])
            if hasattr(self.mc, 'sample_time'):
                stamp = self.mc.sample_time('arm')
                if stamp is not None:
                    self._real_pose_reader.observe_arm(
                        [math.degrees(v) for v in self._startup_pose[:6]], *stamp)
            self.real_transport = RealKeyboardTransport(
                self.mc, self._real_pose_reader, self._mirror_real_pose, JOINT_LIMITS_RAD,
                report_error=self.get_logger().error)
            self._real_thread = threading.Thread(target=self.real_transport.run, daemon=True)
            self._real_mirror = RealPoseBuffer()
            self._sample_publish_lock = threading.Lock()
            self.real_joint_pub = self.create_publisher(
                JointState, "/pro450/real_joint_states", 1)
            self.real_age_pub = self.create_publisher(Float64MultiArray, "/pro450/real_feedback_age", 1)
            self._on_real_sample(self._startup_pose)
            attach_feedback(self, self._real_pose_reader)
            self._real_thread.start()
            self._mirror_thread = threading.Thread(target=self._mirror_loop, daemon=True)
            self._mirror_thread.start()
            self.get_logger().info("Real robot READ ONLY. Wait for Gazebo sync before arming.")
        else:
            self.get_logger().info("Simulation mode; no real connection or startup motion.")

    def _feedback_cb(self, msg):
        positions = dict(zip(msg.name, msg.position))
        if self.mode == "real":
            if all(name in positions and math.isfinite(positions[name])
                   for name in COMMAND_JOINTS):
                with self._state_lock:
                    self._mirror_positions = [positions[name] for name in COMMAND_JOINTS]
                    self._mirror_time = time.monotonic()
            return
        velocities = (dict(zip(msg.name, msg.velocity))
                      if len(msg.velocity) == len(msg.name) else {})
        if not all(name in positions and name in velocities for name in COMMAND_JOINTS):
            return
        p = [float(positions[name]) for name in COMMAND_JOINTS]
        v = [float(velocities[name]) for name in COMMAND_JOINTS]
        if not all(math.isfinite(value) for value in p + v):
            return
        if self.mode == "simulation":
            stamp = (float(msg.header.stamp.sec) +
                     float(msg.header.stamp.nanosec) / 1e9)
            if stamp <= 0.0:
                stamp = time.monotonic()
            observed = self._sim_velocity_estimator.update(stamp, p)
            if observed is None:
                return
            if (not self._sim_velocity_warning_emitted and
                    any(abs(reported - measured) > 0.02
                        for reported, measured in zip(v, observed))):
                self._sim_velocity_warning_emitted = True
                self.get_logger().warning(
                    "Gazebo velocity differs from position change; simulation "
                    "hold control will use position-derived speed.")
            v = observed
        with self._state_lock:
            self._positions, self._velocities = p, v
            self._feedback_time = time.monotonic()

    def _fresh_feedback(self):
        if self.mode == "real" and self.real_transport is not None:
            return self.real_transport.feedback()
        with self._state_lock:
            if (self._positions is None or
                    time.monotonic() - self._feedback_time > MAX_FEEDBACK_AGE):
                return None
            return list(self._positions), list(self._velocities)

    def _mirror_real_pose(self, pose):
        # Only a stationary measured pose may open a newly started launch gate.
        feedback = self.real_transport.feedback() if self.real_transport else None
        if feedback is None:
            return  # Never mirror a stale/unvalidated combined pose.
        if (feedback is not None and not self.real_transport.moving and
                all(abs(v) <= 0.01 for v in feedback[1])):
            self._publish_snapshot(pose)
        # Gazebo receives the measured pose through the delayed render thread.
        self._on_real_sample(pose)

    def arm_real(self):
        if self.mode != "real" or not self.real_hold_enabled:
            return "rejected: real_hold_enabled is false (read-only startup)"
        feedback = self._fresh_feedback()
        with self._state_lock:
            mirror = self._mirror_positions
            age = time.monotonic() - self._mirror_time
        if (feedback is None or mirror is None or age > 0.5 or
                max(abs(a - b) for a, b in zip(feedback[0], mirror)) > 0.01):
            return "rejected: fresh synchronized real/Gazebo feedback required"
        if "slider_control_gazebo" in self.get_node_names():
            return "rejected: competing slider controller"
        if self.get_node_names().count("teleop_keyboard_gazebo") > 1:
            return "rejected: another keyboard controller is running"
        if not self.real_transport.arm():
            return "rejected: hardware feedback is not stationary or fault is latched"
        self._armed = True
        self._stop_requested = False
        return "armed: hold a motion key to move; Space/N stops and locks"

    def _read_real_pose(self):
        raw = self.mc.get_angles()
        grip = self.mc.get_pro_gripper_angle()
        if not isinstance(raw, (list, tuple)) or len(raw) != 6:
            raise RuntimeError(f"Invalid Pro450 angles: {raw!r}")
        gripper_position = valid_gripper_position(grip)
        pose = [math.radians(float(x)) for x in raw] + [gripper_position]
        if not all(math.isfinite(x) and lo <= x <= hi for x, (lo, hi)
                   in zip(pose, JOINT_LIMITS_RAD)):
            raise RuntimeError(f"Pro450 feedback violates joint limits: {pose!r}")
        return pose

    def _publish_snapshot(self, pose):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "pro450:retain:" + getattr(self.mc, "owner_token", "")
        reader = getattr(self, "_real_pose_reader", None)
        if reader is not None:
            if not reader.fresh():
                return
            # JointState has one stamp: use the oldest constituent sample,
            # not a new stamp that would disguise the cached gripper's age.
            age = max(0.0, time.monotonic() - min(reader.arm_time, reader.gripper_time))
            stamp_ns = max(0, int((time.time() - age) * 1e9))
            msg.header.stamp.sec = stamp_ns // 1000000000
            msg.header.stamp.nanosec = stamp_ns % 1000000000
        elif hasattr(self.mc, 'sample_time'):
            stamps = [self.mc.sample_time(kind) for kind in ('arm', 'gripper')]
            if all(stamp is not None for stamp in stamps):
                wall = min(stamp[1] for stamp in stamps)
                seconds = int(wall)
                msg.header.stamp = Time(sec=seconds, nanosec=int((wall - seconds) * 1e9))
        msg.name = list(COMMAND_JOINTS)
        msg.position = list(pose)
        self.snapshot_pub.publish(msg)

    def _connect_read_only(self):
        from pro450_sdk_adapter import Pro450Client
        self.get_logger().info(
            f"Connecting to Pro450 @ {DEFAULT_PRO450_IP}:{DEFAULT_PRO450_PORT} (read-only)")
        self.mc = Pro450Client(DEFAULT_PRO450_IP, DEFAULT_PRO450_PORT)
        if self.mc.is_power_on() != 1:
            raise RuntimeError("Pro450 not powered; automatic power-on is forbidden")
        code = self.mc.get_error_information()
        if code not in (0, None):
            raise RuntimeError(f"Pro450 error: {code}")
        samples = []
        for index in range(5):
            if self.mc.is_moving() != 0:
                raise RuntimeError("Pro450 must be stationary for initial pose")
            samples.append(self._read_real_pose())
            if index < 4:
                time.sleep(0.5)
        if any(max(p[j] for p in samples) - min(p[j] for p in samples)
               > math.radians(0.2) for j in range(7)):
            raise RuntimeError("Pro450 startup pose was not stable")
        self._startup_pose = samples[-1]
        self._publish_snapshot(samples[-1])

    def _refresh_snapshot(self):
        with self._state_lock:
            if self._active:
                return
        try:
            with self._robot_lock:
                if self.mc.is_moving() != 0:
                    return
                pose = self._read_real_pose()
            self._publish_snapshot(pose)
        except Exception as exc:
            self._armed = False
            self.get_logger().error(f"Real feedback failed; keyboard locked: {exc}")

    @staticmethod
    def _duration(current, target):
        d = max(abs(b - a) for a, b in zip(current, target))
        # Zero-endpoint-velocity cubic: peak velocity 1.5d/T, acceleration 6d/T².
        return max(MIN_DURATION, 1.5 * d / MAX_VELOCITY,
                   math.sqrt(6.0 * d / MAX_ACCELERATION))

    def _publish_trajectory(self, target, duration, gripper_only=False, velocities=None):
        sec = int(duration)
        nanosec = int((duration - sec) * 1e9)
        if not gripper_only:
            arm = JointTrajectory()
            arm.joint_names = list(ARM_JOINTS)
            point = JointTrajectoryPoint()
            point.positions = list(target[:6])
            if velocities is not None:
                point.velocities = list(velocities[:6])
            point.time_from_start = Duration(sec=sec, nanosec=nanosec)
            arm.points = [point]
            self.pub_arm.publish(arm)
        gripper = JointTrajectory()
        gripper.joint_names = [GRIPPER_JOINT]
        point = JointTrajectoryPoint()
        point.positions = [target[6]]
        if velocities is not None:
            point.velocities = [velocities[6]]
        point.time_from_start = Duration(sec=sec, nanosec=nanosec)
        gripper.points = [point]
        self.pub_gripper.publish(gripper)

    def _on_real_sample(self, pose):
        publish_feedback(self, pose, self._real_pose_reader, COMMAND_JOINTS)

    def _mirror_loop(self):
        next_tick = time.monotonic()
        while rclpy.ok() and not self._mirror_stop.is_set():
            next_tick += 0.02
            if int(next_tick * 50) % 25 == 0:
                publish_age(self, self._real_pose_reader)
            try:
                rendered = self._real_mirror.render()
                if rendered is not None:
                    pose, velocity = rendered
                    self._publish_trajectory(pose, 0.04, velocities=velocity)
            except Exception:
                self.get_logger().debug("Could not render the measured pose.")
            delay = next_tick - time.monotonic()
            if delay > 0.0:
                self._mirror_stop.wait(delay)
            else:
                next_tick = time.monotonic()

    def request_step(self, index, delta):
        if self.mode == "real":
            self.get_logger().warning("Real terminal step path is disabled; use the hold window.")
            return
        if "slider_control_gazebo" in self.get_node_names():
            self.get_logger().warning(
                "Slider command bridge is also running; stop it before keyboard control.")
            return
        if self.mode == "real" and not self._armed:
            self.get_logger().warning("Real motion locked; press m to arm explicitly.")
            return
        feedback = self._fresh_feedback()
        if feedback is None:
            self.get_logger().warning("Missing/stale finite Gazebo feedback; rejected.")
            return
        current, velocity = feedback
        if any(abs(x) > 0.05 for x in velocity):
            self.get_logger().warning("Robot still moving; wait until settled.")
            return
        with self._state_lock:
            if self._active:
                self.get_logger().warning("Previous command is still active.")
                return
            self._active = True
            self._stop_requested = False
        target = list(current)
        low, high = JOINT_LIMITS_RAD[index]
        target[index] = max(low, min(high, target[index] + delta))
        if abs(target[index] - current[index]) < 1e-8:
            with self._state_lock:
                self._active = False
            self.get_logger().info("Already at joint limit.")
            return
        self._worker = threading.Thread(
            target=self._execute_step,
            args=(current, target, index == 6), daemon=True)
        self._worker.start()

    def _execute_step(self, current, target, gripper_only):
        if self.mode == "real":
            self.get_logger().error("Legacy real step transport is disabled.")
            with self._state_lock:
                self._active = False
            return
        try:
            self.get_logger().info("Validating keyboard path against MoveIt collisions...")
            valid, reason = self._path_is_valid(current, target)
            if not valid:
                self.get_logger().warning(f"Rejected: {reason}")
                return
            fresh = self._fresh_feedback()
            if (self._stop_requested or fresh is None or
                    max(abs(a - b) for a, b in zip(fresh[0], current)) > math.radians(1)):
                self.get_logger().warning("Pose changed during validation; retry.")
                return
            duration = self._duration(current, target)
            if duration > MAX_DURATION:
                self.get_logger().warning("Trajectory duration exceeds safe limit.")
                return
            if self._stop_requested:
                return
            self._publish_trajectory(target, duration, gripper_only)
            end = time.monotonic() + duration
            while time.monotonic() < end and not self._stop_requested:
                time.sleep(0.05)
            feedback = self._fresh_feedback()
            if (not self._stop_requested and feedback is not None and
                    max(abs(a - b) for a, b in zip(feedback[0], target)) <= SETTLE_TOLERANCE):
                self.get_logger().info("Keyboard step completed.")
            elif not self._stop_requested:
                self._armed = False
                self.get_logger().error("Target not reached; real controls locked.")
        except Exception as exc:
            self._armed = False
            self.get_logger().error(f"Keyboard command failed; controls locked: {exc}")
        finally:
            with self._state_lock:
                self._active = False

    def stop(self):
        self._stop_requested = True
        self._armed = False
        with self._hold_lock:
            self._hold_generation += 1
            self._hold_axis = None
            self._hold_pending = None
            self._hold_pressed = False
            self._hold_velocity = 0.0
            self._hold_validation_pending = False
        if self.mode == "real":
            if self.real_transport is not None:
                self.real_transport.stop(emergency=True, lock=True)
        feedback = self._fresh_feedback()
        if feedback is not None and self.mode == "simulation":
            self._publish_trajectory(feedback[0], MIN_DURATION)
        if self.mode == "real":
            self.get_logger().warning("STOP requested; real controls locked.")
        else:
            self.get_logger().warning("Simulation STOP requested; keyboard node locked.")

    def set_hold_speed_gear(self, gear):
        if gear < 1 or gear > len(HOLD_SPEED_GEARS):
            raise ValueError("speed gear must be between 1 and 5")
        with self._hold_lock:
            self._hold_speed_gear = gear
        self.get_logger().info(
            f"Keyboard speed gear {gear}: "
            f"arm {math.degrees(HOLD_SPEED_GEARS[gear - 1]):.1f} deg/s; "
            f"gripper {HOLD_GRIPPER_SPEED_GEARS[gear - 1]:.2f} rad/s; "
            f"real gripper SDK {4 * gear}")

    def hold_speed_status(self):
        with self._hold_lock:
            gear = self._hold_speed_gear
        return gear, math.degrees(HOLD_SPEED_GEARS[gear - 1])

    def _hold_speed_limit(self, axis):
        with self._hold_lock:
            if axis == 6:
                return min(HOLD_GRIPPER_SPEED_GEARS[self._hold_speed_gear - 1],
                           HOLD_GRIPPER_URDF_VELOCITY_LIMIT)
            arm_speed = min(HOLD_SPEED_GEARS[self._hold_speed_gear - 1],
                            HOLD_ARM_URDF_VELOCITY_LIMIT)
        return arm_speed

    def press_hold(self, axis, direction):
        """Accept a hold, or queue a reversal after measured standstill."""
        if self.mode == "real":
            if not self._armed or not self.real_transport.armed:
                return "rejected: real motion locked; press M to arm"
            if axis == 6 and not self.real_gripper_hold_enabled:
                return "rejected: gripper calibration pending; real gripper hold disabled"
        if self._stop_requested:
            return "rejected: STOP is latched; restart the keyboard node"
        if "slider_control_gazebo" in self.get_node_names():
            return "rejected: slider_control_gazebo is also running"
        if self.get_node_names().count("teleop_keyboard_gazebo") > 1:
            return "rejected: another keyboard controller is running"
        feedback = self._fresh_feedback()
        if feedback is None:
            return "rejected: Gazebo feedback is stale"
        positions, velocities = feedback
        replacing = False
        with self._hold_lock:
            if self._hold_axis is not None:
                if self._hold_axis == axis and self._hold_direction == direction:
                    # Auto-repeat presses the held key again. Keep moving.
                    self._hold_pressed = True
                    self._hold_pending = None
                    self._real_braking = False
                    return "accepted: hold continues"
                # Another key takes over. Leaving the old hold queued blocks
                # every later key once a collision scan has finished.
                self._hold_axis = None
                self._hold_pressed = False
                self._hold_pending = None
                self._hold_velocity = 0.0
                replacing = True
        if replacing and self.mode == "real" and self.real_transport is not None:
            self.real_transport.stop(emergency=True)
            self._real_braking = True
        if (not replacing and self._hold_axis is None and self.mode != "real" and any(
                abs(value) > 0.02 for value in velocities)):
            return "rejected: wait for the previous motion to settle"
        with self._hold_lock:
            self._hold_axis = axis
            self._hold_direction = direction
            self._hold_pressed = True
            self._hold_pending = None
            self._hold_velocity = 0.0
            self._hold_validated_limit = positions[axis]
            self._hold_validation_pending = False
            self._hold_generation += 1
            self._hold_last_tick = time.monotonic()
            self._real_braking = False
            self._releasing_axis = None
            self._brake_since = 0.0
            self._hold_validation_origin = list(positions)
            self._hold_first_validation = True
            self._hold_final_target = None
            self._hold_goal_sent = False
        if self.mode != "real":
            self._request_hold_clearance(positions)
        self._request_collision_stop(positions)
        return "accepted: checking collision clearance"

    def release_hold(self):
        """Key-up or focus loss. The held joint is cleared immediately."""
        with self._hold_lock:
            released_axis = self._hold_axis
            self._hold_pending = None
            self._hold_pressed = False
            self._releasing_axis = released_axis
            self._brake_since = time.monotonic()
            self._hold_axis = None
            self._hold_velocity = 0.0
            self._hold_final_target = None
            self._hold_goal_sent = False
        if self.mode == "real" and self.real_transport is not None:
            self.real_transport.stop(emergency=True)
            self._real_braking = True
        elif released_axis is not None:
            feedback = self._fresh_feedback()
            if feedback is not None:
                self._publish_hold_waypoint(
                    feedback[0], released_axis, feedback[0][released_axis], 0.15)

    def hold_idle(self):
        with self._hold_lock:
            return self._hold_axis is None

    def _request_hold_clearance(self, current):
        with self._hold_lock:
            if self._hold_axis is None or self._hold_validation_pending:
                return
            axis = self._hold_axis
            direction = self._hold_direction
            generation = self._hold_generation
            low, high = JOINT_LIMITS_RAD[axis]
            advance = (HOLD_FIRST_VALIDATION_ADVANCE
                       if getattr(self, "_hold_first_validation", False)
                       else HOLD_VALIDATION_ADVANCE)
            self._hold_first_validation = False
            candidate = max(low, min(high, current[axis] + direction * advance))
            joint_boundary = high if direction > 0 else low
            if (abs(candidate - joint_boundary) <= 1e-6 and
                    abs(self._hold_validated_limit - joint_boundary) <= 1e-6):
                # The complete remaining corridor to this URDF boundary is
                # already checked. Rechecking the same endpoint cannot extend it.
                return
            if direction * (candidate - current[axis]) < 0.001:
                self._hold_pressed = False
                return
            self._hold_validation_pending = True
        self._hold_validation_worker = threading.Thread(
            target=self._validate_hold_clearance,
            args=(current, axis, direction, generation, candidate),
            daemon=True,
        )
        self._hold_validation_worker.start()

    def _validate_hold_clearance(self, current, axis, direction,
                                 generation, candidate):
        accepted = None
        reason = "collision ahead"
        try:
            distance = abs(candidate - current[axis])
            # If a full look-ahead intersects the model, probe shorter safe
            # corridors. Service errors are not collisions and fail closed.
            for _ in range(6):
                target = list(current)
                target[axis] = current[axis] + direction * distance
                valid, reason = self._path_is_valid(current, target)
                if valid:
                    accepted = target[axis]
                    break
                if "collision" not in reason.lower():
                    break
                distance *= 0.5
                if distance < math.radians(0.25):
                    break
        except Exception as exc:
            reason = str(exc)
        with self._hold_lock:
            if generation != self._hold_generation or axis != self._hold_axis:
                return
            if getattr(self, "_hold_final_target", None) is not None:
                self._hold_validation_pending = False
                return
            self._hold_validation_pending = False
            if accepted is not None:
                # A valid endpoint need not be farther than the previous one.
                # Keep equal endpoints usable; shrink the corridor if a shorter
                # probe is all that is safe, so braking follows the new limit.
                self._hold_validated_limit = accepted
                self._hold_validation_origin = list(current)
                # The short corridor is the same gate for simulation and the
                # real robot. A collision here cancels the hold in both modes.
                if direction * (accepted - current[axis]) <= HOLD_COLLISION_MARGIN:
                    self._hold_pressed = False
                    self.get_logger().warning(
                        "No remaining collision-checked clearance; hold stopped.")
            else:
                self._hold_pressed = False
                self.get_logger().warning(f"Hold braking before unvalidated path: {reason}")

    def _request_collision_stop(self, positions):
        with self._hold_lock:
            axis = self._hold_axis
            direction = self._hold_direction
            generation = self._hold_generation
        if axis is None:
            return
        low, high = JOINT_LIMITS_RAD[axis]
        goal = high if direction > 0 else low
        worker = threading.Thread(
            target=self._scan_collision_stop,
            args=(list(positions), axis, generation, positions[axis], goal),
            daemon=True,
        )
        self._hold_scan_worker = worker
        worker.start()

    def _scan_collision_stop(self, pose, axis, generation, start, goal):
        """One absolute-grid scan. The same pose always yields the same stop."""
        if axis == 6:
            grid = HOLD_GRIPPER_SCAN_GRID
            fine = HOLD_GRIPPER_SCAN_FINE
            margin = HOLD_GRIPPER_STOP_MARGIN
        else:
            grid, fine, margin = HOLD_SCAN_GRID, HOLD_SCAN_FINE, HOLD_STOP_MARGIN

        def blocked(angle):
            sample = list(pose)
            low, high = JOINT_LIMITS_RAD[axis]
            sample[axis] = min(high, max(low, angle))
            valid, _contacts, reason = self._state_is_valid(sample)
            if reason and "collision" not in reason.lower():
                raise RuntimeError(reason)
            return not valid

        try:
            # Same ground object the short corridor installs. A scan that runs
            # before the ground exists would treat the whole stroke as clear.
            ready, reason = self._ensure_floor_collision_scene()
            if not ready:
                raise RuntimeError(reason)
            target = collision_stop_angle(start, goal, margin, grid, fine, blocked)
        except Exception as exc:
            if self.mode == "real":
                with self._hold_lock:
                    if generation != self._hold_generation or axis != self._hold_axis:
                        return
                    self._hold_pressed = False
                self.get_logger().warning(f"Collision scan failed; no real motion sent: {exc}")
            else:
                self.get_logger().warning(f"Collision scan failed; jog stays inside the short check: {exc}")
            return
        with self._hold_lock:
            if generation != self._hold_generation or axis != self._hold_axis:
                return
            self._hold_final_target = target
            self._hold_validated_limit = target
            self._hold_goal_sent = False

    def _submit_real_gripper_hold(self):
        """One gripper opening at the scanned stop, using the window gear."""
        with self._hold_lock:
            if self._hold_axis != 6 or not self._hold_pressed:
                return
            origin = getattr(self, "_hold_validation_origin", None)
            gear = self._hold_speed_gear
            target = self._hold_final_target
        if origin is None or target is None:
            return
        speed = HOLD_SPEED_GEARS[gear - 1]
        self.real_transport.submit(6, target, speed, origin, target, position_goal=True)

    def _publish_hold_waypoint(self, positions, axis, endpoint, duration):
        if self.mode == "real":
            with self._hold_lock:
                origin = getattr(self, "_hold_validation_origin", None)
                pressed = self._hold_pressed
                gear = self._hold_speed_gear
                final = self._hold_final_target
            if origin is not None and pressed and axis != 6 and final is not None:
                speed = HOLD_SPEED_GEARS[gear - 1]
                self.real_transport.submit(
                    axis, final, speed, origin, final, position_goal=True)
            return
        # Position-only JTC points interpolate linearly. Bound their requested
        # average speed even if a future gear change bypasses the table guard.
        velocity_limit = (HOLD_GRIPPER_URDF_VELOCITY_LIMIT
                          if axis == 6 else HOLD_ARM_URDF_VELOCITY_LIMIT)
        duration = max(duration, abs(endpoint - positions[axis]) / velocity_limit)
        seconds = int(duration)
        nanoseconds = int((duration - seconds) * 1e9)
        msg = JointTrajectory()
        point = JointTrajectoryPoint()
        if axis == 6:
            msg.joint_names = [GRIPPER_JOINT]
            point.positions = [endpoint]
            publisher = self.pub_gripper
        else:
            msg.joint_names = list(ARM_JOINTS)
            point.positions = list(positions[:6])
            point.positions[axis] = endpoint
            publisher = self.pub_arm
        # Position-only points select linear interpolation. In Gazebo the
        # reported velocity may disagree with position change; sending a
        # velocity point would make the controller's cubic interpolation move
        # other joints even when their requested endpoint is unchanged.
        point.time_from_start = Duration(sec=seconds, nanosec=nanoseconds)
        msg.points = [point]
        publisher.publish(msg)

    def _publish_tracked_goal(self, positions, axis, goal, speed):
        """One trapezoid to the scanned stop. Same target the real firmware is given."""
        times, samples = trapezoid_samples(
            positions[axis], goal, speed, HOLD_TRACKED_ACCELERATION)
        msg = JointTrajectory()
        msg.points = []
        for stamp, sample in zip(times, samples):
            point = JointTrajectoryPoint()
            point.time_from_start = Duration(
                sec=int(stamp), nanosec=int((stamp - int(stamp)) * 1e9))
            if axis == 6:
                point.positions = [sample]
            else:
                point.positions = list(positions[:6])
                point.positions[axis] = sample
            msg.points.append(point)
        if axis == 6:
            msg.joint_names = [GRIPPER_JOINT]
            self.pub_gripper.publish(msg)
        else:
            msg.joint_names = list(ARM_JOINTS)
            self.pub_arm.publish(msg)

    def hold_tick(self):
        """Run at 20 Hz; hardware I/O is confined to the transport owner."""
        now = time.monotonic()
        with self._hold_lock:
            axis = self._hold_axis
            if axis is None:
                return
            direction = self._hold_direction
            pressed = self._hold_pressed
            limit = self._hold_validated_limit
            velocity = self._hold_velocity
            dt = max(0.01, now - self._hold_last_tick)
            self._hold_last_tick = now
            pending = self._hold_validation_pending

        feedback = self._fresh_feedback()
        if feedback is None:
            # A slow or missing sample is not a motion fault. Wait for the next tick.
            return
        if "slider_control_gazebo" in self.get_node_names():
            self.stop()
            self.get_logger().error("Hold stopped: slider_control_gazebo is also running.")
            return
        positions, velocities = feedback
        if self.mode == "real":
            if self.real_transport.error:
                self.stop()
                return
            if not pressed and not self._real_braking:
                self.real_transport.stop(emergency=False)
                self._real_braking = True
        if self.mode == "real" and axis == 6:
            # Wait for the scanned opening. One command then holds that target.
            if pressed and self._hold_final_target is not None:
                self._submit_real_gripper_hold()
            return
        speed_limit = self._hold_speed_limit(axis)
        with self._hold_lock:
            final_target = self._hold_final_target
            goal_sent = self._hold_goal_sent
        if self.mode == "real" and final_target is None:
            # A key-down only requests the full collision scan. Do not jog
            # before its finite stop target is available.
            return
        if final_target is not None and self.mode == "real" and pressed and axis != 6:
            self._publish_hold_waypoint(positions, axis, final_target, max(dt, 0.05))
            return
        if final_target is not None and self.mode != "real":
            if not pressed:
                self._publish_hold_waypoint(positions, axis, positions[axis], 0.15)
                with self._hold_lock:
                    pending_key = self._hold_pending
                    self._hold_axis = None
                    self._hold_pending = None
                    self._hold_velocity = 0.0
                    self._hold_goal_sent = False
                    self._hold_final_target = None
                if pending_key is not None:
                    self.press_hold(*pending_key)
                return
            elif not goal_sent:
                self._publish_tracked_goal(positions, axis, final_target, speed_limit)
                with self._hold_lock:
                    self._hold_goal_sent = True
            return
        if (pressed and not pending and final_target is None and
                direction * (limit - positions[axis]) < HOLD_VALIDATION_TRIGGER):
            self._request_hold_clearance(positions)
        if final_target is None and pressed:
            # Constant gear speed, clamped to the short check, until the scan returns.
            endpoint = positions[axis] + direction * speed_limit * dt
            if direction * (endpoint - limit) > 0:
                endpoint = limit
            if abs(endpoint - positions[axis]) > 1e-6:
                self._publish_hold_waypoint(positions, axis, endpoint, max(dt, 0.05))
            return

        endpoint, next_velocity, duration = hold_setpoint(
            positions[axis], velocities[axis], velocity, direction,
            pressed, limit, dt, speed_limit, HOLD_MAX_ACCELERATION,
            collision_margin=HOLD_COLLISION_MARGIN,
        )
        if self.mode == "real" and not pressed:
            # Firmware braking is authoritative; wait for measured standstill.
            next_velocity = 0.0
        pending = None
        stopped = False
        with self._hold_lock:
            if axis != self._hold_axis:
                return
            self._hold_velocity = next_velocity
            if (not self._hold_pressed and abs(next_velocity) < 0.002 and
                    abs(velocities[axis]) < 0.005 and
                    (self.mode != "real" or not self.real_transport.moving)):
                self._hold_axis = None
                self._hold_velocity = 0.0
                pending = self._hold_pending
                self._hold_pending = None
                stopped = True
        if stopped:
            # Always replace the final in-flight waypoint with an explicit
            # stationary endpoint before reporting completion or reversing.
            self._publish_hold_waypoint(positions, axis, positions[axis], 0.15)
            self.get_logger().info(
                "Hold released; hardware standstill confirmed." if self.mode == "real"
                else "Hold released; final hold waypoint sent.")
            if pending is not None:
                self.press_hold(*pending)
            return
        if abs(endpoint - positions[axis]) > 0.00001 or abs(next_velocity) > 0.002:
            self._publish_hold_waypoint(positions, axis, endpoint, duration)


class RawTerminal:
    def __enter__(self):
        self.fd = sys.stdin.fileno()
        self.previous = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        return self

    def __exit__(self, *_):
        termios.tcsetattr(self.fd, termios.TCSANOW, self.previous)


JOINT_KEYS = {"w": (0, 1), "s": (0, -1), "e": (1, 1), "d": (1, -1),
              "r": (2, 1), "f": (2, -1), "t": (3, 1), "g": (3, -1),
              "y": (4, 1), "h": (4, -1), "u": (5, 1), "j": (5, -1)}


def hold_release_action(still_down):
    """Decide a KeyRelease from the physical key state, not the repeat rate.

    True means the key is still down, so an auto-repeat release is ignored.
    False means the key is up and the hold must stop. None means the state
    could not be read and the caller may use a short debounce.
    """
    if still_down is True:
        return "ignore"
    if still_down is False:
        return "release"
    return "debounce"


class X11KeyState:
    """Read the live X11 keymap. Auto-repeat does not clear a held key."""

    def __init__(self):
        self._lib = None
        self._display = None
        self._failed = False

    def down(self, keycode):
        import ctypes
        keycode = int(keycode or 0)
        if keycode <= 0 or keycode >= 256 or self._failed:
            return None
        if self._lib is None:
            try:
                self._lib = ctypes.CDLL("libX11.so.6")
                self._lib.XOpenDisplay.restype = ctypes.c_void_p
                self._lib.XQueryKeymap.argtypes = [
                    ctypes.c_void_p, ctypes.POINTER(ctypes.c_ubyte)]
                self._display = self._lib.XOpenDisplay(None)
            except Exception:
                self._failed = True
                return None
            if not self._display:
                self._failed = True
                return None
        keys = (ctypes.c_ubyte * 32)()
        self._lib.XQueryKeymap(self._display, keys)
        return bool(keys[keycode // 8] & (1 << (keycode % 8)))


class SimulationHoldWindow:
    """Small Qt window that supplies real press/release events to the node."""

    def __init__(self, node):
        from python_qt_binding.QtCore import Qt, QTimer
        from python_qt_binding.QtWidgets import QLabel, QVBoxLayout, QWidget

        class Window(QWidget):
            def __init__(self, owner):
                super().__init__()
                self.owner = owner

            def keyPressEvent(self, event):
                self.owner.key_press(event)

            def keyReleaseEvent(self, event):
                self.owner.key_release(event)

            def focusOutEvent(self, event):
                self.owner.release()
                super().focusOutEvent(event)

            def closeEvent(self, event):
                self.owner.release()
                self.owner.closing = True
                event.accept()

        self.node = node
        self.Qt = Qt
        self.closing = False
        self.pressed_code = None
        self.pressed_scan = 0
        self.key_state = X11KeyState()
        self.window = Window(self)
        real = getattr(node, "mode", "simulation") == "real"
        self.window.setWindowTitle("Pro450 Real Hold Control" if real else
                                   "Pro450 Simulation Hold Control")
        self.window.setFocusPolicy(Qt.StrongFocus)
        layout = QVBoxLayout(self.window)
        layout.addWidget(QLabel(
            "Click this window, then HOLD w/s e/d r/f t/g y/h u/j to move.\n"
            "Release to decelerate. Hold [ or ] for the gripper.\n"
            "- / +: slower / faster; 1-5: speed gear.\n"
            "Space: STOP and lock. Q/Ctrl+C: brake and quit. Focus loss releases.\n" +
            ("REAL: read-only until M arms. N stops and locks. Hardware E-stop required."
             if real else "Simulation only — this window never connects to the real robot.")
        ))
        gear, speed = self.node.hold_speed_status()
        self.status = QLabel(
            f"Gear {gear}: arm {speed:.1f} deg/s; "
            f"gripper {HOLD_GRIPPER_SPEED_GEARS[gear - 1]:.2f} rad/s; "
            f"real gripper SDK {4 * gear}; "
            "awaiting Gazebo feedback.")
        layout.addWidget(self.status)
        self.window.resize(570, 190)
        self.timer = QTimer(self.window)
        self.timer.setInterval(50)
        self.timer.timeout.connect(self.tick)
        self.timer.start()
        self.release_timer = QTimer(self.window)
        self.release_timer.setSingleShot(True)
        self.release_timer.setInterval(70)
        self.release_timer.timeout.connect(self._release_if_key_up)

    def show(self):
        self.window.show()
        self.window.activateWindow()
        self.window.setFocus()

    def release(self):
        self.release_timer.stop()
        self.pressed_code = None
        self.pressed_scan = 0
        self.node.release_hold()

    def _release_if_key_up(self):
        """Debounce fallback only. A key that is still down keeps moving."""
        if hold_release_action(self.key_state.down(self.pressed_scan)) == "ignore":
            self.release_timer.stop()
            return
        self.release()
        ready = getattr(self.node, "mode", "") == "real"
        self.status.setText("Ready: hold a key to move." if ready
                            else "Key released: decelerating.")

    def request_exit(self, source):
        from python_qt_binding.QtWidgets import QApplication
        if self.closing:
            os._exit(0)
        self.closing = True
        self.release()
        self.status.setText(f"{source}: braking, then exiting.")
        app = QApplication.instance()
        if app is not None:
            app.quit()

    def key_press(self, event):
        if event.key() == self.pressed_code:
            # The held key's auto-repeat is another press, not a new joint.
            self.release_timer.stop()
            return
        if event.isAutoRepeat():
            return
        key = event.text().lower()
        if getattr(self.node, "mode", "simulation") == "real" and key in ("m", "n"):
            if key == "m":
                self.status.setText(self.node.arm_real())
            else:
                self.node.stop()
                self.status.setText("STOP requested; real controls locked.")
            return
        if key in ("1", "2", "3", "4", "5", "+", "=", "-", "_"):
            gear, _ = self.node.hold_speed_status()
            if key in ("+", "="):
                gear = min(5, gear + 1)
            elif key in ("-", "_"):
                gear = max(1, gear - 1)
            else:
                gear = int(key)
            self.node.set_hold_speed_gear(gear)
            _, speed = self.node.hold_speed_status()
            self.status.setText(
                f"Gear {gear}: arm {speed:.1f} deg/s; "
                f"gripper {HOLD_GRIPPER_SPEED_GEARS[gear - 1]:.2f} rad/s; "
                f"real gripper SDK {4 * gear}.")
            return
        if event.key() == self.Qt.Key_Space:
            self.pressed_code = None
            self.node.stop()
            self.status.setText("STOP requested (not a hardware E-stop).")
            return
        if event.key() == self.Qt.Key_Q:
            self.request_exit("Q")
            return
        if key in JOINT_KEYS:
            axis, direction = JOINT_KEYS[key]
        elif key == "[":
            axis, direction = 6, 1
        elif key == "]":
            axis, direction = 6, -1
        else:
            return
        self.release_timer.stop()
        result = self.node.press_hold(axis, direction)
        if result.startswith("rejected"):
            self.pressed_code = None
            self.pressed_scan = 0
        else:
            self.pressed_code = event.key()
            self.pressed_scan = int(event.nativeScanCode() or 0)
        self.status.setText(f"Joint {axis + 1}: {result}.")

    def key_release(self, event):
        if event.isAutoRepeat():
            return
        if event.key() != self.pressed_code:
            return
        scan = int(event.nativeScanCode() or self.pressed_scan or 0)
        action = hold_release_action(self.key_state.down(scan))
        if action == "ignore":
            # Linux auto-repeat emits KeyRelease while the key remains down.
            self.release_timer.stop()
            return
        if action == "release":
            self.release()
            ready = getattr(self.node, "mode", "") == "real"
            self.status.setText("Ready: hold a key to move." if ready
                                else "Key released: decelerating.")
            return
        self.pressed_scan = scan
        self.release_timer.start()
        self.status.setText("Key release detected: decelerating after debounce.")

    def tick(self):
        try:
            self.node.hold_tick()
            transport = getattr(self.node, "real_transport", None)
            if transport is not None and transport.error:
                if transport.recovery_ready:
                    self.status.setText(
                        "Feedback recovered and stable; motion remains locked. "
                        "Press M to explicitly re-arm after checking the scene.")
                else:
                    self.status.setText(
                        f"Read/control fault (locked): {transport.error}. "
                        "No automatic motion recovery; hardware E-stop if physical danger.")
        except Exception as exc:
            self.node.stop()
            self.status.setText(f"Control error: {exc}")
            self.node.get_logger().error(f"Simulation hold failed: {exc}")
        if self.closing and self.node.hold_idle():
            self.window.close()


def keyboard_loop(node):
    print("Pro450: w/s e/d r/f t/g y/h u/j = +/-1 deg; uppercase = +/-5 deg")
    print("[ / ] = gripper +/-0.05 rad; Space = STOP; 2 = feedback; q = quit")
    print("Real only: m = arm, n = lock. No reset or collision override.")
    with RawTerminal():
        while rclpy.ok():
            if not select.select([sys.stdin], [], [], 0.05)[0]:
                continue
            key = sys.stdin.read(1)
            if key == "q":
                node._armed = False
                rclpy.shutdown()
                break
            if key == " ":
                node.stop()
            elif key == "m" and node.mode == "real":
                if node._fresh_feedback() is None:
                    node.get_logger().warning("Cannot arm without Gazebo feedback.")
                else:
                    node._armed = True
                    node.get_logger().warning("REAL MOTION ARMED. n = lock; Space = STOP.")
            elif key == "n":
                node._armed = False
                node.get_logger().info("Real motion locked.")
            elif key == "2":
                node.get_logger().info(f"Gazebo feedback (rad): {node._fresh_feedback()}")
            elif key in ("[", "]"):
                node.request_step(6, GRIPPER_STEP if key == "[" else -GRIPPER_STEP)
            elif key.lower() in JOINT_KEYS:
                index, direction = JOINT_KEYS[key.lower()]
                node.request_step(index, direction *
                                  (FAST_STEP if key.isupper() else NORMAL_STEP))


def main(args=None):
    rclpy.init(args=args)
    node = None
    keyboard = None
    ros_thread = None
    previous_sigint = None
    previous_sigterm = None
    exit_code = 0
    try:
        node = TeleopKeyboard()
        if node.mode in ("simulation", "real"):
            from python_qt_binding.QtWidgets import QApplication
            app = QApplication.instance() or QApplication(["pro450_sim_hold"])
            ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
            ros_thread.start()
            window = SimulationHoldWindow(node)
            previous_sigint = signal.getsignal(signal.SIGINT)
            signal.signal(
                signal.SIGINT,
                lambda _signum, _frame: window.request_exit("Ctrl+C"),
            )
            previous_sigterm = signal.getsignal(signal.SIGTERM)
            signal.signal(signal.SIGTERM,
                          lambda _signum, _frame: window.request_exit("SIGTERM"))
            window.show()
            app.exec_()
        else:
            if not sys.stdin.isatty():
                raise RuntimeError("Real keyboard control requires an interactive terminal")
            keyboard = threading.Thread(target=keyboard_loop, args=(node,), daemon=True)
            keyboard.start()
            rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as exc:
        exit_code = 1
        print(f"Failed to start Pro450 keyboard: {exc}", file=sys.stderr)
    finally:
        if previous_sigint is not None:
            signal.signal(signal.SIGINT, previous_sigint)
        if previous_sigterm is not None:
            signal.signal(signal.SIGTERM, previous_sigterm)
        if node is not None:
            if node.real_transport is not None:
                node.real_transport.close()
                node._real_thread.join(timeout=2.0)
            if node._mirror_thread is not None:
                node._mirror_stop.set()
                node._mirror_thread.join(timeout=0.5)
            node._armed = False
            node._stop_requested = True
            if node.mode == "simulation" and not node.hold_idle():
                node.stop()
            with node._state_lock:
                motion_active = node._active
            if motion_active and node.mode == "real":
                try:
                    if node._robot_lock.acquire(timeout=2.0):
                        try:
                            node.mc.stop()
                        finally:
                            node._robot_lock.release()
                    else:
                        node.get_logger().error("Could not acquire real robot lock for STOP")
                except Exception as exc:
                    node.get_logger().error(f"Real STOP on exit failed: {exc}")
            if node._worker is not None:
                node._worker.join(timeout=2.0)
            if node._hold_validation_worker is not None:
                node._hold_validation_worker.join(timeout=2.0)
            # Never release servo torque; the loaded real robot could fall.
            if node.mc is not None and hasattr(node.mc, "close"):
                node.mc.close()
            node.destroy_node()
        rclpy.try_shutdown()
        if ros_thread is not None:
            ros_thread.join(timeout=1.0)
        if keyboard is not None:
            keyboard.join(timeout=1.0)
    if exit_code:
        raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
