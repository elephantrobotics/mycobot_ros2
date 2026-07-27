#!/usr/bin/env python3
"""Publish mimic-expanded joint states for MoveIt2 Goal / Planned-Path preview.

MoveIt's MotionPlanning display often fails to sync parallel-link mimic joints.
This node:
  - listens to Interactive Marker feedback/update (Marker drag + Joints tab)
  - solves IK for Goal preview (seeded from last goal to track Joints sliders)
  - listens to DisplayTrajectory and plays back waypoints for path preview
  - publishes joints on /mimic_preview/joint_states for a prefixed RSP
"""

import math
import threading

import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import DisplayTrajectory, MoveItErrorCodes, RobotState
from moveit_msgs.srv import GetPositionIK
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from visualization_msgs.msg import InteractiveMarkerFeedback, InteractiveMarkerUpdate

ACTIVE_JOINTS = ("J1", "J2", "J3", "J4")
# URDF mimic: child = multiplier * parent + offset (radians)
MIMIC_JOINTS = {
    "center": ("J2", -1.0, 0.0),
    "CENTER_V_2": ("J3", 1.0, 0.0),
    "J3_4": ("J3", -1.0, 0.0),
    "J3_5": ("J3", -1.0, 0.0),
    "J2_2": ("J2", 1.0, 0.0),
    "center_v_1": ("J2", -1.0, 0.0),
}
J2_RANGE = (-18.0, 85.0)
J3_RANGE = (-1.0, 110.0)
ZERO_EPS_DEG = 0.1


def snap_zero(angle_deg):
    return 0.0 if abs(angle_deg) < ZERO_EPS_DEG else angle_deg


def valid_region(j2_deg, j3_deg):
    """Same J2/J3 coupling check as joint_coupling_node (incl. 42 deg hard cut)."""
    a = snap_zero(j2_deg)
    b = snap_zero(j3_deg)
    if not (J2_RANGE[0] <= a <= J2_RANGE[1] and J3_RANGE[0] <= b <= J3_RANGE[1]):
        return False
    if -18 <= a < 0:
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


def active_j2_j3_deg(active):
    return (
        math.degrees(float(active["J2"])),
        math.degrees(float(active["J3"])),
    )


def _pose_changed(a, b, lin_eps=1e-4, ang_eps=1e-3):
    if a is None or b is None:
        return True
    dp = a.position
    dq = b.position
    if abs(dp.x - dq.x) > lin_eps or abs(dp.y - dq.y) > lin_eps or abs(dp.z - dq.z) > lin_eps:
        return True
    oa, ob = a.orientation, b.orientation
    dot = abs(oa.x * ob.x + oa.y * ob.y + oa.z * ob.z + oa.w * ob.w)
    return dot < math.cos(ang_eps / 2.0)


class MimicPreviewNode(Node):
    def __init__(self):
        super().__init__("mimic_preview_node")
        self._lock = threading.Lock()
        self._cb_group = ReentrantCallbackGroup()
        self._current = {name: 0.0 for name in ACTIVE_JOINTS}
        self._goal_active = {name: 0.0 for name in ACTIVE_JOINTS}
        self._mode = "goal"  # goal | trajectory
        self._user_preview = False
        self._traj_points = []
        self._traj_index = 0
        self._last_ee_pose = None
        self._last_ik_time = self.get_clock().now()
        self._last_invalid_pair = None
        self._was_invalid = False

        self.declare_parameter("planning_group", "arm_group")
        self.declare_parameter("ee_link", "J4_Link")
        self.declare_parameter("ik_timeout", 0.05)
        self.declare_parameter("state_display_time", 0.05)
        self.declare_parameter("ik_min_period", 0.05)
        self.declare_parameter(
            "marker_topic_base",
            "/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic",
        )

        self._group = self.get_parameter("planning_group").value
        self._ee_link = self.get_parameter("ee_link").value
        self._ik_timeout = float(self.get_parameter("ik_timeout").value)
        self._state_dt = float(self.get_parameter("state_display_time").value)
        self._ik_min_period = float(self.get_parameter("ik_min_period").value)
        self._marker_base = self.get_parameter("marker_topic_base").value

        self._pub = self.create_publisher(JointState, "/mimic_preview/joint_states", 10)
        self.create_subscription(
            JointState, "/joint_states", self._on_joint_states, 1, callback_group=self._cb_group
        )
        self.create_subscription(
            DisplayTrajectory,
            "/display_planned_path",
            self._on_display_trajectory,
            1,
            callback_group=self._cb_group,
        )
        self.create_subscription(
            InteractiveMarkerFeedback,
            self._marker_base + "/feedback",
            self._on_marker_feedback,
            10,
            callback_group=self._cb_group,
        )
        self.create_subscription(
            InteractiveMarkerUpdate,
            self._marker_base + "/update",
            self._on_marker_update,
            10,
            callback_group=self._cb_group,
        )

        self._ik = self.create_client(GetPositionIK, "/compute_ik", callback_group=self._cb_group)
        if not self._ik.wait_for_service(timeout_sec=30.0):
            self.get_logger().warn("compute_ik not available after 30s; Goal preview may stall")

        self._timer = self.create_timer(self._state_dt, self._on_timer, callback_group=self._cb_group)
        self.get_logger().info(
            "mimic_preview_node ready (group=%s, ee=%s, marker=%s)"
            % (self._group, self._ee_link, self._marker_base)
        )

    def _on_joint_states(self, msg):
        with self._lock:
            for name in ACTIVE_JOINTS:
                if name in msg.name:
                    self._current[name] = msg.position[msg.name.index(name)]
            if not self._user_preview and self._mode == "goal":
                self._goal_active = dict(self._current)

    def _expand(self, active):
        names = list(ACTIVE_JOINTS)
        positions = [float(active[name]) for name in ACTIVE_JOINTS]
        for child, (parent, mult, offset) in MIMIC_JOINTS.items():
            names.append(child)
            positions.append(mult * float(active[parent]) + offset)
        return names, positions

    def _publish_active(self, active):
        names, positions = self._expand(active)
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = names
        msg.position = positions
        self._pub.publish(msg)

    def _active_from_trajectory_point(self, point, joint_names):
        active = {name: 0.0 for name in ACTIVE_JOINTS}
        for name in ACTIVE_JOINTS:
            if name in joint_names:
                active[name] = point.positions[joint_names.index(name)]
        return active

    def _on_display_trajectory(self, msg):
        if not msg.trajectory:
            return
        robot_traj = msg.trajectory[0]
        joint_names = list(robot_traj.joint_trajectory.joint_names)
        points = list(robot_traj.joint_trajectory.points)
        if not points or not joint_names:
            return
        with self._lock:
            traj = []
            for p in points:
                active = self._active_from_trajectory_point(p, joint_names)
                j2, j3 = active_j2_j3_deg(active)
                if not valid_region(j2, j3):
                    self.get_logger().warn(
                        "mimic preview: planned path leaves coupled region at "
                        "J2=%.2f J3=%.2f deg; truncating playback" % (j2, j3)
                    )
                    break
                traj.append(active)
            if not traj:
                self.get_logger().warn("mimic preview: no valid waypoints in planned path")
                return
            self._traj_points = traj
            self._traj_index = 0
            self._mode = "trajectory"
            self._user_preview = True
        self.get_logger().info(
            "mimic preview: playing planned path (%d/%d waypoints)"
            % (len(traj), len(points))
        )

    def _ik_seed(self):
        with self._lock:
            if self._user_preview:
                seed_joints = dict(self._goal_active)
            else:
                seed_joints = dict(self._current)
        state = RobotState()
        state.joint_state.name = list(ACTIVE_JOINTS)
        state.joint_state.position = [seed_joints[name] for name in ACTIVE_JOINTS]
        state.is_diff = True
        return state, seed_joints

    def _is_ee_marker(self, name):
        lower = name.lower()
        return (
            "ee" in lower
            or "goal" in lower
            or self._ee_link.lower() in lower
            or "arm_group" in lower
        )

    def _apply_ee_pose(self, header, pose):
        if not _pose_changed(self._last_ee_pose, pose):
            return
        now = self.get_clock().now()
        if (now - self._last_ik_time).nanoseconds * 1e-9 < self._ik_min_period:
            return
        if not self._ik.service_is_ready():
            return

        pose_stamped = PoseStamped()
        pose_stamped.header = header
        if not pose_stamped.header.frame_id:
            pose_stamped.header.frame_id = "world"
        pose_stamped.header.stamp.sec = 0
        pose_stamped.header.stamp.nanosec = 0
        pose_stamped.pose = pose

        seed, seed_joints = self._ik_seed()
        req = GetPositionIK.Request()
        req.ik_request.group_name = self._group
        req.ik_request.robot_state = seed
        req.ik_request.avoid_collisions = False
        req.ik_request.pose_stamped = pose_stamped
        req.ik_request.timeout = Duration(sec=0, nanosec=int(self._ik_timeout * 1e9))
        req.ik_request.ik_link_name = self._ee_link

        self._last_ee_pose = pose
        self._last_ik_time = now
        future = self._ik.call_async(req)
        future.add_done_callback(
            lambda fut, sj=seed_joints: self._on_ik_done(fut, sj)
        )

    def _on_ik_done(self, future, seed_joints):
        try:
            res = future.result()
        except Exception as exc:  # noqa: BLE001 - log and keep preview alive
            self.get_logger().warn("compute_ik failed: %s" % exc)
            return
        if res is None or res.error_code.val != MoveItErrorCodes.SUCCESS:
            return

        active = dict(seed_joints)
        js = res.solution.joint_state
        for name in ACTIVE_JOINTS:
            if name in js.name:
                active[name] = js.position[js.name.index(name)]

        j2, j3 = active_j2_j3_deg(active)
        if not valid_region(j2, j3):
            should_warn = not self._was_invalid
            if self._last_invalid_pair is not None:
                should_warn = should_warn or abs(j2 - self._last_invalid_pair[0]) >= 0.5
                should_warn = should_warn or abs(j3 - self._last_invalid_pair[1]) >= 0.5
            if should_warn:
                self.get_logger().warn(
                    "mimic preview: reject uncoupled J2/J3 (%.2f, %.2f deg)" % (j2, j3)
                )
                self._last_invalid_pair = (j2, j3)
            self._was_invalid = True
            return

        self._was_invalid = False
        self._last_invalid_pair = None
        with self._lock:
            self._mode = "goal"
            self._user_preview = True
            self._traj_points = []
            self._goal_active = active
        self._publish_active(active)

    def _on_marker_feedback(self, msg):
        if msg.event_type not in (
            InteractiveMarkerFeedback.POSE_UPDATE,
            InteractiveMarkerFeedback.MOUSE_UP,
            InteractiveMarkerFeedback.KEEP_ALIVE,
        ):
            return
        if msg.marker_name and not self._is_ee_marker(msg.marker_name):
            return
        self._apply_ee_pose(msg.header, msg.pose)

    def _on_marker_update(self, msg):
        entries = [(p.name, p.header, p.pose) for p in msg.poses]
        entries.extend((m.name, m.header, m.pose) for m in msg.markers)
        matched = [
            (name, header, pose)
            for name, header, pose in entries
            if not name or self._is_ee_marker(name)
        ]
        if not matched and entries:
            matched = entries[:1]
        for _name, header, pose in matched:
            self._apply_ee_pose(header, pose)

    def _on_timer(self):
        with self._lock:
            mode = self._mode
            if mode == "trajectory" and self._traj_points:
                active = self._traj_points[self._traj_index]
                self._traj_index += 1
                if self._traj_index >= len(self._traj_points):
                    self._mode = "goal"
                    self._goal_active = dict(active)
                    self._traj_points = []
            else:
                active = dict(self._goal_active)
        self._publish_active(active)


def main(args=None):
    rclpy.init(args=args)
    node = MimicPreviewNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
