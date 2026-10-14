#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Confirmed-execution slider GUI for the Pro630 simulator/robot bridge."""

import math
import random
import signal
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty, String, Bool

from python_qt_binding.QtCore import Qt, QTimer
from python_qt_binding.QtWidgets import (
    QApplication,
    QDoubleSpinBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QSlider,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)


from pro630_configuration import ARM_JOINTS, GRIPPER_JOINT, JOINT_LIMITS_RAD, joints, configure_node
COMMAND_JOINTS = joints(True)
RAD_DISPLAY_DECIMALS = 4
RAD_SLIDER_SCALE = 10 ** RAD_DISPLAY_DECIMALS
GRIPPER_DISPLAY_DECIMALS = 2
JOINT_DISPLAY_DECIMALS = [RAD_DISPLAY_DECIMALS] * 6 + [GRIPPER_DISPLAY_DECIMALS]
JOINT_SLIDER_SCALES = [10 ** decimals for decimals in JOINT_DISPLAY_DECIMALS]


class SliderGuiNode(Node):
    """ROS transport for the GUI; targets and actual feedback stay separate."""

    def __init__(self):
        super().__init__("pro630_slider_gui")
        configure_node(self)
        self.declare_parameter("environment", "simulation")
        self.declare_parameter("show_force_execute", False)
        self.show_force_execute = self.get_parameter("show_force_execute").value
        self.environment = str(self.get_parameter("environment").value).strip().lower()
        if self.environment not in ("simulation", "real"):
            raise ValueError("environment must be simulation or real")
        self.target_pub = self.create_publisher(
            JointState, "/pro630/slider_targets", 10
        )
        self.force_target_pub = self.create_publisher(
            JointState, "/pro630/slider_force_targets", 10
        )
        self.stop_pub = self.create_publisher(Empty, "/pro630/slider_stop", 10)
        feedback_topic = (
            "/pro630/real_joint_states" if self.environment == "real" else "/joint_states"
        )
        self.create_subscription(JointState, feedback_topic, self._joint_cb, 1)
        self.create_subscription(String, "/pro630/slider_status", self._status_cb, 10)

        self._lock = threading.Lock()
        self._status_revision = 0
        self._command_busy = False
        self._command_pending = False
        self.create_subscription(Bool, "/pro630/slider_busy", self._busy_cb, 10)
        self._actual = None
        self._actual_time = 0.0
        self._feedback_valid = False
        self._status = (
            "Waiting for real robot feedback..."
            if self.environment == "real"
            else "Waiting for Gazebo joint feedback..."
        )

    def _joint_cb(self, msg):
        values = dict(zip(msg.name, msg.position))
        if not all(name in values for name in self.command_joints):
            return
        positions = [float(values[name]) for name in self.command_joints]
        velocities = dict(zip(msg.name, msg.velocity)) if len(msg.velocity) == len(msg.name) else {}
        feedback_valid = all(math.isfinite(value) for value in positions) and all(
            name in velocities and math.isfinite(float(velocities[name]))
            for name in self.command_joints
        )
        with self._lock:
            age = 0.0
            if self.environment == "real":
                stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
                age = max(0.0, time.time() - stamp) if stamp > 0 else float('inf')
            self._actual = positions
            self._actual_time = time.monotonic() - age
            self._feedback_valid = feedback_valid


    def _status_cb(self, msg):
        with self._lock:
            self._status = msg.data
            self._status_revision += 1
            # A request can be rejected before the controller enters busy.
            # Clearing pending does not override the controller's busy state.
            if msg.data.startswith(("Rejected:", "Cancelled", "Execution complete",
                                    "Execution stopped", "Execution failed:",
                                    "Real robot command failed:")):
                self._command_pending = False

    def _busy_cb(self, msg):
        with self._lock:
            self._command_busy = bool(msg.data)
            if self._command_busy:
                self._command_pending = False

    def command_busy(self):
        with self._lock:
            return self._command_pending or self._command_busy



    def snapshot(self, include_status_revision=False):
        with self._lock:
            actual = list(self._actual) if self._actual is not None else None
            values = (actual, self._actual_time, self._feedback_valid, self._status)
            return values + (self._status_revision,) if include_status_revision else values


    def execute(self, positions_rad, speed_percent, force_collision=False):
        if force_collision and self.environment != "simulation":
            return False
        with self._lock:
            if self._command_pending or self._command_busy:
                return False
            # Block another click immediately, before ROS acknowledges it.
            self._command_pending = True
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.command_joints)
        msg.position = list(positions_rad)
        # On this command topic velocity carries the selected speed ratio.
        # The backend applies each joint's own URDF velocity limit.
        speed_rad_s = max(0.01, min(1.0, speed_percent / 100.0))
        msg.velocity = [speed_rad_s] * len(self.command_joints)
        publisher = self.force_target_pub if force_collision else self.target_pub
        try:
            publisher.publish(msg)
        except Exception:
            with self._lock:
                self._command_pending = False
            raise
        return True

    def stop(self):
        self.stop_pub.publish(Empty())


class Pro630SliderWindow(QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.target_initialized = False
        self.user_edited_target = False
        self.last_status = None
        self.last_status_revision = None
        self.sliders = []
        self.target_boxes = []
        self.actual_labels = []

        self.setWindowTitle("Pro630 Confirmed Slider Control")
        self.resize(980, 430)

        root = QWidget()
        self.setCentralWidget(root)
        outer = QVBoxLayout(root)

        joint_group = QGroupBox("Joint Targets")
        grid = QGridLayout(joint_group)
        grid.addWidget(QLabel("Joint"), 0, 0)
        grid.addWidget(QLabel("Target (rad)"), 0, 1)
        grid.addWidget(QLabel("Slider"), 0, 2)
        grid.addWidget(QLabel("Actual (rad)"), 0, 3)

        labels = ["Joint 1", "Joint 2", "Joint 3", "Joint 4", "Joint 5", "Joint 6", "Gripper"]
        for row, (label, limits) in enumerate(zip(labels, self.node.joint_limits), start=1):
            minimum, maximum = limits
            decimals = JOINT_DISPLAY_DECIMALS[row - 1]
            scale = JOINT_SLIDER_SCALES[row - 1]
            minimum = math.ceil(minimum * scale) / scale
            maximum = math.floor(maximum * scale) / scale
            target_box = QDoubleSpinBox()
            target_box.setDecimals(decimals)
            target_box.setRange(minimum, maximum)
            target_box.setSingleStep(0.01)

            slider = QSlider(Qt.Horizontal)
            slider.setFixedWidth(560)
            slider.setRange(
                round(minimum * scale),
                round(maximum * scale),
            )
            slider.setSingleStep(round(0.01 * scale))

            actual_label = QLabel("--")
            actual_label.setFixedWidth(100)

            slider.valueChanged.connect(
                lambda value, box=target_box, scale=scale: box.setValue(
                    value / scale
                )
            )
            target_box.valueChanged.connect(
                lambda value, control=slider, scale=scale: control.setValue(
                    round(value * scale)
                )
            )
            target_box.valueChanged.connect(self._target_edited)

            grid.addWidget(QLabel(label), row, 0)
            grid.addWidget(target_box, row, 1)
            grid.addWidget(slider, row, 2)
            grid.addWidget(actual_label, row, 3)

            self.target_boxes.append(target_box)
            self.sliders.append(slider)
            self.actual_labels.append(actual_label)

        outer.addWidget(joint_group)
        controls = QHBoxLayout()
        controls.addWidget(QLabel("Speed (%)"))
        self.speed_box = QSpinBox()
        self.speed_box.setRange(1, 100)
        self.speed_box.setValue(20)
        self.speed_box.setSuffix(" %")
        controls.addWidget(self.speed_box)
        controls.addStretch(1)

        self.randomize_button = QPushButton("Randomize Target")
        self.randomize_button.clicked.connect(self._randomize_target)
        controls.addWidget(self.randomize_button)

        self.zero_button = QPushButton("Zero Target")
        self.zero_button.clicked.connect(self._zero_target)
        controls.addWidget(self.zero_button)

        self.execute_button = QPushButton("Execute")
        self.execute_button.setEnabled(False)
        self.execute_button.clicked.connect(self._execute)
        controls.addWidget(self.execute_button)

        self.force_execute_button = QPushButton("Force Execute (Sim Only)")
        self.force_execute_button.setEnabled(self.node.environment != "simulation")
        self.force_execute_button.setStyleSheet(
            "font-weight: bold; color: #7a3e00; background: #ffd180;"
        )
        self.force_execute_button.clicked.connect(self._force_execute)
        controls.addWidget(self.force_execute_button)
        self.force_execute_button.setVisible(self.node.show_force_execute)

        self.stop_button = QPushButton("STOP")
        self.stop_button.setStyleSheet("font-weight: bold; color: white; background: #b00020;")
        self.stop_button.clicked.connect(self._stop)
        controls.addWidget(self.stop_button)
        outer.addLayout(controls)

        self.status_label = QLabel(self.node._status)
        self.status_label.setWordWrap(True)
        outer.addWidget(self.status_label)

        self.refresh_timer = QTimer(self)
        self.refresh_timer.timeout.connect(self._refresh)
        self.refresh_timer.start(100)

    def _target_edited(self, _value):
        if self.target_initialized:
            self.user_edited_target = True

    def _refresh(self):
        actual, actual_time, state_is_finite, status, revision = self.node.snapshot(include_status_revision=True)
        feedback_ok = (
            actual is not None
            and time.monotonic() - actual_time < 1.0
            and state_is_finite
        )

        if actual is not None:
            for index, (label, value) in enumerate(zip(self.actual_labels, actual)):
                label.setText(
                    "NaN"
                    if not math.isfinite(value)
                    else f"{value:.{JOINT_DISPLAY_DECIMALS[index]}f}"
                )
        if feedback_ok and not self.target_initialized:
            self.target_initialized = True
            for index, (box, slider, value) in enumerate(zip(self.target_boxes, self.sliders, actual)):
                box.blockSignals(True)
                slider.blockSignals(True)
                box.setValue(value)
                slider.setValue(round(box.value() * JOINT_SLIDER_SCALES[index]))
                box.blockSignals(False)
                slider.blockSignals(False)
            self.user_edited_target = False

        can_execute = feedback_ok and not self.node.command_busy()
        self.execute_button.setEnabled(can_execute)
        self.force_execute_button.setEnabled(self.node.environment != "simulation" or can_execute)
        if not feedback_ok:
            self.last_status = None
            self.status_label.setText(
                (f"Execute disabled: real robot feedback is stale (age {time.monotonic() - actual_time:.1f}s)."
                 if actual is not None else "Execute disabled: waiting for real robot feedback.")
                if self.node.environment == "real"
                else "Execute disabled: Gazebo position/velocity feedback is missing, stale, or contains NaN. Restart/reset the simulation first."
            )
        elif status != self.last_status or revision != self.last_status_revision:
            self.last_status = status
            self.last_status_revision = revision
            self.status_label.setText(status)

    def _execute(self):
        positions = self._target_positions_rad()
        previous_status = self.node.snapshot(include_status_revision=True)[3:]
        if not self.node.execute(positions, self.speed_box.value()):
            return
        self.execute_button.setEnabled(False)
        self.force_execute_button.setEnabled(self.node.environment != "simulation")
        # Capture before publish so even an immediate reply is rendered.
        self.last_status, self.last_status_revision = previous_status
        self.status_label.setText("Command submitted; waiting for validation...")
        self.user_edited_target = False

    def _force_execute(self):
        if self.node.environment != "simulation":
            QMessageBox.warning(
                self, "Force Execute Unavailable",
                "Force Execute is available only in Simulation mode.\n"
                "This action is disabled in Real Robot mode. No motion command has been sent.",
            )
            return
        if self.node.command_busy():
            return
        answer = QMessageBox.warning(
            self,
            "Simulation Collision Override",
            "This bypasses MoveIt collision rejection and is allowed only in "
            "Gazebo simulation mode. Motion uses the speed selected in the slider window. "
            "Joint limits, finite feedback, and STOP remain active. Gazebo physical self-contact is "
            "disabled, so intersecting robot links will not stop the motion.\n\n"
            "Execute this target anyway?",
            QMessageBox.Yes | QMessageBox.Cancel,
            QMessageBox.Cancel,
        )
        if answer != QMessageBox.Yes:
            return

        positions = self._target_positions_rad()
        forced_speed = self.speed_box.value()
        previous_status = self.node.snapshot(include_status_revision=True)[3:]
        if not self.node.execute(positions, forced_speed, force_collision=True):
            return
        self.execute_button.setEnabled(False)
        self.force_execute_button.setEnabled(False)
        # Capture before publish so even an immediate reply is rendered.
        self.last_status, self.last_status_revision = previous_status
        self.status_label.setText(
            f"FORCE simulation command submitted at {forced_speed}% maximum; "
            "collision validation and physical self-contact will be bypassed."
        )
        self.user_edited_target = False

    def _randomize_target(self):
        """Generate a target locally; motion still requires Execute."""
        for box, limits in zip(self.target_boxes[:6], self.node.joint_limits[:6]):
            box.setValue(random.uniform(*limits))
        # Match the integer 0..100 opening accepted by the gripper interface.
        if self.node.with_gripper:
            self.target_boxes[-1].setValue(random.randint(0, 100) / 100.0)
        self.user_edited_target = True
        self.status_label.setText(
            "Random target generated. Review it, then press Execute to validate and move."
        )

    def _zero_target(self):
        """Load the all-zero target locally; motion still requires Execute."""
        for box in self.target_boxes:
            box.setValue(0.0)
        self.user_edited_target = True
        self.status_label.setText(
            "Zero target loaded. Press Execute to validate and return to zero."
        )

    def _target_positions_rad(self):
        """Read radian targets and clamp display rounding to exact URDF limits."""
        positions = [
            min(max(box.value(), lower), upper)
            for box, (lower, upper) in zip(self.target_boxes, self.node.joint_limits)
        ]
        if self.node.with_gripper:
            positions[-1] = round(positions[-1] * 100) / 100.0
        return positions

    def _stop(self):
        self.node.stop()
        self.status_label.setText("STOP requested.")


def main(args=None):
    rclpy.init(args=args)
    node = SliderGuiNode()
    app = QApplication(sys.argv)
    window = Pro630SliderWindow(node)

    # Let Qt own process termination.  rclpy's default signal path can shut
    # the context down in the middle of a Qt timer callback and print a
    # spurious KeyboardInterrupt traceback during an otherwise normal exit.
    signal.signal(signal.SIGINT, lambda *_args: app.quit())
    signal.signal(signal.SIGTERM, lambda *_args: app.quit())

    ros_timer = QTimer()

    def spin_ros_once():
        if not rclpy.ok():
            app.quit()
            return
        try:
            rclpy.spin_once(node, timeout_sec=0.0)
        except KeyboardInterrupt:
            app.quit()
        except Exception:
            # launch may shut the ROS context down while this Qt callback is
            # already running.  Exit quietly in that expected shutdown race.
            if not rclpy.ok():
                app.quit()
                return
            raise

    ros_timer.timeout.connect(spin_ros_once)
    ros_timer.start(10)

    window.show()
    try:
        exit_code = app.exec_()
    except KeyboardInterrupt:
        exit_code = 0
    finally:
        ros_timer.stop()
        node.destroy_node()
        rclpy.try_shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
