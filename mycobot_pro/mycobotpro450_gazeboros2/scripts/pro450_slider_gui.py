#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Confirmed-execution slider GUI for the Pro450 simulator/robot bridge."""

import math
import random
import signal
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty, String

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


ARM_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
GRIPPER_JOINT = "gripper_controller"
COMMAND_JOINTS = ARM_JOINTS + [GRIPPER_JOINT]
JOINT_LIMITS_RAD = [
    (math.radians(-162.0), math.radians(162.0)),
    (math.radians(-125.0), math.radians(125.0)),
    (math.radians(-154.0), math.radians(154.0)),
    (math.radians(-162.0), math.radians(162.0)),
    (math.radians(-162.0), math.radians(162.0)),
    (math.radians(-165.0), math.radians(165.0)),
    (0.0, 1.0),
]
RAD_DISPLAY_DECIMALS = 4
RAD_SLIDER_SCALE = 10 ** RAD_DISPLAY_DECIMALS
FORCE_EXECUTE_MAX_SPEED_PERCENT = 10


class SliderGuiNode(Node):
    """ROS transport for the GUI; targets and actual feedback stay separate."""

    def __init__(self):
        super().__init__("pro450_slider_gui")
        self.target_pub = self.create_publisher(
            JointState, "/pro450/slider_targets", 10
        )
        self.force_target_pub = self.create_publisher(
            JointState, "/pro450/slider_force_targets", 10
        )
        self.stop_pub = self.create_publisher(Empty, "/pro450/slider_stop", 10)
        self.create_subscription(JointState, "/joint_states", self._joint_cb, 10)
        self.create_subscription(String, "/pro450/slider_status", self._status_cb, 10)

        self._lock = threading.Lock()
        self._actual = None
        self._actual_time = 0.0
        self._feedback_valid = False
        self._status = "Waiting for Gazebo joint feedback..."

    def _joint_cb(self, msg):
        values = dict(zip(msg.name, msg.position))
        if not all(name in values for name in COMMAND_JOINTS):
            return
        positions = [float(values[name]) for name in COMMAND_JOINTS]
        velocities = dict(zip(msg.name, msg.velocity)) if len(msg.velocity) == len(msg.name) else {}
        feedback_valid = all(math.isfinite(value) for value in positions) and all(
            name in velocities and math.isfinite(float(velocities[name]))
            for name in COMMAND_JOINTS
        )
        with self._lock:
            self._actual = positions
            self._actual_time = time.monotonic()
            self._feedback_valid = feedback_valid

    def _status_cb(self, msg):
        with self._lock:
            self._status = msg.data

    def snapshot(self):
        with self._lock:
            actual = list(self._actual) if self._actual is not None else None
            return actual, self._actual_time, self._feedback_valid, self._status

    def execute(self, positions_rad, speed_percent, force_collision=False):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(COMMAND_JOINTS)
        msg.position = list(positions_rad)
        # The dedicated target topic uses velocity as a per-joint speed limit.
        # All URDF velocity limits are currently 1 rad/s, so this also carries
        # the requested 1..100 percent speed scale without a custom message.
        speed_rad_s = max(0.01, min(1.0, speed_percent / 100.0))
        msg.velocity = [speed_rad_s] * len(COMMAND_JOINTS)
        publisher = self.force_target_pub if force_collision else self.target_pub
        publisher.publish(msg)

    def stop(self):
        self.stop_pub.publish(Empty())


class Pro450SliderWindow(QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.target_initialized = False
        self.user_edited_target = False
        self.last_status = None
        self.sliders = []
        self.target_boxes = []
        self.actual_labels = []

        self.setWindowTitle("Pro450 Confirmed Slider Control")
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
        for row, (label, limits) in enumerate(zip(labels, JOINT_LIMITS_RAD), start=1):
            minimum, maximum = limits
            target_box = QDoubleSpinBox()
            target_box.setRange(minimum, maximum)
            target_box.setDecimals(RAD_DISPLAY_DECIMALS)
            target_box.setSingleStep(0.01)

            slider = QSlider(Qt.Horizontal)
            slider.setRange(
                round(minimum * RAD_SLIDER_SCALE),
                round(maximum * RAD_SLIDER_SCALE),
            )
            slider.setSingleStep(round(0.01 * RAD_SLIDER_SCALE))

            actual_label = QLabel("--")
            actual_label.setMinimumWidth(90)

            slider.valueChanged.connect(
                lambda value, box=target_box: box.setValue(
                    value / RAD_SLIDER_SCALE
                )
            )
            target_box.valueChanged.connect(
                lambda value, control=slider: control.setValue(
                    round(value * RAD_SLIDER_SCALE)
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
        self.force_execute_button.setEnabled(False)
        self.force_execute_button.setStyleSheet(
            "font-weight: bold; color: #7a3e00; background: #ffd180;"
        )
        self.force_execute_button.clicked.connect(self._force_execute)
        controls.addWidget(self.force_execute_button)

        self.stop_button = QPushButton("STOP")
        self.stop_button.setStyleSheet("font-weight: bold; color: white; background: #b00020;")
        self.stop_button.clicked.connect(self._stop)
        controls.addWidget(self.stop_button)
        outer.addLayout(controls)

        self.status_label = QLabel("Waiting for Gazebo joint feedback...")
        self.status_label.setWordWrap(True)
        outer.addWidget(self.status_label)

        self.refresh_timer = QTimer(self)
        self.refresh_timer.timeout.connect(self._refresh)
        self.refresh_timer.start(100)

    def _target_edited(self, _value):
        if self.target_initialized:
            self.user_edited_target = True

    def _refresh(self):
        actual, actual_time, state_is_finite, status = self.node.snapshot()
        feedback_ok = (
            actual is not None
            and time.monotonic() - actual_time < 1.0
            and state_is_finite
        )

        if actual is not None:
            for label, value in zip(self.actual_labels, actual):
                label.setText(
                    "NaN"
                    if not math.isfinite(value)
                    else f"{value:.{RAD_DISPLAY_DECIMALS}f}"
                )

        if feedback_ok and not self.target_initialized:
            self.target_initialized = True
            for box, slider, value in zip(self.target_boxes, self.sliders, actual):
                box.blockSignals(True)
                slider.blockSignals(True)
                box.setValue(value)
                slider.setValue(round(value * RAD_SLIDER_SCALE))
                box.blockSignals(False)
                slider.blockSignals(False)
            self.user_edited_target = False

        self.execute_button.setEnabled(feedback_ok)
        self.force_execute_button.setEnabled(feedback_ok)
        if not feedback_ok:
            self.status_label.setText(
                "Execute disabled: Gazebo position/velocity feedback is missing, stale, or contains NaN. Restart/reset the simulation first."
            )
        elif status != self.last_status:
            self.last_status = status
            self.status_label.setText(status)

    def _execute(self):
        positions = self._target_positions_rad()
        self.node.execute(positions, self.speed_box.value())
        self.status_label.setText("Command submitted; waiting for validation...")
        self.user_edited_target = False

    def _force_execute(self):
        answer = QMessageBox.warning(
            self,
            "Simulation Collision Override",
            "This bypasses MoveIt collision rejection and is allowed only in "
            "Gazebo simulation mode. Joint limits, finite feedback, STOP, and "
            "a 10% speed cap remain active. Gazebo physical self-contact is "
            "disabled, so intersecting robot links will not stop the motion.\n\n"
            "Execute this target anyway?",
            QMessageBox.Yes | QMessageBox.Cancel,
            QMessageBox.Cancel,
        )
        if answer != QMessageBox.Yes:
            return

        positions = self._target_positions_rad()
        forced_speed = min(
            self.speed_box.value(), FORCE_EXECUTE_MAX_SPEED_PERCENT
        )
        self.node.execute(positions, forced_speed, force_collision=True)
        self.status_label.setText(
            f"FORCE simulation command submitted at {forced_speed}% maximum; "
            "collision validation and physical self-contact will be bypassed."
        )
        self.user_edited_target = False

    def _randomize_target(self):
        """Generate a target locally; motion still requires Execute."""
        for box, limits in zip(self.target_boxes, JOINT_LIMITS_RAD):
            box.setValue(random.uniform(*limits))
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
        return [
            min(max(box.value(), lower), upper)
            for box, (lower, upper) in zip(self.target_boxes, JOINT_LIMITS_RAD)
        ]

    def _stop(self):
        self.node.stop()
        self.status_label.setText("STOP requested.")


def main(args=None):
    rclpy.init(args=args)
    node = SliderGuiNode()
    app = QApplication(sys.argv)
    window = Pro450SliderWindow(node)

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
