# -*- coding: utf-8 -*-
"""Safe JointState slider GUI for Pro450 force-gripper operation (ROS2).

Mirrors ROS1 ``safe_joint_state_publisher_gui.py``: joint sliders, numeric
fields, and Center — without Randomize.
"""

from __future__ import annotations

import math
import signal
import sys
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

try:
    from PyQt5.QtCore import Qt, QTimer
    from PyQt5.QtWidgets import (
        QAbstractSpinBox,
        QApplication,
        QDoubleSpinBox,
        QFrame,
        QGridLayout,
        QLabel,
        QPushButton,
        QSlider,
        QSpinBox,
        QVBoxLayout,
        QWidget,
    )
except ImportError as exc:
    raise RuntimeError(
        "PyQt5 is required for safe_joint_state_publisher_gui"
    ) from exc


SLIDER_STEPS = 10000
PUBLISH_PERIOD_MS = 50
EXPECTED_JOINTS = (
    "joint1",
    "joint2",
    "joint3",
    "joint4",
    "joint5",
    "joint6",
    "gripper_controller",
)


def load_joints_from_urdf(description: str, logger=None):
    """Read controllable joints and limits from a URDF string."""
    root = ET.fromstring(description)
    discovered = {}
    for joint in root.findall("joint"):
        name = joint.get("name")
        if not name or joint.get("type") == "fixed" or joint.find("mimic") is not None:
            continue
        limit = joint.find("limit")
        if joint.get("type") == "continuous":
            lower, upper = -math.pi, math.pi
        elif (
            limit is not None
            and limit.get("lower") is not None
            and limit.get("upper") is not None
        ):
            lower, upper = float(limit.get("lower")), float(limit.get("upper"))
        else:
            continue
        discovered[name] = (lower, upper)

    joints = [(name, *discovered[name]) for name in EXPECTED_JOINTS if name in discovered]
    if len(joints) != len(EXPECTED_JOINTS):
        if logger is not None:
            logger.warn(
                "Safe slider did not find all expected Pro450 joints; "
                "using all controllable URDF joints"
            )
        joints = [(name, *limits) for name, limits in discovered.items()]
    if not joints:
        raise RuntimeError("No controllable joints in robot_description")
    return joints


class SafeJointStatePublisher(QWidget):
    """A no-randomize JointState GUI."""

    def __init__(self, node: Node, joints):
        super().__init__()
        self.node = node
        self.joints = joints
        self.values = [min(max(0.0, lower), upper) for _, lower, upper in joints]
        self.sliders = []
        self.spins = []
        self._closing = False
        self.publisher = node.create_publisher(JointState, "joint_states", 10)

        self.setWindowTitle("Node: /joint_state_publisher_gui (safe)")
        self.setFixedWidth(360)
        self._build_layout()

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.publish)
        self.timer.start(PUBLISH_PERIOD_MS)

    def _build_layout(self):
        outer = QVBoxLayout(self)
        panel = QFrame()
        panel.setFrameShape(QFrame.StyledPanel)
        panel_layout = QVBoxLayout(panel)
        grid = QGridLayout()
        grid.setHorizontalSpacing(8)
        grid.setVerticalSpacing(2)
        grid.setColumnStretch(1, 1)
        panel_layout.addLayout(grid)

        for row, (name, lower, upper) in enumerate(self.joints):
            grid_row = row * 2
            label = QLabel(name)
            slider = QSlider(Qt.Horizontal)
            slider.setRange(0, SLIDER_STEPS)
            slider.setValue(self._slider_value(row, self.values[row]))

            spin = QDoubleSpinBox()
            spin.setDecimals(3)
            spin.setRange(lower, upper)
            spin.setSingleStep((upper - lower) / 100.0)
            spin.setValue(self.values[row])
            spin.setButtonSymbols(QAbstractSpinBox.NoButtons)
            spin.setMinimumWidth(0)

            slider.valueChanged.connect(
                lambda value, index=row: self._slider_changed(index, value)
            )
            spin.valueChanged.connect(
                lambda value, index=row: self._spin_changed(index, value)
            )

            grid.addWidget(label, grid_row, 0)
            grid.addWidget(spin, grid_row, 1, 1, 2)
            grid.addWidget(slider, grid_row + 1, 0, 1, 3)
            self.sliders.append(slider)
            self.spins.append(spin)

        panel_layout.addStretch(1)
        center = QPushButton("Center")
        center.clicked.connect(self.center)
        panel_layout.addWidget(center)

        joint_count = QSpinBox()
        joint_count.setRange(len(self.joints), len(self.joints))
        joint_count.setValue(len(self.joints))
        joint_count.setReadOnly(True)
        panel_layout.addWidget(joint_count)
        outer.addWidget(panel)

    def _slider_value(self, index, value):
        _, lower, upper = self.joints[index]
        return int(round((value - lower) * SLIDER_STEPS / (upper - lower)))

    def _joint_value(self, index, slider_value):
        _, lower, upper = self.joints[index]
        return lower + (upper - lower) * slider_value / float(SLIDER_STEPS)

    def _slider_changed(self, index, slider_value):
        value = self._joint_value(index, slider_value)
        self.values[index] = value
        self.spins[index].blockSignals(True)
        self.spins[index].setValue(value)
        self.spins[index].blockSignals(False)

    def _spin_changed(self, index, value):
        self.values[index] = value
        self.sliders[index].blockSignals(True)
        self.sliders[index].setValue(self._slider_value(index, value))
        self.sliders[index].blockSignals(False)

    def center(self):
        for index, (_, lower, upper) in enumerate(self.joints):
            self.spins[index].setValue(min(max(0.0, lower), upper))

    def stop(self):
        """Stop timers so shutdown does not keep publishing."""
        self._closing = True
        if self.timer.isActive():
            self.timer.stop()

    def publish(self):
        if self._closing or not rclpy.ok():
            self.stop()
            return
        try:
            message = JointState()
            message.header.stamp = self.node.get_clock().now().to_msg()
            message.name = [name for name, _, _ in self.joints]
            message.position = list(self.values)
            self.publisher.publish(message)
        except Exception:
            # Context already torn down during Ctrl+C / launch shutdown.
            self.stop()

    def closeEvent(self, event):
        self.stop()
        app = QApplication.instance()
        if app is not None:
            app.quit()
        super().closeEvent(event)


class SafeJointStatePublisherNode(Node):
    def __init__(self):
        super().__init__("safe_joint_state_publisher_gui")
        self.declare_parameter("robot_description", "")
        self.declare_parameter("use_gui", True)
        description = (
            self.get_parameter("robot_description").get_parameter_value().string_value
        )
        if not description.strip():
            raise RuntimeError(
                "robot_description parameter is empty; pass the URDF from launch"
            )
        self.joints = load_joints_from_urdf(description, self.get_logger())
        self.use_gui = (
            self.get_parameter("use_gui").get_parameter_value().bool_value
        )


def main(args=None):
    rclpy.init(args=args)
    node = SafeJointStatePublisherNode()
    if not node.use_gui:
        publisher = node.create_publisher(JointState, "joint_states", 10)
        values = [min(max(0.0, lower), upper) for _, lower, upper in node.joints]
        rate = node.create_rate(20)
        try:
            while rclpy.ok():
                message = JointState()
                message.header.stamp = node.get_clock().now().to_msg()
                message.name = [name for name, _, _ in node.joints]
                message.position = values
                publisher.publish(message)
                rclpy.spin_once(node, timeout_sec=0.0)
                rate.sleep()
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        return

    app = QApplication(sys.argv)
    window = SafeJointStatePublisher(node, node.joints)
    window.show()

    spin_timer = QTimer()

    def _shutdown_gui():
        window.stop()
        if spin_timer.isActive():
            spin_timer.stop()
        app.quit()

    def _spin_ros():
        if window._closing or not rclpy.ok():
            _shutdown_gui()
            return
        try:
            rclpy.spin_once(node, timeout_sec=0.0)
        except Exception:
            _shutdown_gui()

    spin_timer.timeout.connect(_spin_ros)
    spin_timer.start(10)

    def _on_signal(_signum, _frame):
        # Qt must quit from the event loop; the next timer tick also covers
        # launch tearing down rclpy before this handler runs.
        _shutdown_gui()

    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)

    exit_code = 0
    try:
        exit_code = app.exec_()
    finally:
        window.stop()
        if spin_timer.isActive():
            spin_timer.stop()
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
