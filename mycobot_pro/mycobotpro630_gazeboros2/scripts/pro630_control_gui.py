#!/usr/bin/env python3
"""Qt input/feedback window. All computation and hardware access stay outside Qt."""
import json
import math
import signal
import sys
import threading
import time
import uuid

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Empty, Float64MultiArray
from sensor_msgs.msg import JointState
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QGridLayout,
    QVBoxLayout, QHBoxLayout, QGroupBox, QLabel, QDoubleSpinBox, QSpinBox,
    QPushButton)

from pro630_control_logging import ControlLog, command_details, track_speed
from pro630_configuration import configure_node, JOINT_LIMITS_RAD, GRIPPER_JOINT


class GuiTransport(Node):
    def __init__(self):
        super().__init__('pro630_control_gui')
        configure_node(self)
        self.lock, self.snapshot, self.received = threading.Lock(), None, 0.
        self.command_pub = self.create_publisher(String, '/pro630/gui/command', 10)
        self.stop_pub = self.create_publisher(Empty, '/pro630/gui_stop', 10)
        self.create_subscription(String, '/pro630/gui/feedback', self.feedback, 1)
        self.gripper_measured = None
        self.gripper_received = self.gripper_sampled = float('-inf')
        if self.with_gripper:
            # Reuse public measurements; this display never queries the SDK.
            self.create_subscription(JointState, '/pro630/real_joint_states', self.gripper_feedback, 1)
            self.create_subscription(Float64MultiArray, '/pro630/real_feedback_age', self.gripper_age, 1)

    def gripper_feedback(self, msg):
        values = dict(zip(msg.name, msg.position))
        value = values.get(GRIPPER_JOINT)
        if type(value) in (int, float) and math.isfinite(value) and 0 <= value <= 1:
            with self.lock:
                self.gripper_measured = value * 100.
                self.gripper_received = time.monotonic()

    def gripper_age(self, msg):
        age = msg.data[1] if len(msg.data) > 1 else float('inf')
        with self.lock:
            self.gripper_sampled = (time.monotonic()-age
                                    if math.isfinite(age) and age >= 0 else float('-inf'))

    def measured_gripper(self):
        with self.lock:
            now = time.monotonic()
            if now-self.gripper_received <= 1. and now-self.gripper_sampled <= 1.:
                return self.gripper_measured
            return None

    def feedback(self, msg):
        try:
            payload = json.loads(msg.data)
            with self.lock:
                self.snapshot, self.received = payload, time.monotonic()
        except (ValueError, TypeError):
            pass


class ControlWindow(QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.node, self.seeded, self.pending = node, False, False
        self.log = ControlLog(node.get_logger())
        self.pending_since = 0.
        self.command_id = ''
        self.setWindowTitle('Pro630 GUI Control')
        central, layout = QWidget(), QVBoxLayout()
        central.setLayout(layout)
        self.setCentralWidget(central)
        grid = QGridLayout()
        layout.addLayout(grid)
        self.joint_inputs, self.coord_inputs = [], []
        self.joint_feedback, self.coord_feedback = [], []
        for col, title, names, unit, inputs, feedback in (
                (0, 'Joint targets / measured angles', [f'Joint {i}' for i in range(1, 7)], '°', self.joint_inputs, self.joint_feedback),
                (1, 'Cartesian targets / measured pose', ['X', 'Y', 'Z', 'RX', 'RY', 'RZ'], None, self.coord_inputs, self.coord_feedback)):
            group, inner = QGroupBox(title), QGridLayout()
            group.setLayout(inner)
            grid.addWidget(group, 0, col)
            inner.addWidget(QLabel('Target'), 0, 1)
            inner.addWidget(QLabel('Measured'), 0, 2)
            for i, name in enumerate(names):
                spin = QDoubleSpinBox()
                spin.setDecimals(2)
                spin.setSingleStep(1.)
                if col == 0:
                    lo, hi = JOINT_LIMITS_RAD[i]
                    spin.setRange(math.degrees(lo), math.degrees(hi))
                else:
                    spin.setRange(-5000., 5000.) if i < 3 else spin.setRange(-180., 180.)
                spin.setSuffix(' ' + (unit or ('mm' if i < 3 else '°')))
                spin.setMinimumWidth(130)
                value = QLabel('—')
                value.setMinimumWidth(130)
                value.setTextInteractionFlags(Qt.TextSelectableByMouse)
                inner.addWidget(QLabel(name), i+1, 0)
                inner.addWidget(spin, i+1, 1)
                inner.addWidget(value, i+1, 2)
                inputs.append(spin)
                feedback.append(value)
        self.joints_button = QPushButton('Set Joints')
        self.coords_button = QPushButton('Set Coordinates')
        grid.addWidget(self.joints_button, 1, 0)
        grid.addWidget(self.coords_button, 1, 1)
        self.joints_button.clicked.connect(lambda: self.submit('joints', self.joint_inputs))
        self.coords_button.clicked.connect(lambda: self.submit('coords', self.coord_inputs))
        options = QHBoxLayout()
        options.addWidget(QLabel('Speed'))
        self.speed = QSpinBox()
        self.speed.setRange(1, 100)
        self.speed.setValue(20)
        self.speed.setSuffix(' %')
        track_speed(self.speed, self.log)
        options.addWidget(self.speed)
        options.addStretch()
        self.clear_button = QPushButton('Clear Errors')
        self.clear_button.clicked.connect(lambda: self.send('clear_errors', []))
        options.addWidget(self.clear_button)
        self.command_buttons = [self.joints_button, self.coords_button, self.clear_button]
        if node.with_gripper:
            group, gripper = QGroupBox('Gripper'), QGridLayout()
            group.setLayout(gripper)
            layout.addWidget(group)
            gripper.addWidget(QLabel('Target (0–100)'), 0, 0)
            self.gripper_target = QSpinBox()
            self.gripper_target.setRange(0, 100)
            self.gripper_button = QPushButton('Set Gripper')
            self.gripper_button.clicked.connect(lambda: self.send_gripper())
            gripper.addWidget(self.gripper_target, 0, 1)
            gripper.addWidget(self.gripper_button, 0, 2, 1, 2)
            self.command_buttons.append(self.gripper_button)
            gripper.addWidget(QLabel('Measured (0–100)'), 0, 4)
            self.gripper_feedback = QLabel('—')
            self.gripper_feedback.setMinimumWidth(65)
            self.gripper_feedback.setTextInteractionFlags(Qt.TextSelectableByMouse)
            gripper.addWidget(self.gripper_feedback, 0, 5)
        layout.addLayout(options)
        stop = QPushButton('STOP')
        stop.setStyleSheet('background: #ca2547; color: white; font-weight: bold; padding: 10px;')
        stop.clicked.connect(self.stop)
        layout.addWidget(stop)
        self.status = QLabel('Connecting to the robot...')
        self.log.event(self.status.text())
        self.status.setWordWrap(True)
        self.status.setMinimumHeight(50)
        layout.addWidget(self.status)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(50)
        self.resize(850, 520 if node.with_gripper else 460)
        self.refresh()

    def submit(self, kind, inputs):
        for control in inputs:
            control.interpretText()
        self.send(kind, [control.value() for control in inputs])

    def send_gripper(self):
        self.gripper_target.interpretText()
        # Keep the existing normalized RPC target; the SDK receives 0..100.
        self.send('gripper', [self.gripper_target.value() / 100.])

    def send(self, kind, target):
        self.log.event(command_details(kind, target, self.speed.value()))
        if self.pending:
            self.log.event("Command not submitted: waiting for command acknowledgement.", "warning")
            return
        self.pending, self.pending_since = True, time.monotonic()
        self.command_id = uuid.uuid4().hex
        self.log.event(f"Command submitted: kind={kind}, id={self.command_id}.")
        self.node.command_pub.publish(String(data=json.dumps({'id': self.command_id, 'kind': kind, 'target': target,
            'speed': self.speed.value()})))
        for button in self.command_buttons:
            button.setEnabled(False)

    def stop(self):
        self.log.event("STOP requested.")
        self.pending = False
        self.node.stop_pub.publish(Empty())

    def refresh(self):
        if self.node.with_gripper:
            measured = self.node.measured_gripper()
            self.gripper_feedback.setText('—' if measured is None else f'{measured:.0f}')
        with self.node.lock:
            data, received = self.node.snapshot, self.node.received
        ready = False
        if data is not None:
            age = data['age'] + max(0., time.monotonic()-received)
            ready = data['ready'] and time.monotonic()-received <= 1.
            for labels, values, cartesian in ((self.joint_feedback, data['angles'], False),
                                             (self.coord_feedback, data['coords'], True)):
                if values is not None:
                    for i, (label, value) in enumerate(zip(labels, values)):
                        label.setText(f'{value:.2f} ' + ('mm' if cartesian and i < 3 else '°'))
            if not self.seeded and data['angles'] is not None and data['coords'] is not None and age <= 1.:
                for controls, values in ((self.joint_inputs, data['angles']), (self.coord_inputs, data['coords'])):
                    for control, value in zip(controls, values):
                        control.setValue(value)
                self.seeded = True
            if self.pending and data.get('command_id') == self.command_id:
                self.pending = False
            text = data['status']
            if data.get('feedback_error'):
                text += '\n' + data['feedback_error']
            self.status.setText(text if ready else 'Control session is unavailable.')
            self.log.state('session', ready, 'Control session available.' if ready else
                           'Control session is unavailable.', 'info' if ready else 'warning')
            if data.get('feedback_error') and ready:
                self.log.state('feedback', data['feedback_error'], data['feedback_error'], 'warning')
            else:
                if ready:
                    self.log.state('feedback', 'available', 'SDK feedback available.')
                else:
                    self.log.clear('feedback')
            ready = ready and not data['busy']
        if self.pending and time.monotonic()-self.pending_since > 5.:
            self.status.setText('Waiting for command acknowledgement...')
            self.log.state('ack', self.command_id, 'Waiting for command acknowledgement...', 'warning')
        else:
            self.log.clear('ack')
        for button in self.command_buttons:
            button.setEnabled(ready and not self.pending)

    def stop_if_busy(self):
        with self.node.lock:
            busy = self.node.snapshot is not None and self.node.snapshot.get("busy", False)
        if self.pending or busy:
            self.stop()

    def closeEvent(self, event):
        self.log.event("GUI window closed.")
        self.stop_if_busy()
        event.accept()


def main():
    from rclpy.signals import SignalHandlerOptions
    from rclpy.executors import ExternalShutdownException
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = GuiTransport()
    app = QApplication(sys.argv)
    window = ControlWindow(node)
    def spin_ros():
        try:
            rclpy.spin(node)
        except ExternalShutdownException:
            pass
    spin = threading.Thread(target=spin_ros, daemon=True)
    spin.start()
    def interrupted(source):
        window.log.event(f'{source}: exit requested.')
        app.quit()
    signal.signal(signal.SIGTERM, lambda *_: interrupted('SIGTERM'))
    signal.signal(signal.SIGINT, lambda *_: interrupted('Ctrl+C'))
    window.show()
    try:
        app.exec_()
    finally:
        window.log.event("GUI event loop exiting.")
        window.stop_if_busy()
        rclpy.try_shutdown()
        spin.join(timeout=2.)
        node.destroy_node()


if __name__ == '__main__':
    main()
