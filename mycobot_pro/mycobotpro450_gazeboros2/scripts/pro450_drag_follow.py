#!/usr/bin/env python3
"""Real-only torque/position mode switching and measured pose mirroring."""
import math
import signal
import sys
import threading
import time

import rclpy
from rclpy.node import Node

from pro450_drag_support import DragState, enable_drag_motors, set_control_mode_checked
from pro450_public_client import PublicPro450Client as Pro450Client
from pro450_session_support import read_initial_gripper
from pro450_configuration import configure_node
from pro450_control_logging import ControlLog

class DragFollow(Node):
    def __init__(self):
        super().__init__('pro450_drag_follow')
        self.log = ControlLog(self.get_logger())
        configure_node(self)
        self.declare_parameter('startup_enabled', False)
        self.declare_parameter('initial_gripper', -1.0)
        self.sdk = None
        self.worker = None
        self.stop_event = threading.Event()
        self._closing = False
        self._clear_errors_on_exit = False
        try:
            if {'slider_control_gazebo', 'teleop_keyboard_gazebo'} & set(self.get_node_names()):
                raise RuntimeError('Close the existing robot controller before starting drag following')
            self.sdk = Pro450Client('192.168.0.232', 4500, feedback_hz=20.0, ownership_timeout=0, feature="drag", with_gripper=self.with_gripper, allow_collision=self.allow_collision)
            self.log.event('Startup error clear requested.')
            result = self.sdk.clear_error_information()
            self.log.event(f'Startup error clear returned {result!r}.',
                           'info' if type(result) is int and result in (0, 1) else 'warning')
            if not self.get_parameter('startup_enabled').value:
                if self.sdk.is_moving() != 0:
                    raise RuntimeError('Stop robot motion before starting drag following')
                self.log.event('Startup motor enable requested.')
                enable_drag_motors(self.sdk, self.with_gripper)
                self.log.event('Startup motor enable acknowledged.')
            else:
                self.log.event('Startup motor enable was completed by the control session.')
            self.log.event('Startup position mode requested.')
            set_control_mode_checked(self.sdk, 0)
            self.log.event('Startup position mode confirmed.')
            if self.with_gripper:
                self.gripper = float(self.get_parameter('initial_gripper').value)
                if not math.isfinite(self.gripper) or not 0 <= self.gripper <= 1:
                    self.gripper = read_initial_gripper(self.sdk) / 100.0
            self.state = DragState(self.sdk, with_gripper=self.with_gripper, report=self.log.event)
            self.log.event(self.state.message)
            self.sdk.set_angle_callback(self._sample)
            self.worker = threading.Thread(target=self._read_loop, daemon=True)
            self.worker.start()
            self._clear_errors_on_exit = True
        except Exception:
            self.close()
            raise

    def _sample(self, sample):
        self.state.observe(sample.angles, sample.monotonic, sample.wall_time)

    def _read_loop(self):
        # One SDK caller: mode switches and angle queries cannot build a queue.
        while not self.stop_event.is_set():
            try:
                self.state.apply_request()
                if self.stop_event.is_set():
                    break
                angles = self.sdk.get_angles()
                stamp = self.sdk.sample_time('arm')
                if stamp and isinstance(angles, (list, tuple)):
                    self.state.observe(angles, *stamp)
            except Exception:
                # The UI reports stale feedback; retry queries without terminal spam.
                self.stop_event.wait(.05)


    def render(self):
        mode, pending, message, sample = self.state.snapshot()
        if sample is None or time.monotonic() - sample[0] > .5:
            return mode, pending, message, False
        return mode, pending, message, True

    def close(self):
        if self._closing:
            return
        self._closing = True
        self.stop_event.set()
        if hasattr(self, 'state'):
            self.state.cancel_pending()
        if self.worker is not None:
            self.worker.join(timeout=1.0)
            if self.worker.is_alive():
                self.log.event('Drag worker is still finishing; further mode-switch steps are cancelled.', 'warning')
        if self.sdk is not None:
            try:
                if self._clear_errors_on_exit:
                    self.log.event('Exit error clear requested.')
                    try:
                        result = self.sdk.clear_error_information()
                        self.log.event(f'Exit error clear returned {result!r}.',
                                       'info' if type(result) is int and result in (0, 1) else 'warning')
                    except Exception as exc:
                        self.log.event(f'Exit error clear failed: {exc}', 'error')
            finally:
                self.sdk.close()
                self.sdk = None
        # Only clear errors: do not implicitly switch mode or motor enable on exit.
        self.log.state('closing', True, 'Drag following closed; control mode and motor enable states are preserved.')


def main():
    from PyQt5.QtCore import Qt, QTimer
    from PyQt5.QtWidgets import QApplication, QLabel, QVBoxLayout, QWidget
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = None
    app = QApplication(sys.argv[:1])

    class Window(QWidget):
        def __init__(self):
            super().__init__()
            self.held_keys = set()
            self.setWindowTitle('Pro450 Drag Follow Control')
            self.setFocusPolicy(Qt.StrongFocus)
            font = self.font()
            font.setPointSize(11)
            self.setFont(font)
            layout = QVBoxLayout(self)
            layout.setContentsMargins(14, 12, 14, 12)
            layout.setSpacing(8)
            profile = 'Arm + gripper' if node.with_gripper else 'Arm only'
            layout.addWidget(QLabel(f'Real | {profile}'))
            self.mode_label = QLabel()
            layout.addWidget(self.mode_label)
            layout.addWidget(QLabel('M  Switch mode     Q  Quit'))
            self.key_label = QLabel('Pressed key: None')
            layout.addWidget(self.key_label)
            self.status_label = QLabel()
            self.status_label.setWordWrap(True)
            layout.addWidget(self.status_label)
            self.feedback_label = QLabel()
            layout.addWidget(self.feedback_label)
            layout.addWidget(QLabel('Closing preserves mode and motor states.'))
            self.update_status(0, False, node.state.message, False)
            self.resize(620, 220)

        def update_status(self, mode, pending, message, fresh):
            if pending:
                title = 'Switching...'
            elif mode == 0:
                title = 'Position'
            elif mode == 1:
                title = 'Torque (zero-force drag)'
            else:
                title = 'Not confirmed'
            self.mode_label.setText('Mode: ' + title)
            normal = not pending and mode in (0, 1) and message == node.state._mode_message()
            self.status_label.setText('' if normal else message)
            self.status_label.setVisible(not normal)
            self.feedback_label.setText('Feedback: live' if fresh else 'Feedback: unavailable')

        def _refresh_keys(self):
            names = [('M' if key == Qt.Key_M else 'Q') for key in sorted(self.held_keys)]
            self.key_label.setText('Pressed key: ' + (' + '.join(names) if names else 'None'))

        def keyPressEvent(self, event):
            if event.isAutoRepeat():
                return
            if event.key() == Qt.Key_M:
                self._pressed(event.key(), 'M')
                node.state.request_toggle()
            elif event.key() == Qt.Key_Q:
                self._pressed(event.key(), 'Q')
                self.close()
            else:
                super().keyPressEvent(event)

        def _pressed(self, key, label):
            if key not in self.held_keys:
                self.held_keys.add(key)
                node.log.event(f'Key {label} pressed.')
                self._refresh_keys()

        def keyReleaseEvent(self, event):
            if not event.isAutoRepeat() and event.key() in self.held_keys:
                self.held_keys.remove(event.key())
                label = 'M' if event.key() == Qt.Key_M else 'Q'
                node.log.event(f'Key {label} released.')
                self._refresh_keys()
            super().keyReleaseEvent(event)

        def _clear_keys(self, source):
            for key in self.held_keys:
                node.log.event(f'Key {"M" if key == Qt.Key_M else "Q"} cleared ({source}; synthetic release).')
            self.held_keys.clear()
            self._refresh_keys()

        def focusOutEvent(self, event):
            self._clear_keys('focus lost')
            super().focusOutEvent(event)

        def closeEvent(self, event):
            node.log.event('Drag follow window closed; control mode and motor enable states are preserved.')
            self._clear_keys('window closed')
            super().closeEvent(event)

    try:
        node = DragFollow()
        window = Window()
        timer = QTimer(window)

        def tick():
            if not rclpy.ok():
                window.close()
                return
            rclpy.spin_once(node, timeout_sec=0)
            mode, pending, message, fresh = node.render()
            node.log.state('feedback', fresh, 'Live joint feedback.' if fresh else
                           'Joint feedback unavailable; Gazebo holds the last valid pose.',
                           'info' if fresh else 'warning')
            window.update_status(mode, pending, message, fresh)

        timer.timeout.connect(tick)
        timer.start(20)
        def interrupted(source):
            node.log.event(f'{source}: exit requested.')
            window.close()
        signal.signal(signal.SIGINT, lambda *_: interrupted('Ctrl+C'))
        signal.signal(signal.SIGTERM, lambda *_: interrupted('SIGTERM'))
        window.show()
        return app.exec_()
    except Exception as exc:
        print(f'Failed to start drag following: {exc}', file=sys.stderr)
        return 1
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
