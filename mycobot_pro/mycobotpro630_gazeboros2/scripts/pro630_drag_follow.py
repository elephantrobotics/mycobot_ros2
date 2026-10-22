#!/usr/bin/env python3
"""Real-only combined arm/gripper enable switching and measured pose mirroring."""
import math
import signal
import sys
import threading
import time

import rclpy
from rclpy.node import Node

from pro630_drag_support import DragState, set_drag_enabled
from pro630_public_client import PublicPro630Client as Pro630Client
from pro630_session_support import read_initial_gripper
from pro630_configuration import configure_node
from pro630_model import DEFAULT_PRO630_IP, DEFAULT_PRO630_PORT
from pro630_control_logging import ControlLog

class DragFollow(Node):
    def __init__(self):
        super().__init__('pro630_drag_follow')
        self.log = ControlLog(self.get_logger())
        configure_node(self)
        self.declare_parameter('startup_enabled', False)
        self.declare_parameter('initial_gripper', -1.0)
        self.sdk = None
        self.worker = None
        self.stop_event = threading.Event()
        self._closing = False
        self._exit_mode_required = False
        try:
            if {'slider_control_gazebo', 'teleop_keyboard_gazebo'} & set(self.get_node_names()):
                raise RuntimeError('Close the existing robot controller before starting drag following')
            self.sdk = Pro630Client(DEFAULT_PRO630_IP, DEFAULT_PRO630_PORT, feedback_hz=20.0, ownership_timeout=0, feature="drag", with_gripper=self.with_gripper)
            if not self.get_parameter('startup_enabled').value:
                if self.sdk.is_moving() != 0:
                    raise RuntimeError('Stop robot motion before starting drag following')
                self._exit_mode_required = True
                self.log.event('Startup motor enable requested.')
                set_drag_enabled(self.sdk, True, self.with_gripper)
                self.log.event('Startup motor enable acknowledged.')
            else:
                self._exit_mode_required = True
                self.log.event('Startup motor enable was completed by the control session.')
            if self.with_gripper:
                self.gripper = float(self.get_parameter('initial_gripper').value)
                if not math.isfinite(self.gripper) or not 0 <= self.gripper <= 1:
                    self.gripper = read_initial_gripper(self.sdk) / 100.0
            self.state = DragState(self.sdk, with_gripper=self.with_gripper, report=self.log.event)
            self.log.event(self.state.message)
            self.sdk.set_angle_callback(self._sample)
            self.worker = threading.Thread(target=self._read_loop, daemon=True)
            self.worker.start()
        except Exception:
            self.close()
            raise

    def _sample(self, sample):
        self.state.observe(sample.angles, sample.monotonic, sample.wall_time)

    def _read_loop(self):
        # One SDK caller: motor switches and angle queries cannot race or build a queue.
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
        enabled, pending, message, sample = self.state.snapshot()
        if sample is None or time.monotonic() - sample[0] > .5:
            return enabled, pending, message, False
        return enabled, pending, message, True

    def close(self):
        if self._closing:
            return
        self._closing = True
        self.stop_event.set()
        if self.worker is not None:
            self.worker.join(timeout=5.0)
        if self.sdk is not None:
            try:
                if self._exit_mode_required:
                    if self.worker is not None and self.worker.is_alive():
                        self.log.event('Exit free-move request not confirmed: control worker is still active.', 'error')
                    else:
                        self.log.event('Exit requested: disabling arm free-move mode.')
                        # The public adapter maps this to set_free_move_mode(False).
                        # Do not call set_drag_enabled: gripper state stays unchanged.
                        result = self.sdk.set_motor_enabled(254, 1)
                        if type(result) is not int or result != 1:
                            raise RuntimeError(f'Free-move exit was not acknowledged (result={result!r})')
                        self.log.event('Arm free-move exit acknowledged.')
            except Exception as exc:
                self.log.event(f'Exit free-move request failed: {exc}', 'error')
            finally:
                self.sdk.close()
                self.sdk = None
        self.log.state('closing', True, 'Drag following closed.')


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

        def keyReleaseEvent(self, event):
            if not event.isAutoRepeat() and event.key() in self.held_keys:
                self.held_keys.remove(event.key())
                label = 'M' if event.key() == Qt.Key_M else 'Q'
                node.log.event(f'Key {label} released.')
            super().keyReleaseEvent(event)

        def _clear_keys(self, source):
            for key in self.held_keys:
                node.log.event(f'Key {"M" if key == Qt.Key_M else "Q"} cleared ({source}; synthetic release).')
            self.held_keys.clear()

        def focusOutEvent(self, event):
            self._clear_keys('focus lost')
            super().focusOutEvent(event)

        def closeEvent(self, event):
            node.log.event('Drag follow window closed; arm free-move exit will be requested.')
            self._clear_keys('window closed')
            super().closeEvent(event)

    try:
        node = DragFollow()
        window = Window()
        window.setWindowTitle('Pro630 Drag Follow Control')
        window.setFocusPolicy(Qt.StrongFocus)
        layout = QVBoxLayout(window)
        layout.addWidget(QLabel(
            ('All six arm joints and the gripper are enabled once at startup.\n' if node.with_gripper else
             'All six arm joints are enabled once at startup.\n') +
            ('M: release / lock all six arm joints and the gripper together. Q: close this window.\n' if node.with_gripper else
             'M: release / lock all six arm joints. Q: close this window.\n') +
            'When disabled, support the arm and drag it by hand. Gazebo follows the measured pose.\n' +
            ('Closing this window exits arm free-move; the gripper enable state is preserved.' if node.with_gripper else
             'Closing this window exits arm free-move.')))
        status = QLabel('Waiting for joint feedback...')
        layout.addWidget(status)
        window.resize(720, 190)
        timer = QTimer(window)

        def tick():
            if not rclpy.ok():
                window.close()
                return
            rclpy.spin_once(node, timeout_sec=0)
            _, _, message, fresh = node.render()
            node.log.state('feedback', fresh, 'Live joint feedback.' if fresh else
                           'Joint feedback unavailable; Gazebo holds the last valid pose.',
                           'info' if fresh else 'warning')
            status.setText(message + ('\nLive joint feedback.' if fresh else
                                      '\nJoint feedback unavailable; Gazebo holds the last valid pose.'))

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
