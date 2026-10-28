"""Hardware-independent state and newest-frame storage for real drag following."""
import math
import threading
import time


def enable_drag_motors(sdk, with_gripper=True):
    """Enable once at startup; M never disables the six arm motors."""
    result = sdk.set_motor_enabled(254, 1)
    if isinstance(result, bool) or result != 1:
        raise RuntimeError(f'Arm motor command was not acknowledged (result={result!r}); state unknown')
    if not with_gripper:
        return
    result = sdk.set_pro_gripper_enabled(1)
    if isinstance(result, bool) or result != 1:
        raise RuntimeError(f'Arm command acknowledged, but gripper command was not acknowledged '
                           f'(result={result!r}); combined state unknown')


class ModeSwitchCancelled(Exception):
    pass


def set_control_mode_checked(sdk, mode, cancelled=lambda: False):
    """Confirm position (0) / torque (1) without querying motor enable state."""
    if type(mode) is not int or mode not in (0, 1):
        raise ValueError('Control mode must be 0 (position) or 1 (torque)')
    if cancelled():
        raise ModeSwitchCancelled()
    reply = sdk.set_control_mode(mode)
    if type(reply) is not int or reply != 1:
        raise RuntimeError(f'Control mode command was not acknowledged (result={reply!r}); mode unknown')
    if cancelled():
        raise ModeSwitchCancelled()
    actual = sdk.get_control_mode()
    if type(actual) is not int or actual != mode:
        raise RuntimeError(f'Control mode was not confirmed (requested={mode}, actual={actual!r}); mode unknown')


class DragState:
    def __init__(self, sdk, clock=time.monotonic, with_gripper=True, report=None):
        self.report = report or (lambda text, level='info': None)
        self.with_gripper = with_gripper
        self.sdk, self.clock = sdk, clock
        self.lock = threading.Lock()
        self.mode = 0
        self.closing = False
        self.pending = False
        self.request = None
        self.message = self._mode_message()
        self.latest = None
        self.sequence = 0

    def request_toggle(self):
        with self.lock:
            if self.closing:
                message, level, accepted = 'Control mode switch ignored: drag following is closing.', 'warning', False
            elif self.pending:
                message, level, accepted = 'Control mode switch ignored: another switch is in progress.', 'warning', False
            elif self.latest is None or self.clock() - self.latest[0] > .5:
                self.message = 'Fresh joint feedback is required before switching control mode.'
                message, level, accepted = self.message, 'warning', False
            else:
                # Position -> torque; an unknown state explicitly recovers to position.
                self.request = 1 if self.mode == 0 else 0
                self.pending = True
                self.message = ('Switching to torque mode...' if self.request == 1
                                else 'Switching to position mode...')
                target = self.request
                message, level, accepted = self.message, 'info', True
        if accepted:
            self.report('Control mode switch requested: ' + ('torque (1).' if target == 1 else 'position (0).'))
        self.report(message, level)
        return accepted

    def apply_request(self):
        with self.lock:
            if not self.pending:
                return
            target = self.request
            cancelled = False
            if self.latest is None or self.clock() - self.latest[0] > .5:
                self.pending = False
                self.message = 'Control mode switch cancelled: joint feedback is stale.'
                cancelled = True
                message = self.message
        if cancelled:
            self.report(message, 'warning')
            return
        level = 'info'
        try:
            set_control_mode_checked(self.sdk, target, self.is_closing)
            if self.is_closing():
                raise ModeSwitchCancelled()
            if self.with_gripper:
                reply = self.sdk.set_pro_gripper_enabled(1 if target == 0 else 0)
                if type(reply) is not int or reply != 1:
                    raise RuntimeError(f'Arm mode confirmed, but gripper command was not acknowledged '
                                       f'(result={reply!r}); combined state unknown')
        except ModeSwitchCancelled:
            with self.lock:
                self.mode = None
                self.message = 'Control mode switch interrupted during close; final mode is not confirmed.'
            level = 'warning'
        except Exception as exc:
            with self.lock:
                self.mode = None
                self.message = str(exc) + '. Press M to request position mode.'
                level = 'error'
        else:
            with self.lock:
                self.mode = target
                self.message = self._mode_message()
        finally:
            with self.lock:
                self.pending = False
                message = self.message
            self.report(message, level)

    def _mode_message(self):
        if self.mode == 1:
            return ('Torque mode (zero-force drag). Drag by hand. Press M to return to position mode.' +
                    (' Gripper released.' if self.with_gripper else ''))
        return ('Position mode. Press M to enter torque (zero-force drag) mode.' +
                (' Gripper enabled.' if self.with_gripper else ''))

    def is_closing(self):
        with self.lock:
            return self.closing

    def cancel_pending(self):
        with self.lock:
            self.closing = True
            self.pending = False
            self.request = None

    def observe(self, angles, measured, wall):
        if (len(angles) != 6 or not all(isinstance(v, (int, float)) and not isinstance(v, bool)
                and math.isfinite(v) for v in angles)
                or not math.isfinite(measured) or not math.isfinite(wall)):
            return False
        with self.lock:
            if self.clock() - measured > .5 or (self.latest and measured <= self.latest[0]):
                return False
            self.sequence += 1
            self.latest = (measured, wall, tuple(math.radians(v) for v in angles), self.sequence)
            return True

    def snapshot(self):
        with self.lock:
            return self.mode, self.pending, self.message, self.latest
