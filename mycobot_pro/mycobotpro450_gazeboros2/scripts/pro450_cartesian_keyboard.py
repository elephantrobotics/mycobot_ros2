"""Single SDK caller for native Cartesian jog; no joint targets or collision API."""
from collections import deque
import math
import threading
import time


CARTESIAN_AXES = ('X', 'Y', 'Z', 'RX', 'RY', 'RZ', 'Gripper')
SPEED_GEARS = (6, 12, 18, 24, 30)  # Native SDK percentages, not physical units.
GRIPPER_SPEED_GEARS = (4, 8, 12, 16, 20)
LOW_Z_MM = 20.0  # Base-frame TCP height, including the installed tool offset.
COORD_POLL_PERIOD = 1.0 / 20.0


class CartesianKeyboardTransport:
    def __init__(self, sdk, with_gripper=True, clock=time.monotonic):
        self.sdk, self.with_gripper, self.clock = sdk, with_gripper, clock
        self.lock = threading.RLock()
        self.shutdown = threading.Event()
        self.armed = False
        self.gear = 2
        self.desired = self.active = None
        self.epoch = 0
        self.last_keepalive = self.clock()
        self.last_feedback = float('-inf')
        self.coords = None
        self.low_z = False
        self.next_feedback = self.next_error = 0.
        self.arm_requested = False
        self.stop_pending = self.force_stop = False
        self.waiting_generation = 0
        self.notices = deque()

    def notice(self, text, level='info'):
        with self.lock:
            self.notices.append((text, level))

    def drain_notices(self):
        with self.lock:
            result = list(self.notices)
            self.notices.clear()
            return result

    def request_arm(self):
        with self.lock:
            self.armed = False
            self.arm_requested = True
            self.stop_pending = True
            self.desired = None
            self.epoch += 1
        return 'Control unlock requested.'

    def request(self, axis, direction):
        with self.lock:
            if type(axis) is not int or not 0 <= axis < (7 if self.with_gripper else 6) or direction not in (-1, 1):
                return 'rejected: invalid coordinate control'
            if not self.armed:
                return 'rejected: controls locked; press M to unlock'
            if self.low_z and (axis, direction) != (2, 1):
                return 'rejected: TCP Z is at or below 20 mm; only R (Z up) is allowed'
            self.desired = (axis, direction, self.speed_for_axis(axis))
            self.last_keepalive = self.clock()
            self.epoch += 1
        return 'requested: native SDK jog'

    def release(self):
        with self.lock:
            self.desired = None
            self.stop_pending = True
            self.epoch += 1

    def stop(self):
        with self.lock:
            self.armed = False
            self.arm_requested = False
            self.desired = None
            self.stop_pending = self.force_stop = True
            self.epoch += 1

    def keepalive(self):
        with self.lock:
            self.last_keepalive = self.clock()

    def set_gear(self, gear):
        with self.lock:
            self.gear = max(1, min(5, int(gear)))
            if self.desired is not None:
                self.desired = (*self.desired[:2], self.speed_for_axis(self.desired[0]))
                self.epoch += 1

    def speed_for_axis(self, axis):
        gears = GRIPPER_SPEED_GEARS if axis == 6 else SPEED_GEARS
        return gears[self.gear - 1]

    def idle(self):
        with self.lock:
            return self.active is None and self.desired is None

    @staticmethod
    def accepted(result, gripper=False, stop=False):
        # The Pro450 gripper SDK has a known non-1 echo decoded as -1.
        allowed = (1, -1) if gripper else ((0, 1) if stop else (1,))
        if type(result) is not int or result not in allowed:
            raise RuntimeError(f'SDK rejected control: {result!r}')

    def _stop_active(self, force=False):
        with self.lock:
            active = self.active
        if active is not None and active[0] == 6:
            opening = self.sdk.get_pro_gripper_angle()
            if type(opening) not in (int, float) or not math.isfinite(opening) or not 0 <= opening <= 100:
                raise RuntimeError('Cannot hold gripper: valid measured opening unavailable')
            self.accepted(self.sdk.set_pro_gripper_angle(int(round(opening))), gripper=True)
            self.notice('Gripper hold sent.')
        elif active is not None or force:
            self.accepted(self.sdk.stop(deceleration=0, _async=True), stop=True)
            self.notice('STOP sent.')
        with self.lock:
            self.active = None

    def _fault(self, exc):
        with self.lock:
            self.armed = False
            self.arm_requested = False
            self.desired = None
            self.epoch += 1
            self.waiting_generation += 1
        try:
            self._stop_active()
        except Exception as stop_error:
            self.notice(f'STOP/hold failed: {stop_error}', 'error')
        self.notice(f'Control fault (locked): {exc}. Press M to retry after recovery.', 'error')

    def _read_coords(self):
        started = self.clock()
        values = self.sdk.get_coords()
        if (not isinstance(values, (list, tuple)) or len(values) != 6 or
                any(type(v) not in (int, float) or not math.isfinite(v) for v in values)):
            raise RuntimeError('SDK coordinate feedback unavailable')
        with self.lock:
            self.coords = tuple(values)
            self.last_feedback = self.clock()
            # Schedule from request start, not reply completion: otherwise SDK
            # latency is added to every 50 ms period. Calls never overlap.
            self.next_feedback = started + COORD_POLL_PERIOD
            was_low = self.low_z
            self.low_z = values[2] <= LOW_Z_MM
            entered = self.low_z and not was_low
            if entered:
                # Latch the entry, not every low sample: M may subsequently
                # unlock an upward escape while feedback is still below 20 mm.
                self.armed = False
                self.arm_requested = False
                self.desired = None
                self.epoch += 1
                self.waiting_generation += 1
                self.notice('TCP Z is at or below 20 mm; controls locked. Press M, then hold R (Z up).', 'warning')
            elif was_low and not self.low_z:
                self.notice('TCP Z is above 20 mm; all motion keys are available.' if self.armed
                            else 'TCP Z is above 20 mm; controls remain locked. Press M to unlock.')
        if entered:
            self._stop_active()

    def _read_error(self):
        error = self.sdk.get_error_information()
        if type(error) is not int:
            raise RuntimeError('SDK controller status unavailable')
        if error:
            if error == 36:
                raise RuntimeError('Motion rejected: singular position')
            raise RuntimeError(f'Controller error {error}')

    def cycle(self):
        with self.lock:
            stop, force = self.stop_pending, self.force_stop
            self.stop_pending = self.force_stop = False
            arm, epoch = self.arm_requested, self.epoch
            self.arm_requested = False
        if stop:
            self._stop_active(force)
        if arm:
            self._read_coords()
            self._read_error()
            with self.lock:
                if epoch == self.epoch and not self.shutdown.is_set():
                    self.armed = True
                    self.notice('Controls unlocked: only R (Z up) is allowed until TCP Z is above 20 mm.'
                                if self.low_z else 'Controls unlocked. Hold a coordinate key to jog.')
        else:
            # Read height before dispatch, so a newly observed low-zone entry
            # cancels an already queued command before any new SDK jog write.
            now = self.clock()
            if now >= self.next_feedback:
                self.next_feedback = now + COORD_POLL_PERIOD
                try:
                    self._read_coords()
                except Exception:
                    if self.armed and self.clock() - self.last_feedback > .5:
                        raise
        with self.lock:
            if self.desired is not None and self.clock() - self.last_keepalive > .5:
                self.desired = None
                self.armed = False
                self.epoch += 1
                self.waiting_generation += 1
                self.notice('Keyboard input heartbeat lost; controls locked.', 'warning')
            desired = self.desired if self.armed else None
            epoch = self.epoch
            active = self.active
        if active is not None and active != desired:
            self._stop_active()
        if desired is not None:
            with self.lock:
                if epoch != self.epoch or self.shutdown.is_set():
                    return
                send = self.active != desired
                if send:
                    if self.low_z and desired[:2] != (2, 1):
                        self.desired = None
                        return
                    self.active = desired  # Ambiguous writes must still be stopped.
            if send:
                axis, direction, speed = desired
                if axis < 6:
                    result = self.sdk.jog_coord(axis + 1, int(direction > 0), speed, _async=True)
                    self.accepted(result)
                else:
                    self.accepted(self.sdk.set_pro_gripper_speed(speed))
                    with self.lock:
                        if epoch != self.epoch or self.shutdown.is_set():
                            self.active = None  # No opening command was sent.
                            return
                    self.accepted(self.sdk.set_pro_gripper_angle(100 if direction > 0 else 0), gripper=True)
                self.notice(f'{CARTESIAN_AXES[axis]}: SDK jog command sent at {speed}%.')
            with self.lock:
                obsolete = epoch != self.epoch or self.shutdown.is_set()
            if obsolete:
                self._stop_active()
                return
        now = self.clock()
        if self.active is not None and now >= self.next_error:
            self.next_error = now + .5
            self._read_error()

    def run(self):
        try:
            while not self.shutdown.is_set():
                try:
                    self.cycle()
                except Exception as exc:
                    self._fault(exc)
                # A fixed 20 ms tick would round 50 ms polling up to 60 ms.
                self.shutdown.wait(min(.02, max(.001, self.next_feedback - self.clock())))
        finally:
            try:
                self._stop_active(force=True)
            except Exception as exc:
                self.notice(f'Exit STOP/hold failed: {exc}', 'error')
