#!/usr/bin/env python3
"""Real-only GUI worker: SDK commands and SDK-reported completion, via the public owner."""
import json
import math
import queue
import signal
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, String
from moveit_msgs.srv import GetStateValidity

from pro450_control_logging import ControlLog
from pro450_configuration import configure_node
from pro450_public_client import PublicPro450Client
from pro450_collision_check import CollisionCheck, DEFAULT_COLLISION_DEPTH_TOLERANCE_M
from pro450_real_keyboard import RealPoseReader


class CommandStopped(Exception):
    pass


class CommandRejected(Exception):
    pass


class GuiBackend(CollisionCheck, Node):
    def __init__(self):
        super().__init__('pro450_gui_backend')
        configure_node(self)
        self._lock = threading.RLock()
        self._closing = threading.Event()
        self._stop = threading.Event()
        self._queue = queue.Queue(maxsize=1)
        self._busy = False
        self._active_kind = None
        self._gripper_active = False
        self._command_id = ''
        self._command_kind = None
        self.log = ControlLog(self.get_logger())
        self._status = 'Ready.'
        self._angles = self._coords = None
        self._feedback_when = 0.
        self._feedback_error = ''
        self._feedback_failure_since = None
        self.mc = None
        self.worker = None
        # This proxy claims the already-running real public bridge; it never
        # constructs an SDK connection or powers/enables any motor.
        self.mc = PublicPro450Client(feature='gui', with_gripper=self.with_gripper, allow_collision=self.allow_collision)
        self.validity_client = self.create_client(GetStateValidity, '/check_state_validity')
        self.collision_depth_tolerance_m = DEFAULT_COLLISION_DEPTH_TOLERANCE_M
        self._joint_pose_reader = RealPoseReader(
            self.mc, self.joint_limits, with_gripper=self.with_gripper,
            report=self.get_logger().debug)
        self.feedback_pub = self.create_publisher(String, '/pro450/gui/feedback', 1)
        self.create_subscription(String, '/pro450/gui/command', self.command, 10)
        self.create_subscription(Empty, '/pro450/gui_stop', self.stop, 10)
        self.create_timer(.05, self.feedback)
        self.worker = threading.Thread(target=self.work, daemon=True)
        self.worker.start()

    def status(self, text, level='info', *, context=None):
        with self._lock:
            self._status = text
            kind, command_id = self._command_kind, self._command_id
        if context is not None:
            kind, command_id = context
        self.log.event(f'[kind={kind}, id={command_id}] {text}', level)

    def feedback(self):
        with self._lock:
            data = dict(angles=self._angles, coords=self._coords,
                        age=min(1e6, time.monotonic()-self._feedback_when),
                        busy=self._busy, ready=True, status=self._status,
                        feedback_error=self._feedback_error, command_id=self._command_id)
        self.feedback_pub.publish(String(data=json.dumps(data, allow_nan=False)))

    @staticmethod
    def six_values(values, label):
        if (not isinstance(values, (list, tuple)) or len(values) != 6
                or any(type(v) not in (int, float) or not math.isfinite(v) for v in values)):
            raise RuntimeError(f'Invalid SDK {label} reply: {values!r}')
        return list(values)

    def refresh_feedback(self):
        # Readback failures are displayed, not turned into a model/feedback gate
        # on the next command. SDK command replies remain authoritative.
        try:
            angles = self.six_values(self.mc.get_angles(), 'angles')
            coords = self.six_values(self.mc.get_coords(), 'coordinates')
            with self._lock:
                self._angles, self._coords = angles, coords
                self._feedback_when, self._feedback_error = time.monotonic(), ''
                self._feedback_failure_since = None
        except Exception:
            with self._lock:
                # A transient missing/invalid SDK reply is not a robot error.
                # Keep the last valid data and its age without flashing raw -1.
                now = time.monotonic()
                if self._feedback_failure_since is None:
                    self._feedback_failure_since = now
                if now - self._feedback_failure_since >= 1.:
                    self._feedback_error = 'SDK feedback unavailable; waiting for a valid response.'

    def command(self, msg):
        data = None
        try:
            data = json.loads(msg.data)
            if not isinstance(data, dict) or set(data) != {'id', 'kind', 'target', 'speed'}:
                raise ValueError('Invalid command fields')
            if not isinstance(data['id'], str) or not 1 <= len(data['id']) <= 64:
                raise ValueError('Invalid command identifier')
            if data['kind'] not in ('joints', 'coords', 'gripper', 'clear_errors'):
                raise ValueError('Unsupported command')
            if type(data['speed']) is not int or not 1 <= data['speed'] <= 100:
                raise ValueError('Speed must be an integer from 1 to 100')
            count = {'gripper': 1, 'clear_errors': 0}.get(data['kind'], 6)
            if (not isinstance(data['target'], list) or len(data['target']) != count
                    or any(type(v) not in (int, float) or not math.isfinite(v) for v in data['target'])):
                raise ValueError('Invalid target values')
            if data['kind'] == 'gripper' and not self.with_gripper:
                raise ValueError('No gripper is installed')
            if data['kind'] == 'gripper' and not 0 <= data['target'][0] <= 1:
                raise ValueError('Invalid gripper target')
            with self._lock:
                if self._busy or self._stop.is_set():
                    raise RuntimeError('A command or STOP is in progress')
                self._busy, self._command_id = True, data['id']
                self._command_kind = data['kind']
                self.status('Sending command...')
                self._queue.put_nowait(data)
        except Exception as exc:
            with self._lock:
                if not self._busy:
                    self._command_id = data.get('id', '') if isinstance(data, dict) else ''
            context = (data.get('kind'), data.get('id')) if isinstance(data, dict) else (None, None)
            self.status(f'Rejected: {exc}', 'warning', context=context)

    def stop(self, _msg):
        self._stop.set()

    def cancelled(self):
        if self._stop.is_set() or self._closing.is_set():
            raise CommandStopped()

    @property
    def _stop_requested(self):
        # Shared collision queries use the existing GUI cancellation events.
        return self._stop.is_set() or self._closing.is_set()

    def _joint_preflight_pose(self):
        self.cancelled()
        try:
            if self.with_gripper:
                self._joint_pose_reader.force_gripper_read()
            pose = self._joint_pose_reader(moving=False)
        except Exception as exc:
            self.cancelled()
            self.get_logger().debug(f'Joint collision preflight feedback: {exc}')
            raise CommandRejected('Current real robot pose is unavailable or stale.') from exc
        self.cancelled()
        return pose

    def _validate_joint_target(self, target):
        arm = [math.radians(value) for value in target]
        if any(not lo <= value <= hi for value, (lo, hi) in zip(arm, self.joint_limits[:6])):
            raise CommandRejected('Joint target exceeds its limits.')
        current = self._joint_preflight_pose()
        if self.allow_collision:
            self.status('Collision checks bypassed (allow_collision=true).')
            return
        # Reuse the slider's full path checks, including its collision-exit rules.
        # Never move the gripper as a side effect of a six-axis GUI command.
        for _ in range(3):
            self.status('Validating joint path for collisions...')
            goal = arm + ([current[6]] if self.with_gripper else [])
            try:
                valid, reason = self._path_is_valid(current, goal)
            except Exception as exc:
                self.cancelled()
                raise CommandRejected(f'Collision validation failed: {exc}') from exc
            self.cancelled()
            if not valid:
                raise CommandRejected(reason)
            confirmed = self._joint_preflight_pose()
            changed = max(abs(a-b) for a, b in zip(current[:6], confirmed[:6])) > 0.01
            if self.with_gripper:
                changed = changed or abs(current[6] - confirmed[6]) > 1e-6
            if not changed:
                return
            current = confirmed
            self.status('Start pose changed; revalidating before motion...')
        raise CommandRejected('Start pose keeps changing; release external control and retry.')

    @staticmethod
    def motion_reply(result, *, asynchronous_send=False):
        accepted = (1,) if asynchronous_send else (0, 1)
        if type(result) is not int or result not in accepted:
            raise RuntimeError(f'SDK motion command returned {result!r}')

    @staticmethod
    def gripper_reply(result):
        # Existing Pro450 firmware may echo the requested register value; the
        # vendor wrapper then returns -1/None. Its status API confirms completion.
        if result not in (1, -1, None):
            raise RuntimeError(f'SDK gripper command returned {result!r}')

    @staticmethod
    def controller_error(code):
        messages = {
            32: 'Motion rejected: no Cartesian solution.',
            33: 'Motion rejected: no continuous linear solution.',
            34: 'Motion failed: velocity fusion error.',
            35: 'Motion rejected: no continuous null-space solution.',
            36: 'Motion rejected: singular position.',
            49: 'Motion rejected: robot disabled.',
            50: 'Motion failed: motor fault.',
            51: 'Motion failed: encoder fault.',
            52: 'Motion failed: motor execution error.',
            53: 'Motion failed: joint position error.',
            64: 'Motion rejected: coordinates exceed limits.',
            10: 'Motion stopped.', 11: 'Motion stopped: emergency stop.',
        }
        for first, last, text in (
                (1, 6, 'Motion rejected: joint {} near limit.'),
                (65, 70, 'Motion failed: joint {} positioning accuracy error.'),
                (81, 86, 'Motion stopped: joint {} collision detected.'),
                (97, 102, 'Motion failed: joint {} communication error.'),
                (113, 118, 'Motion failed: joint {} communication error.'),
                (129, 134, 'Motion rejected: joint {} disabled.'),
                (145, 150, 'Motion failed: joint {} motor fault.'),
                (161, 166, 'Motion failed: joint {} encoder fault.'),
                (193, 198, 'Motion failed: joint {} position error.')):
            if first <= code <= last:
                return text.format(code-first+1)
        return messages.get(code, 'Motion failed: unknown controller error.')

    def arm_error(self, *, joint_motion=False):
        result = self.mc.get_error_information()
        if type(result) is not int or result < 0:
            raise RuntimeError('Command failed: no controller response.')
        # A latched Cartesian error must not invalidate a working joint command.
        if joint_motion and 32 <= result <= 36:
            return
        if result:
            raise RuntimeError(self.controller_error(result))

    def stop_robot(self):
        errors = []
        try:
            self.motion_reply(self.mc.stop(_async=True))
        except Exception as exc:
            errors.append(str(exc))
        if self.with_gripper and self._gripper_active:
            try:
                opening = self.mc.get_pro_gripper_angle()
                if type(opening) not in (int, float) or not math.isfinite(opening) or not 0 <= opening <= 100:
                    raise RuntimeError(f'Invalid SDK gripper angle: {opening!r}')
                # This gripper has no usable SDK stop: retain its measured opening.
                self.gripper_reply(self.mc.set_pro_gripper_angle(int(round(opening))))
                self._gripper_active = False
            except Exception as exc:
                errors.append(str(exc))
        if errors:
            raise RuntimeError('; '.join(errors))

    def execute(self, command):
        self.cancelled()
        kind, target, speed = command['kind'], command['target'], command['speed']
        if kind == 'coords' and target[2] < 0 and not self.allow_collision:
            raise CommandRejected('Target Z must be at least 0 mm.')
        if kind == 'clear_errors':
            self.mc.clear_error_information()
            self.cancelled()
            self.arm_error()
            self.status('Errors cleared: SDK reports no controller error.')
            return
        if kind == 'joints':
            self._validate_joint_target(target)
            self.cancelled()
        self._active_kind = kind
        if kind == 'joints':
            self.motion_reply(self.mc.send_angles(target, speed, _async=True), asynchronous_send=True)
        elif kind == 'coords':
            self.motion_reply(self.mc.send_coords(target, speed, _async=True), asynchronous_send=True)
        else:
            self.gripper_reply(self.mc.set_pro_gripper_speed(speed))
            self.cancelled()
            self._gripper_active = True
            self.gripper_reply(self.mc.set_pro_gripper_angle(int(round(target[0]*100.))))
        # The vendor async path returns 1 immediately after writing the socket;
        # it is neither a controller acknowledgement nor an arrival result.
        self.status('Command sent; waiting for SDK target confirmation.')
        if self._stop.wait(.25):
            raise CommandStopped()
        deadline = time.monotonic()+600.
        idle_since = None
        reply = None
        while time.monotonic() < deadline:
            self.cancelled()
            if kind == 'gripper':
                result = self.mc.get_pro_gripper_status()
                if type(result) is not int or result not in (0, 1, 2, 3):
                    raise RuntimeError(f'SDK gripper status returned {result!r}')
                if result != 0:
                    self._gripper_active = False
                    description = {1: 'stopped', 2: 'stopped; object detected', 3: 'stopped; object lost'}[result]
                    self.status('SDK reports gripper ' + description + '.')
                    return
            else:
                reply = self.mc.get_motion_status()
                if reply is not None and reply['error'] is not None:
                    raise RuntimeError(self.controller_error(reply['error']))
                result = self.mc.is_moving()
                if type(result) is not int or result not in (0, 1):
                    raise RuntimeError('Command failed: no controller response.')
                if result == 0:
                    self.arm_error(joint_motion=kind == 'joints')
                    arrived = self.mc.is_in_position(target, 1 if kind == 'coords' else 0)
                    if type(arrived) is not int or arrived not in (0, 1):
                        raise RuntimeError('Command failed: no controller response.')
                    # A rejection may arrive while the status queries are waiting.
                    reply = self.mc.get_motion_status()
                    if reply is not None and reply['error'] is not None:
                        raise RuntimeError(self.controller_error(reply['error']))
                    if arrived == 1:
                        self.status('Motion complete: SDK confirmed the target position.')
                        return
                    # Allow controller startup/status latency without treating
                    # an idle robot as success or resending the motion command.
                    if idle_since is None:
                        idle_since = time.monotonic()
                    elif time.monotonic() - idle_since >= 3.:
                        raise RuntimeError('Motion not completed: target not reached.' if reply and reply['received']
                                           else 'Command failed: no controller response.')
                else:
                    idle_since = None
            self.refresh_feedback()
            self._stop.wait(.2)
        raise RuntimeError('SDK did not report completion within 600 seconds')

    def work(self):
        while not self._closing.is_set() and rclpy.ok():
            try:
                command = self._queue.get(timeout=.2)
            except queue.Empty:
                if self._closing.is_set():
                    break
                if self._stop.is_set():
                    try:
                        self.stop_robot()
                        self.status('STOP sent.')
                    except Exception as exc:
                        self.status(f'STOP failed: {exc}', 'error')
                    finally:
                        self._stop.clear()
                if not self._closing.is_set():
                    self.refresh_feedback()
                continue
            try:
                self.execute(command)
            except CommandRejected as exc:
                # Validation has not sent any motion: reject without SDK writes.
                self.status(f'Rejected: {exc}', 'warning')
            except CommandStopped:
                try:
                    self.stop_robot()
                    self.status('STOP sent.')
                except Exception as exc:
                    self.status(f'STOP failed: {exc}', 'error')
            except Exception as exc:
                message = str(exc)
                if not message.startswith(('Motion rejected:', 'Motion failed:', 'Motion stopped',
                                           'Motion not completed:', 'Command failed:')):
                    message = f'Command failed: {message}'
                if self._active_kind is not None:
                    try:
                        self.stop_robot()
                    except Exception as stop_error:
                        message += f'; STOP failed: {stop_error}'
                self.status(message, "error")
            finally:
                with self._lock:
                    self._active_kind = None
                    self._busy = False
                    self._stop.clear()
            if not self._closing.is_set():
                self.refresh_feedback()

    def close(self):
        self._closing.set()
        self._stop.set()
        if self.worker is not None:
            self.worker.join(timeout=2.)
        if self.mc is not None:
            # The public owner stops unfinished writes if this worker was lost.
            self.mc.close()


def main():
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = None
    def interrupted(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, interrupted)
    signal.signal(signal.SIGTERM, interrupted)
    try:
        node = GuiBackend()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        if node is not None:
            node.close()
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
