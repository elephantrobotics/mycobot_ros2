"""Launch-session contracts, independent of Qt, ROS and motion controllers."""
import math
import os
import signal
import subprocess
import threading
import time

from pro630_model import COMMAND_JOINTS as JOINTS, LIMITS_DEG


class EnvironmentLatch:
    def __init__(self, environment, clock=time.monotonic):
        self.environment = environment
        self.clock = clock
        self.state = None
        self.received = None
        self.session_id = None

    def observe(self, state):
        if state.get('environment') != self.environment:
            raise RuntimeError('Control and environment modes do not match')
        session = state.get('session_id')
        if not isinstance(session, str) or not session:
            raise ValueError('Missing environment session ID')
        if self.session_id is not None and session != self.session_id:
            raise RuntimeError('Environment restarted; restart the control launch')
        self.session_id = session
        self.state, self.received = state, self.clock()

    def fresh(self):
        return self.received is not None and self.clock() - self.received < 2.0

    def ready(self):
        return self.fresh() and self.state.get('ready') is True


def read_initial_snapshot(sdk, clock=time.monotonic, sleep=time.sleep):
    """Only query hardware. Keep the existing five-sample startup criteria."""
    if sdk.is_power_on() != 1:
        raise RuntimeError('Pro630 is not powered; automatic power-on is forbidden')
    error = sdk.get_error_information()
    if error not in (0, None):
        raise RuntimeError(f'Pro630 reports error {error}')
    samples = []
    for index in range(5):
        if sdk.is_moving() != 0:
            raise RuntimeError('Pro630 must be stationary for the initial snapshot')
        angles = sdk.get_angles()
        if not isinstance(angles, (list, tuple)) or len(angles) != 6:
            raise RuntimeError('Incomplete initial joint feedback')
        if any(isinstance(v, bool) or not isinstance(v, (int, float)) or
               not math.isfinite(v) or not lo <= v <= hi
               for v, (lo, hi) in zip(angles, LIMITS_DEG)):
            raise RuntimeError('Invalid initial joint feedback')
        samples.append(list(angles))
        if index < 4:
            sleep(0.2)
    if any(max(p[i] for p in samples) - min(p[i] for p in samples) > 0.2
           for i in range(6)):
        raise RuntimeError('Pro630 initial pose is not stable')
    gripper = read_initial_gripper(sdk, clock, sleep)
    # Confirm the arm stayed still during a possibly slow gripper read.
    if sdk.is_moving() != 0:
        raise RuntimeError('Pro630 moved during the initial snapshot')
    final = sdk.get_angles()
    if (not isinstance(final, (list, tuple)) or len(final) != 6 or
            any(not isinstance(v, (int, float)) or isinstance(v, bool)
                or not math.isfinite(v) or not lo <= v <= hi or abs(v - old) > 0.2
                for v, old, (lo, hi) in zip(final, samples[-1], LIMITS_DEG))):
        raise RuntimeError('Pro630 pose changed during the initial snapshot')
    return [math.radians(v) for v in final] + [gripper / 100.0]


def read_initial_gripper(sdk, clock=time.monotonic, sleep=time.sleep, timeout=10.0):
    deadline = clock() + timeout
    finished, cancelled = threading.Event(), threading.Event()
    result = {'last': None}
    def read():
        try:
            while not cancelled.is_set() and clock() < deadline:
                try:
                    value = sdk.get_pro_gripper_angle()
                except Exception as exc:
                    value = str(exc)
                result['last'] = value
                if (not isinstance(value, bool) and isinstance(value, (int, float))
                        and math.isfinite(value) and 0 <= value <= 100
                        and not cancelled.is_set() and clock() < deadline):
                    result['value'] = value
                    return
                sleep(min(0.1, max(0.0, deadline - clock())))
        finally:
            finished.set()
    worker = threading.Thread(target=read, daemon=True)
    worker.start()
    if not finished.wait(timeout) or 'value' not in result:
        cancelled.set()
        raise RuntimeError(f'Initial gripper feedback timed out after {timeout:g} s (last={result["last"]!r})')
    worker.join()
    return result['value']


def control_commands(feature, environment, executable_directory, python, show_force_execute=False):
    """Explicit process allowlist: no Gazebo, MoveIt or additional SDK owner."""
    from pathlib import Path
    root = Path(executable_directory)
    def command(script, parameters):
        args = [python, '-B', str(root / script), '--ros-args']
        for name, value in parameters.items():
            args.extend(['-p', f'{name}:={value}'])
        return args
    if feature == 'slider':
        return [command('slider_control_gazebo.py', {
            'mode': environment, 'use_sim_time': 'true'}),
            command('pro630_slider_gui.py', {
                'environment': environment,
                'show_force_execute': 'true' if show_force_execute else 'false',
                'use_sim_time': 'false' if environment == 'real' else 'true'})]
    if feature == 'keyboard':
        return [command('teleop_keyboard_gazebo.py', {
            'mode': environment, 'use_sim_time': 'false',
            'real_hold_enabled': 'true', 'real_gripper_hold_enabled': 'true'})]
    if feature == 'drag':
        if environment != 'real':
            raise ValueError('Drag following requires the real environment')
        return [command('pro630_drag_follow.py', {'use_sim_time': 'false'})]
    raise ValueError('Unsupported control feature')


def stop_owned_children(children, send_signal=None, clock=time.monotonic):
    """Stop all owned process groups together within a shared eight-second budget."""
    send_signal = os.killpg if send_signal is None else send_signal
    remaining = [child for child in children if child.poll() is None]
    for sig, grace in ((signal.SIGINT, 5.0), (signal.SIGTERM, 2.0), (signal.SIGKILL, 1.0)):
        for child in remaining:
            try:
                send_signal(child.pid, sig)
            except ProcessLookupError:
                pass  # Child exited between poll and signal; still reap it.
        deadline = clock() + grace
        pending = []
        for child in remaining:
            try:
                child.wait(timeout=max(0.0, deadline - clock()))
            except subprocess.TimeoutExpired:
                pending.append(child)
        remaining = pending
        if not remaining:
            return
    raise RuntimeError('An owned control process did not exit after shutdown signals')
