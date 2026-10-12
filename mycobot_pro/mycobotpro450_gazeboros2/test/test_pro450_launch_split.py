"""Startup handoff, feature boundaries and failure cases without hardware."""
import ast
from pathlib import Path
import sys
import signal
import subprocess
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
from pro450_session_support import (EnvironmentLatch, control_commands, read_initial_snapshot,
                                   read_initial_gripper, stop_owned_children)
from pro450_sdk_adapter import RobotConnectionLease


class FakeSDK:
    def __init__(self):
        self.calls = []
        self.gripper = iter([-1, -1, 42])
        self.angles = [10, -20, 30, -40, 50, -60]
    def is_power_on(self):
        self.calls.append('power'); return 1
    def get_error_information(self):
        self.calls.append('error'); return 0
    def is_moving(self):
        self.calls.append('moving'); return 0
    def get_angles(self):
        self.calls.append('angles'); return list(self.angles)
    def get_pro_gripper_angle(self):
        self.calls.append('gripper'); return next(self.gripper)


class SplitTests(unittest.TestCase):
    def test_readiness_requires_matching_mode_recent_heartbeat_and_ready(self):
        now = [1.0]
        gate = EnvironmentLatch('real', lambda: now[0])
        self.assertFalse(gate.ready())
        gate.observe(dict(environment='real', session_id='one', ready=False))
        self.assertFalse(gate.ready())
        gate.observe(dict(environment='real', session_id='one', ready=True))
        self.assertTrue(gate.ready())
        now[0] += 2.1
        self.assertFalse(gate.ready())
        with self.assertRaisesRegex(RuntimeError, 'modes do not match'):
            gate.observe(dict(environment='simulation', session_id='one', ready=True))

    def test_restart_requires_new_feature_launch(self):
        gate = EnvironmentLatch('real')
        gate.observe(dict(environment='real', session_id='one', ready=True))
        with self.assertRaisesRegex(RuntimeError, 'restarted'):
            gate.observe(dict(environment='real', session_id='two', ready=True))

    def test_snapshot_retries_minus_one_and_only_queries_sdk(self):
        sdk = FakeSDK()
        pose = read_initial_snapshot(sdk, sleep=lambda _: None)
        self.assertAlmostEqual(pose[0], 0.17453292519943295)
        self.assertEqual(pose[6], 0.42)
        self.assertEqual(sdk.calls.count('gripper'), 3)
        self.assertEqual(sdk.calls.count('angles'), 6)
        self.assertTrue(set(sdk.calls) <= {'power', 'error', 'moving', 'angles', 'gripper'})

    def test_moving_robot_cannot_provide_snapshot(self):
        sdk = FakeSDK(); sdk.is_moving = lambda: 1
        with self.assertRaisesRegex(RuntimeError, 'stationary'):
            read_initial_snapshot(sdk, sleep=lambda _: None)
        self.assertNotIn('gripper', sdk.calls)

    def test_changed_arm_during_gripper_read_is_rejected(self):
        sdk = FakeSDK()
        def gripper():
            sdk.angles[0] += 1.0
            return 42
        sdk.get_pro_gripper_angle = gripper
        with self.assertRaisesRegex(RuntimeError, 'changed'):
            read_initial_snapshot(sdk, sleep=lambda _: None)

    def test_final_sample_must_still_respect_joint_limits(self):
        sdk = FakeSDK(); sdk.angles[0] = 162.0
        def gripper():
            sdk.angles[0] = 162.1
            return 42
        sdk.get_pro_gripper_angle = gripper
        with self.assertRaisesRegex(RuntimeError, 'changed'):
            read_initial_snapshot(sdk, sleep=lambda _: None)

    def test_blocking_gripper_call_has_bounded_deadline(self):
        release = threading.Event()
        sdk = SimpleNamespace(get_pro_gripper_angle=lambda: (release.wait(1), 42)[1])
        start = time.monotonic()
        try:
            with self.assertRaisesRegex(RuntimeError, 'timed out'):
                read_initial_gripper(sdk, timeout=0.03)
            self.assertLess(time.monotonic() - start, 0.3)
        finally:
            release.set()

    def test_invalid_gripper_cannot_be_published(self):
        now = [0.0]
        def sleep(seconds): now[0] += seconds
        sdk = SimpleNamespace(get_pro_gripper_angle=lambda: -1)
        with self.assertRaisesRegex(RuntimeError, '10 s'):
            read_initial_gripper(sdk, lambda: now[0], sleep)

    def test_slider_launch_keeps_active_real_backend_and_only_slider_ui(self):
        commands = control_commands('slider', 'real', '/installed', '/usr/bin/python3')
        self.assertEqual(len(commands), 2)
        self.assertIn('startup_read_only:=false', commands[0])
        self.assertIn('mode:=real', commands[0])
        self.assertTrue(commands[0][2].endswith('slider_control_gazebo.py'))
        self.assertTrue(commands[1][2].endswith('pro450_slider_gui.py'))
        self.assertIn('use_sim_time:=false', commands[1])

    def test_keyboard_does_not_spawn_slider_or_environment(self):
        commands = control_commands('keyboard', 'real', '/installed', '/usr/bin/python3')
        self.assertEqual(len(commands), 1)
        self.assertTrue(commands[0][2].endswith('teleop_keyboard_gazebo.py'))
        self.assertIn('real_hold_enabled:=true', commands[0])
        self.assertIn('real_gripper_hold_enabled:=true', commands[0])
        self.assertIn('use_sim_time:=false', commands[0])

    def test_simulation_feature_lease_is_exclusive_and_reusable(self):
        with tempfile.TemporaryDirectory() as directory:
            lease = RobotConnectionLease('feature-session-domain-test', 0, 0, directory)
            try:
                with self.assertRaisesRegex(RuntimeError, 'already owned'):
                    RobotConnectionLease('feature-session-domain-test', 0, 0, directory)
            finally:
                lease.close()
            RobotConnectionLease('feature-session-domain-test', 0, 0, directory).close()

    def test_base_has_no_slider_keyboard_or_sdk_nodes(self):
        tree = ast.parse((ROOT / 'launch/slider.launch.py').read_text(encoding='utf-8'))
        executables = {kw.value.value for n in ast.walk(tree) if isinstance(n, ast.Call)
                       for kw in n.keywords if kw.arg == 'executable' and isinstance(kw.value, ast.Constant)}
        self.assertNotIn('slider_control_gazebo.py', executables)
        self.assertNotIn('pro450_slider_gui.py', executables)
        self.assertNotIn('teleop_keyboard_gazebo.py', executables)
        state_source = (ROOT / 'scripts/pro450_environment_state.py').read_text()
        self.assertNotIn('pro450_sdk_adapter', state_source)
        self.assertNotIn('pymycobot', state_source)


def session_methods():
    tree = ast.parse((ROOT / 'scripts/pro450_control_session.py').read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef))
    cls.bases = []
    ns = {'time': time, 'String': lambda **kw: SimpleNamespace(**kw)}
    exec(compile(ast.Module(body=[cls], type_ignores=[]), '<session>', 'exec'), ns)
    return ns['ControlSession']


class SessionOrderTests(unittest.TestCase):
    def setUp(self):
        self.node = session_methods().__new__(session_methods())
        n = self.node
        n.failure = None; n.children = []; n.sdk = Mock(owner_token='mine')
        n.released = None; n.consumed = None; n.environment = 'real'
        n.deadline = time.monotonic() + 30; n.base_unready_since = None
        n.snapshot_wall = time.time(); n.snapshot = [0.0] * 7
        n.latch = EnvironmentLatch('real')
        n.latch.observe(dict(environment='real', session_id='one', verified=False, ready=False))
        n.ack_pub = Mock(); n._start_controls = Mock(); n._publish_snapshot = Mock()
        n._prepare_snapshot = Mock(); n._refresh_snapshot = Mock()

    def test_no_control_before_environment_ready(self):
        self.assertTrue(self.node.tick())
        self.node._start_controls.assert_not_called()
        self.node._publish_snapshot.assert_called_once()

    def test_startup_waits_for_first_environment_without_connecting(self):
        self.node.latch.received = None
        self.assertTrue(self.node.tick())
        self.node._prepare_snapshot.assert_not_called()
        self.node._start_controls.assert_not_called()

    def test_startup_environment_loss_does_not_wait_full_startup_timeout(self):
        self.node.latch.received = time.monotonic() - 3
        self.assertTrue(self.node.tick())  # Callback catch-up after a slow SDK read.
        self.node._startup_stale_since = time.monotonic() - 3
        with self.assertRaisesRegex(RuntimeError, 'heartbeat lost during startup'):
            self.node.tick()
        self.node._start_controls.assert_not_called()

    def test_startup_heartbeat_recovery_clears_catchup_deadline(self):
        self.node._startup_stale_since = time.monotonic() - 3
        self.node.tick()
        self.assertIsNone(self.node._startup_stale_since)

    def test_wrong_token_does_not_release_sdk(self):
        self.node._consumed(SimpleNamespace(data='other'))
        self.assertIsNone(self.node.consumed)
        self.node.sdk.close.assert_not_called()

    def test_consumption_releases_before_ack_and_does_not_start_control(self):
        self.node._consumed(SimpleNamespace(data='mine'))
        sdk = self.node.sdk
        events = []
        sdk.close.side_effect = lambda: events.append('close')
        self.node.ack_pub.publish.side_effect = lambda _: events.append('ack')
        self.node.tick()
        self.assertEqual(events, ['close', 'ack'])
        self.assertIsNone(self.node.sdk)
        self.node._start_controls.assert_not_called()
        self.node.tick()
        self.node._prepare_snapshot.assert_not_called()

    def test_ready_closes_any_remaining_bootstrap_connection_before_controls(self):
        self.node.latch.observe(dict(environment='real', session_id='one', verified=True, ready=True))
        events = []
        self.node.sdk.close.side_effect = lambda: events.append('close')
        self.node._start_controls.side_effect = lambda: events.append('start')
        self.node.tick()
        self.assertEqual(events, ['close', 'start'])

    def test_base_loss_shuts_down_feature(self):
        self.node.children = [Mock(poll=lambda: None)]
        self.node.base_unready_since = time.monotonic() - 3
        self.node.get_logger = Mock(return_value=Mock())
        self.assertFalse(self.node.tick())

    def test_lost_heartbeat_does_not_wait_for_a_second_grace_period(self):
        self.node.children = [Mock(poll=lambda: None)]
        self.node.latch.received = time.monotonic() - 3
        self.node.get_logger = Mock(return_value=Mock())
        self.assertFalse(self.node.tick())

    def test_cleanup_failure_still_closes_sdk_and_feature_lease(self):
        self.node.lease = Mock()
        sdk, lease = self.node.sdk, self.node.lease
        self.node.close.__globals__['stop_owned_children'] = Mock(side_effect=RuntimeError('child stuck'))
        with self.assertRaisesRegex(RuntimeError, 'child stuck'):
            self.node.close()
        sdk.close.assert_called_once()
        lease.close.assert_called_once()

    def test_sdk_close_failure_still_releases_feature_lease(self):
        self.node.lease = Mock()
        lease = self.node.lease
        self.node.sdk.close.side_effect = RuntimeError('socket close failed')
        self.node.close.__globals__['stop_owned_children'] = Mock()
        with self.assertRaisesRegex(RuntimeError, 'socket close failed'):
            self.node.close()
        lease.close.assert_called_once()


class ShutdownTests(unittest.TestCase):
    @patch.object(signal, 'SIGKILL', 9, create=True)
    def test_both_children_are_signalled_before_waiting_and_share_deadlines(self):
        now, events = [0.0], []
        class Child:
            def __init__(self, pid): self.pid, self.exited = pid, False
            def poll(self): return 0 if self.exited else None
            def wait(self, timeout):
                events.append(('wait', self.pid, timeout))
                if self.exited: return 0
                now[0] += timeout
                raise subprocess.TimeoutExpired('owned child', timeout)
        children = [Child(1), Child(2)]
        def send(pid, sig):
            events.append(('signal', pid, sig))
            if sig == signal.SIGKILL: children[pid - 1].exited = True
        stop_owned_children(children, send, lambda: now[0])
        self.assertEqual(events[:2], [('signal', 1, signal.SIGINT), ('signal', 2, signal.SIGINT)])
        self.assertLessEqual(now[0], 8.0)
        self.assertTrue(all(c.exited for c in children))
        self.assertIn(('wait', 2, 0.0), events)

    @patch.object(signal, 'SIGKILL', 9, create=True)
    def test_exit_between_poll_and_signal_is_reaped(self):
        child = Mock(pid=9, poll=lambda: None)
        child.wait.return_value = 0
        def already_gone(*_): raise ProcessLookupError()
        stop_owned_children([child], already_gone)
        child.wait.assert_called_once()

    def test_main_ignores_repeated_signals_while_cleaning_up(self):
        source = ROOT / 'scripts/pro450_control_session.py'
        tree = ast.parse(source.read_text())
        main = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'main')
        handlers = {}
        def set_signal(sig, handler):
            previous = handlers.get(sig, signal.SIG_DFL)
            handlers[sig] = handler
            return previous
        node = Mock()
        node.tick.side_effect = KeyboardInterrupt
        def cleanup():
            self.assertEqual(handlers[signal.SIGINT], signal.SIG_IGN)
            self.assertEqual(handlers[signal.SIGTERM], signal.SIG_IGN)
        node.close.side_effect = cleanup
        ns = {'signal': SimpleNamespace(SIGINT=signal.SIGINT, SIGTERM=signal.SIGTERM,
                                        SIG_IGN=signal.SIG_IGN, signal=set_signal),
              'rclpy': Mock(ok=lambda: True), 'ControlSession': lambda: node,
              'interrupted': Mock(), 'sys': sys}
        exec(compile(ast.Module(body=[main], type_ignores=[]), str(source), 'exec'), ns)
        with self.assertRaises(SystemExit) as result: ns['main']()
        self.assertEqual(result.exception.code, 0)
        node.close.assert_called_once()
        node.destroy_node.assert_called_once()

    def test_function_launches_cast_integer_timeout_to_float(self):
        for name in ('slidar_control_gazebo.launch.py', 'teleop_keyboard.launch.py'):
            tree = ast.parse((ROOT / 'launch' / name).read_text())
            timeout = next(value for n in ast.walk(tree) if isinstance(n, ast.Dict)
                           for key, value in zip(n.keys, n.values)
                           if isinstance(key, ast.Constant) and key.value == 'startup_timeout_sec')
            self.assertIsInstance(timeout, ast.Call)
            self.assertEqual(timeout.func.id, 'ParameterValue')
            self.assertTrue(any(k.arg == 'value_type' and k.value.id == 'float' for k in timeout.keywords))

    def test_environment_snapshot_and_verification_timeouts_are_typed(self):
        tree = ast.parse((ROOT / 'launch/slider.launch.py').read_text())
        values = [v for n in ast.walk(tree) if isinstance(n, ast.Dict)
                  for k, v in zip(n.keys, n.values)
                  if isinstance(k, ast.Constant) and k.value in ('timeout_sec', 'tolerance_rad')]
        self.assertEqual(len(values), 3)
        for value in values:
            self.assertIsInstance(value, ast.Call)
            self.assertEqual(value.func.id, 'ParameterValue')
            self.assertTrue(any(k.arg == 'value_type' and k.value.id == 'float' for k in value.keywords))


try:
    import rclpy
except ImportError:
    rclpy = None


@unittest.skipIf(rclpy is None, 'ROS runtime required')
class RosConstructionTests(unittest.TestCase):
    def test_environment_monitor_and_announcer_construct_without_sdk(self):
        from pro450_environment_state import EnvironmentState
        rclpy.init(args=[])
        nodes = []
        try:
            nodes.append(EnvironmentState())
            self.assertEqual(len(nodes[0].moveit_clients), 3)
            self.assertFalse(nodes[0].verified)
            nodes[0].tick()
            nodes.append(EnvironmentState(announce=True))
            nodes[1].tick()
        finally:
            for node in nodes:
                node.destroy_node()
            rclpy.try_shutdown()


if __name__ == '__main__':
    unittest.main()
