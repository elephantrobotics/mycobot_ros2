"""Public ownership, permission boundaries and cleanup without a robot socket."""
import ast
import json
import math
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
from pro450_broker_support import FeatureLease
from pro450_gripper_profile import GripperMotionEstimate, gripper_display_rate


def methods(*names):
    tree = ast.parse((ROOT / 'scripts/pro450_robot_bridge.py').read_text(encoding='utf-8'))
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef))
    ns = dict(json=json, time=time, gripper_display_rate=gripper_display_rate)
    exec(compile(ast.Module(body=[n for n in cls.body if isinstance(n, ast.FunctionDef)
                                 and n.name in names], type_ignores=[]), '<broker>', 'exec'), ns)
    return ns


class PublicBridgeTests(unittest.TestCase):
    def test_public_joint_message_passes_slider_position_and_velocity_gate(self):
        ns = methods('render')
        ns.update(JOINTS=[f'joint{i}' for i in range(1, 7)] + ['gripper_controller'],
                  JointState=lambda: SimpleNamespace(header=SimpleNamespace()),
                  Time=lambda **kw: SimpleNamespace(**kw),
                  Float64MultiArray=SimpleNamespace)
        now, wall = time.monotonic(), time.time()
        b = SimpleNamespace(stamps={'arm': (now, wall), 'gripper': (now, wall)},
            frame=(now, wall, [0.] * 6 + [.5], [.1] * 6 + [0.]), gripper=.6,
            age_pub=Mock(), real_pub=Mock(), arm_pub=Mock(), gripper_pub=Mock(),
            lease=SimpleNamespace(feature='drag', token='owner'), trajectory=Mock())
        ns['render'](b)
        msg = b.real_pub.publish.call_args.args[0]
        self.assertEqual(len(msg.velocity), 7)
        b.trajectory.assert_any_call(ns['JOINTS'][6:], [.6], [0.])
        b.gripper_pub.publish.assert_called_once()
        source = ROOT / 'scripts/pro450_slider_gui.py'
        tree = ast.parse(source.read_text(encoding='utf-8'))
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'SliderGuiNode')
        fn = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == '_joint_cb')
        gui_ns = dict(COMMAND_JOINTS=ns['JOINTS'], math=math, time=time)
        exec(compile(ast.Module(body=[fn], type_ignores=[]), '<slider-gate>', 'exec'), gui_ns)
        gui = SimpleNamespace(_lock=threading.Lock(), environment='real')
        gui_ns['_joint_cb'](gui, msg)
        self.assertTrue(gui._feedback_valid)
        self.assertLess(time.monotonic() - gui._actual_time, 1.)
        msg.velocity = []
        gui_ns['_joint_cb'](gui, msg)
        self.assertFalse(gui._feedback_valid)

    def bridge(self):
        ns = methods('rpc', 'release_owner')
        sdk = Mock(get_pro_gripper_angle=Mock(return_value=50))
        b = SimpleNamespace(lease=FeatureLease(), hardware_lock=threading.RLock(),
            cache_condition=threading.Condition(), sdk=sdk, stamps={},
            mirror=Mock(), gripper_model=GripperMotionEstimate(), gripper=.5,
            arm_active=False, gripper_active=False, arm_goal=None, frame=None,
            gripper_speed=1, environment_ready=True, environment_seen=time.monotonic(),
            update_gripper=Mock(), packet={'sequence': 3, 'value': [0.]*6, 'error': ''})
        b.release_owner = lambda token: ns['release_owner'](b, token)
        def rpc(method, args=(), token='owner'):
            req = SimpleNamespace(method=method, token=token, args_json=json.dumps(list(args)), kwargs_json='{}')
            res = SimpleNamespace(success=False, result_json='', error='', stamps_json='')
            return ns['rpc'](b, req, res)
        return b, sdk, rpc

    def test_one_feature_owner_and_no_enable_on_claim(self):
        b, sdk, rpc = self.bridge()
        self.assertTrue(rpc('claim', ['drag']).success)
        sdk.set_motor_enabled.assert_not_called()
        self.assertFalse(rpc('claim', ['slider'], token='other').success)
        self.assertEqual(b.lease.feature, 'drag')

    def test_not_ready_environment_cannot_claim_control_or_enable(self):
        b, sdk, rpc = self.bridge()
        b.environment_ready = False
        self.assertFalse(rpc('claim', ['drag']).success)
        self.assertIsNone(b.lease.token)
        sdk.set_motor_enabled.assert_not_called()

    def test_drag_can_only_switch_arm_and_gripper_enable_without_motion(self):
        _, sdk, rpc = self.bridge()
        rpc('claim', ['drag'])
        for method, args in [('send_angles', [[0]*6, 10]), ('set_pro_gripper_angle', [100]),
                             ('set_motor_enabled', [1, 0])]:
            self.assertFalse(rpc(method, args).success)
        sdk.set_motor_enabled.return_value = 1
        self.assertTrue(rpc('set_motor_enabled', [254, 0]).success)
        sdk.set_motor_enabled.assert_called_once_with(254, 0)
        sdk.set_pro_gripper_enabled.return_value = 1
        for invalid in ([], [2], [-1], [True], [0, 14]):
            self.assertFalse(rpc('set_pro_gripper_enabled', invalid).success)
        for enable in (0, 1):
            self.assertTrue(rpc('set_pro_gripper_enabled', [enable]).success)
        self.assertEqual(sdk.set_pro_gripper_enabled.call_count, 2)
        sdk.send_angles.assert_not_called(); sdk.set_pro_gripper_angle.assert_not_called()

    def test_other_features_cannot_switch_gripper_enable(self):
        for feature in ('slider', 'keyboard'):
            _, sdk, rpc = self.bridge()
            self.assertTrue(rpc('claim', [feature]).success)
            self.assertFalse(rpc('set_pro_gripper_enabled', [0]).success)
            sdk.set_pro_gripper_enabled.assert_not_called()

    def test_read_angles_uses_public_packet_not_another_sdk_query(self):
        _, sdk, rpc = self.bridge()
        rpc('claim', ['keyboard'])
        reply = rpc('get_angles', [2])
        self.assertTrue(reply.success)
        self.assertEqual(json.loads(reply.result_json)['value'], [0.]*6)
        sdk.get_angles.assert_not_called()

    def test_release_stops_owned_motion_and_keeps_motor_enable_unchanged(self):
        b, sdk, rpc = self.bridge()
        rpc('claim', ['slider'])
        b.arm_active = b.gripper_active = True
        self.assertTrue(rpc('release').success)
        sdk.stop.assert_called_once_with(deceleration=0, _async=True)
        sdk.set_pro_gripper_angle.assert_called_once_with(50)
        sdk.set_motor_enabled.assert_not_called()
        self.assertIsNone(b.lease.token)
        self.assertFalse(rpc('send_angles', [[0]*6, 10]).success)

    def test_heartbeat_and_expiration(self):
        now = [0.]
        lease = FeatureLease(lambda: now[0])
        lease.claim('owner', 'slider')
        now[0] = 2.; lease.touch('owner')
        now[0] = 4.; self.assertFalse(lease.expired())
        now[0] = 5.1; self.assertTrue(lease.expired())

    def test_function_files_cannot_open_hardware_connection(self):
        for name in ('pro450_control_session.py', 'pro450_drag_follow.py',
                     'teleop_keyboard_gazebo.py', 'slider_control_gazebo.py'):
            source = (ROOT / 'scripts' / name).read_text(encoding='utf-8')
            self.assertNotIn('from pro450_sdk_adapter import Pro450Client', source)
            if name != 'pro450_control_session.py':
                self.assertIn('PublicPro450Client', source)
        client = (ROOT / 'scripts/pro450_public_client.py').read_text(encoding='utf-8')
        self.assertNotIn('pymycobot', client)
        self.assertNotIn('socket', client.split('"""', 2)[-1])

    def test_common_launch_owns_bridge_and_scene_without_startup_enable(self):
        source = (ROOT / 'launch/slider.launch.py').read_text(encoding='utf-8')
        self.assertIn("executable='pro450_robot_bridge.py'", source)
        self.assertIn("executable='pro450_collision_scene.py'", source)
        tree = ast.parse((ROOT / 'scripts/pro450_robot_bridge.py').read_text(encoding='utf-8'))
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef))
        init = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == '__init__')
        calls = {n.func.attr for n in ast.walk(init) if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)}
        self.assertFalse(calls & {'send_angles', 'send_angle', 'set_motor_enabled', 'power_on'})

    def test_feature_shutdown_keeps_ros_alive_until_proxy_cleanup(self):
        for name in ('slider_control_gazebo.py', 'teleop_keyboard_gazebo.py', 'pro450_drag_follow.py'):
            source = (ROOT / 'scripts' / name).read_text(encoding='utf-8')
            self.assertIn('signal_handler_options=SignalHandlerOptions.NO', source)
            main = source[source.index('def main('):]
            self.assertLess(main.index('.close()'), main.index('rclpy.try_shutdown()'))


if __name__ == '__main__':
    unittest.main()
