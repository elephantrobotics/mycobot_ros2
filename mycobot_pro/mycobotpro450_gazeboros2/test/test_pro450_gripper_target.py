"""Cached preflight and single-target gripper execution, without hardware."""
import ast
import math
from pathlib import Path
import sys
import time
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

scripts = Path(__file__).resolve().parents[1] / 'scripts'
sys.path.insert(0, str(scripts))
from pro450_real_keyboard import RealPoseReader, sdk_speed_for_rad
from pro450_gripper_profile import gripper_speed_for_duration, GripperMotionEstimate


def methods():
    tree = ast.parse((scripts / 'slider_control_gazebo.py').read_text(encoding='utf-8'))
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'SliderControl')
    names = {'_run_real_motion', '_confirm_gripper_speed', '_validate_real_motion', '_mirror_loop'}
    namespace = dict(math=math, COLLISION_SAMPLE_STEP=.1, WAYPOINT_ARM_TOLERANCE_RAD=.01,
                     sdk_speed_for_rad=sdk_speed_for_rad, REAL_ARRIVAL_MARGIN_S=10,
                     gripper_speed_for_duration=lambda *a: 2,
                     measured_gripper_rate=lambda *a: .3,
                     sdk_motion_accepted=lambda v: v == 1,
                     rclpy=SimpleNamespace(ok=lambda: True))
    namespace.update(time=time, threading=threading, Float64MultiArray=SimpleNamespace, publish_age=Mock())
    exec(compile(ast.Module(body=[n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in names],
                            type_ignores=[]), 'slider_control_gazebo.py', 'exec'), namespace)
    return namespace


class GripperTargetTests(unittest.TestCase):
    def speed_probe(self, readbacks):
        mc = Mock()
        mc.set_pro_gripper_speed.return_value = 1
        mc.get_pro_gripper_speed.side_effect = readbacks
        return SimpleNamespace(mc=mc, _stop_requested=False,
                               _stop_event=Mock(wait=Mock(return_value=False)),
                               _publish_status=Mock(), _read_real_positions=Mock())

    def test_speed_read_timeout_then_success_recovers_without_rewriting(self):
        obj = self.speed_probe([-1, 2])
        self.assertTrue(methods()['_confirm_gripper_speed'](obj, 2))
        obj.mc.set_pro_gripper_speed.assert_called_once_with(2)
        self.assertEqual(obj.mc.get_pro_gripper_speed.call_count, 2)
        obj._read_real_positions.assert_called_once_with(moving=True)
        obj.mc.set_pro_gripper_angle.assert_not_called()
        obj.mc.send_angles.assert_not_called()

    def test_speed_read_exception_and_invalid_response_then_success(self):
        obj = self.speed_probe([TimeoutError('no reply'), None, 2])
        self.assertTrue(methods()['_confirm_gripper_speed'](obj, 2))
        self.assertEqual(obj.mc.get_pro_gripper_speed.call_count, 3)

    def test_old_speed_readback_can_recover_on_retry(self):
        obj = self.speed_probe([8, 2])
        self.assertTrue(methods()['_confirm_gripper_speed'](obj, 2))

    def test_persistent_speed_mismatch_reports_values_after_all_retries(self):
        obj = self.speed_probe([8, 8, 8])
        with self.assertRaisesRegex(RuntimeError, r'requested=2, set_result=1, readbacks=\[8, 8, 8\]'):
            methods()['_confirm_gripper_speed'](obj, 2)
        self.assertEqual(obj.mc.get_pro_gripper_speed.call_count, 3)
        obj.mc.send_angles.assert_not_called()

    def test_all_failed_speed_reads_report_failure_only_after_retries(self):
        obj = self.speed_probe([-1, -1, -1])
        with self.assertRaisesRegex(RuntimeError, 'after 3 reads'):
            methods()['_confirm_gripper_speed'](obj, 2)
        self.assertEqual(obj.mc.get_pro_gripper_speed.call_count, 3)

    def test_stop_during_speed_retry_cancels_without_next_read(self):
        obj = self.speed_probe([-1, 2])
        obj._stop_event.wait.return_value = True
        self.assertFalse(methods()['_confirm_gripper_speed'](obj, 2))
        self.assertEqual(obj.mc.get_pro_gripper_speed.call_count, 1)

    def test_stop_during_speed_read_prevents_motion_even_with_matching_speed(self):
        obj = self.speed_probe([])
        def stopped_read():
            obj._stop_requested = True
            return 2
        obj.mc.get_pro_gripper_speed.side_effect = stopped_read
        self.assertFalse(methods()['_confirm_gripper_speed'](obj, 2))

    def test_failed_new_read_cannot_be_satisfied_by_valid_cache(self):
        sdk = SimpleNamespace(get_pro_gripper_angle=Mock(return_value=-1))
        reader = RealPoseReader(sdk, [(-3, 3)] * 6 + [(0, 1)], clock=lambda: 10)
        reader.seed_gripper(.5)
        with self.assertRaises(RuntimeError):
            reader.read_fresh_gripper()
        self.assertEqual(reader.gripper, .5)

    def test_successful_preflight_commits_new_measurement(self):
        sdk = SimpleNamespace(get_pro_gripper_angle=Mock(return_value=56),
                              sample_time=lambda _: (9.9, 100))
        reader = RealPoseReader(sdk, [(-3, 3)] * 6 + [(0, 1)], clock=lambda: 10)
        self.assertEqual(reader.read_fresh_gripper(), .56)
        self.assertEqual(reader.gripper_time, 9.9)

    def test_stale_response_cannot_pass_preflight(self):
        sdk = SimpleNamespace(get_pro_gripper_angle=lambda **_: 56,
                              sample_time=lambda _: (1, 100))
        reader = RealPoseReader(sdk, [(-3, 3)] * 6 + [(0, 1)], clock=lambda: 10)
        with self.assertRaises(RuntimeError):
            reader.read_fresh_gripper()

    def test_cached_start_does_not_query_gripper_or_refresh_old_timestamp(self):
        sdk = SimpleNamespace(get_angles=lambda: [0] * 6,
                              get_pro_gripper_angle=Mock(side_effect=AssertionError('unexpected gripper query')))
        reader = RealPoseReader(sdk, [(-3, 3)] * 6 + [(0, 1)], clock=lambda: 10)
        reader.seed_gripper(.5)
        reader.cache.gripper_time = 1
        reader.gripper_valid = False  # A later idle read failed; last valid sample survives.
        self.assertEqual(reader(moving=True)[6], .5)
        self.assertEqual(reader.cache.gripper_time, 1)
        sdk.get_pro_gripper_angle.assert_not_called()

    def test_missing_cache_does_not_trigger_a_gripper_read(self):
        sdk = SimpleNamespace(get_angles=lambda: [0] * 6, get_pro_gripper_angle=Mock())
        reader = RealPoseReader(sdk, [(-3, 3)] * 6 + [(0, 1)], clock=lambda: 10)
        with self.assertRaises(RuntimeError):
            reader.cached_gripper()
        sdk.get_pro_gripper_angle.assert_not_called()

    def test_gripper_only_target_does_not_send_any_arm_command(self):
        mc = Mock(); mc.get_pro_gripper_speed.return_value = 2
        current = [0.] * 6 + [.5]; target = [0.] * 6 + [.6]
        obj = SimpleNamespace(_stop_requested=False, get_node_names=lambda: [],
                              _publish_status=Mock(), _pose_reader=SimpleNamespace(cached_gripper=Mock(return_value=.5)),
                              _read_real_positions=Mock(return_value=current), _gazebo_matches=lambda _: True,
                              _path_is_valid=lambda *a: (True, ''),
                              _validate_real_motion=lambda *a: (True, ''),
                              _trajectory_duration=lambda *a: 2, mc=mc,
                              _gripper_model=GripperMotionEstimate(),
                              _send_gripper=mc.set_pro_gripper_angle,
                              _follow_real_line=Mock(return_value=True), _stop_real_motion=Mock())
        obj._confirm_gripper_speed = lambda speed: methods()['_confirm_gripper_speed'](obj, speed)
        methods()['_run_real_motion'](obj, target, .2)
        mc.send_angles.assert_not_called()
        mc.set_pro_gripper_angle.assert_called_once_with(60)
        mc.set_pro_gripper_speed.assert_called_once_with(2)
        self.assertEqual(obj._pose_reader.cached_gripper.call_count, 1)
        self.assertTrue(all(call.kwargs == {'moving': True} for call in obj._read_real_positions.call_args_list))

    def test_long_collision_scan_keeps_real_arm_feedback_fresh_on_owner_thread(self):
        owner = threading.get_ident(); polls = []; validation_threads = []
        sdk = SimpleNamespace(get_angles=Mock(side_effect=lambda: polls.append(
            (time.monotonic(), threading.get_ident())) or [0] * 6),
            get_pro_gripper_angle=Mock(side_effect=AssertionError('gripper query during validation')))
        reader = RealPoseReader(sdk, [(-3, 3)] * 6 + [(0, 1)])
        reader.seed_gripper(.5); original_stamp = reader.gripper_time
        def scan(*_):
            validation_threads.append(threading.get_ident())
            threading.Event().wait(1.25)  # Longer than GUI's one-second stale threshold.
            return True, ''
        obj = SimpleNamespace(_stop_requested=False, _stop_event=threading.Event(),
                              _publish_status=Mock(), _path_is_valid=scan,
                              _read_real_positions=lambda **kw: reader(**kw))
        began = time.monotonic()
        valid, _ = methods()['_validate_real_motion'](obj, [0]*6+[.5], [0]*6+[.6])
        self.assertTrue(valid)
        self.assertGreaterEqual(len(polls), 5)
        self.assertTrue(all(thread == owner for _, thread in polls))
        self.assertTrue(all(thread != owner for thread in validation_threads))
        times = [began] + [stamp for stamp, _ in polls] + [time.monotonic()]
        self.assertLess(max(b-a for a,b in zip(times,times[1:])), .5)
        self.assertLess(time.monotonic()-reader.arm_time, .5)
        sdk.get_pro_gripper_angle.assert_not_called()
        self.assertEqual(reader.gripper_time, original_stamp)

    def test_stop_during_scan_prevents_execution_and_finishes_validation_thread(self):
        finished = threading.Event()
        obj = SimpleNamespace(_stop_requested=False, _stop_event=threading.Event(),
                              _publish_status=Mock())
        def scan(*_):
            while not obj._stop_requested:
                time.sleep(.005)
            finished.set()
            return True, ''
        def poll(**_):
            obj._stop_requested = True
        obj._path_is_valid = scan
        obj._read_real_positions = poll
        valid, reason = methods()['_validate_real_motion'](obj, [0]*6+[.5], [0]*6+[.6])
        self.assertFalse(valid)
        self.assertEqual(reason, 'STOP requested.')
        self.assertTrue(finished.is_set())

    def test_feedback_failure_cancels_scan_before_returning_to_command_loop(self):
        obj = SimpleNamespace(_stop_requested=False, _stop_event=threading.Event(),
                              _publish_status=Mock(), _read_real_positions=Mock(side_effect=RuntimeError('arm read failed')))
        finished = threading.Event()
        def scan(*_):
            while not obj._stop_requested:
                time.sleep(.005)
            finished.set()
            return False, 'cancelled'
        obj._path_is_valid = scan
        with self.assertRaisesRegex(RuntimeError, 'arm read failed'):
            methods()['_validate_real_motion'](obj, [0]*7, [0]*7)
        self.assertTrue(finished.is_set())
        self.assertTrue(obj._stop_event.is_set())

    def test_changed_start_pose_is_revalidated_before_any_motion_write(self):
        mc = Mock(); mc.get_pro_gripper_speed.return_value = 2
        first = [0.] * 6 + [.5]; changed = [0.] * 6 + [.53]; target = [0.] * 6 + [.6]
        stages = []
        def validate(current, goal):
            stages.append(('validated', current[6]))
            self.assertEqual(mc.method_calls, [])
            return True, ''
        obj = SimpleNamespace(_stop_requested=False, get_node_names=lambda: [],
                              _publish_status=Mock(), _pose_reader=SimpleNamespace(cached_gripper=lambda: .5),
                              _read_real_positions=Mock(side_effect=[first, changed, changed]),
                              _gazebo_matches=lambda _: True, _validate_real_motion=validate,
                              _trajectory_duration=lambda *a: 2, mc=mc,
                              _gripper_model=GripperMotionEstimate(), _send_gripper=mc.set_pro_gripper_angle,
                              _follow_real_line=Mock(return_value=True), _stop_real_motion=Mock())
        obj._confirm_gripper_speed = lambda speed: methods()['_confirm_gripper_speed'](obj, speed)
        methods()['_run_real_motion'](obj, target, .2)
        self.assertEqual(stages, [('validated', .5), ('validated', .53)])
        mc.set_pro_gripper_angle.assert_called_once_with(60)
        mc.send_angles.assert_not_called()

    def test_arm_and_gripper_target_runs_only_one_moveit_path_scan(self):
        path = Mock(return_value=(True, 'valid'))
        obj = SimpleNamespace(_stop_requested=False, _stop_event=threading.Event(),
                              _publish_status=Mock(), _path_is_valid=path,
                              _read_real_positions=Mock())
        current = [0.] * 6 + [.5]; target = [.2] * 6 + [.8]
        self.assertEqual(methods()['_validate_real_motion'](obj,current,target), (True,'valid'))
        path.assert_called_once_with(current,target)

    def test_speed_uses_measured_direction_and_never_extrapolates(self):
        profile = {'opening': [[1, 20], [4, 40], [8, 60]],
                   'closing': [[1, 10], [4, 30], [8, 50]]}
        self.assertEqual(gripper_speed_for_duration(10, 50, 1, profile), 4)
        self.assertEqual(gripper_speed_for_duration(50, 10, 1, profile), 4)
        self.assertEqual(gripper_speed_for_duration(10, 50, 100, profile), 1)
        self.assertEqual(gripper_speed_for_duration(10, 50, .01, profile), 8)
        self.assertIsNone(gripper_speed_for_duration(10, 10, 1, profile))

    def test_estimate_continues_without_any_angle_reads_and_clamps_target(self):
        now = [10.0]; model = GripperMotionEstimate(clock=lambda: now[0])
        model.start(.5, .6, .2)
        now[0] = 10.25
        value, velocity, done = model.sample()
        self.assertAlmostEqual(value, .55)
        self.assertEqual(velocity, .2)
        self.assertFalse(done)
        model.observe(.51, 10.25)
        self.assertAlmostEqual(model.sample()[0], .55)
        now[0] = 11
        self.assertEqual(model.sample(), (.6, 0, True))
        model.observe(.51, 11)
        self.assertIsNotNone(model.sample())
        model.observe(.59, 11)
        self.assertIsNone(model.sample())

    def test_stop_holds_the_estimate_without_continuing_toward_goal(self):
        now = [0.0]; model = GripperMotionEstimate(clock=lambda: now[0])
        model.start(.6, .4, .1)
        now[0] = 1
        model.hold()
        now[0] = 10
        self.assertAlmostEqual(model.sample()[0], .5)
        self.assertEqual(model.sample()[1], 0)

    def test_simulated_gripper_keeps_rendering_when_arm_feedback_is_missing(self):
        ns = methods(); stop = Mock()
        stop.is_set.side_effect = [False, True]
        obj = SimpleNamespace(_mirror_stop=stop, _pose_reader=None,
                              _real_mirror=SimpleNamespace(render=lambda: None),
                              _gripper_model=SimpleNamespace(sample=lambda: (.55, .29, False)),
                              gripper_estimate_pub=Mock(), _publish_trajectory=Mock(),
                              _publish_gripper_trajectory=Mock(), get_logger=Mock(return_value=Mock()))
        ns['_mirror_loop'](obj)
        obj._publish_trajectory.assert_not_called()
        obj._publish_gripper_trajectory.assert_called_once_with(.55, .04, .29)
        self.assertEqual(obj.gripper_estimate_pub.publish.call_args[0][0].data, [.55, .29, 1])


if __name__ == '__main__':
    unittest.main()
