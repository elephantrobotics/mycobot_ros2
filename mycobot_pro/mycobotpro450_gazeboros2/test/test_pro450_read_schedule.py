"""No ROS/hardware: virtual clock checks scheduling, freshness and recovery."""
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from pro450_real_keyboard import RealPoseReader, RealKeyboardTransport


class ReadSdk:
    def __init__(self):
        self.gripper = 31
        self.arm_reads = self.gripper_reads = 0
        self.moving = 0
        self.writes = []
        self.delay = lambda: None

    def get_angles(self):
        self.arm_reads += 1
        return list(getattr(self, 'angles', [-90, -120, 120, -90, 90, 0]))

    def get_pro_gripper_angle(self, gripper_id=14):
        assert gripper_id == 14
        self.gripper_reads += 1
        self.delay()
        return self.gripper

    def is_moving(self):
        return self.moving

    def is_power_on(self):
        return 1

    def get_error_information(self):
        return 0

    def stop(self, **kwargs):
        self.writes.append(('stop', kwargs))


class ReadScheduleTests(unittest.TestCase):
    def setUp(self):
        self.now = 10.0
        self.sdk = ReadSdk()
        self.logs = []
        self.reader = RealPoseReader(self.sdk, [(-3, 3)] * 6 + [(0, 1)],
                                     clock=lambda: self.now, report=self.logs.append)

    def test_gripper_2hz_arm_each_cycle(self):
        for i in range(11):
            self.now = 10 + i * .1
            self.reader()
        self.assertEqual(self.sdk.arm_reads, 11)
        self.assertEqual(self.sdk.gripper_reads, 3)

    def test_cached_gripper_does_not_refresh_its_timestamp(self):
        self.reader()
        stamp = self.reader.gripper_time
        self.now += .1
        self.reader()
        self.assertEqual(self.reader.gripper_time, stamp)
        self.assertGreater(self.reader.arm_time, stamp)

    def test_minus_one_read_reuses_last_valid_gripper(self):
        self.reader()
        stamp = self.reader.gripper_time
        self.sdk.gripper = -1
        for _ in range(4):
            self.now += .5
            pose = self.reader()
            self.assertEqual(pose[6], .31)
            self.assertEqual(self.reader.gripper, .31)
            self.assertEqual(self.reader.gripper_time, stamp)
            self.assertTrue(self.reader.fresh())
        self.assertEqual(self.sdk.gripper_reads, 5)
        self.assertEqual(self.reader.failure_count, 0)
        self.assertTrue(self.reader.gripper_rejected_minus_one)

    def test_motion_reads_arm_live_and_gripper_from_cache(self):
        self.reader()
        arm_reads = self.sdk.arm_reads
        gripper_reads = self.sdk.gripper_reads
        self.sdk.angles = [10, -120, 120, -90, 90, 0]
        self.sdk.gripper = -1
        self.now += 1.5
        pose = self.reader(moving=True)
        self.assertAlmostEqual(pose[0], math.radians(10))
        self.assertEqual(pose[6], .31)
        self.assertEqual(self.sdk.arm_reads, arm_reads + 1)
        self.assertEqual(self.sdk.gripper_reads, gripper_reads)
        self.assertTrue(self.reader.gripper_from_cache)
        self.assertTrue(self.reader.fresh())

    def test_transport_tracks_arm_while_moving(self):
        transport = RealKeyboardTransport(self.sdk, self.reader, lambda _: None,
            self.reader.limits, clock=lambda: self.now)
        for i in range(4):
            self.now = 10 + i * .1
            transport.cycle()
        self.assertTrue(transport.arm())
        self.sdk.moving = 1
        self.now += .1
        transport.cycle()
        arm_reads = self.sdk.arm_reads
        gripper_reads = self.sdk.gripper_reads
        for i in range(5):
            self.now += .1
            self.sdk.angles = [-90 + i, -120, 120, -90, 90, 0]
            transport.cycle()
        self.assertEqual(self.sdk.arm_reads, arm_reads + 5)
        self.assertEqual(self.sdk.gripper_reads, gripper_reads)
        self.assertAlmostEqual(transport.pose[0], math.radians(-86))
        self.assertIsNone(transport.error)
        self.assertTrue(transport.armed)
        self.assertIsNotNone(transport.feedback())

    def test_minus_one_without_any_valid_value_still_fails(self):
        self.sdk.gripper = -1
        with self.assertRaisesRegex(RuntimeError, 'raw=-1'):
            self.reader()

    def test_seeded_gripper_covers_first_minus_one(self):
        self.reader.seed_gripper(.4)
        self.sdk.gripper = -1
        self.now += .5
        self.assertEqual(self.reader()[6], .4)

    def test_minus_one_does_not_disarm_transport(self):
        transport = RealKeyboardTransport(self.sdk, self.reader, lambda _: None,
            self.reader.limits, clock=lambda: self.now)
        for i in range(4):
            self.now = 10 + i * .1
            transport.cycle()
        self.assertTrue(transport.arm())
        self.sdk.gripper = -1
        for i in range(1, 12):
            self.now = 10.3 + i * .1
            transport.cycle()
        self.assertIsNone(transport.error)
        self.assertTrue(transport.armed)
        self.assertIsNotNone(transport.feedback())

    def test_two_spaced_retries_then_cooldown(self):
        self.sdk.gripper = -1
        for stamp, expected_reads in [(10, 1), (10.1, 1), (10.21, 2),
                                      (10.31, 2), (10.42, 3), (10.9, 3), (11.43, 4)]:
            self.now = stamp
            with self.assertRaises(RuntimeError):
                self.reader()
            self.assertEqual(self.sdk.gripper_reads, expected_reads)
        self.assertIn('failed_attempt=3/3', self.logs[2])

    def test_stale_cache_blocks_without_overwriting_last_read(self):
        self.reader()
        self.now += .9
        self.reader.next_gripper = self.now + 1
        with self.assertRaisesRegex(RuntimeError, 'stale'):
            self.reader()
        self.assertEqual(self.sdk.gripper_reads, 1)
        self.assertFalse(self.reader.fresh())

    def test_slow_gripper_does_not_make_old_arm_sample_new(self):
        self.sdk.delay = lambda: setattr(self, 'now', self.now + .6)
        with self.assertRaisesRegex(RuntimeError, 'stale'):
            self.reader()
        self.assertAlmostEqual(self.reader.arm_time, 10)
        self.assertAlmostEqual(self.reader.gripper_time, 10.6)

    def test_callback_observes_current_pose_and_moving_state(self):
        observed = []
        transport = RealKeyboardTransport(self.sdk, self.reader,
            lambda pose: observed.append((transport.pose, transport.moving, pose)),
            self.reader.limits, clock=lambda: self.now)
        for i in range(4):
            self.now = 10 + i * .1
            transport.cycle()
        self.sdk.moving = 1
        self.now += .1
        transport.cycle()
        self.assertTrue(observed[-1][1])
        self.assertEqual(observed[-1][0], observed[-1][2])

    def test_fault_recovery_requires_three_distinct_gripper_reads_and_manual_arm(self):
        transport = RealKeyboardTransport(self.sdk, self.reader, lambda _: None,
            self.reader.limits, clock=lambda: self.now)
        for i in range(4):
            self.now = 10 + i * .1
            transport.cycle()
        transport._latch_fault(RuntimeError('test gripper read failure'))
        for i in range(1, 12):
            self.now = 10.3 + i * .1
            transport.cycle()
        self.assertFalse(transport.recovery_ready)
        self.assertFalse(transport.armed)
        # Third new physical gripper read after the fault.
        self.now = 11.6
        transport.cycle()
        self.assertTrue(transport.recovery_ready)
        self.assertIsNotNone(transport.error)
        self.assertFalse(transport.armed)
        self.assertEqual(self.sdk.writes, [])
        self.assertTrue(transport.arm())
        self.assertIsNone(transport.error)
        self.assertEqual(self.sdk.writes, [])


if __name__ == '__main__':
    unittest.main()
