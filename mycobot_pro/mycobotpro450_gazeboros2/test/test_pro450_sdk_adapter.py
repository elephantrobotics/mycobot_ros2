"""Actual SDK parsing against an in-memory transport; no robot socket is opened."""
import math
from pathlib import Path
import socket
import struct
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import pro450_sdk_adapter as adapter
from pro450_real_keyboard import RealPoseReader


class QueueTests(unittest.TestCase):
    def test_repeated_batches_have_bounded_latest_angles(self):
        queue = adapter.ResponseList()
        for i in range(10000):
            for n in range(3):
                queue.offer(bytearray([254, 254, 4, 32, n, 0, 0]), i, i)
            self.assertEqual(len(queue), 1)
        self.assertEqual(queue[0][0][4], 2)
        self.assertEqual(len(queue.metadata), 1)
        self.assertEqual(queue.dropped, 29999)

    def test_ack_data_and_arrival_are_separate(self):
        queue = adapter.ResponseList()
        for frame in ([254, 254, 5, 34, 255, 1, 0, 0],
                      [254, 254, 4, 34, 0, 0, 0], [254, 254, 4, 91, 0, 0, 0]):
            queue.offer(bytearray(frame), 10, 20)
        self.assertEqual(len(queue), 3)
        queue.remove(queue[0])
        self.assertEqual(queue.consumed[34], (10, 20))
        queue.discard(34)
        self.assertEqual(len(queue), 1)
        self.assertNotIn(34, queue.consumed)

    def test_custom_modbus_frames_are_keyed_by_register(self):
        queue = adapter.ResponseList()
        for register in (12, 13):
            queue.offer(bytearray([254, 254, 14, 181, 254, 254, 8,
                                   14, 3, 0, register, 0, 31, 0, 0, 0, 0]), 10, 20)
        self.assertEqual(len(queue), 2)
        self.assertEqual(adapter.ResponseList.register_key(queue[0][0]), (14, 3, 0, 12))

    def test_ownership_requires_release_even_with_same_process(self):
        with tempfile.TemporaryDirectory() as directory:
            first = adapter.RobotConnectionLease('test-endpoint', 4500, 0, directory)
            try:
                with self.assertRaisesRegex(RuntimeError, 'already owned'):
                    adapter.RobotConnectionLease('test-endpoint', 4500, 0, directory)
            finally:
                first.close()
            second = adapter.RobotConnectionLease('test-endpoint', 4500, 0, directory)
            second.close()


@unittest.skipIf(adapter._VendorClient is object, 'pymycobot not installed')
class SdkAdapterTests(unittest.TestCase):
    def setUp(self):
        from pymycobot.pro450_close_loop import Pro450CloseLoop
        self.client = adapter.Pro450Client.__new__(adapter.Pro450Client)
        Pro450CloseLoop.__init__(self.client)
        self.client._adapter_init()
        self.client.read_command = adapter.ResponseList()
        self.client._angle_period = 0
        self.client.language = 'en_US'
        self.sent = []
        self.threads = []
        self.reply = lambda genre, raw: [self.angles(20)] if genre == 32 else []

        def write(raw, method=None):
            self.sent.append(bytes(raw))
            frames = self.reply(raw[3], raw)
            def deliver():
                self.client._receive_frames(frames, time.monotonic(), time.time())
            t = threading.Thread(target=deliver)
            self.threads.append(t)
            t.start()
        self.client._write = write

    def tearDown(self):
        for t in self.threads:
            t.join(timeout=1)
        self.client.close()

    def frame(self, genre, payload):
        data = [254, 254, len(payload) + 3, genre] + list(payload)
        return bytearray(data + self.client.crc_check(data))

    def angles(self, degrees):
        return self.frame(32, struct.pack('>6h', *[round(degrees * 100)] * 6))

    def test_old_angle_is_discarded_before_new_query(self):
        self.client.read_command.offer(self.angles(10), time.monotonic() - 5, time.time() - 5)
        self.assertEqual(self.client.get_angles(), [20] * 6)
        self.assertLess(time.monotonic() - self.client.sample_time('arm')[0], .5)
        self.assertEqual(len(self.client.read_command), 0)
        self.assertEqual(self.client.write_command, [])

    def test_batch_returns_latest_and_does_not_accumulate(self):
        self.reply = lambda genre, raw: [self.angles(1), self.angles(2), self.angles(3)]
        for _ in range(100):
            self.assertEqual(self.client.get_angles(), [3] * 6)
            self.assertEqual(len(self.client.read_command), 0)
            self.assertEqual(self.client.write_command, [])
            self.assertLessEqual(len(self.client.read_command.metadata), 1)

    def test_expired_query_does_not_admit_late_response(self):
        self.reply = lambda genre, raw: []
        self.assertEqual(self.client.get_angles(), -1)
        self.assertEqual(self.client.write_command, [])
        self.client._receive_frames([self.angles(10)], time.monotonic(), time.time())
        self.assertEqual(len(self.client.read_command), 0)
        self.reply = lambda genre, raw: [self.angles(20)]
        self.assertEqual(self.client.get_angles(), [20] * 6)

    def test_unrequested_frames_update_feedback_without_sdk_backlog(self):
        samples = []
        self.client.set_angle_callback(samples.append)
        mono, wall = time.monotonic() - .1, time.time() - .1
        self.client._receive_frames([self.angles(1), self.angles(2)], mono, wall)
        self.assertEqual(len(samples), 1)
        self.assertEqual(samples[0].angles, (2,) * 6)
        self.assertEqual(samples[0].monotonic, mono)
        self.assertEqual(samples[0].wall_time, wall)
        self.assertEqual(len(self.client.read_command), 0)

    def test_callback_runs_without_sdk_writer_lock(self):
        def callback(sample):
            acquired = self.client.lock.acquire(blocking=False)
            self.assertTrue(acquired)
            if acquired:
                self.client.lock.release()
        self.client.set_angle_callback(callback)
        self.client._receive_frames([self.angles(1)], time.monotonic(), time.time())
        self.assertEqual(self.client.feedback_error, '')

    def test_crc_and_split_frame_handling(self):
        frame = self.angles(20)
        frames, remain = self.client._parse_frames(frame[:8])
        self.assertEqual(frames, [])
        frames, remain = self.client._parse_frames(remain + frame[8:] + frame)
        self.assertEqual(len(frames), 2)
        self.assertEqual(remain, b'')
        broken = bytearray(frame)
        broken[-1] ^= 1
        frames, _ = self.client._parse_frames(broken + frame)
        self.assertEqual(frames, [frame])
        self.assertGreater(self.client.invalid_frames, 0)

    def test_tool_register_response_cannot_replace_angle_response(self):
        wrong = self.frame(181, [14, 3, 0, 13, 0, 99, 0, 0])
        right = self.frame(181, [14, 3, 0, 12, 0, 31, 0, 0])
        self.reply = lambda genre, raw: [wrong, right]
        self.assertEqual(self.client.get_pro_gripper_angle(), 31)
        self.assertIsNotNone(self.client.sample_time('gripper'))

    def test_synchronous_move_preserves_ack_and_arrival(self):
        self.reply = lambda genre, raw: [self.frame(34, [255, 1]), self.frame(91, [0])]
        self.assertEqual(self.client.send_angles([0] * 6, 1), 0)
        self.assertEqual(self.client.write_command, [])

    def test_async_stop_bypasses_waiting_api_lock(self):
        result = []
        with self.client._api_lock:
            t = threading.Thread(target=lambda: result.append(self.client.stop(_async=True)))
            t.start()
            t.join(timeout=.5)
            self.assertFalse(t.is_alive())
        self.assertEqual(result, [1])
        self.assertEqual(self.sent[-1][3], adapter.STOP)

    def test_socket_eof_exits_receiver_and_close_releases_lease(self):
        left, right = socket.socketpair()
        self.client.sock = left
        thread = threading.Thread(target=self.client.read_thread, args=('socket',))
        self.client.read_threading = thread
        thread.start()
        right.close()
        thread.join(timeout=.5)
        self.assertFalse(thread.is_alive())
        self.assertIn('disconnected', self.client.feedback_error)

    def test_real_constructor_and_receiver_with_fake_socket(self):
        left, right = socket.socketpair()
        with patch.object(adapter.Pro450Client, 'connect_socket', return_value=left):
            client = adapter.Pro450Client('in-memory-constructor-test', 0, ownership_timeout=0)
        samples = []
        try:
            client.set_angle_callback(samples.append)
            right.sendall(self.angles(25))
            deadline = time.monotonic() + .5
            while not samples and time.monotonic() < deadline:
                time.sleep(.001)
            self.assertEqual(samples[-1].angles, (25,) * 6)
            self.assertEqual(client.diagnostics()['queued_responses'], 0)
        finally:
            right.close()
            client.close()
        self.assertFalse(client.read_threading.is_alive())

    def test_four_byte_angles_use_vendor_conversion(self):
        frame = self.frame(32, struct.pack('>6i', *[1000] * 6))
        samples = []
        self.client.set_angle_callback(samples.append)
        self.client._receive_frames([frame], time.monotonic(), time.time())
        self.assertEqual(samples[-1].angles, (self.client._int2angle(1000),) * 6)


class SampleTimeTests(unittest.TestCase):
    def test_old_read_cannot_overwrite_new_receiver_sample(self):
        reader = RealPoseReader(object(), [(-3, 3)] * 6 + [(0, 1)], clock=lambda: 10)
        reader.seed_gripper(.3)
        reader.observe_arm([20] * 6, 9.9, 100)
        reader.observe_arm([10] * 6, 9.8, 99)
        self.assertAlmostEqual(reader.arm[0], math.radians(20))
        self.assertEqual(reader.arm_time, 9.9)
        self.assertEqual(reader.arm_wall_time, 100)

    def test_sdk_timestamp_not_return_time_controls_freshness(self):
        class Fake:
            def get_angles(self):
                return [0] * 6
            def sample_time(self, kind):
                return (4, 104)
        reader = RealPoseReader(Fake(), [(-3, 3)] * 6 + [(0, 1)], clock=lambda: 10)
        reader.seed_gripper(.3)
        with self.assertRaisesRegex(RuntimeError, 'stale'):
            reader(moving=True)
        self.assertEqual(reader.arm_time, 4)


if __name__ == '__main__':
    unittest.main()
