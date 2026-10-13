"""Feature-side ROS proxy. Never imports the hardware SDK or opens a robot socket."""
import json
import math
import threading
import time
import uuid
from types import SimpleNamespace

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from pro630_model import ARM_JOINTS
from mycobotpro630_gazeboros2.srv import Pro630Rpc


class PublicPro630Client:
    public_feedback = True

    def __init__(self, ip=None, netport=None, debug=False, feedback_hz=10.,
                 ownership_timeout=15., feature='slider'):
        self.owner_token = uuid.uuid4().hex
        self._call_lock = threading.Lock()
        self._condition = threading.Condition()
        self._callback = None
        self._latest = None
        self._queried_sequence = -1
        self._rpc_sequence = -1
        self._stamps = {}
        self._closed = False
        self._claimed = False
        self._heartbeat_pending = None
        self.node = Node('pro630_feature_proxy_' + self.owner_token[:8])
        self.client = self.node.create_client(Pro630Rpc, '/pro630/robot_rpc')
        self.node.create_subscription(JointState, '/pro630/real_joint_states', self._sample, 1)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()
        try:
            if not self.client.wait_for_service(timeout_sec=5.):
                raise RuntimeError('Public robot service unavailable; start slider.launch.py environment:=real')
            self._rpc('claim', [feature])
            self._claimed = True
            self.node.create_timer(.5, self._heartbeat)
        except Exception:
            self.close()
            raise

    def _request(self, method, args=(), kwargs=None):
        req = Pro630Rpc.Request()
        req.token, req.method = self.owner_token, method
        req.args_json = json.dumps(list(args), allow_nan=False)
        req.kwargs_json = json.dumps(kwargs or {}, allow_nan=False)
        return req

    def _rpc(self, method, args=(), kwargs=None):
        with self._call_lock:
            if self._closed:
                raise RuntimeError('Feature proxy closed')
            completed = threading.Event()
            future = self.client.call_async(self._request(method, args, kwargs))
            future.add_done_callback(lambda _: completed.set())
            if not completed.wait(15.):
                future.cancel()
                raise RuntimeError(f'Public robot request timed out: {method}')
            response = future.result()
            if response is None or not response.success:
                raise RuntimeError(response.error if response is not None else 'Public robot service failed')
            self._stamps.update(json.loads(response.stamps_json))
            return json.loads(response.result_json)

    def _heartbeat(self):
        if not self._claimed or self._closed:
            return
        if self._heartbeat_pending is None or self._heartbeat_pending.done():
            self._heartbeat_pending = self.client.call_async(self._request('heartbeat'))

    def _sample(self, msg):
        values = dict(zip(msg.name, msg.position))
        if not all(name in values for name in ARM_JOINTS):
            return
        wall = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        angles = tuple(math.degrees(values[name]) for name in ARM_JOINTS)
        measured = time.monotonic() - max(0., time.time() - wall)
        with self._condition:
            if self._latest is not None and wall <= self._latest.wall_time:
                return
            sequence = 1 if self._latest is None else self._latest.sequence + 1
            sample = SimpleNamespace(angles=angles, monotonic=measured, wall_time=wall, sequence=sequence)
            self._latest = sample
            self._stamps['arm'] = [measured, wall]
            self._condition.notify_all()
            callback = self._callback
        if callback:
            try:
                callback(sample)
            except Exception:
                pass  # The normal get_angles path validates and reports the same data.

    def get_angles(self):
        packet = self._rpc('get_angles', [self._rpc_sequence])
        self._rpc_sequence = packet['sequence']
        if packet['error']:
            raise RuntimeError(packet['error'])
        return packet['value']

    def sample_time(self, kind):
        return self._stamps.get(kind)

    def set_angle_callback(self, callback):
        with self._condition:
            self._callback = callback

    def __getattr__(self, name):
        from pro630_broker_support import READS, WRITES
        if name not in READS | set().union(*WRITES.values()):
            raise AttributeError(name)
        return lambda *args, **kwargs: self._rpc(name, args, kwargs)

    def close(self):
        if self._closed:
            return
        try:
            if self._claimed:
                self._rpc('release')
        except Exception as exc:
            self.node.get_logger().warning(f'Feature lease release incomplete: {exc}')
        finally:
            self._closed = True
            self._claimed = False
            with self._condition:
                self._condition.notify_all()
            self.executor.shutdown(timeout_sec=1.)
            self.thread.join(timeout=1.)
            self.node.destroy_node()
