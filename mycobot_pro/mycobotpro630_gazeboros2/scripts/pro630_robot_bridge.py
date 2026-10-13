#!/usr/bin/env python3
"""Public real-pose owner: one SDK connection, serial calls and Gazebo mirror."""
import json
import math
import signal
import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from builtin_interfaces.msg import Duration, Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from mycobotpro630_gazeboros2.srv import Pro630Rpc

from pro630_sdk_adapter import Pro630Client
from pro630_session_support import JOINTS, LIMITS_DEG, read_initial_snapshot
from pro630_broker_support import FeatureLease
from pro630_real_mirror import RealPoseBuffer
from pro630_model import DEFAULT_PRO630_IP, DEFAULT_PRO630_PORT


class RobotBridge(Node):
    def __init__(self):
        super().__init__('pro630_robot_bridge')
        self.sdk = None
        self.hardware_lock = threading.RLock()
        self.cache_condition = threading.Condition()
        self.stop_event = threading.Event()
        self.lease = FeatureLease()
        self.stamps = {}
        self.packet = {'sequence': 0, 'value': None, 'error': 'No arm feedback'}
        self.arm_active = self.gripper_active = False
        self.arm_goal = None
        self.mirror = RealPoseBuffer()
        self.next_gripper = 0.
        self.worker = None
        self.frame = None
        self.environment_ready = False
        self.environment_seen = 0.
        self._last_lease_error_log = float('-inf')
        try:
            self.declare_parameter('pro630_ip', DEFAULT_PRO630_IP)
            self.declare_parameter('pro630_port', DEFAULT_PRO630_PORT)
            self.sdk = Pro630Client(self.get_parameter('pro630_ip').value,
                                   int(self.get_parameter('pro630_port').value),
                                   feedback_hz=20., ownership_timeout=0)
            # Startup is strictly read-only; feature launches are not needed for the snapshot.
            self.initial_pose = read_initial_snapshot(self.sdk)
            self.gripper = self.initial_pose[6]
            self.stamps = {kind: self.sdk.sample_time(kind) for kind in ('arm', 'gripper')}
            self.arm_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 1)
            self.gripper_pub = self.create_publisher(JointTrajectory, '/pro_gripper_controller/joint_trajectory', 1)
            self.real_pub = self.create_publisher(JointState, '/pro630/real_joint_states', 1)
            self.age_pub = self.create_publisher(Float64MultiArray, '/pro630/real_feedback_age', 1)
            qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.snapshot_pub = self.create_publisher(JointState, '/pro630/real_state_snapshot', qos)
            self.create_subscription(String, '/pro630/environment_state', self.environment_state, qos)
            self.create_service(Pro630Rpc, '/pro630/robot_rpc', self.rpc,
                                callback_group=ReentrantCallbackGroup())
            self.create_timer(.02, self.render)
            self.create_timer(.2, self.publish_snapshot)
            self.create_timer(.2, self.check_transport)
            self.worker = threading.Thread(target=self.sample_loop, daemon=True)
            self.worker.start()
            self.get_logger().info('Public robot connection acquired; continuous tracking active. No startup motion sent.')
        except Exception:
            self.close()
            raise

    def environment_state(self, msg):
        try:
            state = json.loads(msg.data)
            if state.get('environment') == 'real':
                self.environment_ready = state.get('ready') is True
                self.environment_seen = time.monotonic()
        except (ValueError, TypeError):
            pass

    def check_transport(self):
        if self.sdk is not None and self.sdk._closed.is_set():
            raise RuntimeError('Pro630 transport closed; restart the public environment after checking the connection')

    def update_gripper(self, value):
        if isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value) and 0 <= value <= 100:
            stamp = self.sdk.sample_time('gripper')
            if stamp is not None:
                self.gripper = value / 100.
                self.stamps['gripper'] = stamp

    def sample_loop(self):
        while not self.stop_event.is_set():
            if self.lease.expired():
                try:
                    with self.hardware_lock:
                        if self.lease.expired():
                            self.release_owner(self.lease.token)
                except Exception as exc:
                    now = time.monotonic()
                    if now - self._last_lease_error_log >= 5.:
                        self.get_logger().error(f'Expired feature stop failed: {exc}')
                        self._last_lease_error_log = now
            value, error = None, ''
            try:
                with self.hardware_lock:
                    value = self.sdk.get_angles()
                    stamp = self.sdk.sample_time('arm')
                    self.stamps['arm'] = stamp
                    valid = (isinstance(value, (list, tuple)) and len(value) == 6
                             and all(not isinstance(v, bool) and isinstance(v, (int, float))
                                     and math.isfinite(v) and lo <= v <= hi
                                     for v, (lo, hi) in zip(value, LIMITS_DEG)))
                    if valid and stamp is not None:
                        self.stamps['arm'] = stamp
                        pose = [math.radians(v) for v in value] + [self.gripper]
                        previous = self.frame
                        velocity = [0.] * 7 if previous is None else [
                            (current - old) / max(.001, stamp[0] - previous[0])
                            for current, old in zip(pose, previous[2])]
                        self.frame = (stamp[0], stamp[1], pose, velocity)
                        self.mirror.push(pose, when=stamp[0])
                        if self.arm_goal and max(abs(a-b) for a,b in zip(value, self.arm_goal)) < .5:
                            self.arm_active = False
            except Exception as exc:
                error = str(exc)
            with self.cache_condition:
                self.packet = {'sequence': self.packet['sequence'] + 1, 'value': value, 'error': error}
                self.cache_condition.notify_all()
            # Publish the arm packet before a possibly slow idle gripper query.
            if time.monotonic() >= self.next_gripper:
                with self.hardware_lock:
                    if not self.stop_event.is_set():
                        try:
                            self.update_gripper(self.sdk.get_pro_gripper_angle())
                        except Exception:
                            pass  # Preserve the last measured opening and timestamp.
                        self.next_gripper = time.monotonic() + .5
            self.stop_event.wait(.005)

    def release_owner(self, token):
        # Called under the hardware lock. Never change motor enable on feature exit.
        self.lease.touch(token)
        if self.arm_active:
            self.sdk.stop(deceleration=0, _async=True)
            self.arm_active = False
        if self.gripper_active:
            opening = self.sdk.get_pro_gripper_angle()
            if not isinstance(opening, (int, float)) or isinstance(opening, bool) or not 0 <= opening <= 100:
                raise RuntimeError('Cannot hold gripper: fresh opening unavailable')
            self.sdk.set_pro_gripper_angle(int(round(opening)))
            self.gripper_active = False
            self.update_gripper(opening)
        self.lease.clear(token)

    def rpc(self, req, res):
        try:
            args, kwargs = json.loads(req.args_json or '[]'), json.loads(req.kwargs_json or '{}')
            if not isinstance(args, list) or not isinstance(kwargs, dict):
                raise ValueError('Invalid request payload')
            method = req.method
            if method == 'heartbeat':
                self.lease.touch(req.token)
                result = True
            elif method == 'claim':
                with self.hardware_lock:
                    if not self.environment_ready or time.monotonic() - self.environment_seen > 2.:
                        raise RuntimeError('Public environment is not ready for a control feature')
                    self.lease.claim(req.token, args[0])
                    self.mirror.clear()
                result = True
            elif method == 'get_angles':
                self.lease.authorize(req.token, method)
                deadline = time.monotonic() + .6
                with self.cache_condition:
                    while (self.packet['sequence'] == 0 or self.packet['sequence'] <= args[0]) and time.monotonic() < deadline:
                        self.cache_condition.wait(max(0., deadline - time.monotonic()))
                    if self.packet['sequence'] == 0 or self.packet['sequence'] <= args[0]:
                        raise RuntimeError('Public arm sampling timed out')
                    result = dict(self.packet)
            else:
                with self.hardware_lock:
                    if method == 'release':
                        self.release_owner(req.token)
                        result = True
                    else:
                        self.lease.authorize(req.token, method)
                        if method == 'set_motor_enabled' and (args not in ([254, 0], [254, 1]) or kwargs):
                            raise ValueError('Drag mode may switch only all six arm motors')
                        if method == 'set_pro_gripper_enabled' and (
                                len(args) != 1 or type(args[0]) is not int or args[0] not in (0, 1) or kwargs):
                            raise ValueError('Drag mode may only enable or disable the default gripper')
                        result = getattr(self.sdk, method)(*args, **kwargs)
                        if method == 'get_pro_gripper_angle':
                            self.update_gripper(result)
                        elif method == 'set_pro_gripper_angle' and result == 1:
                            self.gripper_active = True
                        elif method in ('send_angles', 'send_angle', 'jog_angle'):
                            self.arm_active = True
                            self.arm_goal = list(args[0]) if method == 'send_angles' else None
                            if method == 'send_angle' and self.frame:
                                self.arm_goal = [math.degrees(v) for v in self.frame[2][:6]]
                                self.arm_goal[int(args[0])-1] = args[1]
                        elif method == 'stop':
                            self.arm_active = False
            res.success = True
            res.result_json = json.dumps(result, allow_nan=False)
            res.stamps_json = json.dumps(self.stamps)
        except Exception as exc:
            res.success, res.error = False, str(exc)
            res.stamps_json = '{}'
        return res

    @staticmethod
    def trajectory(names, pose, velocity):
        msg = JointTrajectory()
        msg.joint_names = list(names)
        msg.points = [JointTrajectoryPoint(positions=list(pose), velocities=list(velocity),
                         time_from_start=Duration(nanosec=40000000))]
        return msg

    def render(self):
        now = time.monotonic()
        arm_stamp, grip_stamp = self.stamps.get('arm'), self.stamps.get('gripper')
        ages = Float64MultiArray()
        ages.data = [now - arm_stamp[0] if arm_stamp else -1., now - grip_stamp[0] if grip_stamp else -1.]
        self.age_pub.publish(ages)
        frame = self.frame
        if frame is None or now - frame[0] > .5:
            return
        measured, wall, pose, measured_velocity = frame
        msg = JointState()
        sec = int(wall)
        msg.header.stamp = Time(sec=sec, nanosec=int((wall-sec)*1e9))
        msg.name, msg.position = list(JOINTS), list(pose[:6]) + [self.gripper]
        msg.velocity = list(measured_velocity)
        self.real_pub.publish(msg)
        if self.lease.feature == 'drag':
            shown, velocity = list(pose[:6]) + [self.gripper], [0.] * 7
        else:
            rendered = self.mirror.render()
            if rendered is None:
                return
            shown, velocity = rendered
        # Gripper rendering uses only the latest successfully measured opening.
        shown[6], velocity[6] = self.gripper, 0.0
        self.arm_pub.publish(self.trajectory(JOINTS[:6], shown[:6], velocity[:6]))
        self.gripper_pub.publish(self.trajectory(JOINTS[6:], shown[6:], velocity[6:]))

    def publish_snapshot(self):
        if self.frame is None or time.monotonic() - self.frame[0] > .5:
            return
        # This is an initial-pose source, not an SDK ownership handoff.
        msg = JointState()
        wall = min(self.frame[1], self.stamps['gripper'][1])
        sec = int(wall); msg.header.stamp = Time(sec=sec, nanosec=int((wall-sec)*1e9))
        msg.header.frame_id = 'pro630:retain:' + self.sdk.owner_token
        msg.name, msg.position = list(JOINTS), list(self.frame[2][:6]) + [self.gripper]
        self.snapshot_pub.publish(msg)

    def close(self):
        self.stop_event.set()
        if self.worker is not None:
            self.worker.join(timeout=1.)
        if self.sdk is not None:
            try:
                with self.hardware_lock:
                    if self.lease.token is not None:
                        self.release_owner(self.lease.token)
            finally:
                self.sdk.close()
                self.sdk = None


def main():
    rclpy.init()
    node = None
    executor = MultiThreadedExecutor(num_threads=6)
    code = 0
    def interrupted(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, interrupted)
    signal.signal(signal.SIGTERM, interrupted)
    try:
        node = RobotBridge()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f'Public Pro630 bridge failed: {exc}')
        code = 1
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        executor.shutdown(timeout_sec=1.)
        if node is not None:
            node.close(); node.destroy_node()
        rclpy.try_shutdown()
    raise SystemExit(code)


if __name__ == '__main__':
    main()
