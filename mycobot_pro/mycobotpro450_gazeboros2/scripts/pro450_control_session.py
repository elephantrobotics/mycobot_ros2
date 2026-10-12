#!/usr/bin/env python3
"""One function launch: read-only snapshot, readiness, unchanged control programs."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import rclpy
from ament_index_python.packages import get_package_prefix
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from pro450_sdk_adapter import Pro450Client, RobotConnectionLease
from pro450_session_support import (EnvironmentLatch, JOINTS, control_commands,
                                   read_initial_snapshot, stop_owned_children)

PACKAGE = 'mycobotpro450_gazeboros2'


class ControlSession(Node):
    def __init__(self):
        super().__init__('pro450_control_session')
        self.declare_parameter('feature', 'slider')
        self.declare_parameter('environment', 'simulation')
        self.declare_parameter('startup_timeout_sec', 600.0)
        self.feature = self.get_parameter('feature').value
        self.environment = self.get_parameter('environment').value
        if self.feature not in ('slider', 'keyboard') or self.environment not in ('simulation', 'real'):
            raise ValueError('Invalid feature or environment')
        self.latch = EnvironmentLatch(self.environment)
        self.failure = None
        self.children = []
        self.sdk = None
        self.snapshot = None
        self.snapshot_wall = None
        self.consumed = None
        self.released = None
        self.lease = None
        self.base_unready_since = None
        self._startup_stale_since = None
        self.deadline = time.monotonic() + float(self.get_parameter('startup_timeout_sec').value)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/pro450/environment_state', self._state, qos)
        self.snapshot_pub = self.create_publisher(JointState, '/pro450/real_state_snapshot', qos)
        self.ack_pub = self.create_publisher(String, '/pro450/snapshot_released', 10)
        self.create_subscription(String, '/pro450/snapshot_consumed', self._consumed, 10)
        try:
            domain = int(os.environ.get('ROS_DOMAIN_ID', '0'))
            self.lease = RobotConnectionLease(f'feature-session-domain-{domain}', 0, timeout=0)
        except Exception as exc:
            raise RuntimeError('Another Pro450 function launch is already running in this ROS domain') from exc
        self.get_logger().info(
            f'{self.feature}: waiting for the {self.environment} environment; no motion is permitted.')

    def _state(self, msg):
        try:
            self.latch.observe(json.loads(msg.data))
        except Exception as exc:
            self.failure = str(exc)

    def _consumed(self, msg):
        if self.sdk is not None and msg.data == self.sdk.owner_token:
            self.consumed = msg.data

    def _prepare_snapshot(self):
        self.get_logger().info('Acquiring an initial real pose using read-only SDK queries.')
        self.sdk = Pro450Client('192.168.0.232', 4500, ownership_timeout=0)
        self._refresh_snapshot()

    def _refresh_snapshot(self):
        self.snapshot = read_initial_snapshot(self.sdk)
        stamps = [self.sdk.sample_time(kind) for kind in ('arm', 'gripper')]
        self.snapshot_wall = min(stamp[1] for stamp in stamps if stamp is not None)

    def _publish_snapshot(self):
        msg = JointState()
        seconds = int(self.snapshot_wall)
        msg.header.stamp.sec = seconds
        msg.header.stamp.nanosec = int((self.snapshot_wall - seconds) * 1e9)
        msg.header.frame_id = 'pro450:handoff:' + self.sdk.owner_token
        msg.name, msg.position = list(JOINTS), list(self.snapshot)
        self.snapshot_pub.publish(msg)

    def _start_controls(self):
        if {'slider_control_gazebo', 'teleop_keyboard_gazebo', 'pro450_slider_gui'} & set(self.get_node_names()):
            raise RuntimeError('A legacy control process is still running; close it before starting this function')
        directory = Path(get_package_prefix(PACKAGE)) / 'lib' / PACKAGE
        for command in control_commands(self.feature, self.environment, directory, sys.executable):
            self.children.append(subprocess.Popen(command, start_new_session=True))
        self.get_logger().info(f'{self.feature}: environment ready; control programs started.')

    def tick(self):
        now = time.monotonic()
        if self.failure:
            raise RuntimeError(self.failure)
        if self.children:
            for child in self.children:
                result = child.poll()
                if result is not None:
                    if result != 0:
                        raise RuntimeError(f'Control program exited with code {result}')
                    return False
            if not self.latch.fresh():
                self.get_logger().warning('Environment heartbeat lost; closing this control session.')
                return False
            if not self.latch.ready():
                if self.base_unready_since is None:
                    self.base_unready_since = now
                if now - self.base_unready_since >= 2.0:
                    self.get_logger().warning('Environment unavailable; closing this control session.')
                    return False
            else:
                self.base_unready_since = None
            return True
        if now >= self.deadline:
            raise RuntimeError('Timed out waiting for the public environment')
        if not self.latch.fresh():
            # Before the first heartbeat, allow either launch order. After an
            # observed environment disappears, do not hold the SDK/feature
            # lease until the full 600 s startup timeout. Give callbacks time
            # to catch up after a slow read-only snapshot query.
            if self.latch.received is not None:
                stale_since = getattr(self, '_startup_stale_since', None)
                if stale_since is None:
                    self._startup_stale_since = now
                elif now - stale_since >= 2.0:
                    raise RuntimeError('Environment heartbeat lost during startup; restart the control launch')
            return True
        self._startup_stale_since = None
        if self.latch.ready():
            if self.sdk is not None:
                self.sdk.close()
                self.sdk = None
            self._start_controls()
            return True
        if self.environment == 'real' and not self.latch.state.get('verified'):
            if self.released:
                self.ack_pub.publish(String(data=self.released))
            elif self.consumed:
                self.released = self.consumed
                self.sdk.close()
                self.sdk = None
                self.ack_pub.publish(String(data=self.released))
            else:
                if self.sdk is None:
                    self._prepare_snapshot()
                if time.time() - self.snapshot_wall > 2.0:
                    self._refresh_snapshot()
                self._publish_snapshot()
        return True

    def close(self):
        # Only owned children receive signals; the public environment is retained.
        try:
            stop_owned_children(self.children)
        finally:
            try:
                if self.sdk is not None:
                    self.sdk.close()
                    self.sdk = None
            finally:
                if self.lease is not None:
                    self.lease.close()
                    self.lease = None


def interrupted(*_args):
    raise KeyboardInterrupt


def main():
    rclpy.init()
    node = None
    code = 0
    old_term = signal.signal(signal.SIGTERM, interrupted)
    old_int = signal.signal(signal.SIGINT, interrupted)
    try:
        node = ControlSession()
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if not node.tick():
                break
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f'Failed to run Pro450 control session: {exc}', file=sys.stderr)
        code = 1
    finally:
        # launch can send SIGTERM while SIGINT cleanup is still in progress.
        # A second KeyboardInterrupt here would leave detached children alive.
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if node is not None:
            try:
                node.close()
            except Exception as exc:
                print(f'Pro450 session cleanup failed: {exc}', file=sys.stderr)
                code = 1
            finally:
                node.destroy_node()
        signal.signal(signal.SIGTERM, old_term)
        signal.signal(signal.SIGINT, old_int)
        rclpy.try_shutdown()
    raise SystemExit(code)


if __name__ == '__main__':
    main()
