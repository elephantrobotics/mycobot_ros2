"""Exclusive feature leases and a closed SDK method allowlist."""
import threading
import time

READS = {'get_angles', 'is_power_on', 'is_moving', 'get_error_information',
         'get_pro_gripper_angle', 'get_pro_gripper_speed'}
WRITES = {
    'gui': {'send_angles', 'set_pro_gripper_angle', 'set_pro_gripper_speed', 'stop',
            'get_coords', 'get_pro_gripper_status', 'send_coords', 'is_in_position',
            'get_motion_status'},
    'slider': {'send_angles', 'set_pro_gripper_angle', 'set_pro_gripper_speed', 'stop'},
    'keyboard': {'send_angle', 'jog_angle', 'set_pro_gripper_angle',
                 'set_pro_gripper_speed', 'stop'},
    'drag': {'set_motor_enabled', 'set_pro_gripper_enabled'},
}


class FeatureLease:
    def __init__(self, clock=time.monotonic, timeout=3.0):
        self.clock, self.timeout = clock, timeout
        self.lock = threading.RLock()
        self.token = None
        self.feature = None
        self.seen = 0.0

    def claim(self, token, feature):
        with self.lock:
            if feature not in WRITES or not token:
                raise ValueError('Invalid control feature')
            if self.token is not None and token != self.token:
                raise RuntimeError('Another feature owns the public robot connection')
            self.token, self.feature, self.seen = token, feature, self.clock()

    def touch(self, token):
        with self.lock:
            if self.token != token or self.token is None:
                raise RuntimeError('Control ownership expired; restart the feature launch')
            self.seen = self.clock()

    def authorize(self, token, method):
        with self.lock:
            self.touch(token)
            if method not in READS | WRITES[self.feature]:
                raise ValueError(f'{method} is not allowed for {self.feature}')

    def expired(self):
        with self.lock:
            return self.token is not None and self.clock() - self.seen > self.timeout

    def clear(self, token):
        with self.lock:
            if token != self.token:
                raise RuntimeError('Wrong feature owner')
            self.token = self.feature = None
