"""Bounded, single-axis hardware transport. No ROS and no startup writes.

Arm holds wait until the collision scan finishes, then send one send_angle
to the same stop angle the simulation tracks. A gripper
hold sends one opening command at the keyboard gear's SDK speed. Release
stops the arm, or reads and writes the measured gripper opening on release.
Firmware braking is not a hardware E-stop.
"""
import math
import threading
import time

from pro450_hold_profile import PositionVelocityEstimator

SDK_RAD_PER_SPEED = math.pi / 120.0  # User calibration: 100 -> 150 deg/s.


def valid_gripper_position(raw):
    """Check SDK read status before mapping 0..100 to the URDF coordinate."""
    if isinstance(raw, bool) or not isinstance(raw, (int, float)):
        raise RuntimeError(f"Pro450 gripper read failed: invalid SDK response {raw!r}")
    if raw < 0:
        raise RuntimeError(
            f"Pro450 gripper read failed: SDK returned {raw!r}; "
            "Gazebo startup and real motion are blocked")
    if not math.isfinite(raw) or raw > 100:
        raise RuntimeError(f"Pro450 gripper read failed: invalid SDK position {raw!r}")
    return float(raw) / 100.0


def sdk_speed_for_rad(velocity, urdf_limit=1.0):
    if not math.isfinite(velocity) or not math.isfinite(urdf_limit) or urdf_limit <= 0:
        raise ValueError("invalid speed")
    # Round down, never exceed either the requested or the URDF speed.
    return max(0, min(100, int((min(abs(velocity), urdf_limit) + 1e-12) /
                             SDK_RAD_PER_SPEED)))


class RosPoseCache:
    """ROS-side joint cache. Only successful SDK reads are stored."""

    def __init__(self):
        self.lock = threading.RLock()
        self.arm = None
        self.arm_time = None
        self.gripper = None
        self.gripper_time = None

    def ready(self):
        return self.arm is not None and self.gripper is not None

    def store_arm(self, arm, when):
        with self.lock:
            if self.arm_time is None or when > self.arm_time:
                self.arm = list(arm)
                self.arm_time = when

    def store_gripper(self, value, when):
        with self.lock:
            self.gripper = float(value)
            self.gripper_time = when

    def pose(self):
        with self.lock:
            return list(self.arm) + [self.gripper]


class RealPoseReader:
    """Single-owner reads with a ROS-side pose cache; never writes to SDK.

    Arm angles are read from the SDK every cycle. While the robot is moving,
    the gripper opening comes from the cache and the slow gripper query is
    skipped. A gripper read of -1 is discarded and
    does not replace the cached opening. Other gripper failures still retry
    twice, then cool down for 1 s. Cache timestamps change only when a
    successful reading is stored.
    """
    supports_motion_cache = True

    def __init__(self, sdk, limits, clock=time.monotonic, report=None):
        self.sdk, self.limits, self.clock = sdk, limits, clock
        self.report = report or (lambda _: None)
        self.cache = RosPoseCache()
        self.gripper_valid = False
        self.gripper_rejected_minus_one = False
        self.gripper_from_cache = False
        self.next_gripper = 0.0
        self.gripper_period = 0.5
        self.gripper_max_age = 0.8
        self.retry_count = 0
        self.read_count = self.failure_count = self.success_count = 0
        self.last_raw = None
        self.last_error = ''
        self.arm_wall_time = None

    def _sample_stamp(self, kind):
        if hasattr(self.sdk, 'sample_time'):
            stamp = self.sdk.sample_time(kind)
            if stamp is None:
                raise RuntimeError(f"Pro450 {kind} sample has no receive timestamp")
            return stamp
        return self.clock(), time.time()

    def observe_arm(self, raw, when, wall_time):
        """Commit a received arm sample without waiting for a gripper query."""
        if not isinstance(raw, (list, tuple)) or len(raw) != 6:
            raise RuntimeError(f"Invalid Pro450 arm response: {raw!r}")
        if any(isinstance(v, bool) or not isinstance(v, (int, float)) for v in raw):
            raise RuntimeError(f"Invalid Pro450 arm response: {raw!r}")
        arm = [math.radians(v) for v in raw]
        if not all(math.isfinite(v) and lo <= v <= hi
                   for v, (lo, hi) in zip(arm, self.limits[:6])):
            raise RuntimeError(f"Pro450 arm feedback violates joint limits: {raw!r}")
        with self.cache.lock:
            if self.cache.arm_time is None or when > self.cache.arm_time:
                self.cache.store_arm(arm, when)
                self.arm_wall_time = wall_time
            return self.cache.pose() if self.cache.ready() else None

    @property
    def arm(self):
        return None if self.cache.arm is None else list(self.cache.arm)

    @property
    def arm_time(self):
        return self.cache.arm_time

    @property
    def gripper(self):
        return self.cache.gripper

    @property
    def gripper_time(self):
        return self.cache.gripper_time

    def fresh(self):
        now = self.clock()
        if self.cache.arm_time is None or now - self.cache.arm_time > 0.5:
            return False
        if self.gripper_from_cache:
            try:
                self.cached_gripper()
                return True
            except RuntimeError:
                return False
        if not self.gripper_valid or self.cache.gripper is None:
            return False
        # Motion skips the gripper query; -1 is refused. Either way the
        # previous opening stays usable.
        if self.gripper_from_cache or self.gripper_rejected_minus_one:
            return True
        return now - self.cache.gripper_time <= self.gripper_max_age

    def force_gripper_read(self):
        self.next_gripper = self.clock()

    def cached_gripper(self):
        """Last successful measured opening; no SDK call or age refresh."""
        with self.cache.lock:
            value = self.cache.gripper
            if (value is None or self.cache.gripper_time is None or
                    not math.isfinite(value) or not 0.0 <= value <= 1.0):
                raise RuntimeError("No valid gripper angle is available in cache")
            return float(value)

    def read_fresh_gripper(self):
        """One successful hardware read; never satisfy a preflight with cache."""
        raw = self.sdk.get_pro_gripper_angle(gripper_id=14)
        value = valid_gripper_position(raw)
        stamp, _ = self._sample_stamp('gripper')
        if self.clock() - stamp > self.gripper_max_age:
            raise RuntimeError("new gripper response is stale")
        self.cache.store_gripper(value, stamp)
        self.gripper_valid = True
        self.gripper_rejected_minus_one = False
        self.gripper_from_cache = False
        self.next_gripper = self.clock() + self.gripper_period
        return value

    def __call__(self, moving=False):
        raw = self.sdk.get_angles()
        stamp, wall = self._sample_stamp('arm')
        self.observe_arm(raw, stamp, wall)
        if moving and self.cache.gripper is not None:
            self.cached_gripper()
            self.gripper_from_cache = True
            # Use a known valid historical sample even after an idle read
            # fails. Do not turn that failure into a new pre-motion query.
            return self._combined()
        self.gripper_from_cache = False
        if self.clock() >= self.next_gripper:
            started = self.clock()
            self.read_count += 1
            self.last_raw = None
            try:
                self.last_raw = self.sdk.get_pro_gripper_angle(gripper_id=14)
                if self.last_raw == -1:
                    if self.cache.gripper is None:
                        raise RuntimeError(
                            "Pro450 gripper read failed: SDK returned -1; "
                            "Gazebo startup and real motion are blocked")
                    # Do not store -1. Keep the last successful opening.
                    self.gripper_rejected_minus_one = True
                    self.retry_count = 0
                    self.next_gripper = self.clock() + self.gripper_period
                    self.report(
                        f"gripper read returned -1; cache unchanged at "
                        f"{self.cache.gripper:.3f}, "
                        f"read_elapsed={self.clock() - started:.3f}s")
                    return self._combined()
                value = valid_gripper_position(self.last_raw)
            except Exception as exc:
                self.gripper_valid = False
                self.gripper_rejected_minus_one = False
                self.failure_count += 1
                self.retry_count += 1
                attempt = self.retry_count
                delay = 0.2 if attempt <= 2 else 1.0
                self.next_gripper = self.clock() + delay
                if attempt >= 3:
                    self.retry_count = 0
                self.last_error = (
                    f"{exc}; raw={self.last_raw!r}, read_elapsed="
                    f"{self.clock() - started:.3f}s, failed_attempt={attempt}/3, "
                    f"total_failures={self.failure_count}, next_read_in={delay:.1f}s")
                self.report(self.last_error)
                raise RuntimeError(self.last_error) from exc
            self.cache.store_gripper(value, self._sample_stamp('gripper')[0])
            self.gripper_valid = True
            self.gripper_rejected_minus_one = False
            self.retry_count = 0
            self.success_count += 1
            self.next_gripper = self.clock() + self.gripper_period
            self.last_error = ''
            self.report(
                f"gripper read valid: raw={self.last_raw!r}, "
                f"read_elapsed={self.clock() - started:.3f}s, reads={self.read_count}, "
                f"total_failures={self.failure_count}")
        return self._combined()

    def seed_gripper(self, value):
        self.cache.store_gripper(float(value), self._sample_stamp('gripper')[0])
        self.gripper_valid = True
        self.gripper_rejected_minus_one = False
        self.next_gripper = self.clock() + self.gripper_period

    def _combined(self):
        if not self.fresh():
            raise RuntimeError(self.last_error or "Pro450 arm/gripper feedback stale")
        return self.cache.pose()


class RealKeyboardTransport:
    """One SDK owner; latest command only, finite goals, read-only by default."""

    def __init__(self, sdk, read_pose, publish_pose, limits, clock=time.monotonic,
                 report_error=None):
        self.sdk, self.read_pose, self.publish_pose = sdk, read_pose, publish_pose
        self.limits, self.clock = limits, clock
        self.report_error = report_error or (lambda _: None)
        self.lock = threading.Lock()
        self.pose = None
        self.velocity = None
        self.received = 0.0
        self.error = None
        self.armed = False
        self.command = None
        self.stop_pending = None
        self.motion_sent = False
        self.motion_axis = None
        self.generation = 0
        self.closed = threading.Event()
        self.estimator = PositionVelocityEstimator(window=0.25)
        self.last_send = 0.0
        self.last_signature = None
        self._gripper_speed_sent = None
        self.command_time = 0.0
        self.moving = True
        self.last_health = 0.0
        self.recovery_ready = False
        self._recovery_samples = 0
        self._recovery_origin = None
        self._last_recovery_read = -1
        self._interpolation_ready = False
        self._handoff_pending = False

    def feedback(self):
        with self.lock:
            if ((self.error and not self.recovery_ready) or self.pose is None or
                    self.clock() - self.received > 0.5 or
                    (hasattr(self.read_pose, 'fresh') and not self.read_pose.fresh())):
                return None
            return list(self.pose), list(self.velocity)

    def arm(self):
        with self.lock:
            if ((self.error and not self.recovery_ready) or self.pose is None or
                    self.clock() - self.received > 0.5 or
                    (hasattr(self.read_pose, 'fresh') and not self.read_pose.fresh())
                    or any(abs(v) > 0.01 for v in self.velocity)
                    or self.moving or self.motion_sent or self.stop_pending is not None):
                return False
            # Only this explicit operator action clears the motion fault latch.
            self.error = None
            self.recovery_ready = False
            self.armed = True
            return True

    def submit(self, axis, endpoint, velocity, origin, validated_limit,
               position_goal=False):
        with self.lock:
            if not self.armed or self.error or self.stop_pending is not None:
                return False
            self.command = (self.generation, axis, endpoint, abs(velocity),
                            list(origin), validated_limit, bool(position_goal))
            self.command_time = self.clock()
            return True

    def stop(self, emergency=False, lock=False):
        with self.lock:
            self.generation += 1
            self.command = None
            self._handoff_pending = False
            self.stop_pending = bool(emergency) or bool(self.stop_pending)
            if lock:
                self.armed = False

    def _ensure_interpolation_mode(self):
        """jog_angle is rejected while the controller is in refresh mode."""
        if self._interpolation_ready:
            return
        mode = self.sdk.get_fresh_mode()
        if mode != 0:
            switched = self.sdk.set_fresh_mode(0)
            if isinstance(switched, str) or switched == 0:
                raise RuntimeError(
                    f"could not switch to interpolation mode for jog: {switched}")
        self._interpolation_ready = True

    def _brake_active_motion(self):
        """Stop an in-flight arm jog at the checked boundary. Keep the arm armed."""
        with self.lock:
            self.generation += 1
            self.command = None
            if self.motion_sent and self.motion_axis != 6 and self.stop_pending is None:
                self.stop_pending = False

    def _stop_active_motion(self, emergency):
        if self.motion_axis == 6:
            self._stop_gripper()
        else:
            self.sdk.stop(deceleration=0 if emergency else 1, _async=True)

    def _stop_gripper(self):
        """Original key-up behavior: read current opening and hold that angle."""
        if hasattr(self.read_pose, 'force_gripper_read'):
            self.read_pose.force_gripper_read()
        pose = self.read_pose()
        self.sdk.set_pro_gripper_angle(round(pose[6] * 100))

    def _acquire_pose(self):
        """During motion, only the gripper opening comes from the ROS cache."""
        if getattr(self.read_pose, 'supports_motion_cache', False):
            arm_moving = bool(self.motion_sent or self.moving) and self.motion_axis != 6 and not self.error
            pending_start = self.command is not None and not self.motion_sent
            if pending_start:
                self.read_pose.cached_gripper()
            return self.read_pose(moving=arm_moving or pending_start)
        return self.read_pose()

    def cycle(self):
        """Only the owner thread calls this; reads and writes cannot overlap."""
        with self.lock:
            if (self.motion_sent and self.command is not None and
                    self.clock() - self.command_time > 0.5):
                self.armed = False
                self.command = None
                self.stop_pending = True
            stop = self.stop_pending
            self.stop_pending = None
        if stop is not None and self.motion_sent:
            # Arm uses STOP; gripper restores the original measured-angle hold.
            self._stop_active_motion(stop)
            self.motion_sent = False
            self.last_signature = None
            self._gripper_speed_sent = None
        pose = self._acquire_pose()
        sample_time = getattr(self.read_pose, 'arm_time', None) or self.clock()
        now = self.clock()
        if now - self.last_health >= 1.0:
            if self.sdk.is_power_on() != 1 or self.sdk.get_error_information() not in (0, None):
                raise RuntimeError("hardware power/error check failed")
            self.last_health = now
        moving = self.sdk.is_moving()
        if moving not in (0, 1):
            raise RuntimeError("invalid hardware moving status")
        if len(pose) != 7 or not all(math.isfinite(p) and lo <= p <= hi
                                    for p, (lo, hi) in zip(pose, self.limits)):
            raise RuntimeError("invalid hardware feedback")
        velocities = self.estimator.update(sample_time, pose)
        now = self.clock()
        with self.lock:
            # Do not expose an unmeasured zero velocity during estimator warmup.
            if velocities is not None:
                self.pose, self.velocity, self.received = pose, velocities, sample_time
                self.moving = bool(moving)
                if self.error:
                    read_id = getattr(self.read_pose, 'success_count', sample_time)
                    if read_id != self._last_recovery_read:
                        self._last_recovery_read = read_id
                        stable = (not moving and not self.motion_sent and
                                  all(abs(v) <= 0.01 for v in velocities))
                        if self._recovery_origin is not None:
                            stable = stable and all(
                                abs(a - b) <= (math.radians(0.2) if j < 6 else 0.005)
                                for j, (a, b) in enumerate(zip(pose, self._recovery_origin)))
                        if stable:
                            if self._recovery_origin is None:
                                self._recovery_origin = list(pose)
                            self._recovery_samples += 1
                        else:
                            self._recovery_samples = 0
                            self._recovery_origin = None
                        self.recovery_ready = self._recovery_samples >= 3
            # State is committed before callbacks use it for launch gating.
            command = self.command
        if velocities is not None:
            self.publish_pose(pose)
        if command is None or velocities is None:
            return
        if len(command) == 7:
            generation, axis, endpoint, velocity, origin, boundary, position_goal = command
        else:
            generation, axis, endpoint, velocity, origin, boundary = command
            position_goal = False
        direction = 1 if boundary > origin[axis] else -1
        # Additional communication/feedback travel reserve beyond the shared
        # mathematical profile. Firmware stop distance still needs calibration.
        # A position goal is the firmware's own stop; do not brake ahead of it.
        reserve = velocity * 0.5 + velocity * velocity / (2 * 1.4) + 0.01
        if (not position_goal and axis != 6 and
                direction * (boundary - pose[axis]) <= reserve):
            self._brake_active_motion()
            return
        if position_goal and axis != 6:
            # The scanned stop is the same angle simulation tracks. Stop a jog
            # before sending it, and stop again if the arm runs past it.
            if direction * (pose[axis] - endpoint) > 0.0:
                if self.motion_sent or self.last_signature is not None or self._handoff_pending:
                    self.sdk.stop(deceleration=1, _async=True)
                self.motion_sent = False
                self.last_signature = None
                self._handoff_pending = False
                return
            was_jogging = (isinstance(self.last_signature, tuple) and
                           self.last_signature and self.last_signature[0] == 'jog')
            if was_jogging or self._handoff_pending:
                if not self._handoff_pending:
                    self.sdk.stop(deceleration=1, _async=True)
                    self._handoff_pending = True
                    self.last_signature = None
                still = self.sdk.is_moving()
                if still == 1:
                    return
                if still != 0:
                    raise RuntimeError("invalid hardware moving status")
                self._handoff_pending = False
        if now - self.last_send < 0.20:
            return
        speed = sdk_speed_for_rad(velocity)
        if speed == 0:
            return
        if axis == 6:
            # One opening command for the whole hold. A scanned goal uses that
            # opening; otherwise 100 follows a positive direction and 0 a negative one.
            if position_goal:
                opening = max(0, min(100, int(round(endpoint * 100))))
            else:
                opening = 100 if direction > 0 else 0
            signature = ('gripper', opening)
        elif position_goal:
            signature = ('angle', axis + 1, round(math.degrees(endpoint), 2), speed)
        else:
            # SDK direction: 1 increases, 0 decreases. One jog runs until stop.
            signature = ('jog', axis + 1, 1 if direction > 0 else 0, speed)
        if signature != self.last_signature and hasattr(self.read_pose, 'cached_gripper'):
            try:
                self.read_pose.cached_gripper()
            except Exception as exc:
                self.report_error(f"Gripper cache unavailable before motion: {exc}")
                return
        with self.lock:
            if (generation != self.generation or not self.armed or self.error
                    or self.stop_pending is not None):
                return
            speed_changed = axis == 6 and self._gripper_speed_sent != speed
            if signature == self.last_signature and not speed_changed:
                return
            # Mark before calling: even an ambiguous timeout requires a STOP.
            self.motion_sent, self.motion_axis = True, axis
        if axis == 6:
            if speed_changed or self.last_signature != signature:
                if self.sdk.set_pro_gripper_speed(speed) != 1:
                    raise RuntimeError("gripper speed setting rejected")
                self._gripper_speed_sent = speed
            if self.last_signature == signature:
                self.last_send = now
                return
            with self.lock:
                if generation != self.generation or not self.armed:
                    return
            result = self.sdk.set_pro_gripper_angle(opening)
        elif signature[0] == 'angle':
            self._ensure_interpolation_mode()
            result = self.sdk.send_angle(signature[1], signature[2], speed, _async=True)
            if isinstance(result, str) or result == 0:
                result = self.sdk.send_angle(signature[1], signature[2], speed, _async=True)
        else:
            self._ensure_interpolation_mode()
            result = self.sdk.jog_angle(signature[1], signature[2], speed, _async=True)
        if isinstance(result, str) or result == 0:
            raise RuntimeError(f"SDK rejected motion: {result}")
        self.last_send, self.last_signature = now, signature

    def _latch_fault(self, exc):
        with self.lock:
            changed = self.error != str(exc)
            self.error, self.armed, self.command = str(exc), False, None
            self.generation += 1
            self.recovery_ready = False
            self._recovery_samples = 0
            self._recovery_origin = None
            # A pre-fault cached sample cannot count as successful recovery.
            self._last_recovery_read = getattr(self.read_pose, 'success_count', -1)
            self._handoff_pending = False
        if changed:
            self.report_error(f"Real feedback/control failed; motion locked: {exc}")

    def run(self):
        while not self.closed.is_set():
            try:
                self.cycle()
            except Exception as exc:
                self._latch_fault(exc)
                # A failed read/send may have left a finite goal active.
                if self.motion_sent:
                    try:
                        self._stop_active_motion(True)
                        self.motion_sent = False
                    except Exception:
                        pass  # Keep latch; operator must use hardware E-stop.
            self.closed.wait(0.02)

    def close(self):
        self.stop(emergency=True, lock=True)
        # Owner must service STOP before being asked to exit.
        deadline = self.clock() + 2.0
        while (self.stop_pending is not None or self.motion_sent) and self.clock() < deadline:
            time.sleep(0.02)
        self.closed.set()
