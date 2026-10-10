"""Shared measured-feedback publication for the slider and keyboard bridges."""
import time

from builtin_interfaces.msg import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


def attach_feedback(node, reader):
    def received(sample):
        pose = reader.observe_arm(sample.angles, sample.monotonic, sample.wall_time)
        if pose is not None:
            node._on_real_sample(pose)
    node.mc.set_angle_callback(received)


def publish_feedback(node, pose, reader, joints):
    """Use measurement time, deduplicate samples, and never overwrite newer data."""
    with node._sample_publish_lock:
        now = time.monotonic()
        with reader.cache.lock:
            measured = reader.arm_time if reader.arm_time is not None else now
            wall = reader.arm_wall_time if reader.arm_wall_time is not None else time.time()
            if reader.cache.ready():
                pose = reader.cache.pose()
        if now - measured > 0.5:
            return
        previous = node._real_mirror.latest()
        if previous is not None and measured <= previous[0]:
            return
        velocity = [0.0] * 7 if previous is None else [
            (value - old) / max(0.001, measured - previous[0])
            for value, old in zip(pose, previous[1])]
        node._real_mirror.push(pose, when=measured)
        node._sample_velocity = velocity
        msg = JointState()
        seconds = int(wall)
        msg.header.stamp = Time(sec=seconds, nanosec=int((wall - seconds) * 1e9))
        msg.name, msg.position, msg.velocity = list(joints), list(pose), velocity
        node.real_joint_pub.publish(msg)
        publish_age(node, reader)


def publish_age(node, reader):
    now = time.monotonic()
    arm = reader.arm_time
    gripper = reader.gripper_time
    msg = Float64MultiArray()
    msg.data = [max(0.0, now - arm) if arm is not None else -1.0,
                max(0.0, now - gripper) if gripper is not None else -1.0]
    node.real_age_pub.publish(msg)
