"""Pro630 model coordinates; all feature modules share these limits.

The old Pro630 bridge maps hardware J2/J4 to URDF by adding 90 degrees.
Keep that conversion at the hardware adapter boundary only.
"""
import math

ARM_JOINTS = ['joint1_to_base', 'joint2_to_joint1', 'joint3_to_joint2', 'joint4_to_joint3', 'joint5_to_joint4', 'joint6_to_joint5']
GRIPPER_JOINT = 'gripper_controller'
COMMAND_JOINTS = ARM_JOINTS + [GRIPPER_JOINT]
JOINT_LIMITS_RAD = [(-3.14, 3.14159), (-3.14159, 3.14159), (-2.61, 2.618), (-2.967, 2.967), (-2.93, 2.9321), (-3.03, 3.0368)] + [(0.0, 1.0)]
LIMITS_DEG = [(math.degrees(lo), math.degrees(hi)) for lo, hi in JOINT_LIMITS_RAD[:6]]
MAX_VELOCITY_RAD_S = [3.14] * 6 + [1.0]
HARDWARE_OFFSETS_DEG = (0.0, 90.0, 0.0, 90.0, 0.0, 0.0)
DEFAULT_PRO630_IP = '192.168.1.159'
DEFAULT_PRO630_PORT = 5001
GRIPPER_ID = 14
# Preserve the old project's 0..1000 arm command scale. The SDK documents
# no angular speed conversion; this is nominal scaling, not measured deg/s.
ARM_SDK_SPEED_MAX = 1000
