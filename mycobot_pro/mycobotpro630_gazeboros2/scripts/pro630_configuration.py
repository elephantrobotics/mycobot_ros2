"""One installation profile shared by model generation and control processes."""
import copy
import math
import xml.etree.ElementTree as ET

from pro630_model import ARM_JOINTS, GRIPPER_JOINT, LIMITS_DEG, JOINT_LIMITS_RAD


def require_flag(value):
    if type(value) is not bool:
        raise ValueError('Missing or invalid with_gripper capability; restart the matching public environment')
    return value


def joints(with_gripper):
    return ARM_JOINTS + ([GRIPPER_JOINT] if require_flag(with_gripper) else [])


def controllers(with_gripper):
    return ['arm_controller', 'joint_state_broadcaster'] + (
        ['pro_gripper_controller'] if require_flag(with_gripper) else [])


def geometry_xml(path, with_gripper=True):
    """Keep offline geometry tools compatible with the conditional URDF source."""
    root = ET.parse(path).getroot()
    for parent in list(root.iter()):
        for child in list(parent):
            if child.tag == '{http://www.ros.org/wiki/xacro}if' and child.get('value') == '$(arg with_gripper)':
                index = list(parent).index(child)
                parent.remove(child)
                if require_flag(with_gripper):
                    for item in list(child):
                        parent.insert(index, item)
                        index += 1
    return root


def configure_node(node):
    node.declare_parameter('with_gripper', True)
    node.with_gripper = require_flag(node.get_parameter('with_gripper').value)
    node.command_joints = joints(node.with_gripper)
    node.joint_limits = JOINT_LIMITS_RAD[:len(node.command_joints)]


def controller_config(source, with_gripper):
    result = copy.deepcopy(source)
    if not require_flag(with_gripper):
        result['controller_manager']['ros__parameters'].pop('pro_gripper_controller', None)
        result.pop('pro_gripper_controller', None)
        result['joint_state_broadcaster']['ros__parameters']['joints'] = list(ARM_JOINTS)
    return result


def project_semantic(text, with_gripper):
    if require_flag(with_gripper):
        return text
    root = ET.fromstring(text)
    links = {'world', 'base', *[f'link{i}' for i in range(1, 7)]}
    names = {'world_to_base', *ARM_JOINTS}
    for child in list(root):
        if (child.tag == 'group' and child.get('name') == 'gripper' or
                child.tag == 'group_state' and child.get('group') == 'gripper' or
                child.tag == 'end_effector' or
                child.tag == 'passive_joint' and child.get('name') not in names or
                child.tag == 'disable_collisions' and
                (child.get('link1') not in links or child.get('link2') not in links)):
            root.remove(child)
        elif child.tag == 'group' and child.get('name') == 'arm':
            for item in list(child):
                if item.tag == 'joint' and item.get('name') not in names:
                    child.remove(item)
                elif item.tag == 'chain':
                    item.set('tip_link', 'link6')
    return ET.tostring(root, encoding='unicode')


def project_moveit(config, with_gripper):
    config.robot_description_semantic['robot_description_semantic'] = project_semantic(
        config.robot_description_semantic['robot_description_semantic'], with_gripper)
    if not with_gripper:
        config.joint_limits['robot_description_planning']['joint_limits'].pop(GRIPPER_JOINT, None)
        manager = config.trajectory_execution.get('moveit_simple_controller_manager', {})
        manager['controller_names'] = [n for n in manager.get('controller_names', [])
                                       if n != 'pro_gripper_controller']
        manager.pop('pro_gripper_controller', None)
        config.robot_description_kinematics.get('robot_description_kinematics', {}).pop('gripper', None)
    return config


def public_profile(timeout=2.0):
    """Auxiliary visualizers inherit a live environment; standalone tools default to gripper."""
    import json
    import time
    import rclpy
    from rclpy.context import Context
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from std_msgs.msg import String
    context = Context()
    rclpy.init(context=context)
    node = Node('pro630_auxiliary_profile', context=context)
    result = []
    def received(msg):
        result.append(json.loads(msg.data))
    node.create_subscription(String, '/pro630/environment_state', received,
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    try:
        deadline = time.monotonic() + timeout
        while not result and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        return require_flag(result[-1].get('with_gripper')) if result else True
    finally:
        node.destroy_node()
        context.try_shutdown()
