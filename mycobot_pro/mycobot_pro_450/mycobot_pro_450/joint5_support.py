# -*- coding: utf-8 -*-
"""Shared Pro 450 J5 offset / firmware-limit helpers for ROS2.

Logical (URDF/ROS) J5 and firmware J5 differ by ``joint5_offset_deg`` on v2:
``firmware = logical - offset``, ``logical = firmware + offset``.
"""

from __future__ import annotations

from typing import Optional, Tuple

EXPECTED_LIMITS_BY_OFFSET = {
    0.0: (-162.0, 162.0),
    90.0: (-252.0, 72.0),
}


def expected_firmware_limits(configured_offset: float) -> Optional[Tuple[float, float]]:
    """Return expected J5 firmware limits for a configured offset, or None."""
    for offset, limits in EXPECTED_LIMITS_BY_OFFSET.items():
        if abs(configured_offset - offset) < 0.01:
            return limits
    return None


def read_joint5_firmware_limits(client, logger=None):
    """Read J5 software limits from the connected controller.

    Returns:
        (lower, upper) in degrees, or None on failure.
    """
    try:
        lower = client.get_joint_min_angle(5)
        upper = client.get_joint_max_angle(5)
    except Exception as exc:
        if logger is not None:
            logger.error("Unable to read J5 firmware limits: %s" % exc)
        return None
    if (
        not isinstance(lower, (int, float))
        or not isinstance(upper, (int, float))
        or lower >= upper
    ):
        if logger is not None:
            logger.error(
                "Invalid J5 firmware limits returned by controller: %r, %r"
                % (lower, upper)
            )
        return None
    if logger is not None:
        logger.info("J5 firmware limits: [%.2f, %.2f] degrees" % (lower, upper))
    return float(lower), float(upper)


def resolve_joint5_offset(limits, configured_offset, logger=None):
    """Require controller limits to match the selected URDF profile.

    Returns:
        configured_offset if matched, otherwise None (profile incompatible).
    """
    if limits is None:
        return None
    expected = expected_firmware_limits(configured_offset)
    if expected is None:
        if logger is not None:
            logger.error(
                "Unsupported J5 offset configured: %.2f degrees" % configured_offset
            )
        return None
    lower, upper = limits
    expected_lower, expected_upper = expected
    if abs(lower - expected_lower) < 0.01 and abs(upper - expected_upper) < 0.01:
        return float(configured_offset)
    if logger is not None:
        logger.error(
            "J5 firmware limits [%.2f, %.2f] do not match the selected URDF "
            "profile; expected [%.2f, %.2f]"
            % (lower, upper, expected_lower, expected_upper)
        )
    return None


def logical_to_firmware_j5(logical_deg: float, offset_deg: float) -> float:
    return logical_deg - offset_deg


def firmware_to_logical_j5(firmware_deg: float, offset_deg: float) -> float:
    return firmware_deg + offset_deg


def angles_logical_to_firmware(angles_deg, offset_deg):
    """Copy a 6-length angle list and convert index 4 (J5) to firmware."""
    converted = list(angles_deg)
    converted[4] = logical_to_firmware_j5(converted[4], offset_deg)
    return converted


def angles_firmware_to_logical(angles_deg, offset_deg):
    """Copy a 6-length angle list and convert index 4 (J5) to logical."""
    converted = list(angles_deg)
    converted[4] = firmware_to_logical_j5(converted[4], offset_deg)
    return converted


class Joint5ProfileGuard:
    """Stateful helper used by control nodes after connecting to the arm."""

    def __init__(self, logger=None):
        self.logger = logger
        self.configured_offset = 0.0
        self.joint5_offset_deg = 0.0  # None means profile rejected
        self.firmware_limits = None
        self._logged_profile_refuse = False
        self._logged_limits_unavailable = False
        self._last_rejected_joint5 = None

    def configure(self, client, configured_offset: float):
        """Read limits immediately after connect and validate profile."""
        self.configured_offset = float(configured_offset)
        self.firmware_limits = read_joint5_firmware_limits(client, self.logger)
        if self.firmware_limits is None:
            self.joint5_offset_deg = None
            if self.logger is not None:
                self.logger.error(
                    "Refusing motion/publish because J5 firmware limits are unavailable"
                )
            return False
        resolved = resolve_joint5_offset(
            self.firmware_limits, self.configured_offset, self.logger
        )
        self.joint5_offset_deg = resolved
        if resolved is None:
            return False
        if self.logger is not None:
            self.logger.info(
                "J5 profile OK: offset=%.2f, firmware limits=[%.2f, %.2f]"
                % (resolved, self.firmware_limits[0], self.firmware_limits[1])
            )
        return True

    def is_ready(self) -> bool:
        return self.joint5_offset_deg is not None and self.firmware_limits is not None

    def refuse_reason(self) -> Optional[str]:
        if self.joint5_offset_deg is None:
            return "profile"
        if self.firmware_limits is None:
            return "limits"
        return None

    def log_refuse_once(self, reason: str):
        if self.logger is None:
            return
        if reason == "profile" and not self._logged_profile_refuse:
            self.logger.error(
                "Refusing J5 command because the selected URDF profile is "
                "incompatible with firmware limits"
            )
            self._logged_profile_refuse = True
        elif reason == "limits" and not self._logged_limits_unavailable:
            self.logger.error(
                "Refusing J5 command because firmware limits are unavailable"
            )
            self._logged_limits_unavailable = True

    def validate_firmware_j5(self, firmware_j5_deg: float) -> bool:
        """Return True if firmware J5 is within limits; log on change."""
        if not self.is_ready():
            self.log_refuse_once(self.refuse_reason() or "limits")
            return False
        lower, upper = self.firmware_limits
        if lower <= firmware_j5_deg <= upper:
            self._last_rejected_joint5 = None
            return True
        if self._last_rejected_joint5 != firmware_j5_deg and self.logger is not None:
            self.logger.error(
                "Refusing J5 command %.2f degrees outside firmware limits [%.2f, %.2f]"
                % (firmware_j5_deg, lower, upper)
            )
            self._last_rejected_joint5 = firmware_j5_deg
        return False

    def convert_logical_angles_to_firmware(self, logical_angles_deg):
        if not self.is_ready():
            self.log_refuse_once(self.refuse_reason() or "profile")
            return None
        firmware = angles_logical_to_firmware(
            logical_angles_deg, self.joint5_offset_deg
        )
        if not self.validate_firmware_j5(firmware[4]):
            return None
        return firmware

    def convert_firmware_angles_to_logical(self, firmware_angles_deg):
        if not self.is_ready():
            self.log_refuse_once(self.refuse_reason() or "profile")
            return None
        return angles_firmware_to_logical(
            firmware_angles_deg, self.joint5_offset_deg
        )


def load_robot_limit_profile(configured_offset: float):
    """Return (profile_name, limits_dict) from pymycobot RobotLimit."""
    from pymycobot.robot_info import RobotLimit

    profiles = RobotLimit.robot_limit
    if abs(configured_offset - 90.0) < 0.01:
        profile = "Pro450ClientNew"
        if profile not in profiles:
            raise RuntimeError(
                "Current pymycobot version does not support Pro450ClientNew. "
                "Please upgrade pymycobot to version 4.0.8 or later."
            )
    else:
        profile = "Pro450Client"
    try:
        return profile, profiles[profile]
    except (AttributeError, KeyError, TypeError) as exc:
        raise RuntimeError(
            "Unable to load limits for {} from pymycobot: {}".format(profile, exc)
        ) from exc


# Launch files rename the driver to listen_real_service; the class default
# name is mycobot_driver_node. Try both for ros2-run clients.
DEFAULT_DRIVER_NODE_CANDIDATES = (
    "listen_real_service",
    "mycobot_driver_node",
)


def _fetch_parameter_from_driver(
    node,
    driver_node_name: str,
    parameter_name: str,
    timeout_sec: float = 2.0,
):
    """Read one parameter from an already-running driver node.

    Returns:
        Decoded Python value, or None if unavailable.
    """
    import rclpy
    from rcl_interfaces.msg import ParameterType
    from rcl_interfaces.srv import GetParameters

    service_name = "/{}/get_parameters".format(driver_node_name.lstrip("/"))
    client = node.create_client(GetParameters, service_name)
    if not client.wait_for_service(timeout_sec=timeout_sec):
        return None

    request = GetParameters.Request()
    request.names = [parameter_name]
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
    if not future.done() or future.result() is None:
        return None

    values = future.result().values
    if not values:
        return None
    value = values[0]
    if value.type == ParameterType.PARAMETER_NOT_SET:
        return None
    if value.type == ParameterType.PARAMETER_DOUBLE:
        return float(value.double_value)
    if value.type == ParameterType.PARAMETER_INTEGER:
        return float(value.integer_value)
    if value.type == ParameterType.PARAMETER_STRING:
        return str(value.string_value)
    if value.type == ParameterType.PARAMETER_BOOL:
        return bool(value.bool_value)
    return None


def fetch_joint5_offset_from_driver(
    node,
    driver_node_name: str,
    timeout_sec: float = 2.0,
):
    """Read ``joint5_offset_deg`` from one already-running driver node."""
    value = _fetch_parameter_from_driver(
        node, driver_node_name, "joint5_offset_deg", timeout_sec=timeout_sec
    )
    if value is None:
        return None
    return float(value)


def resolve_joint5_offset_for_client_node(
    node,
    driver_node_names=DEFAULT_DRIVER_NODE_CANDIDATES,
    timeout_sec: float = 2.0,
) -> float:
    """Resolve J5 offset for a client started outside the description launch.

    Preference order (ROS1-like):
    1. ``joint5_offset_deg`` on the running driver (set by launch from urdf:=)
    2. Local node parameter (``-p joint5_offset_deg:=...`` or default 0.0)
    """
    node.declare_parameter("joint5_offset_deg", 0.0)
    for driver_node_name in driver_node_names:
        remote = fetch_joint5_offset_from_driver(
            node, driver_node_name=driver_node_name, timeout_sec=timeout_sec
        )
        if remote is not None:
            node.get_logger().info(
                "Using joint5_offset_deg={} from /{} (launch URDF profile)".format(
                    remote, driver_node_name
                )
            )
            return float(remote)

    local = node.get_parameter("joint5_offset_deg").get_parameter_value().double_value
    node.get_logger().warn(
        "Could not read joint5_offset_deg from {}; using local value {}. "
        "Start teleop_keyboard.launch.py (or simple_gui.launch.py) first.".format(
            list(driver_node_names), local
        )
    )
    return float(local)


def resolve_gripper_profile_for_client_node(
    node,
    driver_node_names=DEFAULT_DRIVER_NODE_CANDIDATES,
    timeout_sec: float = 2.0,
    local_default: str = "none",
) -> str:
    """Resolve gripper profile for a separately started client node.

    Preference order:
    1. ``gripper`` on the running driver (set by launch from gripper:=)
    2. Local node parameter (default ``none``, or ``f100`` for force-gripper entry)
    """
    node.declare_parameter("gripper", local_default)
    for driver_node_name in driver_node_names:
        remote = _fetch_parameter_from_driver(
            node, driver_node_name, "gripper", timeout_sec=timeout_sec
        )
        if remote is None:
            continue
        profile = str(remote).strip().lower()
        if profile in ("none", "f100"):
            node.get_logger().info(
                "Using gripper={} from /{} (launch gripper profile)".format(
                    profile, driver_node_name
                )
            )
            return profile
        node.get_logger().warn(
            "Ignoring invalid gripper profile '{}' from /{}".format(
                remote, driver_node_name
            )
        )

    local = node.get_parameter("gripper").get_parameter_value().string_value
    local = (local or local_default).strip().lower()
    if local not in ("none", "f100"):
        node.get_logger().warn(
            "Invalid local gripper='{}'; falling back to '{}'".format(
                local, local_default
            )
        )
        local = local_default
    node.get_logger().warn(
        "Could not read gripper from {}; using local value '{}'.".format(
            list(driver_node_names), local
        )
    )
    return local
