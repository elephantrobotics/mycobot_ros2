#!/usr/bin/env python3

"""Deterministically probe the MoveIt collision matrix with random joint states."""

import argparse
import random

import rclpy
from moveit_msgs.srv import GetStateValidity
from rclpy.node import Node


JOINTS = [
    "joint1",
    "joint2",
    "joint3",
    "joint4",
    "joint5",
    "joint6",
    "gripper_controller",
]
LIMITS = [
    (-2.9496, 2.9496),
    (-2.2689, 2.2689),
    (-2.6878, 2.6878),
    (-2.8274, 2.8274),
    (-2.8274, 2.8274),
    (-2.8797, 2.8797),
    (0.0, 1.0),
]


def pair(contact):
    return frozenset((contact.contact_body_1, contact.contact_body_2))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--samples", type=int, default=3000)
    parser.add_argument("--seed", type=int, default=450)
    args = parser.parse_args()

    rclpy.init()
    node = Node("pro450_collision_probe")
    client = node.create_client(GetStateValidity, "/check_state_validity")
    if not client.wait_for_service(timeout_sec=10.0):
        raise SystemExit("/check_state_validity is unavailable")

    rng = random.Random(args.seed)
    targets = {
        "link2/link5": frozenset(("link2", "link5")),
        "link2/gripper": None,
    }
    found = {}
    invalid = 0
    for index in range(args.samples):
        positions = [rng.uniform(low, high) for low, high in LIMITS]
        request = GetStateValidity.Request()
        request.group_name = "arm"
        request.robot_state.joint_state.name = JOINTS
        request.robot_state.joint_state.position = positions
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
        response = future.result()
        if response is None:
            raise SystemExit(f"state validity request {index} failed")
        if response.valid:
            continue
        invalid += 1
        pairs = {pair(contact) for contact in response.contacts}
        if targets["link2/link5"] in pairs and "link2/link5" not in found:
            found["link2/link5"] = positions
        if "link2/gripper" not in found:
            for bodies in pairs:
                if "link2" in bodies and any(name.startswith("gripper_") for name in bodies):
                    found["link2/gripper"] = positions
                    break
        if len(found) == len(targets):
            break

    print(f"samples={index + 1} invalid={invalid}")
    for name in targets:
        print(f"{name}: {found.get(name, 'NOT FOUND')}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
