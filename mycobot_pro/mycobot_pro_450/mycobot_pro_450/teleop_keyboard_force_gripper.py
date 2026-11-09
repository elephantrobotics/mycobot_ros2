# -*- coding: utf-8 -*-
"""Compatibility entry for force-gripper keyboard teleop.

Uses the unified teleop_keyboard implementation. When the driver parameter
cannot be read, defaults local gripper profile to f100 so g/h stay available.
Prefer:

    ros2 launch mycobot_pro_450 teleop_keyboard.launch.py urdf:=v2 gripper:=f100
    ros2 run mycobot_pro_450 teleop_keyboard
"""

from mycobot_pro_450.teleop_keyboard import main as teleop_main


def main(args=None):
    teleop_main(args=args, gripper_local_default="f100")


if __name__ == "__main__":
    main()
