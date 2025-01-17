#! /usr/bin/env python3
import time
import os
import sys
import signal
import threading

import rclpy
from rclpy.node import Node

from mycobot_interfaces.msg import (
    MycobotAngles,
    MycobotCoords,
    MycobotSetAngles,
    MycobotSetCoords,
    MycobotGripperStatus,
    MycobotPumpStatus,
)

import pymycobot
from packaging import version

# min low version require
MAX_REQUIRE_VERSION = '3.5.3'

current_verison = pymycobot.__version__
print('current pymycobot library version: {}'.format(current_verison))
if version.parse(current_verison) > version.parse(MAX_REQUIRE_VERSION):
    raise RuntimeError('The version of pymycobot library must be less than {} . The current version is {}. Please downgrade the library version.'.format(MAX_REQUIRE_VERSION, current_verison))
else:
    print('pymycobot library version meets the requirements!')
    from pymycobot.mycobot import MyCobot


class Watcher:
    """this class solves two problems with multithreaded
    programs in Python, (1) a signal might be delivered
    to any thread (which is just a malfeature) and (2) if
    the thread that gets the signal is waiting, the signal
    is ignored (which is a bug).
    The watcher is a concurrent process (not thread) that
    waits for a signal and the process that contains the
    threads.  See Appendix A of The Little Book of Semaphores.
    http://greenteapress.com/semaphores/
    I have only tested this on Linux.  I would expect it to
    work on the Macintosh and not work on Windows.
    """

    def __init__(self):
        """Creates a child thread, which returns.  The parent
        thread waits for a KeyboardInterrupt and then kills
        the child thread.
        """
        self.child = os.fork()
        if self.child == 0:
            return
        else:
            self.watch()

    def watch(self):
        try:
            os.wait()
        except KeyboardInterrupt:
            # I put the capital B in KeyBoardInterrupt so I can
            # tell when the Watcher gets the SIGINT
            print("KeyBoardInterrupt")
            self.kill()
        sys.exit()

    def kill(self):
        try:
            os.kill(self.child, signal.SIGKILL)
        except OSError:
            pass


class MycobotTopics(Node):
    def __init__(self):
        super().__init__("mycobot_topics")

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', '115200')
        self.get_logger().info("start ...")

        port = self.get_parameter("port").get_parameter_value().string_value
        baud = self.get_parameter("baud").get_parameter_value().integer_value

        self.get_logger().info("%s,%d" % (port, baud))
        self.mc = MyCobot(port, str(baud))
        self.lock = threading.Lock()

    def start(self):
        pa = threading.Thread(target=self.pub_real_angles)
        pb = threading.Thread(target=self.pub_real_coords)
        sa = threading.Thread(target=self.sub_set_angles)
        sb = threading.Thread(target=self.sub_set_coords)
        sg = threading.Thread(target=self.sub_gripper_status)
        sp = threading.Thread(target=self.sub_pump_status)

        pa.setDaemon(True)
        pa.start()
        pb.setDaemon(True)
        pb.start()
        sa.setDaemon(True)
        sa.start()
        sb.setDaemon(True)
        sb.start()
        sg.setDaemon(True)
        sg.start()
        sp.setDaemon(True)
        sp.start()

        pa.join()
        pb.join()
        sa.join()
        sb.join()
        sg.join()
        sp.join()

    def pub_real_angles(self):
        pub = self.create_publisher(
            topic="mycobot/angles_real",
            msg_type=MycobotAngles,
            qos_profile=5
        )
        ma = MycobotAngles()
        while rclpy.ok():
            rclpy.spin_once(self)
            self.lock.acquire()
            angles = self.mc.get_angles()
            self.lock.release()

            if angles:
                ma.joint_1 = angles[0]
                ma.joint_2 = angles[1]
                ma.joint_3 = angles[2]
                ma.joint_4 = angles[3]
                ma.joint_5 = angles[4]
                ma.joint_6 = angles[5]
                pub.publish(ma)
            time.sleep(0.25)

    def pub_real_coords(self):
        pub = self.create_publisher(
            topic="mycobot/coords_real",
            msg_type=MycobotCoords,
            qos_profile=5
        )
        ma = MycobotCoords()

        while rclpy.ok():
            rclpy.spin_once(self)
            self.lock.acquire()
            coords = self.mc.get_coords()
            self.lock.release()
            if coords:
                ma.x = coords[0]
                ma.y = coords[1]
                ma.z = coords[2]
                ma.rx = coords[3]
                ma.ry = coords[4]
                ma.rz = coords[5]
                pub.publish(ma)
            time.sleep(0.25)

    def sub_set_angles(self):
        def callback(data):
            angles = [
                data.joint_1,
                data.joint_2,
                data.joint_3,
                data.joint_4,
                data.joint_5,
                data.joint_6,
            ]
            sp = int(data.speed)
            self.mc.send_angles(angles, sp)

        sub = self.create_subscription(
            topic="mycobot/angles_goal",
            msg_type=MycobotSetAngles,
            callback=callback,
            qos_profile=1
        )
        rclpy.spin(sub)

    def sub_set_coords(self):
        def callback(data):
            angles = [data.x, data.y, data.z, data.rx, data.ry, data.rz]
            sp = int(data.speed)
            model = int(data.model)
            self.mc.send_coords(angles, sp, model)

        sub = self.create_subscription(
            topic="mycobot/coords_goal",
            msg_type=MycobotSetCoords,
            callback=callback,
            qos_profile=1
        )
        rclpy.spin(sub)

    def sub_gripper_status(self):
        def callback(data):
            if data.status:
                self.mc.set_gripper_state(0, 80)
            else:
                self.mc.set_gripper_state(1, 80)

        sub = self.create_subscription(
            topic="mycobot/gripper_status",
            msg_type=MycobotGripperStatus,
            callback=callback,
            qos_profile=1
        )
        rclpy.spin(sub)

    def sub_pump_status(self):
        def callback(data):
            if data.Status:
                self.mc.set_basic_output(data.pin1, 0)
                self.mc.set_basic_output(data.pin2, 0)
            else:
                self.mc.set_basic_output(data.pin1, 1)
                self.mc.set_basic_output(data.pin2, 1)

        sub = self.create_subscription(
            topic="mycobot/pump_status",
            msg_type=MycobotPumpStatus,
            callback=callback,
            qos_profile=1
        )
        rclpy.spin(sub)


def main(args=None):
    rclpy.init(args=args)
    Watcher()
    mc_topics = MycobotTopics()
    rclpy.spin(mc_topics)

    mc_topics.start()
    # while True:
    #     mc_topics.pub_real_coords()
    # mc_topics.sub_set_angles()

    mc_topics.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()