#! /usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from mycobot_interfaces.srv import (
    SetAngles,
    GetAngles,
    SetCoords,
    GetCoords,
    GripperStatus,
    PumpStatus,
)

from pymycobot.mycobot import MyCobot


class Mycobot_Service(Node):
    def __init__(self):
        super().__init__("mycobot_services")
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', '115200')
        self.get_logger().info("start ...")

        port = self.get_parameter("port").get_parameter_value().string_value
        baud = self.get_parameter("baud").get_parameter_value().integer_value
        self.get_logger().info("%s,%d" % (port, baud))

        self.mc = MyCobot(port, str(baud))

    def create_services(self):
        self.set_joint_angles_service = self.create_service(
            srv_type=SetAngles,
            srv_name="set_joint_angles",
            callback=self.set_angles
        )

        self.get_joint_angles_service = self.create_service(
            srv_type=GetAngles,
            srv_name="get_joint_angles",
            callback=self.get_angles
        )

        self.set_joint_coords_service = self.create_service(
            srv_type=SetCoords,
            srv_name="set_joint_coords",
            callback=self.set_coords
        )

        self.get_joint_coords_service = self.create_service(
            srv_type=GetCoords,
            srv_name="get_joint_coords",
            callback=self.get_coords
        )

        self.switch_gripper_status_service = self.create_service(
            srv_type=GripperStatus,
            srv_name="switch_gripper_status",
            callback=self.switch_status
        )

        self.switch_pump_status_service = self.create_service(
            srv_type=PumpStatus,
            srv_name="switch_pump_status",
            callback=self.toggle_pump
        )

        self.get_logger().info("ready")

    def set_angles(self, req):
        angles = [
            req.joint_1,
            req.joint_2,
            req.joint_3,
            req.joint_4,
            req.joint_5,
            req.joint_6,
        ]
        sp = req.speed

        if self.mc:
            self.mc.send_angles(angles, sp)

        self.get_logger().info("set_angles, over")
        return True

    def get_angles(self, req):
        if self.mc:
            angles = self.mc.get_angles()
            self.get_logger().info("get_angles, over")
            return list(angles)

    def set_coords(self, req):
        coords = [
            req.x,
            req.y,
            req.z,
            req.rx,
            req.ry,
            req.rz,
        ]
        sp = req.speed
        mod = req.model

        if self.mc:
            self.mc.send_coords(coords, sp, mod)

        self.get_logger().info("set_coords, over")
        return True

    def get_coords(self, req):
        if self.mc:
            coords = self.mc.get_coords()
            self.get_logger().info("get_coords, over")
            return list(coords)

    def switch_status(self, req):
        if self.mc:
            if req.status:
                self.mc.set_gripper_state(0, 80)
            else:
                self.mc.set_gripper_state(1, 80)

        self.get_logger().info("switch_status, over")
        return True

    def toggle_pump(self, req):
        if self.mc:
            if req.status:
                self.mc.set_basic_output(req.pin1, 0)
                self.mc.set_basic_output(req.pin2, 0)
            else:
                self.mc.set_basic_output(req.pin1, 1)
                self.mc.set_basic_output(req.pin2, 1)

        self.get_logger().info("toggle_pump, over")
        return True

    def output_robot_message(self):
        robot_msg = """
        MyCobot Status
        --------------------------------
        Joint Limit:
            joint 1: -170 ~ +170
            joint 2: -170 ~ +170
            joint 3: -170 ~ +170
            joint 4: -170 ~ +170
            joint 5: -170 ~ +170
            joint 6: -180 ~ +180
        Connect Status: %s
        Servo Infomation: %s
        Servo Temperature: %s
        Atom Version: %s
        """
        connect_status = False
        servo_infomation = "unknown"
        servo_temperature = "unknown"
        atom_version = "unknown"

        if self.mc:
            cn = self.mc.is_controller_connected()
            if cn == 1:
                connect_status = True
            time.sleep(0.1)
            si = self.mc.is_all_servo_enable()
            if si == 1:
                servo_infomation = "all connected"

        self.get_logger().info(
            robot_msg % (connect_status, servo_infomation,
                         servo_temperature, atom_version)
        )


def main(args=None):
    # print(MyCobot.__dict__)
    rclpy.init(args=args)
    mycobot_services = Mycobot_Service()
    mycobot_services.output_robot_message()
    mycobot_services.create_services()
    rclpy.spin(mycobot_services)
    mycobot_services.get_logger().info("结束")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
