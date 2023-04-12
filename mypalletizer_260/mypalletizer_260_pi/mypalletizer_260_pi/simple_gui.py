#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import tkinter as tk
import time
# from pymycobot import MyCobotSocket
from pymycobot.mypalletizer import MyPalletizer
from mypalletizer_interfaces.srv import (
    SetAngles,
    GetAngles,
    SetCoords,
    GetCoords,
    GripperStatus,
    PumpStatus,
)
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

# class Window(Node): 
class Window(): 

    def __init__(self, handle):
        self.mc = MyPalletizer("/dev/ttyAMA0", 1000000)    
        self.win = handle
        self.win.resizable(0, 0)  # 固定窗口大小

        self.model = 0
        self.speed = 50

        # 设置默认速度123456
        self.speed_d = tk.StringVar()
        self.speed_d.set(str(self.speed))
        
        # self.connect_ser() # new add

        # 获取机械臂数据
        self.record_coords = [
            [0, 0, 0, 0],
            self.speed,
            self.model
        ]
        self.res_angles = [
            [0, 0, 0, 0],
            self.speed,
            self.model
        ]
        self.get_date()

        # get screen width and height
        self.ws = self.win.winfo_screenwidth()  # width of the screen
        self.hs = self.win.winfo_screenheight()  # height of the screen
        
        # calculate x and y coordinates for the Tk root window
        x = (self.ws / 2) - 190
        y = (self.hs / 2) - 250
        self.win.geometry("450x420+{}+{}".format(int(x), int(y)))
        # 布局
        self.set_layout()
        # 输入部分
        self.need_input()
        # 展示部分
        self.show_init()

        # joint 设置按钮
        tk.Button(self.frmLT, text="设置", width=5, command=self.get_joint_input).grid(
            row=6, column=1, sticky="w", padx=3, pady=2
        )

        # coordination 设置按钮
        tk.Button(self.frmRT, text="设置", width=5, command=self.get_coord_input).grid(
            row=6, column=1, sticky="w", padx=3, pady=2
        )

        # 夹爪开关按钮
        tk.Button(self.frmLB, text="夹爪(开)", command=self.gripper_open, width=5).grid(
            row=1, column=0, sticky="w", padx=3, pady=50
        )
        tk.Button(self.frmLB, text="夹爪(关)", command=self.gripper_close, width=5).grid(
            row=1, column=1, sticky="w", padx=3, pady=2
        )

    
    def set_layout(self):
        self.frmLT = tk.Frame(width=200, height=200)
        self.frmLC = tk.Frame(width=200, height=200)
        self.frmLB = tk.Frame(width=200, height=200)
        self.frmRT = tk.Frame(width=200, height=200)
        self.frmLT.grid(row=0, column=0, padx=1, pady=3)
        self.frmLC.grid(row=1, column=0, padx=1, pady=3)
        self.frmLB.grid(row=1, column=1, padx=2, pady=3)
        self.frmRT.grid(row=0, column=1, padx=2, pady=3)

    def need_input(self):
        # 输入提示
        tk.Label(self.frmLT, text="Joint 1 ").grid(row=0)
        tk.Label(self.frmLT, text="Joint 2 ").grid(row=1)  # 第二行
        tk.Label(self.frmLT, text="Joint 3 ").grid(row=2)
        tk.Label(self.frmLT, text="Joint 4 ").grid(row=3)

        tk.Label(self.frmRT, text=" x ").grid(row=0)
        tk.Label(self.frmRT, text=" y ").grid(row=1)  # 第二行
        tk.Label(self.frmRT, text=" z ").grid(row=2)
        tk.Label(self.frmRT, text=" rx ").grid(row=3)

        # 设置输入框的默认值
        self.j1_default = tk.StringVar()
        self.j1_default.set(self.res_angles[0][0])
        self.j2_default = tk.StringVar()
        self.j2_default.set(self.res_angles[0][1])
        self.j3_default = tk.StringVar()
        self.j3_default.set(self.res_angles[0][2])
        self.j4_default = tk.StringVar()
        self.j4_default.set(self.res_angles[0][3])

        self.x_default = tk.StringVar()
        self.x_default.set(self.record_coords[0][0])
        self.y_default = tk.StringVar()
        self.y_default.set(self.record_coords[0][1])
        self.z_default = tk.StringVar()
        self.z_default.set(self.record_coords[0][2])
        self.rx_default = tk.StringVar()
        self.rx_default.set(self.record_coords[0][3])

        # joint 输入框
        self.J_1 = tk.Entry(self.frmLT, textvariable=self.j1_default)
        self.J_1.grid(row=0, column=1, pady=3)
        self.J_2 = tk.Entry(self.frmLT, textvariable=self.j2_default)
        self.J_2.grid(row=1, column=1, pady=3)
        self.J_3 = tk.Entry(self.frmLT, textvariable=self.j3_default)
        self.J_3.grid(row=2, column=1, pady=3)
        self.J_4 = tk.Entry(self.frmLT, textvariable=self.j4_default)
        self.J_4.grid(row=3, column=1, pady=3)

        # coord 输入框
        self.x = tk.Entry(self.frmRT, textvariable=self.x_default)
        self.x.grid(row=0, column=1, pady=3, padx=0)
        self.y = tk.Entry(self.frmRT, textvariable=self.y_default)
        self.y.grid(row=1, column=1, pady=3)
        self.z = tk.Entry(self.frmRT, textvariable=self.z_default)
        self.z.grid(row=2, column=1, pady=3)
        self.rx = tk.Entry(self.frmRT, textvariable=self.rx_default)
        self.rx.grid(row=3, column=1, pady=3)

        # 所有输入框，用于拿输入的数据
        self.all_j = [self.J_1, self.J_2, self.J_3, self.J_4]
        self.all_c = [self.x, self.y, self.z, self.rx]

        # 速度输入框
        tk.Label(
            self.frmLB,
            text="speed",
        ).grid(row=0, column=0)
        self.get_speed = tk.Entry(self.frmLB, textvariable=self.speed_d, width=10)
        self.get_speed.grid(row=0, column=1)

    def show_init(self):
        # 显示
        tk.Label(self.frmLC, text="Joint 1 ").grid(row=0)
        tk.Label(self.frmLC, text="Joint 2 ").grid(row=1)  # 第二行
        tk.Label(self.frmLC, text="Joint 3 ").grid(row=2)
        tk.Label(self.frmLC, text="Joint 4 ").grid(row=3)

        # get数据

        # ，展示出来
        self.cont_1 = tk.StringVar(self.frmLC)
        self.cont_1.set(str(self.res_angles[0][0]) + "°")
        self.cont_2 = tk.StringVar(self.frmLC)
        self.cont_2.set(str(self.res_angles[0][1]) + "°")
        self.cont_3 = tk.StringVar(self.frmLC)
        self.cont_3.set(str(self.res_angles[0][2]) + "°")
        self.cont_4 = tk.StringVar(self.frmLC)
        self.cont_4.set(str(self.res_angles[0][3]) + "°")
        
        self.cont_all = [
            self.cont_1,
            self.cont_2,
            self.cont_3,
            self.cont_4,
            self.speed,
            self.model,
        ]

        self.show_j1 = tk.Label(
            self.frmLC,
            textvariable=self.cont_1,
            font=("Arial", 9),
            width=7,
            height=1,
            bg="white",
        ).grid(row=0, column=1, padx=0, pady=5)

        self.show_j2 = tk.Label(
            self.frmLC,
            textvariable=self.cont_2,
            font=("Arial", 9),
            width=7,
            height=1,
            bg="white",
        ).grid(row=1, column=1, padx=0, pady=5)
        self.show_j3 = tk.Label(
            self.frmLC,
            textvariable=self.cont_3,
            font=("Arial", 9),
            width=7,
            height=1,
            bg="white",
        ).grid(row=2, column=1, padx=0, pady=5)
        self.show_j4 = tk.Label(
            self.frmLC,
            textvariable=self.cont_4,
            font=("Arial", 9),
            width=7,
            height=1,
            bg="white",
        ).grid(row=3, column=1, padx=0, pady=5)

        self.all_jo = [
            self.show_j1,
            self.show_j2,
            self.show_j3,
            self.show_j4,
        ]

        # 显示
        tk.Label(self.frmLC, text="  x ").grid(row=0, column=3)
        tk.Label(self.frmLC, text="  y ").grid(row=1, column=3)  # 第二行
        tk.Label(self.frmLC, text="  z ").grid(row=2, column=3)
        tk.Label(self.frmLC, text="  rx ").grid(row=3, column=3)
        self.coord_x = tk.StringVar()
        self.coord_x.set(str(self.record_coords[0][0]))
        self.coord_y = tk.StringVar()
        self.coord_y.set(str(self.record_coords[0][1]))
        self.coord_z = tk.StringVar()
        self.coord_z.set(str(self.record_coords[0][2]))
        self.coord_rx = tk.StringVar()
        self.coord_rx.set(str(self.record_coords[0][3]))

        self.coord_all = [
            self.coord_x,
            self.coord_y,
            self.coord_z,
            self.coord_rx,
            self.speed,
            self.model,
        ]

        self.show_x = tk.Label(
            self.frmLC,
            textvariable=self.coord_x,
            font=("Arial", 9),
            width=7,
            height=1,
            bg="white",
        ).grid(row=0, column=4, padx=5, pady=5)
        self.show_y = tk.Label(
            self.frmLC,
            textvariable=self.coord_y,
            font=("Arial", 9),
            width=7,
            height=1,
            bg="white",
        ).grid(row=1, column=4, padx=5, pady=5)
        self.show_z = tk.Label(
            self.frmLC,
            textvariable=self.coord_z,
            font=("Arial", 9),
            width=7,
            height=1,
            bg="white",
        ).grid(row=2, column=4, padx=5, pady=5)
        self.show_rx = tk.Label(
            self.frmLC,
            textvariable=self.coord_rx,
            font=("Arial", 9),
            width=7,
            height=1,
            bg="white",
        ).grid(row=3, column=4, padx=5, pady=5)

        # mm 单位展示
        self.unit = tk.StringVar()
        self.unit.set("mm")
        for i in range(4):
            tk.Label(self.frmLC, textvariable=self.unit, font=("Arial", 9)).grid(
                row=i, column=5
            )

    def gripper_open(self):
        try:
            self.switch_gripper(True)
        except Exception as e:
            # 可能由于该方法没有返回值，服务抛出无法处理的错误
            pass

    def gripper_close(self):
        try:
            self.switch_gripper(False)
        except Exception as e:
            pass

    def get_coord_input(self):
        # 获取 coord 输入的数据，发送给机械臂
        c_value = []
        for i in self.all_c:
            c_value.append(float(i.get()))
        self.speed = (
            int(float(self.get_speed.get())) if self.get_speed.get() else self.speed
        )
        
        try:
            self.mc.send_coords(c_value,self.speed, self.model)
        except Exception as e:
            pass
        self.show_j_date(c_value, "coord")

    def get_joint_input(self):
        # 获取joint输入的数据，发送给机械臂
        j_value = []
        for i in self.all_j:
            j_value.append(float(i.get()))
            
        self.speed = (
            int(float(self.get_speed.get())) if self.get_speed.get() else self.speed
        )
        
        res = [j_value, self.speed]

        try:
            self.mc.send_angles(*res)
        except Exception as e:
            pass
        self.show_j_date(j_value)
        # return j_value,c_value,speed

    def get_date(self):
        # 拿机械臂的数据，用于展示
        t = time.time()
        while time.time() - t < 2:
            self.res = self.mc.get_coords()
            if self.res != []:
                break
            time.sleep(0.1)

        t = time.time()
        while time.time() - t < 2:
            self.angles = self.mc.get_angles()
            if self.angles != []:
                break
            time.sleep(0.1)
        
        self.record_coords[0] = self.res
        self.res_angles[0] = self.angles
        # print("self.res_angles[0]%s"%self.res_angles[0])
 

    # def send_input(self,dates):
    def show_j_date(self, date, way=""):
        # 展示数据
        if way == "coord":
            for i, j in zip(date, self.coord_all):
                j.set(str(i))
        else:
            for i, j in zip(date, self.cont_all):
                j.set(str(i) + "°")

    def run(self):
        while True:
            try:
                self.win.update()
                time.sleep(0.001)
            except tk.TclError as e:
                if "application has been destroyed" in str(e):
                    break
                else:
                    raise


def main():
    window = tk.Tk()
    window.title("mycobot ros GUI")
    Window(window).run()

if __name__ == "__main__":
    main()
