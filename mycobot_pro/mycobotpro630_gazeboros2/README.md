# MyCobot Pro 630 Gazebo & ROS 2 仿真控制包

本启动指南适用于 `mycobotpro630_gazeboros2` 控制包，主要包含在 Gazebo 下对大象机器人 MyCobot Pro 630 以及夹爪进行联动的仿真控制功能。

## 1. 编译工作空间
在每次修改了 `scripts` 或 `launch` 文件后，都需要重新编译以确保文件正确安装至 `lib` / `share` 目录中。

请在您的工作空间根目录（例如 `C:\Users\elephant\Desktop\mycobot_630gazeboros2`）打开 ROS 2 专属终端（或确保终端已配置好了 ROS 2 的基础环境），运行：

```bash
# 1. 编译工程目录（如果您使用的是 Linux 则无需使用 .bat 格式）
colcon build --packages-select mycobotpro630_gazeboros2

# 2. 刷新你的环境变量 (Windows下通常为 setup.bat，基于具体的终端可能为 .ps1。Linux为 .bashrc 或 install/setup.bash)
call install/setup.bat
```

---

## 2. 功能启动指引

以下所有功能**均需要两个终端共同协作**：
- **终端 1**：用于启动 Gazebo 环境及 MoveIt 组件配置。
- **终端 2**：用于运行负责收发指令逻辑的 Python 脚本。
*(记得在每一个新打开的终端中执行 `call install/setup.bat`)*

### 功能一：滑块控制 (Slider Control)
在该模式下，能够配合弹出 GUI 滑块或者通过真实机械臂返回的坐标，联动控制仿真环境中的模型和它的夹爪。

* **终端 1（启动仿真环境）**
  ```bash
  ros2 launch mycobotpro630_gazeboros2 slider.launch.py
  ```

* **终端 2（启动控制脚本）**
  ```bash
  ros2 run mycobotpro630_gazeboros2 slider_control_gazebo.py
  # 按屏幕提示，选择模式 1 (Gazebo 纯仿真) 或 2 (连接真实机械臂)。
  ```

---

### 功能二：键盘遥控 (Teleop Keyboard)
在这个模式中，您将使用各种按键快捷键来进行空间位姿的远程遥控（包括关节的微调和夹爪的快捷开合）。

* **终端 1（启动仿真环境）**
  ```bash
  ros2 launch mycobotpro630_gazeboros2 teleop_keyboard.launch.py
  ```

* **终端 2（启动控制脚本）**
  ```bash
  ros2 run mycobotpro630_gazeboros2 teleop_keyboard_gazebo.py
  # 注意：必须保持在此终端激活的情况下按键才会生效。
  # 按键指令示例：用 `w/s` 控制关节，`o/p` 控制夹爪，`q` 退出。
  ```

---

### 功能三：示教跟随 (Follow Display)
这是一种被动更新数据的显示模式，程序将连接真实的 Pro 630 设备，读取当前各个关节及夹爪的角度大小，实时一比一映射进 Gazebo 模型里。

* **终端 1（启动仿真环境）**
  ```bash
  ros2 launch mycobotpro630_gazeboros2 follow.launch.py
  ```

* **终端 2（启动控制脚本）**
  ```bash
  ros2 run mycobotpro630_gazeboros2 follow_display_gazebo.py
  ```

---

## 常见问题
1. **机械臂没有动或控制失败**：
   - 检查并确保真实机械臂或对应树莓派开启了 `IP: 192.168.137.182, 端口: 5001` 的 Server 控制服务。
   - 确保你的电脑和树莓派处于同一个网段能够正常 Ping 通。
2. **找不到可执行文件**：
   - 如果遇到 `executable not found`，意味着你没有进行 `colcon build` 或者由于缓存未能生效配置。请尝试清理 `build/` 文件夹并重新打包编译。
