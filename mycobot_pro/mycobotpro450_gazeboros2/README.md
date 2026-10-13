# Pro450 Gazebo ROS 2

## 安装依赖

使用该工程前，需要安装最新的pymycobot python api库

```bash
pip install pymycobot --user
```

## 公共环境

```bash
# 纯仿真环境：
ros2 launch mycobotpro450_gazeboros2 slider.launch.py environment:=simulation
# 真机环境：
ros2 launch mycobotpro450_gazeboros2 slider.launch.py environment:=real
```

真机环境下，公共环境启动时会读取一次真机姿态快照映射到Gazebo环境中，使用前需确认机械臂固件版本、末端固件版本、夹爪固件版本为最新，确保机械臂和夹爪通信正常。
纯仿真环境下，默认从零位开始。

## 滑块与随机目标

公共环境就绪后，另开终端：

```bash
ros2 launch mycobotpro450_gazeboros2 slidar_control_gazebo.launch.py environment:=real
# 纯仿真时使用 environment:=simulation
```

environment参数需要与公共环境一致，检测路径碰撞会显示 `Rejected: collision detected` 拒接执行。
Force Execute 测试按钮默认隐藏。仅在启动滑块功能时显式设置 `show_force_execute:=true` 才显示，功能为即使检测到碰撞也执行，仅在纯仿真环境下开放。

## 键盘控制

公共环境就绪后，另开终端：

```bash
ros2 launch mycobotpro450_gazeboros2 teleop_keyboard.launch.py environment:=real
# 纯仿真时使用 environment:=simulation
```

真机模式 `M` 解锁控制，`N` 或 `Space` 停止并锁定；`Q` 关闭窗口退出。
聚焦窗口，按住 `w/s、e/d、r/f、t/g、y/h、u/j` 控制六轴，按住 `[ / ]` 控制夹爪。
松键停止，失焦停止；`1–5` 切换速度档位，`- / +` 调整档位。界面持续显示当前设定速度。

## 拖动跟随

先启动真机公共环境，再另开终端：

```bash
ros2 launch mycobotpro450_gazeboros2 drag_follow_gazebo.launch.py
```

仅支持真机，`M` 控制六轴和夹爪使能/失能，六轴失能后托住机械臂手动拖动；夹爪失能后可手动掰动，Gazebo公共环境中的机械臂跟随真机运动，再次按下 `M` 锁紧六轴和夹爪，`Q` 关闭窗口退出。

