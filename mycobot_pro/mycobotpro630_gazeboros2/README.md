# Pro630 Gazebo ROS 2

## 安装依赖

使用该工程前，需要安装最新的 pymycobot Python API 库：

```bash
pip install pymycobot --user
```

创建工作空间并拉取代码：

```bash
mkdir -p ~/colcon_ws/src
cd ~/colcon_ws/src
git clone --branch Pro630gazebo https://github.com/elephantrobotics/mycobot_ros2.git
```

编译并加载 ROS 2 环境：

```bash
cd ~/colcon_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select mycobot_description mycobotpro630_gazeboros2
# 或全量编译
# colcon build
source install/setup.bash
```

## 前置上电准备

参考[机器上电步骤](https://docs.elephantrobotics.com/docs/pro630-cn/5-BasicApplication/5.2-ApplicationUse.html#11%E7%B3%BB%E7%BB%9F%E4%B8%8A%E7%94%B5)，执行到 `1.1 系统上电` 即可。

## 公共环境

如果机械臂选择末端装有myGripper F100 力控夹爪，则调用以下指令拉起公共环境：

```bash
# 纯仿真环境 带夹爪
ros2 launch mycobotpro630_gazeboros2 slider.launch.py environment:=simulation with_gripper:=true

# 真机环境 带夹爪
ros2 launch mycobotpro630_gazeboros2 slider.launch.py environment:=real with_gripper:=true
```

如果机械臂选择末端没有myGripper F100 力控夹爪，则调用以下指令拉起公共环境：

```bash
# 纯仿真环境 不带夹爪
ros2 launch mycobotpro630_gazeboros2 slider.launch.py environment:=simulation with_gripper:=false

# 真机环境 不带夹爪
ros2 launch mycobotpro630_gazeboros2 slider.launch.py environment:=real with_gripper:=false
```

- 启动成功后，会拉起 Gazebo 仿真环境，并实时跟踪真机机械臂姿态，期间可切换控制器来控制机械臂。
- 真机环境：启动时会读取一次真机姿态快照，并映射到 Gazebo 环境中。使用前需确认机械臂、末端、夹爪的固件版本为最新，并确保机械臂和夹爪通信正常。
- 纯仿真环境：默认从零位开始。

## 滑块与随机目标

公共环境就绪后，另开终端：

```bash
ros2 launch mycobotpro630_gazeboros2 slider_control_gazebo.launch.py environment:=real
# 纯仿真时使用 environment:=simulation
```

- 启动成功后，会拉起用于`滑块和随机数操作`的 Qt 界面，界面和实际功能会自动适配公共环境是否有 myGripper F100 力控夹爪的情况。
- `environment` 参数需要与公共环境一致。检测到路径碰撞时，会显示 `Rejected: collision detected` 并拒绝执行。
- 滑块控制逻辑发生变化：移动滑块、随机运动、回零时，需要点击 `Execute` 按钮后才会运动。
- `Stop` 按键全程可用，点击后机械臂停止运动。
- Force Execute 测试按钮默认隐藏，仅在启动滑块功能时显式设置 `show_force_execute:=true` 才显示。其功能为即使检测到碰撞也执行，且仅在纯仿真环境下开放。

## 键盘控制

公共环境就绪后，另开终端：

```bash
ros2 launch mycobotpro630_gazeboros2 teleop_keyboard.launch.py environment:=real
# 纯仿真时使用 environment:=simulation
```

- 启动成功后，会拉起用于`键盘控制`的 Qt 界面，界面和实际功能会自动适配公共环境是否有 myGripper F100 力控夹爪的情况。

真机模式下：

- `M`：解锁控制
- `N` 或 `Space`：停止并锁定
- `Q`：关闭窗口退出

聚焦窗口后：

- 按住 `w` / `s`、`e` / `d`、`r` / `f`、`t` / `g`、`y` / `h`、`u` / `j` 控制六轴
- 按住 `[` / `]` 控制夹爪，无夹爪时没有该功能
- 松键停止，失焦停止
- `1`–`5` 切换速度档位，`-` / `+` 调整档位
- 108 键布局下，小键盘区的 `-` / `+`，以及主键盘区的 `-`（`_`）、`+`（`=`），均可用于速度档位调整。
- 按键控制不区分大小写。
- 界面持续显示当前设定速度

## 拖动跟随

公共环境就绪后，另开终端：

```bash
ros2 launch mycobotpro630_gazeboros2 drag_follow_gazebo.launch.py
```

仅支持真机。

- 启动成功后，会拉起用于`拖动跟随`的 Qt 界面，界面和实际功能会自动适配公共环境是否有 myGripper F100 力控夹爪的情况。
- `M`：控制六轴和夹爪使能 / 失能。六轴失能后，托住机械臂手动拖动；夹爪失能后可手动掰动。Gazebo 公共环境中的机械臂跟随真机运动。再次按下 `M`，锁紧六轴和夹爪。
- `Q`：关闭窗口退出。
- 按键控制不区分大小写。

## GUI 控制

公共环境就绪后，另开终端：

```bash
ros2 launch mycobotpro630_gazeboros2 gui_control_gazebo.launch.py
```

仅支持真机。

- 启动成功后，会拉起用于 `GUI 控制` 的 Qt 界面，界面和实际功能会自动适配公共环境是否有 myGripper F100 力控夹爪。
- 输入速度、目标角度或坐标后，点击 `Set Gripper`、`Set Joints` 或 `Set Coordinates`，执行相应运动。
- 当输入坐标为奇异位置时，机械臂会运动到一半，显示 `Motion rejected: singular position` 后停止运动，link5 的 LED 光圈变成蓝色。
- 角度运动的路径检测到碰撞时，会显示 `Rejected: collision detected` 并拒绝执行。
- 630 进入奇异位置时，不会改变 LED 屏颜色。如果机械臂静止但六轴坐标跳变，或调用坐标运动时打印 `The current pose is singular. Please use joint control to move away`，即当前为奇异姿态，需要用角度运动脱离。可以参考[奇异姿态典例](https://docs.elephantrobotics.com/docs/pro630-cn/2-ProductFeature/2.5-CoordinateSystem.html)
