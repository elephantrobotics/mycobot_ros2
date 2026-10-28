# Pro450 Gazebo ROS 2

## 安装依赖

使用该工程前，需要安装最新的 pymycobot Python API 库：

```bash
pip install pymycobot --user
```

创建工作空间并拉取代码：

```bash
mkdir -p ~/colcon_ws/src
cd ~/colcon_ws/src
git clone --branch Pro450gazebo https://github.com/elephantrobotics/mycobot_ros2.git
```

编译并加载 ROS 2 环境：

```bash
cd ~/colcon_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select mycobot_description mycobotpro450_gazeboros2
# 或全量编译
# colcon build
source install/setup.bash
```

## 公共环境

如果机械臂选择末端装有myGripper F100 力控夹爪，则调用以下指令拉起公共环境：

```bash
# 纯仿真环境 带夹爪
ros2 launch mycobotpro450_gazeboros2 slider.launch.py environment:=simulation with_gripper:=true

# 真机环境 带夹爪
ros2 launch mycobotpro450_gazeboros2 slider.launch.py environment:=real with_gripper:=true
```

如果机械臂选择末端没有myGripper F100 力控夹爪，则调用以下指令拉起公共环境：

```bash
# 纯仿真环境 不带夹爪
ros2 launch mycobotpro450_gazeboros2 slider.launch.py environment:=simulation with_gripper:=false

# 真机环境 不带夹爪
ros2 launch mycobotpro450_gazeboros2 slider.launch.py environment:=real with_gripper:=false
```

- 启动成功后，会拉起 Gazebo 仿真环境，并实时跟踪真机机械臂姿态，期间可切换控制器来控制机械臂。
- 真机环境：启动时会读取一次真机姿态快照，并映射到 Gazebo 环境中。使用前需确认机械臂、末端、夹爪的固件版本为最新，并确保机械臂和夹爪通信正常。
- 纯仿真环境：默认从零位开始。

如果坐标控制期间机械臂本体陷入水平面以下，则调用以下指令临时强制拉起公共环境：

```bash
# 真机环境 带夹爪
ros2 launch mycobotpro450_gazeboros2 slider.launch.py environment:=real with_gripper:=true allow_collision:=true

# 真机环境 不带夹爪
ros2 launch mycobotpro450_gazeboros2 slider.launch.py environment:=real with_gripper:=false allow_collision:=true
```

- 启动后的初始阶段，角度控制会被拒绝。需使用坐标控制器（键盘控制或 GUI 控制）沿 Z 轴上移，使机械臂本体回到水平面以上，再改用正常指令重新拉起公共环境。
- `allow_collision:=true` 仅用于应急：坐标运动导致机械臂本体低于水平面的特殊情况。该参数默认为 `false`，且仅支持真机，即 `environment:=real`。
- 该模式仅临时用于坐标运动导致机械臂本体陷入水平面以下的特殊情况。脱离异常区域后，请切换回正常公共环境使用机械臂。
- 本工程的坐标运动仅限制末端不陷入水平面以下。具体见键盘控制的操作说明。

## 滑块与随机目标

公共环境就绪后，另开终端：

```bash
ros2 launch mycobotpro450_gazeboros2 slider_control_gazebo.launch.py environment:=real
# 纯仿真时使用 environment:=simulation
```

- 启动成功后，会拉起用于`滑块和随机数操作`的 Qt 界面，界面和实际功能会自动适配公共环境是否有 myGripper F100 力控夹爪的情况。
- `Randomize Target` 按钮用于生成六轴与夹爪的随机角度。
- `Zero Target` 按钮用于生成机械臂的零位关节角度。
- 移动滑块、生成随机目标、回零生成关节角度后，均需点击 `Execute` 按钮，机械臂才会运动。
- `Stop` 按钮用于停止正在运动的机械臂。
- `environment` 参数需要与公共环境一致。检测到路径碰撞时，会显示 `Rejected: collision detected` 并拒绝执行。

## 键盘控制

公共环境就绪后，另开终端：

```bash
ros2 launch mycobotpro450_gazeboros2 teleop_keyboard.launch.py
```

- 启动成功后，会拉起用于`键盘控制`的 Qt 界面，界面和实际功能会自动适配公共环境是否有 myGripper F100 力控夹爪的情况。
- 装有 myGripper F100 力控夹爪时，该控制器进行坐标运动的末端会自动延伸至夹爪末端。

仅支持真机。

聚焦窗口后：

- `M`：解锁控制
- `N` 或 `Space`：停止并锁定
- `C`：清除奇异位姿导致的报错
- `Q`：关闭窗口并退出
- 按住 `w` / `s`、`a` / `d`、`z` / `x` 分别控制位置 X、Y、Z；按住 `u` / `j`、`i` / `k`、`o` / `l` 分别控制姿态角 RX、RY、RZ
- 按住 `g` / `h` 控制夹爪开合，未安装夹爪时无此功能
- `1`：回到机械臂零位
- `2`：回到坐标控制 Home 位
- `3`：自定义坐标控制 Home 位，将当前位置设置为坐标运动 Home 位，下次启动键盘控制时需重新设置
- 松开按键即停止，窗口失焦即停止
- 108 键布局下，主键盘区的 `-`（`_`）、`+`（`=`），以及小键盘区的 `-` / `+`，均可切换速度档位
- 坐标控制进入奇异位姿时，机械臂 link5 的 LED 光圈变为蓝色，Qt 界面底部显示 `Motion rejected: singular position`。需先按 `C` 清除错误，再按 `M` 解锁，此时允许键盘控制运动。建议按 `2` 回到坐标控制 Home 位，否则在该位姿下继续运动容易再次进入奇异位姿。
- 执行零位运动后也会进入奇异位姿，需依次按 `C`、`M` 进入坐标控制 Home 位，才能恢复功能。
- 控制期间不允许运动到水平面以下。末端距地面 20 mm 时会触发保护机制，需先长按 `X` 向上运动并脱离触发区域，才能继续其他运动按键；此时仍可调整速度档位。
- 本工程的保护机制仅限制末端，不能保证机械臂本体不与地面碰撞，操作时需谨慎。
- 按键控制不区分大小写。
- 界面持续显示可操作按键、当前设定速度、速度档位、当前有效按键、提示信息等。

错误码 33 的特殊情况：

- 若抛出错误码 33，表示无法求得连续的直线运动解。LED 光圈可能先变为蓝色，再闪一下红色，随后恢复绿色，Qt 界面底部显示 `Controller error 33`。此时需先按 `C` 清除错误，再按 `M` 解锁，随后允许键盘控制运动。建议按 `2` 回到坐标控制 Home 位，否则在该位姿下继续运动容易再次抛出错误码，甚至导致机械臂重启。

## 拖动跟随

公共环境就绪后，另开终端：

```bash
ros2 launch mycobotpro450_gazeboros2 drag_follow_gazebo.launch.py
```

仅支持真机。

- 启动成功后，会拉起用于`拖动跟随`的 Qt 界面，界面和实际功能会自动适配公共环境是否有 myGripper F100 力控夹爪的情况。
- `M`：控制六轴进入或退出零力拖动模式，并控制夹爪使能或失能。六轴进入零力拖动模式后，听到机械臂发出两声咔咔声，再托住机械臂手动拖动；夹爪失能后可手动掰动。Gazebo 公共环境中的机械臂跟随真机运动。再次按下 `M`，锁紧六轴和夹爪。
- `Q`：关闭窗口退出。
- 拖动跟随期间，如果关节超限，LED 光圈会变为蓝色。此时仍可正常失能关节并运动。退出拖动跟随界面前，请将机械臂运动到零位附近，并确认关节未超限。程序退出时会自动清错。

## GUI 控制

公共环境就绪后，另开终端：

```bash
ros2 launch mycobotpro450_gazeboros2 gui_control_gazebo.launch.py
```

仅支持真机。

- 启动成功后，会拉起用于 `GUI 控制` 的 Qt 界面，界面和实际功能会自动适配公共环境是否有 myGripper F100 力控夹爪。
- 装有 myGripper F100 力控夹爪时，该控制器进行坐标运动的末端会自动延伸至夹爪末端。
- 坐标控制需先点击 `Home`，进入坐标控制 Home 位后再开始坐标运动。该按钮也可用于脱离奇异位姿。
- 输入速度、目标角度或坐标后，点击 `Set Gripper`、`Set Joints` 或 `Set Coordinates`，执行相应运动。
- `Stop` 按钮用于停止正在运动的机械臂。
- 当输入坐标为奇异位置时，机械臂会运动到一半，显示 `Motion rejected: singular position` 后停止运动。
- 进入奇异位姿后，link5 的 LED 光圈变为蓝色并保持。需通过角度控制脱离奇异位姿，才能继续坐标运动；也可直接点击 `Home`，回到坐标控制 Home 位。随后点击 `Clear Errors` 清错，使 LED 光圈恢复绿色。
- 如果进入奇异位姿后 LED 光圈自动恢复绿色，则无需点击 `Clear Errors` 清错，即可继续运动。坐标运动特殊情况的详细说明参见键盘控制的操作说明。
- 角度运动的路径检测到碰撞时，会显示 `Rejected: collision detected` 并拒绝执行。
