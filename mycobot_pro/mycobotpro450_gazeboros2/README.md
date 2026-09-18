### 本地 1.操作流程

#### 1.1 安装前提



要使用此包，需先安装[Python api](https://github.com/elephantrobotics/pymycobot.git)库。

夹爪的使用：请将夹爪的模式切换为modbus(enable)模式

```bash

pip install pymycobot --user

```



#### 1.2 包的下载与安装



下载包到你的ros2工作空间中



```bash

$ cd ~/catkin_ws/src
$ cd ~/catkin_ws
$ colcon build
$ source install/setup.bash

```

MyCobot_450_m5-Gazebo使用说明

1. 滑块控制

使用项目自带的 Pro450 Confirmed Slider Control 页面控制机械臂。滑块只编辑目标，
点击英文 `Execute` 按钮后才执行；`Speed (%)` 同时控制 Gazebo 轨迹速度和真机速度。
执行前会检查关节限位、反馈 NaN，并通过 MoveIt 检查插值路径碰撞。
控制程序还会向 MoveIt 规划场景加入与 Gazebo `z=0` 地面对应的碰撞体；只有固定
底座 `base` 允许接触地面。Pro450 本体和力控夹爪的 15 个可视部件使用由 DAE
外形生成的封闭凸碰撞网格（外扩 0.5 mm），不再使用尺寸、位置偏差较大的圆柱和
方盒近似。默认另外保留 20 mm 地面净空。执行路径按不大于 1 度的关节步长检查，
因此 link5/link2、夹爪/link2 以及夹爪/地面的中途碰撞会在下发轨迹前被拒绝。


打开通信，给脚本添加执行权限



```bash
在src/mycobot_ros/mycobot_pro路径下执行
sudo chmod -R 777 mycobotpro450_gazeboros2/scripts/follow_display_gazebo.py
sudo chmod -R 777 mycobotpro450_gazeboros2/scripts/slider_control_gazebo.py
sudo chmod -R 777 mycobotpro450_gazeboros2/scripts/teleop_keyboard_gazebo.py
sudo chmod -R 777 mycobotpro450_gazeboros2/scripts/coords_broadcaster.py

```
每次开新终端就必须执行环境配置
```bash
source install/setup.bash


以下步骤请在ros目录下执行
```bash

ros2 launch mycobotpro450_gazeboros2 slider.launch.py

```



接着打开另外一个终端，输入如下命令：

```bash

ros2 run mycobotpro450_gazeboros2 coords_broadcaster.py

```

接着打开另外一个终端，输入如下命令：

```bash

ros2 run mycobotpro450_gazeboros2 slider_control_gazebo.py

```

输入 `1` 仅控制 Gazebo（默认、安全）；输入 `2` 同时控制 Gazebo 与真机。
此时在 `Pro450 Confirmed Slider Control` 页面设置目标角度和 `Speed (%)`，然后点击
`Execute`。`Randomize Target` 只生成随机目标，不会立即运动，仍需点击 `Execute`
并通过碰撞校验；`Zero Target` 同样只装载全零目标，确认执行后才回零。
移动滑块本身不会让机械臂运动；紧急停止使用 `STOP`。

`Force Execute (Sim Only)` 用于核对碰撞模型：它通过独立话题绕过 MoveIt
碰撞拒绝，但仅允许 Gazebo 模式，速度强制不超过 10%，且仍检查关节限位、
反馈超时和 NaN。真机模式会在后台强制拒绝该命令。Gazebo 内部物理自碰撞已关闭，
Force Execute 可能让机器人连杆视觉穿透；它只能用于验证 MoveIt 拒绝结果，不能
用来验证 ODE 接触力。

机器人自碰撞由 MoveIt 和 `firefighter.srdf` 负责，在轨迹下发前完成。Gazebo 只
负责机器人与地面、工作台及其他外部物体的物理接触。不要把 link1～link6 的
`selfCollide` 改回 `true`：力控夹爪的 mimic 机构与安装端凸碰撞包络存在预期重叠，
ODE 接触约束会与位置控制器互相对抗，表现为夹爪抖动、越过关节限位并拖慢仿真。

`/joint_states` 仅作为 Gazebo 实际反馈，不再作为滑块命令。GUI 的确认目标使用
`/pro450/slider_targets`，因此不会再由两个 `/joint_states` 发布者形成反馈回路。

真机 IP、端口和坐标读取频率可使用 ROS 参数设置，例如：

```bash
ros2 run mycobotpro450_gazeboros2 coords_broadcaster.py --ros-args \
  -p pro450_ip:=192.168.0.232 -p pro450_port:=4500 -p broadcast_rate:=10.0

ros2 run mycobotpro450_gazeboros2 slider_control_gazebo.py --ros-args \
  -p pro450_ip:=192.168.0.232 -p pro450_port:=4500
```

仿真地面保护参数可按模型标定结果调整（通常不建议小于 10 mm）：

```bash
ros2 run mycobotpro450_gazeboros2 slider_control_gazebo.py --ros-args \
  -p floor_clearance_m:=0.02 -p floor_frame:=world
```

碰撞网格维护与复核（开发时使用，运行仿真不需要安装这些 Python 依赖）：

```bash
python3 scripts/generate_collision_hulls.py \
  config/mycobot_pro_450_force_gripper.urdf \
  ../../mycobot_description \
  ../../mycobot_description/urdf/mycobot_pro_450/collision

python3 scripts/validate_collision_hulls.py \
  config/mycobot_pro_450_force_gripper.urdf \
  ../../mycobot_description
```

生成脚本需要 `numpy`、`scipy`、`trimesh` 和 `pycollada`。验证必须显示全部可视顶点
均位于 watertight 碰撞网格内；视觉模型或原点发生变化后必须重新生成并验证。



2\. Gazebo模型跟随

通过如下的命令可以实现Gazebo中的模型跟随实际机械臂的运动而发生位姿的改变，首先运行launch文件：



```bash

ros2 launch mycobotpro450_gazeboros2 follow.launch.py

```



如果程序运行成功，Gazebo界面将成功加载机械臂模型，机械臂模型的所有关节都处于原始位姿，即\[0,0,0,0,0,0]. 此后我们打开第二个终端并运行：



```bash

ros2 run mycobotpro450_gazeboros2 follow_display_gazebo.py

```



现在当我们操控实际机械臂的位姿，我们可以看到Gazebo中的机械臂也会跟着一起运动到相同的位姿。



3. 键盘控制

我们还可以使用键盘输入的方式同时操控Gazebo中机械臂模型与实际机械臂的位姿，首先打开一个终端并输入：



```bash

ros2 launch mycobotpro450_gazeboros2 teleop_keyboard.launch.py

```



同上一部分相同，我们会看到机械臂模型被加载到Gazebo中，并且所有关节都在初始的位姿上，紧接着我们打开另外一个终端并输入：



```bash

ros2 run mycobotpro450_gazeboros2 teleop_keyboard_gazebo.py

```



如果运行成功，我们将在终端看到如下的输出信息：



```shell

╔══════════════════════════════════════════════════════════╗
║   MyCobot Pro 450 键盘控制器 (Gazebo + 真实机械臂同步)   ║
╚══════════════════════════════════════════════════════════╝

关节控制 (普通步长: 5.0°, 快速步长: 15.0°):
  ┌─────────────────────────────────────────────────┐
  │ w/s: joint1 +/-     W/S: joint1 +/-  (快速)    │
  │ e/d: joint2 +/-     E/D: joint2 +/-  (快速)    │
  │ r/f: joint3 +/-     R/F: joint3 +/-  (快速)    │
  │ t/g: joint4 +/-     T/G: joint4 +/-  (快速)    │
  │ y/h: joint5 +/-     Y/H: joint5 +/-  (快速)    │
  │ u/j: joint6 +/-     U/J: joint6 +/-  (快速)    │
  └─────────────────────────────────────────────────┘

夹爪控制 (Pro力控夹爪 ID=14):
  ┌─────────────────────────────────────────────────┐
  │ o: 夹爪完全打开 (100°)                          │
  │ p: 夹爪完全关闭 (0°)                            │
  │ [: 夹爪开启 +10°                                │
  │ ]: 夹爪关闭 -10°                                │
  └─────────────────────────────────────────────────┘


```



根据上面的提示我们可以知道如何操控机械臂运动了，这里我设置每点击一下机械臂与Gazebo中的机械臂模型会运动1角度，可以尝试长按上述键位中的其中一个键来到达某一位姿。


















