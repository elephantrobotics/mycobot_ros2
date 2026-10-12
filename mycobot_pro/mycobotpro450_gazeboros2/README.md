## 解耦后的启动方式（当前入口）

公共环境与功能端分别启动。两个终端必须使用相同的 ROS_DOMAIN_ID、工作空间和 environment 参数。
公共环境不启动滑块页面，不建立真机连接；功能端在环境就绪后启动原有控制程序。

真机滑块／随机数控制：

```bash
# 终端 1：公共环境，等待真实初始姿态快照
ros2 launch mycobotpro450_gazeboros2 slidar.launch.py environment:=real
# 终端 2：提供只读快照，随后启动滑块控制后端和页面
ros2 launch mycobotpro450_gazeboros2 slidar_control_gazebo.launch.py environment:=real
```

真机键盘控制：

```bash
# 终端 1：与上面相同；公共环境已运行时无需再启动
ros2 launch mycobotpro450_gazeboros2 slidar.launch.py environment:=real
# 终端 2：独立提供快照并启动键盘页面，不启动滑块控制器
ros2 launch mycobotpro450_gazeboros2 teleop_keyboard.launch.py environment:=real
```

纯仿真使用同样的两个 launch，将两个 environment 参数都改为 simulation。
`slider.launch.py` 保留为公共环境入口，`slidar.launch.py` 是同一入口的拼写别名。
键盘依然需要先按 M 解锁，再按住原有运动键；松开夹爪按键仍使用既有读取角度后保持逻辑。
Randomize Target、Execute、Force Execute 和碰撞校验的行为不变。

同一 ROS 域只允许一个功能 launch。切换功能时，先退出当前功能 launch，再启动另一功能 launch；
公共环境可继续运行，不重新生成 Gazebo 模型。启动时会检查旧版 ros2 run 控制进程，避免重复控制。
真机初始快照只查询电源、错误、运动状态、六轴角度和夹爪角度；不会自动上电或使能／失能。
快照被环境消费后先释放 SDK 连接，环境完成初始姿态验证、控制器激活和 MoveIt 服务准备后才开放功能。
关闭功能端只关闭其自身启动的控制程序；公共环境失去就绪状态时，功能端通过原有退出流程停止。
本次只拆分启动职责，不提供失能拖动功能；SDK 源码、模型、碰撞矩阵和运动算法没有改动。

实现边界：

- launch/slider.launch.py：公共 Gazebo、MoveIt、RViz、初始姿态验证。
- scripts/pro450_environment_state.py：环境就绪心跳，无 SDK 依赖。
- scripts/pro450_control_session.py：功能互斥、只读快照交接、等待环境、管理自身控制进程。
- scripts/pro450_session_support.py：公共启动契约，不依赖滑块或键盘控制类。
- scripts/slider_control_gazebo.py、pro450_slider_gui.py、teleop_keyboard_gazebo.py：原有功能实现。

下面保留历史说明供查阅；以本节两 launch 的启动方式为准，不再额外启动历史 ros2 run 控制器。

---

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
在 `slider.launch.py` 启动的 MoveIt 中，每次新建规划都会以监测中的当前关节状态
为起点（与白色透明的 Scene Robot 相同来源），不使用 RViz 留存的启动姿态。
橙色 Query Goal 仍是用户自定义目标。工程内的 `UseCurrentStartState` 请求适配器
在原有规划适配器之前处理起点，不替换碰撞检查或时间参数化；适用于 OMPL、CHOMP
和 Pilz 管线。Plan 后若机械臂位置再次变化，应重新 Plan，原有 Execute 起点检查保留。
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
反馈超时和 NaN。真机模式保留该按钮，点击仅弹出“Force Execute Unavailable”，不发布
运动命令；后台也会强制拒绝外部强制执行请求。Gazebo 内部物理自碰撞已关闭，
Force Execute 可能让机器人连杆视觉穿透；它只能用于验证 MoveIt 拒绝结果，不能
用来验证 ODE 接触力。

机器人自碰撞由 MoveIt 和 `firefighter.srdf` 负责，在轨迹下发前完成。Gazebo 只
负责机器人与地面、工作台及其他外部物体的物理接触。不要把 link1～link6 的
`selfCollide` 改回 `true`：力控夹爪的 mimic 机构与安装端凸碰撞包络存在预期重叠，
ODE 接触约束会与位置控制器互相对抗，表现为夹爪抖动、越过关节限位并拖慢仿真。

夹爪的五个从动关节由 `GazeboSystem` 原生 mimic 支持与主关节同步更新，
正反方向及启动开度在 `firefighter.ros2_control.xacro` 中配置；已在 er 的
`gazebo_ros2_control 0.4.10` 上验证。不要重新加载旧的
`roboticsgroup_upatras_gazebo_mimic_joint_plugin`：它会把从动关节的 `fmax`
设成 effort 上限 1000，覆盖原来的摩擦值 0.1，并独立改写关节位置，引发大开度
抖动及腕部扰动。`/joint_states` 保留六轴和夹爪主关节共七个坐标，夹指通过
URDF mimic 关系显示；夹爪控制器只接收主关节目标。更新此配置后需重新启动
Gazebo 和控制器，运行中的模型不会自动重载 URDF 插件。

真机模式下，夹爪显示使用校准速度估算运动；到位后保留独立的夹爪输出，
只接收带新测量时间戳的真实夹爪读回。估算与读回存在量化差异时，显示平滑校正，
Actual 数值保留估算与校正结果，界面不再额外显示夹爪状态行。夹爪不再切回六轴的延时历史，
避免旧开度和缓存插值速度造成到位回跳。显示估算不会写入真实角度缓存，也不代替
真机到位判断。修改控制脚本后需要重新启动 slider 控制器及 GUI。

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



















夹爪速度：滑块页面 Speed (%) 直接对应夹爪 SDK 的 1–100 速度参数，例如 20% → 20、50% → 50。夹爪速度不再由机械臂轨迹时长反推，因此机械臂与夹爪不保证同时到位。Gazebo 夹爪低速显示采用已有测量数据，未标定的高速区间采用模型速度上限插值估计；该估计不参与 SDK 速度指令计算。
