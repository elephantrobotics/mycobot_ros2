# Pro630 Gazebo 控制架构

本包采用 Pro450 当前版本的四模块架构，保留 Pro630 原有的模型尺寸、
关节名称、轴向、安装变换和关节限位。README 未在本次重构中修改。

## 四个入口与职责

| 入口 | 职责 | 环境 |
| --- | --- | --- |
| `slider.launch.py` | Gazebo、ros2_control、MoveIt、唯一碰撞场景；真机模式下独占 SDK 连接、获取初始快照并持续同步位姿 | simulation / real |
| `slider_control_gazebo.launch.py` | 滑块界面、随机目标、单层 MoveIt 路径碰撞校验、速度选择、执行及 STOP | simulation / real |
| `teleop_keyboard.launch.py` | 按住关节或夹爪按键运动、松开停止、速度档位、碰撞边界校验和锁定 | simulation / real |
| `drag_follow_gazebo.launch.py` | 启动时使六轴退出自由拖动并使能夹爪一次；M 同时切换六轴自由拖动与夹爪使能 | real only |

每个功能使用公共环境加一个功能 launch。功能 launch 之间互斥，
退出功能后可切换到另一个功能。公共环境持续维护真实位姿；功能模块
通过 `/pro630/robot_rpc` 调用公共连接，不打开第二条机器人 TCP 连接。

### 纯仿真

```bash
source /opt/ros/humble/setup.bash
source ~/xzh/install/setup.bash
ros2 launch mycobotpro630_gazeboros2 slider.launch.py
```

另一个终端启动滑块或键盘功能：

```bash
ros2 launch mycobotpro630_gazeboros2 slider_control_gazebo.launch.py
# 或
ros2 launch mycobotpro630_gazeboros2 teleop_keyboard.launch.py
```

Force Execute 默认隐藏，只在明确指定测试参数时显示：

```bash
ros2 launch mycobotpro630_gazeboros2 slider_control_gazebo.launch.py show_force_execute:=true
```

它使用界面所选速度，只允许纯仿真跳过 MoveIt 碰撞拒绝。
真机模式下，界面和后端均阻止此调用路径。

### 真机配置

```bash
ros2 launch mycobotpro630_gazeboros2 slider.launch.py environment:=real \
  pro630_ip:=192.168.1.191 pro630_port:=5001

# 另一个终端任选一个功能：
ros2 launch mycobotpro630_gazeboros2 slider_control_gazebo.launch.py environment:=real
ros2 launch mycobotpro630_gazeboros2 teleop_keyboard.launch.py environment:=real
ros2 launch mycobotpro630_gazeboros2 drag_follow_gazebo.launch.py
```

拖动 launch 不需要 environment 参数，也不支持仿真。
公共环境启动只读取硬件，不自动上电、校准或发送角度指令。
它要求机械臂已上电且静止，获取稳定六轴快照，并最多等待 10 秒获取
合法夹爪角度；控制器完成初始化和位姿核对后才放行功能模块。

450 与 630 共用标准控制器、TF 和 MoveIt 服务名称，因此同一个 ROS 域内
只能运行一套公共环境。需要同时运行时，必须分别设置 ROS_DOMAIN_ID
和 GAZEBO_MASTER_URI。重构验收使用域 93 / 94 和端口 11346 / 11347，
没有停止 er 原有的 450 环境。

## Pro630 SDK 适配

依据旧 Pro630 工程与 er 已安装的 `pymycobot/elephantrobot.py`。
SDK 源码没有修改。适配文件为 `scripts/pro630_sdk_adapter.py`。

| 工程操作 | ElephantRobot 接口 |
| --- | --- |
| 六轴角度读取 | `get_angles` 的文本请求，严格校验六个有限值 |
| 六轴目标下发 | `write_angles` |
| 单轴目标下发 | 读取六轴后修改对应元素，再 `write_angles` |
| 单轴持续点动 | `jog_angle('J1'..'J6', -1/1, speed)` |
| 六轴停止 | `task_stop` |
| 运动 / 电源状态 | `check_running` / `is_power_on` 的文本请求 |
| 错误检查 | `get_robot_state` 的 SDK 文档状态位，不清空错误队列 |
| 夹爪目标 / 实际角度 | `force_set_angle(14, value)` / `force_get_angle(14)` |
| 夹爪速度 / 读回 | `force_set_speed(14, 1..100)` / `force_get_speed(14)` |
| 夹爪使能 | `force_set_enabled(14, 0/1)` |
| 六轴拖动 / 保持 | `set_free_move_mode(True/False)`；保持时先 `_state_on` 并用 `state_check` 确认 |

630 的拖动使用 SDK 的自由拖动模式，不使用未经确认的数字制动 IO，
也不使用 SDK 中不存在的 `release_all_servos`。这与 450 的电机失能接口
存在硬件语义差异，需要 Pro630 实机验收。

硬件 J2、J4 到模型坐标加 90°，目标下发时减 90°，转换只发生在适配层。
其余模块始终使用模型坐标。夹爪为 0..100 的整数，模型为 0..1，
滑块目标与实际开度显示两位小数。夹爪速度直接使用所选 SDK 百分比，
没有开度预测、速度标定或补偿模型。

六轴接口速度沿用旧工程的 1000 上限并按模型速度做名义换算；
SDK 源码没有给出该参数与真实角速度的标定关系。本次未进行真机
速度测量，界面中的关节 deg/s 是控制目标，不是实测速度。

## 碰撞与模型

模型和控制关节限位在 `config/mycobot_pro_630_force_gripper.urdf`，
运行模块共用 `scripts/pro630_model.py` 的相同限位。
`firefighter.srdf` 提供唯一允许碰撞矩阵，公共场景加入地面物体，
滑块和键盘共用 MoveIt `/check_state_validity`，没有另加运动包络校验层。

- 15 个视觉组件从原始 DAE 的完整场景导出，包含单位缩放和节点变换。
- MoveIt 使用 `collision_exact` 的完整三角网格。
- Gazebo 使用从同一几何生成的凸包及夹爪包络，避免 ODE 处理复杂凹网格。
- 底座包络保留 CAD 的 z=0 安装底面，避免向下膨胀产生持续地面穿透。
- 端子为同款夹爪凹槽直径、25 mm 高的碰撞圆柱，没有新增视觉模型。
- 手指与 link5/link6、端子与机械臂及地面的碰撞检查保持开启。
- Gazebo 取消重复 mimic 插件，由 GazeboSystem 唯一维护五个从动关节。
- 同款夹爪惯性参数与 450 对齐。位置跟踪模型关闭重力；它不用于
  验证真实负载、重力补偿或接触力学。

离线重新生成网格：

```bash
python3 scripts/build_collision_meshes.py \
  config/mycobot_pro_630_force_gripper.urdf ../../mycobot_description
```

生成工具需要 numpy 和 scipy，正常运行使用已生成的网格。
编译需要 `mycobot_description` 与本包：

```bash
colcon build --packages-select mycobot_description mycobotpro630_gazeboros2
```

## 验收范围

本次完成 er 编译、隔离 Gazebo 纯仿真、Qt 界面、随机碰撞采样及
仅连接 127.0.0.1 假机器人的 SDK 和公共连接链路检查。
真实 Pro630 的速度、制动距离、自由拖动效果及现场碰撞距离尚未实机验证。
RViz MoveIt Execute 仍连接 Gazebo 控制器，没有新增真机轨迹执行通道。
