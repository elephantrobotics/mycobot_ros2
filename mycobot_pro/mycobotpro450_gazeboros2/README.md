### 本地 1.操作流程

#### 1.1 安装前提



要使用此包，需先安装[Python api](https://github.com/elephantrobotics/pymycobot.git)库。

夹爪的使用：请将夹爪的模式切换为modbus(enable)模式

```bash

pip install pymycobot --user

```



#### 1.2 包的下载与安装



下载包到你的ros工作空间中



```bash

$ cd ~/catkin_ws/src
$ cd ~/catkin_ws
$ catkin_make
$ source devel/setup.bash

```

MyCobot_450_m5-Gazebo使用说明

1. 滑块控制

现已实现通过joint_state_publisher_gui的滑块控制机械臂模型在Gazebo中的位姿


打开通信，给脚本添加执行权限



```bash
在src/mycobot_ros/mycobot_pro路径下执行
sudo chmod -R 777 mycobotpro450_gazebo/scripts/follow_display_gazebo.py
sudo chmod -R 777 mycobotpro450_gazebo/scripts/slider_control_gazebo.py
sudo chmod -R 777 mycobotpro450_gazebo/scripts/teleop_keyboard_gazebo.py
sudo chmod -R 777 mycobotpro450_gazebo/scripts/coords_broadcaster.py
roscore

```



以下步骤请在ros目录下执行
```bash

source devel/setup.bash
roslaunch mycobotpro450_gazebo slider.launch

```



接着打开另外一个终端，输入如下命令：

```bash

source devel/setup.bash
rosrun mycobotpro450_gazebo coords_broadcaster.py

```

接着打开另外一个终端，输入如下命令：

```bash

source devel/setup.bash
rosrun  mycobotpro450_gazebo slider_control_gazebo.py

```

此时便可通过操控joint\_state\_publisher\_gui的滑块来同时操控Gazebo中机械臂模型的位姿了。



2\. Gazebo模型跟随

通过如下的命令可以实现Gazebo中的模型跟随实际机械臂的运动而发生位姿的改变，首先运行launch文件：



```bash

source devel/setup.bash
roslaunch  mycobotpro450_gazebo follow.launch 

```



如果程序运行成功，Gazebo界面将成功加载机械臂模型，机械臂模型的所有关节都处于原始位姿，即\[0,0,0,0,0,0]. 此后我们打开第二个终端并运行：



```bash

source devel/setup.bash
rosrun  mycobotpro450_gazebo follow_display_gazebo.py 

```



现在当我们操控实际机械臂的位姿，我们可以看到Gazebo中的机械臂也会跟着一起运动到相同的位姿。



3. 键盘控制

我们还可以使用键盘输入的方式同时操控Gazebo中机械臂模型与实际机械臂的位姿，首先打开一个终端并输入：



```bash

source devel/setup.bash
roslaunch  mycobotpro450_gazebo teleop_keyboard.launch 

```



同上一部分相同，我们会看到机械臂模型被加载到Gazebo中，并且所有关节都在初始的位姿上，紧接着我们打开另外一个终端并输入：



```bash

source devel/setup.bash
rosrun  mycobotpro450_gazebo teleop_keyboard_gazebo.py 

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


















