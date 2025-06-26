# mycobot_ros2

[![jaywcjlove/sb](https://jaywcjlove.github.io/sb/lang/chinese.svg)](https://docs.elephantrobotics.com/docs/gitbook/12-ApplicationBaseROS/12.2-ROS2/12.2.1-ROS2%E7%9A%84%E5%AE%89%E8%A3%85.html)
[![jaywcjlove/sb](https://jaywcjlove.github.io/sb/lang/english.svg)](https://docs.elephantrobotics.com/docs/gitbook-en/12-ApplicationBaseROS/12.2-ROS2/12.2.3-ROS2%E4%BB%8B%E7%BB%8D.html)

[中文文档](https://docs.elephantrobotics.com/docs/gitbook/12-ApplicationBaseROS/12.2-ROS2/12.2.1-ROS2%E7%9A%84%E5%AE%89%E8%A3%85.html) | [English Documentation](https://docs.elephantrobotics.com/docs/gitbook-en/12-ApplicationBaseROS/12.2-ROS2/12.2.3-ROS2%E4%BB%8B%E7%BB%8D.html)

myCobot ROS2 package

**Notes**:

* Make sure that `Atom` is flashed into the top Atom and `Transponder` or `minirobot` is flashed into the base Basic .The tool download address: [https://github.com/elephantrobotics/myCobot/tree/main/Software](https://github.com/elephantrobotics/myCobot/tree/main/Software)
* Supported ROS2 versions:
  * Ubuntu 20.04 / ROS2 Foxy - branch `foxy`
  * Ubuntu 20.04 / ROS2 Galactic - branch `galactic`
  * Ubuntu 22.04 / ROS2 Humble - branch `humble`

## Installation

### 1.1 Pre-Requriements

For using this package, the [Python api](https://github.com/elephantrobotics/pymycobot) library should be installed first.

```bash
pip install pymycobot --user
```

### 1.2 Package Download and Install

Install ros package in your src folder of your Colcon workspace.

```bash
$ cd ~/colcon_ws/src
$ git clone --depth 1 https://github.com/elephantrobotics/mycobot_ros2.git
$ cd ~/colcon_ws
$ colcon build
$ source ~/colcon_ws/install/setup.bash
$ sudo echo 'source ~/colcon_ws/install/setup.bash' >> ~/.bashrc
```

## Troubleshooting

1. On ROS2 Humble if slider_control does not show GUI properly, update file
   `/opt/ros/humble/lib/python3.10/site-packages/joint_state_publisher_gui/joint_state_publisher_gui.py`
   from here: https://github.com/ros/joint_state_publisher/blob/ros2/joint_state_publisher_gui/joint_state_publisher_gui/joint_state_publisher_gui.py


## URDF Model Graph

[mycobot 280 m5](./mycobot_description/urdf/mycobot_280_m5/mycobot_280_m5.urdf)

![280 m5](./demo_img/280m5/280_m5.png)

[mycobot 280 m5 pump](./mycobot_description/urdf/mycobot_280_m5/mycobot_280_m5_with_pump.urdf)

![280 m5 pump](./demo_img/280m5/280_m5_pump.png)

[mycobot 280 m5 camera flange](./mycobot_description/urdf/mycobot_280_m5/mycobot_280_m5_with_camera_flange.urdf)

![280 m5 camera flange](./demo_img/280m5/280_m5_camera_flange.png)

[mycobot 280 m5 camera flange & pump](./mycobot_description/urdf/mycobot_280_m5/mycobot_280_m5_with_camera_flange_pump.urdf)

![280 m5 camera flange & pump](./demo_img/280m5/280_m5_camera_flange_pump.png)

[mycobot 280 pi](./mycobot_description/urdf/mycobot_280_pi/mycobot_280_pi.urdf)

![280 pi](./demo_img/280pi/280_pi.png)

[mycobot 280 pi pump](./mycobot_description/urdf/mycobot_280_pi/mycobot_280_pi_with_pump.urdf)

![280 pi pump](./demo_img/280pi/280_pi_pump.png)

[mycobot 280 pi camera flange](./mycobot_description/urdf/mycobot_280_pi/mycobot_280_pi_with_camera_flange.urdf)

![280 pi camera flange](./demo_img/280pi/280_pi_camera_flange.png)

[mycobot 280 pi camera flange & pump](./mycobot_description/urdf/mycobot_280_pi/mycobot_280_pi_with_camera_flange_pump.urdf)

![280 pi camera flange pump](./demo_img/280pi/280_pi_camera_flange_pump.png)

[mycobot 280 JetsonNano](./mycobot_description/urdf/mycobot_280_jn/mycobot_280_jn.urdf)

![280 jn](./demo_img/280jn/280jn.png)

[mycobot 280 Arduino](./mycobot_description/urdf/mycobot_280_arduino/mycobot_280_arduino.urdf)

![280 jn](./demo_img/280ar/280ar.png)

[mycobot 280 x3pi](./mycobot_description/urdf/mycobot_280_x3pi/mycobot_280_x3pi.urdf)

![280 x3pi](./demo_img/280pi/280_pi.png)

[mycobot 280 x5pi](./mycobot_description/urdf/mycobot_280_x5pi/mycobot_280_x5pi.urdf)

![280 x5pi](./demo_img/280pi/280_pi.png)

[mechArm 270 m5](./mycobot_description/urdf/mecharm_270_m5/mecharm_270_m5.urdf)

![270 m5](./demo_img/270m5/270m5.png)

[mechArm 270 pi](./mycobot_description/urdf/mecharm_270_pi/mecharm_270_pi.urdf)

![270 pi](./demo_img/270pi/270pi.png)

[myPalletizer 260 m5](./mycobot_description/urdf/mypalletizer_260_m5/mypalletizer_260_m5.urdf)

![260 m5](./demo_img/260m5/260m5.png)

[myPalletizer 260 pi](./mycobot_description/urdf/mypalletizer_260_pi/mypalletizer_260_pi.urdf)

![260 pi](./demo_img/260pi/260pi.png)

[mycobot 320 m5 2022](./mycobot_description/urdf/mycobot_320_m5_2022/mycobot_320_m5_2022.urdf)

![320 m5 2022](./demo_img/320m5_2022/320m5_2022.png)

[mycobot 320 pi 2022](./mycobot_description/urdf/mycobot_320_pi_2022/mycobot_320_pi_2022.urdf)

![320 pi 2022](./demo_img/320pi_2022/320pi_2022.png)

[ultraArm P340](./mycobot_description/urdf/ultraArm_p340/ultraArm_p340.urdf)

![p340](./demo_img/ultraArm_p340/ultraArmp340.png)

[mybuddy](./mycobot_description/urdf/mybuddy/mybuddy.urdf)

![mybuddy](./demo_img/mybuddy/mybuddy.png)

[mycobot pro 600](./mycobot_description/urdf/mycobot_pro_600/mycobot_pro_600.urdf)

![pro600](./demo_img/pro600/pro600.png)

[myArm 300 Pi](./mycobot_description/urdf/myarm_300_pi/myarm_300_pi.urdf)

![myarm](./demo_img/myarm300/myarm300.png)

[mycobot pro 630](./mycobot_description/urdf/mycobot_pro_630/mycobot_pro_630.urdf)

![pro630](./demo_img/pro630/pro630.png)
