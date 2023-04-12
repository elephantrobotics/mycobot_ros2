#!/bin/bash

# >>> fishros initialize >>>
source /opt/ros/foxy/setup.bash 
# <<< fishros initialize <<<
source ~/colcon_ws/install/setup.bash

cd $HOME/colcon_ws/src/mycobot_ros2/ultraArm/ultraArm_Testtool
python3 main.py
