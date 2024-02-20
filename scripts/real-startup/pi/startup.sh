#!/bin/bash
cd /home/ubuntu/rift2024/
source /opt/ros/humble/setup.bash
source ./install/setup.bash
export ROS_NAMESPACE=real
export ROS_DOMAIN_ID=0
ros2 launch rift_bringup real.launch.py