#!/bin/bash
source /opt/ros/${ROS_DISTRO}/setup.bash
source install/setup.bash
ros2 launch rift_bringup real-vslam.launch.py
```