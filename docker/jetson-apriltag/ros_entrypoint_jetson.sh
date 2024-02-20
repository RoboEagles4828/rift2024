#!/bin/bash
source /opt/ros/${ROS_DISTRO}/setup.bash
source /root/isaac_ros_ws/install/setup.bash

ros2 launch isaac_ros_apriltag isaac_ros_apriltag_usb_cam.launch.py