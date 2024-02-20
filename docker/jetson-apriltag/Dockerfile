ARG IMAGE_NAME=nvcr.io/nvidia/isaac/ros:aarch64-ros2_humble_42f50fd45227c63eb74af1d69ddc2970
ARG ROS2_DIST=humble

FROM ${IMAGE_NAME}

RUN mkdir -p /root/isaac_ros_ws/src
WORKDIR /root/isaac_ros_ws/src

RUN git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_apriltag.git

WORKDIR /root/isaac_ros_ws

RUN apt-get update -y && rosdep update && rosdep install --from-paths src --ignore-src -r -y

RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && \
  colcon build --parallel-workers $(nproc) --symlink-install \
  --event-handlers console_direct+ --base-paths src \
  --cmake-args ' -DCMAKE_BUILD_TYPE=Release' \
  ' -DCMAKE_LIBRARY_PATH=/usr/local/cuda/lib64/stubs' \
  ' -DCMAKE_CXX_FLAGS="-Wl,--allow-shlib-undefined"' \
  ' --no-warn-unused-cli' "

# Setup environment variables 
COPY ros_entrypoint_jetson.sh /root/isaac_ros_ws/ros_entrypoint.sh
RUN sudo chmod 755 /root/isaac_ros_ws/ros_entrypoint.sh

ENTRYPOINT ["/root/isaac_ros_ws/ros_entrypoint.sh"]
CMD ["bash"]