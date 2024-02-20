# JETSON IMAGE
# FROM stereolabs/zed:4.0-devel-jetson-jp5.1.1
# PC IMAGE 
FROM stereolabs/zed:4.0-devel-cuda12.1-ubuntu22.04

# Install dependencies
RUN apt update && apt install --yes git cmake make build-essential libnvinfer-dev libnvonnxparsers-dev

WORKDIR /
RUN git clone https://github.com/RoboEagles4828/zed-engine-builder.git

WORKDIR /zed-engine-builder
# RUN cmake . && make