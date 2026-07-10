# AirSLAM Dockerfile — ROS2 Humble + TensorRT 10.9 + CUDA 13.0
#
# 构建:
#   docker build -t airslam:ros2 .
#
# 运行:
#   docker run -it --gpus all --net=host --env DISPLAY=$DISPLAY \
#     -v /tmp/.X11-unix:/tmp/.X11-unix \
#     -v /path/to/datasets:/data \
#     -v ~/lib/Deploy/TensorRT-10.9.0.34:/opt/tensorrt \
#     airslam:ros2 /bin/bash

FROM nvidia/cuda:13.0-devel-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Shanghai

# 1. 系统依赖: 编译器 + C++ 库 + ONNX 修复工具
RUN apt-get update && apt-get install -y \
    build-essential cmake git wget unzip \
    locales curl gnupg2 lsb-release software-properties-common \
    libopencv-dev libeigen3-dev libyaml-cpp-dev \
    libboost-dev libboost-serialization-dev \
    libgflags-dev libgoogle-glog-dev libsuitesparse-dev \
    python3-pip \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

ENV LANG=en_US.UTF-8

# 2. ROS2 Humble + 功能包 + G2O
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt-get update && apt-get install -y \
    ros-humble-ros-base \
    ros-humble-cv-bridge \
    ros-humble-tf2-ros \
    ros-humble-visualization-msgs \
    ros-humble-libg2o \
    ros-humble-desktop \
    python3-colcon-common-extensions python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init || true && rosdep update
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# 3. Python 工具: ONNX 模型修复依赖 (polygraphy, onnx, onnxruntime)
RUN pip3 install onnx onnxruntime onnx_graphsurgeon polygraphy

# 4. TensorRT 10.9 — 通过 volume 挂载: -v ~/lib/Deploy/TensorRT-10.9.0.34:/opt/tensorrt
ENV TENSORRT_DIR=/opt/tensorrt
ENV LD_LIBRARY_PATH=${TENSORRT_DIR}/lib:/usr/local/cuda/lib64:${LD_LIBRARY_PATH}
ENV PATH=/usr/local/cuda/bin:${PATH}

WORKDIR /workspace

# 5. 构建 AirSLAM (取消注释以在镜像构建时编译)
# COPY . /workspace/src/AirSLAM
# RUN . /opt/ros/humble/setup.bash && \
#     cd /workspace && \
#     colcon build --packages-select air_slam --cmake-args -DCMAKE_BUILD_TYPE=Release
