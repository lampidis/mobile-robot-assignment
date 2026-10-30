FROM ros:humble-ros-base-jammy@sha256:566b87e6dfc48392189a9c32e594e84060fa9948de93b04e536f39475a1f6963

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=humble \
    PYTHONUNBUFFERED=1 \
    MPLBACKEND=Agg

# One image for the C++ action application, perception pipeline, and RViz2.
# Keep source, bag data, results, and model weights on runtime mounts.
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    python3-colcon-common-extensions \
    python3-pip \
    python3-yaml \
    ros-humble-ament-cmake \
    ros-humble-cv-bridge \
    ros-humble-geometry-msgs \
    ros-humble-launch-ros \
    ros-humble-message-filters \
    ros-humble-robot-state-publisher \
    ros-humble-rosbag2 \
    ros-humble-rviz2 \
    ros-humble-rosidl-default-generators \
    ros-humble-sensor-msgs \
    ros-humble-tf2-geometry-msgs \
    ros-humble-tf2-ros \
    ros-humble-tf2-sensor-msgs \
    ros-humble-turtlesim \
    ros-humble-vision-msgs \
    && rm -rf /var/lib/apt/lists/*

# ROS Humble's cv_bridge is built against NumPy 1.x. Install a CPU-only
# PyTorch wheel first so the detector does not pull CUDA runtime packages.
RUN python3 -m pip install --no-cache-dir \
    'numpy==1.26.4' \
    'torch==2.2.2+cpu' \
    'torchvision==0.17.2+cpu' \
    --extra-index-url https://download.pytorch.org/whl/cpu \
    && python3 -m pip install --no-cache-dir \
    'numpy==1.26.4' \
    'opencv-python==4.10.0.84' \
    'ultralytics==8.3.220'

WORKDIR /workspace

ENV YOLO_CONFIG_DIR=/tmp

CMD ["bash"]
