# =============================================================================
# Go2 CycloneDDS ROS2 SDK — Optimized Dockerfile for Jetson Orin NX (ARM64)
#
# Cache strategy:
#   1. System deps (rarely changes — cached)
#   2. Package manifests only → rosdep install (cached unless deps change)
#   3. Source code → colcon build (rebuilds only when source changes)
# =============================================================================

# ---------- Stage 1: System dependencies (cached) ----------
FROM ros:humble-ros-base AS deps

ENV DEBIAN_FRONTEND=noninteractive

# System + ROS packages (this layer is cached unless packages change)
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    # envsubst for CycloneDDS config templating
    gettext-base \
    # CycloneDDS RMW
    ros-humble-rmw-cyclonedds-cpp \
    # Core ROS2 packages
    ros-humble-pointcloud-to-laserscan \
    ros-humble-robot-state-publisher \
    ros-humble-tf2-ros \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-compressed-image-transport \
    # GUI
    ros-humble-rviz2 \
    # Teleop
    ros-humble-twist-mux \
    ros-humble-joy \
    ros-humble-teleop-twist-joy \
    # Navigation & SLAM
    ros-humble-slam-toolbox \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    # PCL for lidar_processor_cpp
    ros-humble-pcl-ros \
    ros-humble-pcl-conversions \
    libpcl-dev \
    # Audio for speech_processor
    ffmpeg \
    portaudio19-dev \
    && rm -rf /var/lib/apt/lists/*

# Foxglove bridge (not in apt for ARM64, install via pip)
RUN pip3 install --no-cache-dir foxglove-bridge 2>/dev/null || \
    echo 'WARN: foxglove-bridge pip install failed, skipping (optional)'

# Python deps for speech_processor and lidar_processor
RUN pip3 install --no-cache-dir requests pydub wasmtime open3d

# ---------- Stage 2: rosdep from manifests only (cached unless deps change) ----------
FROM deps AS rosdep

WORKDIR /ros2_ws

# Copy ONLY package manifests first — this layer is cached unless deps change
COPY unitree_go/package.xml             src/unitree_go/package.xml
COPY unitree_go/CMakeLists.txt          src/unitree_go/CMakeLists.txt
COPY unitree_api/package.xml            src/unitree_api/package.xml
COPY unitree_api/CMakeLists.txt         src/unitree_api/CMakeLists.txt
COPY go2_robot_sdk/package.xml          src/go2_robot_sdk/package.xml
COPY go2_robot_sdk/setup.py             src/go2_robot_sdk/setup.py
COPY go2_robot_sdk/setup.cfg            src/go2_robot_sdk/setup.cfg
COPY lidar_processor_cpp/package.xml    src/lidar_processor_cpp/package.xml
COPY lidar_processor_cpp/CMakeLists.txt src/lidar_processor_cpp/CMakeLists.txt
COPY speech_processor/package.xml       src/speech_processor/package.xml
COPY speech_processor/setup.py          src/speech_processor/setup.py
COPY speech_processor/setup.cfg         src/speech_processor/setup.cfg

RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    rosdep install --from-paths src --ignore-src -r -y 2>/dev/null || true

# ---------- Stage 3: Build (rebuilds only when source changes) ----------
FROM rosdep AS build

# Copy official unitree message packages (C++ build, changes rarely)
COPY unitree_go/ src/unitree_go/
COPY unitree_api/ src/unitree_api/

# Build unitree_go + unitree_api first (cached unless .msg files change)
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build --packages-select unitree_go unitree_api \
        --cmake-args -DCMAKE_BUILD_TYPE=Release

# Copy lidar_processor_cpp (C++)
COPY lidar_processor_cpp/ src/lidar_processor_cpp/

# Build lidar_processor_cpp
RUN echo "Bust Cache 1" && \
    . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build --packages-select lidar_processor_cpp \
        --cmake-args -DCMAKE_BUILD_TYPE=Release

# Copy speech_processor (Python, changes rarely)
COPY speech_processor/ src/speech_processor/

# Build speech_processor
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    . /ros2_ws/install/setup.sh && \
    colcon build --packages-select speech_processor

# Copy go2_robot_sdk (Python driver — most likely to change during dev)
COPY go2_robot_sdk/ src/go2_robot_sdk/

# Build go2_robot_sdk (fast Python install, only this rebuilds on driver changes)
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    . /ros2_ws/install/setup.sh && \
    colcon build --packages-select go2_robot_sdk --symlink-install

# Copy CycloneDDS config template (rendered at startup by entrypoint.sh)
COPY docker/cyclonedds.xml.template /ros2_ws/cyclonedds.xml.template

# Copy and setup entrypoint
COPY docker/entrypoint.sh /ros2_ws/entrypoint.sh
RUN chmod +x /ros2_ws/entrypoint.sh

# Source workspace in bashrc (for interactive shells)
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc && \
    echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /root/.bashrc && \
    echo "export CYCLONEDDS_URI=file:///ros2_ws/cyclonedds.xml" >> /root/.bashrc

# Environment
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV CYCLONEDDS_URI=file:///ros2_ws/cyclonedds.xml
ENV ROS_DOMAIN_ID=0

ENTRYPOINT ["/ros2_ws/entrypoint.sh"]
CMD ["ros2", "launch", "go2_robot_sdk", "robot.launch.py"]
