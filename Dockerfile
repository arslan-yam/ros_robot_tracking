FROM ubuntu:24.04

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# Basic OS tools
RUN apt-get update && apt-get install -y \
    locales \
    curl \
    gnupg \
    lsb-release \
    software-properties-common \
 && locale-gen en_US en_US.UTF-8 \
 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
 && rm -rf /var/lib/apt/lists/*

ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# ROS 2 repository
RUN add-apt-repository universe && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" \
      > /etc/apt/sources.list.d/ros2.list

# Install ROS 2 Jazzy + cv_bridge + build tools
RUN apt-get update && apt-get install -y \
    ros-jazzy-ros-base \
    ros-jazzy-cv-bridge \
    ros-jazzy-nav-msgs \
    ros-jazzy-sensor-msgs \
    ros-jazzy-geometry-msgs \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-numpy \
    python3-opencv \
    git \
 && rm -rf /var/lib/apt/lists/*

# rosdep
RUN rosdep init 2>/dev/null || true && rosdep update

# Workspace
WORKDIR /ws
COPY src/ /ws/src/

# Install deps from package.xml and build
RUN source /opt/ros/jazzy/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build --symlink-install

# Run your node by default
CMD ["bash", "-lc", "source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash && ros2 run robot_marker_tracking marker_tracker_node"]