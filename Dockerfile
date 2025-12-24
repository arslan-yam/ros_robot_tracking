FROM osrf/ros:jazzy-ros-base

SHELL ["/bin/bash", "-c"]

# build + python
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-numpy \
    python3-opencv \
    git \
 && rm -rf /var/lib/apt/lists/*

# rosdep init/update
RUN rosdep init 2>/dev/null || true && rosdep update

# Workspace
WORKDIR /ws

# copy workspace sources
COPY src/ /ws/src/

# install ROS deps from package.xml, then build
RUN source /opt/ros/jazzy/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build --symlink-install

# run command: launch your app
CMD source /opt/ros/jazzy/setup.bash && \
    source /ws/install/setup.bash && \
    ros2 launch robot_marker_tracking tracking.launch.py
