# ROS 2 Humble + Gazebo Fortress (ros_gz) + RViz2 for the CPS lab robot models.
FROM osrf/ros:humble-desktop

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-ros-gz \
        ros-humble-xacro \
        ros-humble-robot-state-publisher \
        ros-humble-joint-state-publisher \
        ros-humble-joint-state-publisher-gui \
        python3-yaml \
        python3-colcon-common-extensions \
        mesa-utils \
        xvfb imagemagick x11-utils procps && \
    rm -rf /var/lib/apt/lists/*

# workspace: the repo is bind-mounted at /ws/src/All_Robot by docker/compose.yaml
WORKDIR /ws
COPY docker/ros_gz/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
