# FROM ros:humble-ros-base
FROM osrf/ros:humble-desktop


RUN apt-get update && apt-get install -y \
    ros-humble-kortex-bringup \ 
    ros-humble-ros-gz-sim \
    ros-humble-ros-gz-bridge \
    ros-humble-ros-gz-interfaces\
    ros-humble-ign-ros2-control \
    xvfb \ 
    && rm -rf /var/lib/apt/lists/*

ENTRYPOINT ["/bin/bash"]