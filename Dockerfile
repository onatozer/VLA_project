# FROM ros:humble-ros-base
FROM osrf/ros:humble-desktop

# ROS library come with python, but not the venv/pip libraries
RUN apt-get update && apt-get install -y \
    python3-venv \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# More robust/clean way of activating the virtual environment, as suggested in (https://pythonspeed.com/articles/activate-virtualenv-dockerfile/)
ENV VIRTUAL_ENV=/opt/venv
RUN python3 -m venv --system-site-packages $VIRTUAL_ENV 
ENV PATH="$VIRTUAL_ENV/bin:$PATH"

COPY octo-requirements.txt workspace/
RUN pip install --no-cache-dir -r workspace/octo-requirements.txt
COPY octo/ workspace/octo
RUN pip install --no-cache-dir workspace/octo/.

RUN apt-get update && apt-get install -y \
    ros-humble-kortex-bringup \ 
    ros-humble-ros-gz-sim \
    ros-humble-ros-gz-bridge \
    ros-humble-ros-gz-interfaces\
    ros-humble-ign-ros2-control \
    xvfb \ 
    && rm -rf /var/lib/apt/lists/*


ENTRYPOINT ["/bin/bash"]