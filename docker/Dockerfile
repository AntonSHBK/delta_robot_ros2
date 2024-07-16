# FROM ros:jazzy
FROM osrf/ros:humble-desktop-full
# FROM ros:jazzy

# **************************************************************************** #
# set environment variables
ENV PYTHONDONTWRITEBYTECODE 1
ENV PYTHONUNBUFFERED 1

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

# **************************************************************************** #

# Development tools
RUN apt-get update && apt-get install -y \
    cmake \
    build-essential \
    curl \
    wget \
    zsh \
    vim \
    git \
    tmux \
    tree \
    && rm -rf /var/lib/apt/lists/*

# **************************************************************************** #

# Gazebo tools
RUN apt-get update && apt-get install -y \
    wget \
    gnupg \
    lsb-release \
    && rm -rf /var/lib/apt/lists/*

RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

RUN apt-get update && apt-get install -y \
    ignition-fortress \
    ros-humble-desktop-full \
    ros-humble-simulation \
    ros-humble-joint-state-publisher \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-control-toolbox \
    ros-humble-realtime-tools \
    ros-humble-ros-gz \
    ros-humble-xacro \
    ros-humble-moveit \
    && rm -rf /var/lib/apt/lists/*

RUN echo "source /usr/share/ignition/fortress/setup.sh" >> ~/.bashrc

# **************************************************************************** #

# Install Python 3 and pip
RUN apt-get update && apt-get install -y \
    python3 \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# install python dependencies
# COPY ../requirements.txt /tmp/requirements.txt

# RUN pip install --upgrade pip --break-system-packages && \
#     pip install -r /tmp/requirements.txt --break-system-packages && \
#     rm -rf /root/.cache/pip

# # **************************************************************************** #

# set work directory
ENV WORKSPACE /workspace/src
RUN mkdir -p $WORKSPACE
WORKDIR $WORKSPACE

# copy project
COPY . $WORKSPACE

# **************************************************************************** #

# Source the ROS 2 setup file for the entry point
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "alias sros='source install/setup.bash'" >> ~/.bashrc