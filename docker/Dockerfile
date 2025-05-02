# Start from official ROS 2 Humble desktop image
FROM osrf/ros:humble-desktop

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8
ENV TZ=UTC

# Install prerequisites for Gazebo package source
RUN apt-get update && apt-get install -y \
    wget \
    lsb-release \
    && rm -rf /var/lib/apt/lists/*

# Add OSRF Gazebo package source (alternative installation process)
RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list' \
    && wget https://packages.osrfoundation.org/gazebo.key -O - | apt-key add -

# Install Gazebo 11 and ROS-Gazebo integration
RUN apt-get update && apt-get install -y \
    gazebo \
    libgazebo-dev \
    ros-humble-gazebo-ros-pkgs \
    && rm -rf /var/lib/apt/lists/*

# Install development tools
RUN apt-get update && apt-get install -y \
    build-essential \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    git \
    nano \
    && rosdep update --rosdistro $ROS_DISTRO \
    && rm -rf /var/lib/apt/lists/*

# Install X11 for GUI applications
RUN apt-get update && apt-get install -y \
    x11-apps \
    && rm -rf /var/lib/apt/lists/*


RUN apt-get update && apt-get install -y \
    libx11-xcb1 \
    libxcb-xfixes0 \
    libqt5x11extras5 \
    qtbase5-dev \
    && rm -rf /var/lib/apt/lists/*

# Create a non-root user
ARG USERNAME=rosdev
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Create ROS 2 workspace
USER $USERNAME
WORKDIR /home/$USERNAME/workspace
RUN mkdir -p src

# Source ROS 2 and Gazebo in .bashrc
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/$USERNAME/.bashrc \
    && echo "source /usr/share/gazebo-11/setup.sh" >> /home/$USERNAME/.bashrc 
COPY docker/entrypoint.sh /entrypoint.sh
RUN sudo chmod +x /entrypoint.sh


# Entrypoint and default command
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

