FROM ubuntu:20.04

# Install required packages for poetry, sudo and non-root user creation
RUN apt-get update && apt-get install -y    \
    curl                                    \
    sudo                                    \
    software-properties-common              \
    locales                                 \
    tmux                                    \
    && rm -rf /var/lib/apt/lists/*

# Set the locale
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# set the timezone
RUN ln -sf /usr/share/zoneinfo/Asia/Seoul /etc/localtime
RUN ln -sf /usr/share/zoneinfo/GMT /etc/localtime

# Create a non-root user 'vscode' and add to sudoers group
RUN useradd -m vscode && echo "vscode ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/nopasswd && chmod 0440 /etc/sudoers.d/nopasswd
RUN chsh -s /bin/bash vscode
USER vscode

# ensure the Ubuntu Universe repository is enabled
RUN sudo add-apt-repository universe

# Add the ROS 2 GPG key wit apt
RUN sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add the ROS 2 repository to sources list
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN sudo apt update && sudo apt upgrade -y && sudo rm -rf /var/lib/apt/lists/*

# Install dependencies for f1tenth gym
RUN sudo apt update && sudo apt install -y  \
    ros-foxy-desktop                        \
    python3-argcomplete                     \
    ros-dev-tools                           \
    ros-foxy-ackermann-msgs                 \
    ros-foxy-joint-state-publisher          \
    ros-foxy-xacro                          \
    ros-foxy-nav2-map-server                \
    ros-foxy-nav2-lifecycle-manager         \
    python3-pip                             \
    && sudo rm -rf /var/lib/apt/lists/*

# Install ros dependencies for devices
RUN sudo apt update && sudo apt install -y  \
    ros-foxy-laser-proc                     \
    ros-foxy-serial-driver                  \
    ros-foxy-diagnostic-updater             \
    && sudo rm -rf /var/lib/apt/lists/*

RUN sudo rosdep init
RUN rosdep update

# set bashrc
RUN echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc

# Set the default working directory
WORKDIR /workspace