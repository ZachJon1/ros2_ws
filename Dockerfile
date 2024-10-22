FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ARG USERNAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN if ! id -u $USER_UID >/dev/null 2>&1; then \
        groupadd --gid $USER_GID $USERNAME && \
        useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME; \
    fi
# Add sudo support for the non-root user
RUN apt-get update && \
    apt-get install -y sudo && \
    echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME

# Switch from root to user
USER $USERNAME

# Add user to video group to allow access to webcam
RUN sudo usermod --append --groups video $USERNAME

# Update all packages
RUN sudo apt update && sudo apt upgrade -y

# Install Git
RUN sudo apt install -y git

# Rosdep update
RUN rosdep update
# ROS2
# Create a workspace
WORKDIR /ros2/src

# Gazebo Fortress
# Set the working directory
WORKDIR /root

RUN sudo apt-get install  -y ros-${ROS_DISTRO}-ros-gz


RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
RUN echo "source /ros2_humble_ws/install/setup.bash" >> ~/.bashrc

# Set Gazebo models path environment variable
# see https://gazebosim.org/api/sim/8/resources.html
# Modify here to add your own models path
RUN echo "export GZ_SIM_RESOURCE_PATH=/ros2_ws/src/robot_description/models:/ros2_ws/src/robot_description/meshes:/home/azakaria/Documents/sim_v2/ros2_ws/src/robot_description/urdf:/ros2_ws/src/robot_description/worlds:/ros2_ws/src" >> ~/.bashrc

# An alias to source the bashrc file
RUN  echo "alias sb='source ~/.bashrc'" >> ~/.bashrc

WORKDIR /ros2_ws
COPY src/robot_description ./src