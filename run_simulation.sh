#!/bin/bash

# Set the GZ_SIM_RESOURCE_PATH environment variable
export GZ_SIM_RESOURCE_PATH=/home/azakaria/Documents/sim_v2/ros2_ws/src/robot_description/models:home/azakaria/Documents/sim_v2/ros2_ws/src/robot_description/meshes:/home/azakaria/Documents/sim_v2/ros2_ws/src/robot_description/urdf:/ros2_ws/src/robot_description/worlds:/ros2_ws/src

# Build the workspace
colcon build

# Source the setup script
source install/setup.bash

# Launch the project
#ros2 launch robot_description project.launch.py
ros2 launch robot_description project_world.launch.py