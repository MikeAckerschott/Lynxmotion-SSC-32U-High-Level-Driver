#!/bin/bash

# This is a simple script to compile the project for the first time. 
# It will first build the custom srv and msg files, then build the project.

# Build custom srv and msg files
colcon build --packages-select msg_srv

# Build the project
colcon build --packages-select robo_driver

# Print message to user
echo "Project built for the first time. Remember to source the workspace before running the project. (. install/setup.bash)"