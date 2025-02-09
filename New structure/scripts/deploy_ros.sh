#!/bin/bash
# Build and deploy the ROS package
echo "Building ROS package..."
colcon build --packages-select slm_ros
echo "Sourcing ROS workspace..."
source install/setup.bash
echo "Launching ROS nodes..."
ros2 launch slm_ros launch_file.launch.py
