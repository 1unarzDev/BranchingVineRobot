#!/usr/bin/env bash

# Make script non-interactive (automatically answer prompts)
export DEBIAN_FRONTEND=noninteractive

# Update apt packages
echo "Updating apt packages..."
sudo apt update && sudo apt upgrade -y

# Install all necessary Debian/ROS packages
sudo apt -y install \
ros-humble-librealsense2* \
ros-humble-gazebo-ros-pkgs

# Install Python dependencies as defined in the requirements.txt
if [ -f "requirements.txt" ]; then
    echo "Installing Python dependencies from requirements.txt..."
    python3 -m pip install --upgrade pip
    python3 -m pip install -r requirements.txt
else
    echo "requirements.txt not found. Skipping Python dependencies installation."
fi