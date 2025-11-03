#!/bin/bash

set -e

echo "Setting up ROS2 environment..."
source /opt/ros/humble/setup.bash

export LD_LIBRARY_PATH=/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/lib

echo "Environment configured:"
echo "  ROS_DISTRO: $ROS_DISTRO"
echo "  RMW_IMPLEMENTATION: $RMW_IMPLEMENTATION"
echo "  ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "  LD_LIBRARY_PATH: $LD_LIBRARY_PATH"
echo "  DISPLAY: $DISPLAY"

echo "X11 Display check:"
if [ -n "$DISPLAY" ]; then
    echo "  DISPLAY is set to: $DISPLAY"
else
    echo "  WARNING: DISPLAY variable not set"
fi

# Ensure ROS is sourced in interactive shells
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

cd /workspace
