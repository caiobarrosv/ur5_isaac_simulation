#!/bin/bash
set -e

# Internal script that runs inside the container to launch Isaac Sim

echo "=========================================="
echo "UR5 Simulation Container Started"
echo "=========================================="

# Source ROS2 environment
source /opt/ros/humble/setup.bash
source /workspace/ur5_ws/install/setup.bash 2>/dev/null || echo "Workspace not built yet"

# Set environment variables
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export OMNI_KIT_ALLOW_ROOT=1
export LD_LIBRARY_PATH=/home/user/isaacsim/exts/isaacsim.ros2.bridge/humble/lib:$LD_LIBRARY_PATH

# USD scene path
USD_SCENE="/workspace/ur5_ws/src/ur5_isaac_simulation/usd/ur5_isaac_sim.usd"

echo ""
echo "Configuration:"
echo "  Isaac Sim: /home/user/isaacsim (container installation)"
echo "  USD Scene: $USD_SCENE"
echo "  ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "  DISPLAY: $DISPLAY"
echo ""

# Check if Isaac Sim exists in container
if [ ! -d "/home/user/isaacsim" ]; then
    echo "ERROR: Isaac Sim not found in container at /home/user/isaacsim"
    echo "The container should have Isaac Sim installed during build"
    exit 1
fi

# Check if USD file exists
if [ ! -f "$USD_SCENE" ]; then
    echo "ERROR: USD file not found at $USD_SCENE"
    exit 1
fi

echo "Launching Isaac Sim with UR5 scene..."
echo "=========================================="

# Launch Isaac Sim with the UR5 scene
cd /home/user/isaacsim
exec ./isaac-sim.sh "$USD_SCENE" "$@"
