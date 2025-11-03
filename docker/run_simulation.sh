#!/bin/bash
set -e

# UR5 Isaac Sim Docker Simulation Launcher
# This script runs the Docker container and launches Isaac Sim with the UR5 scene

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "=========================================="
echo "UR5 Isaac Sim Docker Launcher"
echo "=========================================="

# Setup X11 for GUI
echo "Setting up X11 display..."
xhost +local:docker 2>/dev/null || echo "Warning: xhost command failed"

# Create cache directories if they don't exist
mkdir -p ~/docker/isaac-sim/cache ~/docker/isaac-sim/logs ~/docker/isaac-sim/data ~/docker/isaac-sim/documents ~/docker/isaac-sim/pkg

# Docker image name
IMAGE_NAME="docker-ur5_sim:latest"

echo "Starting Docker container with UR5 simulation..."
echo "Isaac Sim will be loaded from container installation at /home/user/isaacsim"

# Run the container directly
docker run --rm \
    --name ur5_sim_session \
    --runtime=nvidia \
    --gpus all \
    --network host \
    --privileged \
    --ipc host \
    --pid host \
    -e DISPLAY="${DISPLAY}" \
    -e QT_X11_NO_MITSHM=1 \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -e ROS_DOMAIN_ID=0 \
    -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    -e OMNI_KIT_ALLOW_ROOT=1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v ~/docker/isaac-sim/cache:/home/user/.cache:rw \
    -v ~/docker/isaac-sim/cache:/home/user/.nv:rw \
    -v ~/docker/isaac-sim/logs:/home/user/.nvidia-omniverse/logs:rw \
    -v ~/docker/isaac-sim/data:/home/user/.local/share/ov/data:rw \
    -v ~/docker/isaac-sim/documents:/home/user/Documents:rw \
    -v ~/docker/isaac-sim/pkg:/home/user/.local/share/ov/pkg:rw \
    -v /dev/shm:/dev/shm \
    -u user \
    "$IMAGE_NAME" \
    bash -c '
set -e

echo "=========================================="
echo "UR5 Simulation Container Started"
echo "=========================================="

# Set environment variables for Isaac Sim with internal ROS2 bridge
# IMPORTANT: Must be set BEFORE starting Isaac Sim
export ROS_DISTRO=humble
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export OMNI_KIT_ALLOW_ROOT=1

# Use Isaac Sim internal ROS2 libraries (Python 3.11 compatible)
# These paths match what Isaac Sim expects for internal rclpy
export LD_LIBRARY_PATH=/home/user/isaacsim/exts/isaacsim.ros2.bridge/humble/lib:$LD_LIBRARY_PATH
export PYTHONPATH=/home/user/isaacsim/exts/isaacsim.ros2.bridge/humble/rclpy:$PYTHONPATH

# Source system ROS2 for additional tools and message definitions
source /opt/ros/humble/setup.bash 2>/dev/null || echo "ROS2 not sourced"
source /workspace/ur5_ws/install/setup.bash 2>/dev/null || echo "Workspace not built yet"

# Override PYTHONPATH again after sourcing to ensure Isaac Sim rclpy is prioritized
export PYTHONPATH=/home/user/isaacsim/exts/isaacsim.ros2.bridge/humble/rclpy:$PYTHONPATH

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
echo ""

# Launch Isaac Sim with the UR5 scene
cd /home/user/isaacsim
exec ./isaac-sim.sh "$USD_SCENE"
'
