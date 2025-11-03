#!/bin/bash

set -e

echo "Installing NVIDIA Container Toolkit..."

# Check if NVIDIA drivers are installed
if ! command -v nvidia-smi &> /dev/null; then
    echo "Error: NVIDIA drivers not found. Please install NVIDIA drivers first."
    exit 1
fi

echo "NVIDIA drivers found:"
nvidia-smi --query-gpu=name,driver_version --format=csv,noheader

# Add NVIDIA repository
echo "Adding NVIDIA Container Toolkit repository..."
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg

curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

# Update package list
echo "Updating package list..."
sudo apt-get update

# Install NVIDIA Container Toolkit (latest version)
echo "Installing NVIDIA Container Toolkit..."
sudo apt-get install -y nvidia-container-toolkit

# Configure Docker to use NVIDIA runtime
echo "Configuring Docker runtime..."
sudo nvidia-ctk runtime configure --runtime=docker

# Restart Docker service
echo "Restarting Docker service..."
sudo systemctl restart docker

# Wait for Docker to fully restart
sleep 5

# Test NVIDIA Docker integration
echo "Testing NVIDIA Docker integration..."
if sudo docker run --rm --gpus all ubuntu:20.04 nvidia-smi; then
    echo "✅ NVIDIA Container Toolkit installed and configured successfully!"
    echo ""
    echo "You can now run GPU-enabled containers with:"
    echo "  docker run --gpus all <image>"
    echo "  docker compose up (with GPU configuration in compose file)"
else
    echo "❌ Installation completed but GPU test failed."
    echo "You may need to reboot your system or check your Docker configuration."
fi
