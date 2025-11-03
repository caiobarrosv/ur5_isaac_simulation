#!/bin/bash
# Fix X11 authentication for Isaac Sim container

echo "Fixing X11 authentication for Isaac Sim..."

# Get the current display
CURRENT_DISPLAY=${DISPLAY:-:0}
echo "Current DISPLAY: $CURRENT_DISPLAY"

# Create .Xauthority file if it doesn't exist
if [ ! -f ~/.Xauthority ]; then
    echo "Creating .Xauthority file..."
    touch ~/.Xauthority
    chmod 600 ~/.Xauthority
fi

# Allow local connections temporarily (less secure but works)
echo "Allowing local X11 connections..."
xhost +local:root
xhost +local:docker

# Give some permissions to the X11 socket for all displays
echo "Setting X11 socket permissions..."
for socket in /tmp/.X11-unix/X*; do
    if [ -e "$socket" ]; then
        echo "Setting permissions for $socket"
        sudo chmod 666 "$socket" 2>/dev/null || true
    fi
done

echo "X11 setup complete!"
echo "DISPLAY is set to: $CURRENT_DISPLAY"
