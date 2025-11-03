#!/bin/bash

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

show_usage() {
    echo "Usage: $0 [OPTION] [SERVICE] [SIM_ARGS]"
    echo ""
    echo "Run Isaac Sim services using docker compose"
    echo ""
    echo "Options:"
    echo "  -h, --help     Show this help message"
    echo "  -d, --daemon   Run in daemon mode (detached)"
    echo "  -b, --build    Build images before starting"
    echo "  --no-cache     Build images with no cache"
    echo "  -x, --fix-x11  Setup X11 authentication before starting (default)"
    echo "  --no-x11       Skip X11 setup"
    echo ""
    echo "Services:"
    echo "  sim            Run ur5_sim service"
    echo "  dev            Run ur5_sim-dev service"
    echo "  both           Run both services"
    echo ""
    echo "Examples:"
    echo "  $0 sim                 # Run sim service with default settings"
    echo "  $0 sim --no-ui         # Run sim headless"
    echo "  $0 sim --environment single_beverage_shelf"
    echo "  $0 dev                 # Run dev service"
    echo "  $0 both                # Run both services"
    echo "  $0 -d sim              # Run sim in daemon mode"
    echo "  $0 -b dev              # Build and run dev service"
}

DAEMON_MODE=false
BUILD_MODE=false
NO_CACHE=false
FIX_X11=true
SERVICE=""
SIM_ARGS=""

while [[ $# -gt 0 ]]; do
    case $1 in
        -h|--help)
            show_usage
            exit 0
            ;;
        -d|--daemon)
            DAEMON_MODE=true
            shift
            ;;
        -b|--build)
            BUILD_MODE=true
            shift
            ;;
        --no-cache)
            NO_CACHE=true
            BUILD_MODE=true
            shift
            ;;
        -x|--fix-x11)
            FIX_X11=true
            shift
            ;;
        --no-x11)
            FIX_X11=false
            shift
            ;;
        sim|dev|both)
            if [[ -n "$SERVICE" ]]; then
                echo "Error: Only one service can be specified"
                exit 1
            fi
            SERVICE="$1"
            shift
            # Collect remaining arguments as simulation parameters
            while [[ $# -gt 0 ]]; do
                SIM_ARGS="$SIM_ARGS $1"
                shift
            done
            ;;
        *)
            echo "Unknown option: $1"
            show_usage
            exit 1
            ;;
    esac
done

if [[ -z "$SERVICE" ]]; then
    echo "Error: Service must be specified"
    show_usage
    exit 1
fi

COMPOSE_ARGS=""
if [[ "$DAEMON_MODE" == true ]]; then
    COMPOSE_ARGS="$COMPOSE_ARGS -d"
fi

check_nvidia_toolkit() {
    # Check if nvidia-ctk command exists
    if ! command -v nvidia-ctk &> /dev/null; then
        echo "NVIDIA Container Toolkit not found."
        echo "Installing NVIDIA Container Toolkit..."

        if [[ -f "./install_nvidia_container_toolkit.sh" ]]; then
            ./install_nvidia_container_toolkit.sh
            if [[ $? -ne 0 ]]; then
                echo "Error: Failed to install NVIDIA Container Toolkit"
                exit 1
            fi
        else
            echo "Error: install_nvidia_container_toolkit.sh not found"
            echo "Please install NVIDIA Container Toolkit manually or ensure the install script is present"
            exit 1
        fi
    else
        echo "NVIDIA Container Toolkit is installed"
    fi
}

check_git_lfs() {
    # Check if git-lfs is installed
    if ! command -v git-lfs &> /dev/null; then
        echo "Git LFS not found. Installing..."
        sudo apt update && sudo apt install -y git-lfs
        if [[ $? -ne 0 ]]; then
            echo "Error: Failed to install Git LFS"
            exit 1
        fi
        echo "Git LFS installed successfully"
    else
        echo "Git LFS is already installed"
    fi

    # Check if LFS has been pulled before (check if .git/lfs/objects has content)
    if [[ ! -d ".git/lfs/objects" ]] || [[ -z "$(ls -A .git/lfs/objects 2>/dev/null)" ]]; then
        echo "Pulling Git LFS files..."
        git lfs pull
        if [[ $? -ne 0 ]]; then
            echo "Warning: Failed to pull Git LFS files"
        else
            echo "Git LFS files pulled successfully"
        fi
    else
        echo "Git LFS files already pulled"
    fi
}

setup_cache_directories() {
    echo "Setting up Isaac Sim cache directories..."

    # Define cache directories
    CACHE_DIRS=(
        "$HOME/docker/isaac-sim/cache/kit"
        "$HOME/docker/isaac-sim/cache/ov"
        "$HOME/docker/isaac-sim/cache/pip"
        "$HOME/docker/isaac-sim/cache/warp"
        "$HOME/docker/isaac-sim/cache/glcache"
        "$HOME/docker/isaac-sim/cache/computecache"
        "$HOME/docker/isaac-sim/cache/asset_browser"
        "$HOME/docker/isaac-sim/logs"
        "$HOME/docker/isaac-sim/data"
        "$HOME/docker/isaac-sim/documents"
        "$HOME/docker/isaac-sim/pkg"
    )

    # Create directories and fix ownership if needed
    for dir in "${CACHE_DIRS[@]}"; do
        if [[ ! -d "$dir" ]]; then
            mkdir -p "$dir" 2>/dev/null || {
                echo "Creating $dir with sudo..."
                sudo mkdir -p "$dir"
                sudo chown -R $USER:$USER "$dir"
            }
        else
            # Check if directory is owned by root and fix it
            if [[ $(stat -c '%U' "$dir") == "root" ]]; then
                echo "Fixing ownership of $dir..."
                sudo chown -R $USER:$USER "$dir"
            fi
        fi
    done

    echo "Cache directories setup complete"
}

# Check NVIDIA Container Toolkit before proceeding
check_nvidia_toolkit

# Check Git LFS before proceeding
check_git_lfs

# Setup cache directories before proceeding
setup_cache_directories

if [[ "$FIX_X11" == true ]]; then
    if [[ -f "./fix_x11.sh" ]]; then
        echo "Running X11 setup..."
        ./fix_x11.sh
    else
        echo "Warning: fix_x11.sh not found, skipping X11 setup"
    fi
fi

if [[ "$BUILD_MODE" == true ]]; then
    if [[ "$NO_CACHE" == true ]]; then
        echo "Building images with no cache..."
        docker compose build --no-cache
    else
        echo "Building images..."
        docker compose build
    fi
fi

RUN_ARGS="--rm"
if [[ "$BUILD_MODE" == true ]]; then
    RUN_ARGS="$RUN_ARGS --build --remove-orphans"
fi

case "$SERVICE" in
    sim)
        echo "Starting ur5_sim service..."
        if [[ -n "$SIM_ARGS" ]]; then
            echo "Simulation arguments: $SIM_ARGS"
        fi
        if [[ "$DAEMON_MODE" == true ]]; then
            SIM_ARGS="$SIM_ARGS" docker compose up -d ur5_sim
        else
            SIM_ARGS="$SIM_ARGS" docker compose run $RUN_ARGS ur5_sim
        fi
        ;;
    dev)
        echo "Starting ur5_sim-dev service..."
        if [[ "$DAEMON_MODE" == true ]]; then
            docker compose up -d ur5_sim-dev
        else
            docker compose run $RUN_ARGS ur5_sim-dev
        fi
        ;;
    both)
        echo "Starting both services..."
        if [[ -n "$SIM_ARGS" ]]; then
            echo "Simulation arguments: $SIM_ARGS"
            SIM_ARGS="$SIM_ARGS" docker compose up $COMPOSE_ARGS ur5_sim ur5_sim-dev
        else
            docker compose up $COMPOSE_ARGS ur5_sim ur5_sim-dev
        fi
        ;;
esac

if [[ "$DAEMON_MODE" == true ]]; then
    echo "Services started in daemon mode"
    echo "To view logs: docker compose logs -f [service-name]"
    echo "To stop services: docker compose down"
fi
