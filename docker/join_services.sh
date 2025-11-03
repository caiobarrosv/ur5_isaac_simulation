#!/bin/bash

set -e

# Get the directory where this script is located (docker/)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

show_usage() {
    echo "Usage: $0 [OPTIONS] SERVICE"
    echo ""
    echo "Attach to running Isaac Sim services launched by run_services.sh"
    echo ""
    echo "Note: Run this script from the docker/ directory"
    echo ""
    echo "Options:"
    echo "  -h, --help     Show this help message"
    echo "  -s, --shell    Shell to use (default: bash)"
    echo ""
    echo "Services:"
    echo "  sim            Attach to ur5_isaac_sim service"
    echo "  dev            Attach to ur5_isaac_sim-dev service"
    echo ""
    echo "Examples:"
    echo "  ./join_services.sh sim                 # Attach to sim service with bash"
    echo "  ./join_services.sh dev                 # Attach to dev service with bash"
    echo "  ./join_services.sh -s zsh sim          # Attach to sim service with zsh"
}

SHELL_CMD="bash"
SERVICE=""

while [[ $# -gt 0 ]]; do
    case $1 in
        -h|--help)
            show_usage
            exit 0
            ;;
        -s|--shell)
            SHELL_CMD="$2"
            shift 2
            ;;
        sim|dev)
            if [[ -n "$SERVICE" ]]; then
                echo "Error: Only one service can be specified"
                exit 1
            fi
            SERVICE="$1"
            shift
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

check_service_running() {
    local service_name="$1"
    if ! docker compose ps --services --filter "status=running" | grep -q "^${service_name}$"; then
        echo "Error: Service '$service_name' is not running"
        echo "Start it first with: ./run_services.sh $SERVICE"
        exit 1
    fi
}

case "$SERVICE" in
    sim)
        SERVICE_NAME="ur5_sim"
        ;;
    dev)
        SERVICE_NAME="ur5_sim-dev"
        ;;
esac

echo "Checking if $SERVICE_NAME service is running..."
check_service_running "$SERVICE_NAME"

echo "Attaching to $SERVICE_NAME service with $SHELL_CMD..."
docker compose exec "$SERVICE_NAME" "$SHELL_CMD"
