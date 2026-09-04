#!/bin/bash
# Launch the IFSSIM ROS2 bridge
# Usage: ./launch_bridge.sh [host] [port]

WS=/Users/raulmoran/Documents/Github/IFSSIM/ros2/install

source /Users/raulmoran/miniforge3/etc/profile.d/conda.sh
conda activate ifssim

export AMENT_PREFIX_PATH="$WS/ifssim_bridge:$WS/fs_msgs:$AMENT_PREFIX_PATH"
export DYLD_LIBRARY_PATH="$WS/ifssim_bridge/lib:$WS/fs_msgs/lib:$DYLD_LIBRARY_PATH"

HOST=${1:-localhost}
PORT=${2:-41451}

exec ros2 launch ifssim_bridge ifssim_bridge.launch.py host:=$HOST port:=$PORT
