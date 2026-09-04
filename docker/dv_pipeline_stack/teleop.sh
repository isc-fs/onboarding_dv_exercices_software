#!/bin/bash
source /opt/ros/humble/setup.bash
source /dv_pipeline_stack_ws/install/setup.bash
export AMENT_PREFIX_PATH="/dv_pipeline_stack_ws/install/fs_msgs:/dv_pipeline_stack_ws/install/ifssim_bridge:$AMENT_PREFIX_PATH"
exec python3 /dv_pipeline_stack_ws/teleop_keyboard.py
