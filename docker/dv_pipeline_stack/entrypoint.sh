#!/bin/bash
set -e

source /opt/ros/humble/setup.bash

# Wipe Fast DDS' shared-memory droppings before anything else touches
# /dev/shm. `docker compose restart` keeps the tmpfs intact, and Fast
# DDS doesn't reliably reap its own segments when participants are
# stopped abruptly — every ros2 CLI invocation, foxglove restart, and
# rclpy script run leaves a fresh pair of segments behind. After a
# couple of dev iterations we'd see 10–20 stale files, and Fast DDS
# Participants in the new bridge would discover the ghosts and fail
# to negotiate large-message endpoints with the live publisher (the
# /lidar/Lidar1 PointCloud2 firehose specifically — small topics like
# /imu kept working). Symptom we chased: bridge publishing healthily
# at 10.7 Hz internally but external subscribers seeing 0 msgs;
# `docker compose down + up` was the only fix because it wiped the
# tmpfs as a side effect.
#
# Safe to nuke unconditionally: the bridge is the first DDS
# participant in this container; nothing legitimate could be using
# these files yet.
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true

# Optional rebuild step. With docker-compose bind-mounting the host's
# pipeline/ and ros2/src/ packages over the image's COPY'd baseline,
# Python edits go live via --symlink-install without any rebuild. But
# changes that need re-running colcon — new packages, setup.py edits,
# .msg regeneration, C++ source changes — require this. Off by default
# because it adds ~3–10s to startup; opt in via DV_REBUILD_ON_STARTUP=true
# in compose / shell env.
if [ "${DV_REBUILD_ON_STARTUP:-false}" = "true" ]; then
    echo "DV_REBUILD_ON_STARTUP=true — running colcon build before startup..."
    cd /dv_pipeline_stack_ws
    colcon build --symlink-install
    cd - >/dev/null
fi

source /dv_pipeline_stack_ws/install/setup.bash

# ament_cmake packages (fs_msgs, ifssim_bridge, dv_msgs) are not added
# to AMENT_PREFIX_PATH by the colcon-generated setup scripts — add
# them explicitly so the lifecycle launch's autonomy nodes can resolve
# the action / srv interfaces.
export AMENT_PREFIX_PATH="/dv_pipeline_stack_ws/install/fs_msgs:/dv_pipeline_stack_ws/install/ifssim_bridge:/dv_pipeline_stack_ws/install/dv_msgs:$AMENT_PREFIX_PATH"

echo "IFSSIM ROS stack starting — full lifecycle launch..."
echo "  Simulator: $IFSSIM_HOST:$IFSSIM_PORT"
echo "  Mission:   $MISSION_NAME / track $TRACK_NAME"

# Bring everything up in one launch (per docs/AUTONOMY.md
# §"Lifecycle orchestration mechanism"):
#
#   • bridge + foxglove_bridge  — always-active C++ Nodes
#   • robot_state_publisher + joint_state_publisher — coche_urdf TF tree
#   • mode_manager / mission_control / sim_supervisor — LifecycleNodes
#     auto-configured + activated by launch event handlers
#   • cone_detection / slam / path_planning / control — LifecycleNodes
#     parked in `unconfigured`, driven through the StartMission
#     action chain when the user clicks Start Session in Mission
#     Control
#
# Pre-#381 entrypoint.sh ran a polling loop watching
# /pipeline_ctrl/enable, forked pipeline_only.launch.py via setsid
# when the flag appeared, kill-TERM'd the process group when it
# went away. That mechanism existed because the pre-lifecycle
# autonomy nodes had no concept of `unconfigured`/`active` — the
# only way to "stop the autonomy" was to kill the processes. Post-
# feat/359 (lifecycle) + feat/360 (Phase 1 /odom) + feat/389 (event_start
# action-chain integration), the autonomy's runtime state is governed
# by the StartMission action chain, not by process existence. The
# flag-file is gone; entrypoint.sh just exec's the launch in the
# foreground and waits.
exec ros2 launch /dv_pipeline_stack_ws/pipeline.launch.py \
    host:=$IFSSIM_HOST \
    port:=$IFSSIM_PORT \
    mission_name:=$MISSION_NAME \
    track_name:=$TRACK_NAME
