# Mission Control backend (FastAPI + uvicorn) — ROS 2 base.
#
# History: this used to be a vanilla python:3.12-slim image. Step 5 of
# the DV-pipeline-alignment series turned the backend into an action
# client for sim_supervisor.StartMission, which means it needs rclpy +
# DDS + the codegen'd dv_msgs/fs_msgs interfaces. rclpy ships only as
# part of a ROS 2 install, so the base flips to ros:humble-ros-base.
#
# Trade-offs accepted with this switch:
#   • Python pinned to 3.10 (Humble's interpreter) instead of 3.12.
#     FastAPI / uvicorn / pytest all support 3.10 with no observable
#     downside.
#   • Image grows from ~120 MB → ~700 MB. Worth it for a real DDS
#     client; the alternative (out-of-band ros2 CLI subprocess) has
#     worse failure modes.

FROM ros:humble-ros-base

# ---------------------------------------------------------------------
# System deps: pip + colcon (we build dv_msgs / fs_msgs in-image so the
# generated Python bindings sit in the install tree this container
# sources).
# ---------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-pip \
        python3-colcon-common-extensions \
        ros-humble-rosidl-default-generators \
    && rm -rf /var/lib/apt/lists/*
# `docker` Python SDK (#498) — backend talks directly to the host
# docker daemon over /var/run/docker.sock to copy a finalised bag out
# of the dv_pipeline_stack container into the host-bind-mounted
# `/host_bags/` directory. We use the SDK (HTTP API) rather than the
# CLI because Linux `docker cp` from inside a container hits a
# colon-parsing failure on Windows host paths (`C:/Users/...` is
# parsed as container `C` + path `/Users/...`) and the daemon
# doesn't recognise `/c/Users/...` as a host mount. The SDK writes
# the tarball directly to mc_backend's filesystem (bind-mounted
# `/host_bags/` → host `./bags/`), so cross-filesystem path
# translation isn't involved.
RUN pip3 install --no-cache-dir "docker>=7,<8"
# Note: ros-humble-ros2bag / rosbag2-transport / rosbag2-storage-mcap
# are NOT installed here — bag recording was moved to dv_pipeline_stack
# in #465 v2 (the recorder needs to share the publisher's SHM-tuned
# DDS context to capture full-fidelity LiDAR). mc_backend is now a
# pure service client; the recorder bits live in
# pipeline/bag_recorder_node/.

# ---------------------------------------------------------------------
# Build dv_msgs + fs_msgs in their own little workspace under /msgs_ws.
# Keeping it separate from /app means a backend source edit doesn't
# touch the install tree (no risky rebuild), and conversely an
# interface change shows up as a clean rebuild step.
# ---------------------------------------------------------------------
WORKDIR /msgs_ws
RUN mkdir -p src
COPY pipeline/dv_msgs/   src/dv_msgs/
COPY pipeline/fs_msgs/   src/fs_msgs/
RUN . /opt/ros/humble/setup.sh \
    && colcon build --packages-select fs_msgs dv_msgs \
                    --cmake-args -DCMAKE_BUILD_TYPE=Release

# ---------------------------------------------------------------------
# Backend: ifssim Python client + pinned requirements + source.
# ---------------------------------------------------------------------
WORKDIR /app
COPY python/ /ifssim_client/
RUN pip3 install --no-cache-dir /ifssim_client

COPY tools/mission_control/backend/requirements.txt .
RUN pip3 install --no-cache-dir -r requirements.txt

COPY tools/mission_control/backend/ .

# Random-track-generator submodule, baked into the image at /track_gen
# (matches TRACK_GEN_PATH in docker-compose.yml). Used to be a
# read-only bind mount; #490 moved it in to cut one more 9p/virtiofs
# hit on Windows + macOS. Updating the submodule (rare — once a
# quarter at most) now requires a `docker compose build
# mission_control_backend`.
COPY tools/random-track-generator/ /track_gen/

ENV IFSSIM_HOST=host.docker.internal
ENV IFSSIM_PORT=41451

EXPOSE 8000

# Source ROS + the locally-built msgs workspace before launching uvicorn,
# otherwise `import rclpy` and `from dv_msgs.action import StartMission`
# both fail at backend startup. /bin/bash -lc gives us a login shell so
# /opt/ros/humble/setup.bash and /msgs_ws/install/setup.bash run as
# expected; the inner `exec` hands PID 1 to uvicorn so SIGTERM from
# `docker compose down` lands cleanly on the actual web server.
CMD ["/bin/bash", "-lc", \
     "source /opt/ros/humble/setup.bash && \
      source /msgs_ws/install/setup.bash && \
      exec uvicorn main:app --host 0.0.0.0 --port 8000"]
