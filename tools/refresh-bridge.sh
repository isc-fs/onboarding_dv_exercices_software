#!/bin/bash
# Refresh the dv_pipeline_stack container after editing bridge code,
# any pipeline ROS source, the launch files, the entrypoint, or the
# Fast DDS profile.
#
# `docker compose restart` is NOT enough — Docker Desktop on macOS
# accumulates UDP proxy state and Fast DDS leaves SHM segments around;
# both eventually wedge in ways that make /lidar/Lidar1 disappear from
# external subscribers even when the bridge process is publishing
# internally. Only a full container teardown + recreate consistently
# recovers (verified by an afternoon of chasing this).
#
# #490 — pipeline source is not bind-mounted from the host; each
# package uses a Docker named volume sync'd by `docker compose watch`
# (see docker-compose.yml develop.watch) so Python edits across all
# pipeline/* packages apply without rebuilding. Use this script when
# you change docker/dv_pipeline_stack/*, need a full colcon rebuild
# (C++, .msg, setup.py), or want a clean recreate to clear DDS state.
# A repo-wide .dockerignore keeps `docker compose build` fast.
#
# What this script does, in order:
#   1. `docker compose build dv_pipeline_stack` — rebuild the image
#      against the current host source. BuildKit's layer cache makes
#      this fast for incremental Python edits (only the final COPY
#      + colcon build layers re-run).
#   2. `docker compose up -d --force-recreate dv_pipeline_stack` —
#      destroys + recreates the container, drops any wedged DDS SHM
#      / UDP proxy state, mounts the new image.
#   3. Wait for the container to report healthy.
#
# Run this any time you edit:
#   - pipeline/* ROS Python or C++ source
#   - ros2/src/* (fs_msgs / ifssim_bridge)
#   - docker/dv_pipeline_stack/{bridge,pipeline,pipeline_only}.launch.py
#   - docker/dv_pipeline_stack/entrypoint.sh
#   - docker/dv_pipeline_stack/fastdds_profile.xml
#
# For iterative Python work, `docker compose watch` syncs all pipeline
# packages into the container (see docker-compose.yml); only use this
# script when you need an image rebuild or a hard recreate.

set -euo pipefail

# Git Bash on Windows rewrites Linux-looking paths like /entrypoint.sh
# before native Windows executables see them. Docker commands need those
# paths to reach the Linux container unchanged.
export MSYS_NO_PATHCONV=1
export MSYS2_ARG_CONV_EXCL="*"

cd "$(git -C "$(dirname "$0")" rev-parse --show-toplevel)"

echo "→ docker compose build dv_pipeline_stack"
docker compose build dv_pipeline_stack 2>&1 | tail -3

# Stale-volume guard (added 2026-05-23 after silently driving a lap
# against the pre-edit sigma_bg_walk because refresh-bridge rebuilt
# the image, recreated the container — and the container still
# read from a stale named volume).
#
# The story: docker-compose mounts named volumes (
# `ifssim_dv_pipeline_<pkg>_src`, see develop.watch in
# docker-compose.yml, introduced in #490) over each pipeline
# package's source dir inside the container. Those volumes persist
# across container recreates AND across image rebuilds. They were
# designed for `compose watch` to sync into so Python edits go live
# without a rebuild. But that means `compose build` updates the
# image and `compose up -d --force-recreate` recreates the
# container, yet the running container still sees whatever the
# volume holds — which is the LAST sync, not the new image.
#
# Fix: drop the named source volumes after the build, so the
# subsequent `compose up` re-populates them from the new image's
# baked-in source. Cost is the volume re-init time (~1 s), tiny
# relative to wasting a drive on stale code.
echo "→ dropping stale pipeline source volumes (so they re-populate from new image)"
docker compose down dv_pipeline_stack 2>&1 | tail -3
VOLS=$(docker volume ls -q | grep -E '^ifssim_dv_pipeline_.+_src$' || true)
if [[ -n "$VOLS" ]]; then
    echo "$VOLS" | xargs -r docker volume rm >/dev/null 2>&1 || true
    echo "  removed $(echo "$VOLS" | wc -l | tr -d ' ') volume(s)"
fi

echo "→ docker compose up -d dv_pipeline_stack"
docker compose up -d dv_pipeline_stack 2>&1 | tail -3

echo "→ waiting for healthy..."
for _ in $(seq 1 30); do
    status=$(docker compose ps --format '{{.Name}} {{.Status}}' 2>/dev/null \
             | awk '/dv_pipeline_stack-1/ {for (i=2;i<=NF;i++) printf "%s ", $i; print ""}' \
             | tr -d '()')
    if [[ "$status" == *"healthy"* ]]; then
        echo "  ready: $status"
        break
    fi
    sleep 1
done


# Post-recreate verification — check the RUNNING CONTAINER (not the
# image alone). The named-volume failure mode masks the issue if
# you only check the image, since the volume overlays the image's
# COPY'd source. `docker exec` sees the effective FS the
# autonomy nodes will use.
echo "→ verifying container source == host source (post-recreate)"
CANARIES=(
    "pipeline/odometry_filter/include/odometry_filter/odometry_filter.hpp"
    "pipeline/cone_slam/cone_slam/data_association.py"
    "pipeline/cone_slam/cone_slam/cone_graph_slam_node.py"
)
STALE=0
for f in "${CANARIES[@]}"; do
    if [[ ! -f "$f" ]]; then continue; fi
    target="/dv_pipeline_stack_ws/src/${f#pipeline/}"
    container_sha=$(docker exec ifssim-dv_pipeline_stack-1 \
        sha256sum "$target" 2>/dev/null | awk '{print $1}')
    host_sha=$(sha256sum "$f" 2>/dev/null | awk '{print $1}')
    if [[ -z "$container_sha" || -z "$host_sha" || "$container_sha" != "$host_sha" ]]; then
        echo "  ✘ STALE: $f"
        echo "    host:      $host_sha"
        echo "    container: $container_sha"
        STALE=1
    fi
done
if [[ "$STALE" -ne 0 ]]; then
    echo
    echo "✘ ABORT: running container's source doesn't match host. Don't" >&2
    echo "  drive — autonomy is running on stale code." >&2
    echo "  Try: docker compose down dv_pipeline_stack && docker volume prune -f" >&2
    echo "  and re-run this script." >&2
    exit 1
fi
echo "  ✓ all canaries match in running container"

echo
echo "✓ bridge refreshed. Tail logs with:"
echo "    docker compose logs -f dv_pipeline_stack"
