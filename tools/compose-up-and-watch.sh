#!/usr/bin/env bash
# Compose cannot attach `docker compose watch` to `up -d` from the compose file
# (watch is a host-side process; `-d` and `--watch` are mutually exclusive on the
# CLI). Use this instead of plain `docker compose up -d` when you want pipeline
# sources synced into the dv_pipeline_stack named volumes.
#
# Usage (from anywhere):
#   tools/compose-up-and-watch.sh
#   tools/compose-up-and-watch.sh dv_pipeline_stack
#
# Detaches containers, then runs watch in the foreground (this terminal). Ctrl+C
# stops watch only; containers keep running. Use `docker compose down` to stop.

set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
docker compose up -d "$@"
docker compose watch "$@"
