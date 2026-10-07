#!/usr/bin/env bash
# Start the live RSSI monitor in its own container on NUC1, domain 42.
#
#   rssi/ros/run_rssi_node.sh                 # paper n/alpha, measured P0, noise on
#   rssi/ros/run_rssi_node.sh --no-noise      # clean model only
#   rssi/ros/run_rssi_node.sh --P0 -30        # paper's P0
#
# Extra args go straight to rssi_monitor.py. Watch it with:
#   docker logs -f fair_rssi
# Read it from any container on domain 42:
#   ros2 topic echo /rssi/links
#   ros2 topic echo /rssi/end_to_end/relay2
#
# --ipc host is MANDATORY (see CLAUDE.md trap 8): without it FastDDS shared memory
# between this and safe_robot silently drops every message.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
docker image inspect fair_rssi:humble >/dev/null 2>&1 || docker build -t fair_rssi:humble "$ROOT/rssi/ros"
docker rm -f fair_rssi >/dev/null 2>&1 || true
docker run -d --name fair_rssi --network host --ipc host \
  -e ROS_DOMAIN_ID=42 -e ROS_LOCALHOST_ONLY=0 \
  -v "$ROOT":/fair:ro \
  fair_rssi:humble \
  /entrypoint.sh python3 -u /fair/rssi/ros/rssi_monitor.py "$@" >/dev/null
echo "fair_rssi started. logs: docker logs -f fair_rssi"
