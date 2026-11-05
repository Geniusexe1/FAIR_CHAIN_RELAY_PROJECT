#!/usr/bin/env bash
# The exploration coordinator, in its container on this host. run_exploration.sh calls it.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
if [ -n "${RELAY_HOSTS:-}" ]; then DDS_ENV="-e ROS_LOCALHOST_ONLY=1"
else DDS_ENV="-e ROS_LOCALHOST_ONLY=0 -e FASTRTPS_DEFAULT_PROFILES_FILE=/fair/hospital_world/mesh_peers.xml"; fi
docker image inspect fair_rssi:humble >/dev/null 2>&1 || docker build -t fair_rssi:humble "$ROOT/rssi/ros"
docker rm -f fair_coord >/dev/null 2>&1 || true
docker run -d --name fair_coord --network host --ipc host \
  -e ROS_DOMAIN_ID=42 $DDS_ENV \
  -v "$ROOT":/fair:ro -v "$ROOT/rssi/out/live_runs":/live_runs \
  fair_rssi:humble \
  /entrypoint.sh python3 -u /fair/rssi/ros/exploration_coordinator.py "$@" \
  --ros-args -p use_sim_time:=true >/dev/null
echo "fair_coord started (logs: docker logs -f fair_coord). Waiting for /coord/navigate_to_pose ..."
for _ in $(seq 60); do
  docker logs fair_coord 2>&1 | grep -q "exploration_coordinator\]: P0" && break
  sleep 1
done

