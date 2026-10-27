#!/usr/bin/env bash
# Communication-aware exploration: coordinator + chain_robot's frontier explorer.
#
#   rssi/ros/run_exploration.sh                 # paper P0 (-30), engaged relays re-placed (paper)
#   rssi/ros/run_exploration.sh --keep-deployed # relays never move once placed
#   rssi/ros/run_exploration.sh --P0 -60.8      # measured 5 GHz P0
#
# Needs, already running: the chain stack WITHOUT --explore (gazebo, FAST-LIO, Nav2 ...),
# both relays spawned with their Nav2 up (run_relay1_nuc1.sh, run_relay2_nuc2.sh).
#
# 1. fair_coord container: exploration_coordinator.py (extra args go to it)
# 2. in safe_robot: the frontier detector + explorer, with the explorer's
#    navigate_to_pose remapped to /coord/navigate_to_pose -- their code is unmodified.
#
# Watch:   docker logs -f fair_coord
#          ros2 topic echo /coord/state
# Stop:    docker rm -f fair_coord; docker exec safe_robot pkill -f frontier
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
docker image inspect fair_rssi:humble >/dev/null 2>&1 || docker build -t fair_rssi:humble "$ROOT/rssi/ros"
docker rm -f fair_coord >/dev/null 2>&1 || true
docker run -d --name fair_coord --network host --ipc host \
  -e ROS_DOMAIN_ID=42 -e ROS_LOCALHOST_ONLY=0 \
  -v "$ROOT":/fair:ro \
  fair_rssi:humble \
  /entrypoint.sh python3 -u /fair/rssi/ros/exploration_coordinator.py "$@" >/dev/null
echo "fair_coord started (logs: docker logs -f fair_coord). Waiting for /coord/navigate_to_pose ..."
for _ in $(seq 60); do
  docker logs fair_coord 2>&1 | grep -q "exploration_coordinator\]: P0" && break
  sleep 1
done

# same detector parameters as frontier_exploration/classical_exploration.launch.py
docker exec -d safe_robot bash -ic 'ros2 run frontier_exploration classical_frontier_detector --ros-args \
  -p region_size_thresh_:=3 -p robot_width_:=0.3 -p safe_radius_m_:=0.8 -p min_dist_thresh_:=0.3 \
  -p max_dist_thresh_:=50.0 -p occupancy_map_topic_:=map > /tmp/7a_detector.log 2>&1'
docker exec -d safe_robot bash -ic 'ros2 run frontier_exploration frontier_exploration_node.py --ros-args \
  -r navigate_to_pose:=/coord/navigate_to_pose > /tmp/7b_explore.log 2>&1'
echo "explorer started in safe_robot (/tmp/7a_detector.log, /tmp/7b_explore.log)"
