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
#    navigate_to_pose action remapped to /coord/navigate_to_pose -- their code is unmodified.
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
  /entrypoint.sh python3 -u /fair/rssi/ros/exploration_coordinator.py "$@" \
  --ros-args -p use_sim_time:=true >/dev/null
echo "fair_coord started (logs: docker logs -f fair_coord). Waiting for /coord/navigate_to_pose ..."
for _ in $(seq 60); do
  docker logs fair_coord 2>&1 | grep -q "exploration_coordinator\]: P0" && break
  sleep 1
done

# same detector parameters as frontier_exploration/classical_exploration.launch.py
: | docker exec -i safe_robot bash -c 'pkill -f "fair_explore_loop|frontier_exploration_node|classical_frontier_detector"; true' 2>/dev/null || true
docker exec -d safe_robot bash -ic 'ros2 run frontier_exploration classical_frontier_detector --ros-args \
  -p region_size_thresh_:=3 -p robot_width_:=0.3 -p safe_radius_m_:=0.8 -p min_dist_thresh_:=0.3 \
  -p max_dist_thresh_:=50.0 -p occupancy_map_topic_:=map -r map:=/coord/map -r /map:=/coord/map \
  > /tmp/7a_detector.log 2>&1'
# the explorer quits ("No frontiers") if its first request beats the detector's first /map
docker exec safe_robot bash -ic 'timeout 30 ros2 topic echo --once /map --no-arr >/dev/null' 2>/dev/null \
  || echo "WARNING: no /map yet - is occ_grid_mapping running?"
sleep 8
# Humble does not apply `-r navigate_to_pose:=...` to an action's hidden services/topics,
# so each of the five is remapped explicitly.
A=navigate_to_pose/_action; C=/coord/navigate_to_pose/_action
# Explorer supervision. It reads /coord/map (the coordinator's cropped /map -- the 1.5 MB
# original does not reach every participant on this host, see exploration_coordinator.py).
#  - "No frontiers" within its first minute = it had no map yet; it then HANGS (its exit()
#    runs in an executor thread), so kill and restart it;
#  - any other exit (e.g. a crash in its own code) -> restart.
cat > /tmp/fair_explore_loop.sh <<EOS
#!/usr/bin/env bash
LOG=/tmp/7b_explore.log
for i in \$(seq 50); do
  n0=\$(grep -c "No frontiers" \$LOG 2>/dev/null); n0=\${n0:-0}
  ros2 run frontier_exploration frontier_exploration_node.py --ros-args \\
    -r $A/send_goal:=$C/send_goal -r $A/cancel_goal:=$C/cancel_goal -r $A/get_result:=$C/get_result \\
    -r $A/feedback:=$C/feedback -r $A/status:=$C/status -r /map:=/coord/map >> \$LOG 2>&1 &
  pid=\$!; early=0
  for s in \$(seq 60); do
    sleep 1
    [ "\$(grep -c "No frontiers" \$LOG)" -gt "\$n0" ] && { early=1; break; }
    kill -0 \$pid 2>/dev/null || break
  done
  if [ \$early = 0 ]; then
    wait \$pid
    echo "[run_exploration] explorer exited (code \$?), restarting" >> \$LOG
  else
    pkill -f frontier_exploration_node
    echo "[run_exploration] explorer quit early (no map yet), restarting" >> \$LOG
  fi
  sleep 10
done
EOS
docker cp /tmp/fair_explore_loop.sh safe_robot:/tmp/fair_explore_loop.sh
docker exec -d safe_robot bash -ic 'bash /tmp/fair_explore_loop.sh'
echo "explorer started in safe_robot (/tmp/7a_detector.log, /tmp/7b_explore.log)"
