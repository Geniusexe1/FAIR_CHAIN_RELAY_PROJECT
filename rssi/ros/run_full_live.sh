#!/usr/bin/env bash
# The whole live system, in order, with the Nav2 start-up retries it needs at low RTF:
#   1. chain stack (gazebo, FAST-LIO, mapping, controller, planner, Nav2)
#   2. both relays spawned at their parking spots (nodes.yaml `park`), facing south
#   3. relay brains: relay1 on NUC1 (nuc1), relay2 on NUC2 (nuc2)
#   4. every Nav2 checked ACTIVE, restarted through its lifecycle manager if not
#   5. RSSI monitor + exploration coordinator + explorer + ground-truth logger
#
#   rssi/ros/run_full_live.sh [run_name] [coordinator args...]
#   rssi/ros/run_full_live.sh run2 --P0 -30
#
# Another world (UNIST): the world/mapping/offset env vars go to the hospital_world scripts,
# NODES picks the parking spots, and the args go to the coordinator and the RSSI monitor:
#   WORLD_FILE=/hw/unist/unist_indoor.world MAPPING_LAUNCH=/hw/unist/unist_mapping.launch.xml \
#   MAP_ODOM_X=0.5 MAP_ODOM_Y=6.0 NODES=rssi/ros/nodes_unist.yaml PARK_YAW=1.5708 rssi/ros/run_full_live.sh unist1 \
#     --P0 -45 --config /fair/rssi/ros/nodes_unist.yaml --gt hospital_world/unist/gt/unist_gt \
#     --map-offset -0.5 -6 --marker-offset 0.5 6
#
# Stop everything: hospital_world/stop_all.sh; docker rm -f fair_coord fair_rssi;
#                  pkill -f live_logger.sh
set -uo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
HW="$ROOT/hospital_world"
RUN="${1:-live}"; shift || true
LOGCSV="$ROOT/rssi/out/live_runs/${RUN}.csv"

nav_active() {   # $1 = namespace ('' for chain_robot); prints the bt_navigator state word
  docker exec nuc1 /entrypoint.sh bash -c "export ROS_DOMAIN_ID=42; timeout 8 ros2 lifecycle get $1/bt_navigator" \
    2>/dev/null | awk '{print $1}' | tail -n 1
}
nav_up() {       # bring a Nav2 stack to ACTIVE: wait, STARTUP, then RESET + STARTUP
  local ns="$1" s
  for i in $(seq 9); do s=$(nav_active "$ns"); [ "$s" = active ] && { echo "  ok  Nav2 ${ns:-/ (chain_robot)} active"; return 0; }; sleep 5; done
  if [ -z "$ns" ]; then
    # chain_robot: a lost change_state reply freezes the lifecycle manager and it ignores
    # manage_nodes too -- relaunching Nav2 (~20 s) is far quicker than waiting it out
    docker exec safe_robot bash -c 'pkill -9 -f "[n]avigation_exploration_launch|[l]ifecycle_manager|[b]t_navigator|[c]ontroller_server|[p]lanner_server|[b]ehavior_server|[s]moother_server|[w]aypoint_follower|[v]elocity_smoother"; true'
    docker exec -d safe_robot bash -ic "export GAZEBO_IP=127.0.0.1; ros2 launch nav2_bringup navigation_exploration_launch.py > /tmp/6_nav2.log 2>&1"
    for i in $(seq 20); do s=$(nav_active ""); [ "$s" = active ] && { echo "  ok  Nav2 / active (relaunched)"; return 0; }; sleep 5; done
  fi
  for cmds in "0" "3 0"; do
    for c in $cmds; do
      docker exec nuc1 /entrypoint.sh bash -c "export ROS_DOMAIN_ID=42; timeout 200 ros2 service call \
        $ns/lifecycle_manager_navigation/manage_nodes nav2_msgs/srv/ManageLifecycleNodes '{command: $c}'" >/dev/null 2>&1
    done
    for i in $(seq 24); do s=$(nav_active "$ns"); [ "$s" = active ] && { echo "  ok  Nav2 ${ns:-/} active (after restart)"; return 0; }; sleep 5; done
  done
  echo "  !!  Nav2 ${ns:-/} not active (state: $s)"; return 1
}

spawn() {        # spawn_entity can hang: time out, delete the half-spawned model, retry once
  for try in 1 2; do
    timeout 180 "$HW/spawn_saye.sh" "$1" "$2" "$3" 0.10 "${PARK_YAW:--1.5708}" >/dev/null 2>&1 && { echo "  ok  $1 at ($2, $3)"; return 0; }
    docker exec safe_robot bash -c 'pkill -f "[s]pawn_entity"; true'
    docker exec safe_robot bash -ic "timeout 30 ros2 service call /delete_entity gazebo_msgs/srv/DeleteEntity '{name: $1}'" >/dev/null 2>&1
  done
  echo "  !!  could not spawn $1"; return 1
}
FROM="${FROM:-1}"     # FROM=3 rssi/ros/run_full_live.sh ...  resumes at stage 3

if [ "$FROM" -le 1 ]; then
echo "[1/5] chain stack"
"$HW/run_chain_stack.sh" 2>&1 | grep -E "ok|!!"
nav_up ""
fi

if [ "$FROM" -le 2 ]; then
echo "[2/5] relays at their parking spots"
read -r P1X P1Y P2X P2Y < <(python3 -c "
import yaml; c = yaml.safe_load(open('${NODES:-$ROOT/rssi/ros/nodes.yaml}'))
print(*c['relay1']['park'], *c['relay2']['park'])")
spawn relay1 "$P1X" "$P1Y"
spawn relay2 "$P2X" "$P2Y"
fi

if [ "$FROM" -le 3 ]; then
echo "[3/5] relay brains"
"$HW/run_relay1_nuc1.sh" relay1 >/dev/null 2>&1
"$HW/run_relay2_nuc2.sh" relay2 >/dev/null 2>&1
fi

if [ "$FROM" -le 4 ]; then
echo "[4/5] Nav2 of both relays"
nav_up /relay1
nav_up /relay2
fi

echo "[5/5] RSSI monitor, coordinator, explorer, logger"
"$ROOT/rssi/ros/run_rssi_node.sh" "$@" >/dev/null
"$ROOT/rssi/ros/run_exploration.sh" "$@" | tail -n 1
pkill -f "[l]ive_logger.sh" 2>/dev/null
setsid nohup "$ROOT/rssi/ros/live_logger.sh" "$LOGCSV" "${LOG_PERIOD:-20}" >/dev/null 2>&1 < /dev/null &   # survives this shell
echo "logging ground truth to ${LOGCSV#$ROOT/}"
