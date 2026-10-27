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
  for i in $(seq 18); do s=$(nav_active "$ns"); [ "$s" = active ] && { echo "  ok  Nav2 ${ns:-/ (chain_robot)} active"; return 0; }; sleep 5; done
  for cmds in "0" "3 0"; do
    for c in $cmds; do
      docker exec nuc1 /entrypoint.sh bash -c "export ROS_DOMAIN_ID=42; timeout 200 ros2 service call \
        $ns/lifecycle_manager_navigation/manage_nodes nav2_msgs/srv/ManageLifecycleNodes '{command: $c}'" >/dev/null 2>&1
    done
    for i in $(seq 24); do s=$(nav_active "$ns"); [ "$s" = active ] && { echo "  ok  Nav2 ${ns:-/} active (after restart)"; return 0; }; sleep 5; done
  done
  echo "  !!  Nav2 ${ns:-/} not active (state: $s)"; return 1
}

echo "[1/5] chain stack"
"$HW/run_chain_stack.sh" 2>&1 | grep -E "ok|!!"
nav_up ""

echo "[2/5] relays at their parking spots"
read -r P1X P1Y P2X P2Y < <(python3 -c "
import yaml; c = yaml.safe_load(open('$ROOT/rssi/ros/nodes.yaml'))
print(*c['relay1']['park'], *c['relay2']['park'])")
"$HW/spawn_saye.sh" relay1 "$P1X" "$P1Y" 0.10 -1.5708 >/dev/null 2>&1 && echo "  ok  relay1 at ($P1X, $P1Y)"
"$HW/spawn_saye.sh" relay2 "$P2X" "$P2Y" 0.10 -1.5708 >/dev/null 2>&1 && echo "  ok  relay2 at ($P2X, $P2Y)"

echo "[3/5] relay brains"
"$HW/run_relay1_nuc1.sh" relay1 >/dev/null 2>&1
"$HW/run_relay2_nuc2.sh" relay2 >/dev/null 2>&1

echo "[4/5] Nav2 of both relays"
nav_up /relay1
nav_up /relay2

echo "[5/5] RSSI monitor, coordinator, explorer, logger"
"$ROOT/rssi/ros/run_rssi_node.sh" "$@" >/dev/null
"$ROOT/rssi/ros/run_exploration.sh" "$@" | tail -n 1
pkill -f "[l]ive_logger.sh" 2>/dev/null
setsid nohup "$ROOT/rssi/ros/live_logger.sh" "$LOGCSV" 20 >/dev/null 2>&1 < /dev/null &   # survives this shell
echo "logging ground truth to ${LOGCSV#$ROOT/}"
