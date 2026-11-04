#!/usr/bin/env bash
# The whole live system, in order, with the Nav2 start-up retries it needs at low RTF:
# Run it on the big NUC, which holds the sim, the map, chain_robot and the coordinator:
#   1. chain stack (gazebo, FAST-LIO, mapping, controller, planner, Nav2)
#   2. every relay in the nodes file spawned at its `park`, heading `park_yaw` (else PARK_YAW)
#   3. relay brains on the NUCs in hospital_world/relay_hosts.conf
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
  docker exec safe_robot bash -ic "export ROS_DOMAIN_ID=42; timeout 8 ros2 lifecycle get $1/bt_navigator" \
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
      docker exec safe_robot bash -ic "export ROS_DOMAIN_ID=42; timeout 200 ros2 service call \
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
NODES="${NODES:-$ROOT/rssi/ros/nodes.yaml}"
mapfile -t RELAYS < <(python3 -c "import yaml; print(*list(yaml.safe_load(open('$NODES')))[1:-1], sep='\n')")
FROM="${FROM:-1}"     # FROM=3 rssi/ros/run_full_live.sh ...  resumes at stage 3

if [ "$FROM" -le 1 ]; then
# containers draw on the desktop's X server; the grant does not survive a new login
DISPLAY=:0 XAUTHORITY="/run/user/$(id -u)/gdm/Xauthority" xhost +local:docker >/dev/null 2>&1 \
  || echo "  !!  no desktop session on :0 - log in on the big NUC's screen (Gazebo needs it)"
# no monitor plugged in: X has no display modes and gzserver segfaults at start (Ogre's GLX set-up)
if ! DISPLAY=:0 XAUTHORITY="/run/user/$(id -u)/gdm/Xauthority" xrandr -q | grep -q ' connected'; then
  export DISPLAY=:0 XAUTHORITY="/run/user/$(id -u)/gdm/Xauthority"
  xrandr --newmode 2560x1440R 241.50 2560 2608 2640 2720 1440 1443 1448 1481 +hsync -vsync 2>/dev/null
  xrandr --addmode HDMI-2 2560x1440R 2>/dev/null
  xrandr --output HDMI-2 --mode 2560x1440R --pos 0x0 --primary && echo "  ok  virtual 2560x1440 display"
fi
# Gazebo's Ignition Transport multicasts discovery to 224.0.0.7 about 16 times a second. Tiny, but it
# chokes the Rajant radio: 70% loss and 100+ ms for every node, 0% and 4 ms the moment it is blocked.
# It stays on this host. The iptables rule is what proved it; add it if it is gone after a reboot:
#   sudo iptables -I OUTPUT -o enp86s0 -d 224.0.0.7 -j DROP
# DDS discovery multicast chokes it the same way: with brains on other NUCs everything uses unicast
# peers (mesh_peers.xml); with every brain here (RELAY_HOSTS=relay_hosts_bignuc.conf) nothing leaves
# this host. Unicast still collapsed the big NUC's link with ~27 remote Nav2 processes (2026-11-05).
if [ -n "${RELAY_HOSTS:-}" ]; then DDS="ROS_LOCALHOST_ONLY=1"; else DDS="FASTRTPS_DEFAULT_PROFILES_FILE=/hw/mesh_peers.xml"; fi
# ulimit: gzserver opens a socket per gz client; newer Docker gives containers 1024 files and at that
# it dies with "Too many open files" (big_u_full, after an hour)
docker exec safe_robot bash -c "sed -i '/IGN_IP/d' /root/.bashrc; echo 'export IGN_IP=127.0.0.1 $DDS; ulimit -n 524288' >> /root/.bashrc"
echo "[1/5] chain stack"
"$HW/run_chain_stack.sh" 2>&1 | grep -E "ok|!!"
nav_up ""
# chain_robot is differential drive but its DWB config (min_vel_x = min_speed_xy = 0.2) forbids
# turning in place: nose-first at a dead end every arc hits the wall and it is stuck for good
# and its costmaps model it as a 5 cm robot (robot_radius 0.05), so paths and DWB hug walls
docker exec safe_robot bash -ic "export ROS_DOMAIN_ID=42; for p in FollowPath.min_vel_x FollowPath.min_speed_xy; do
  timeout 20 ros2 param set /controller_server \$p 0.0; done
  timeout 20 ros2 param set /global_costmap/global_costmap robot_radius 0.2
  timeout 20 ros2 param set /local_costmap/local_costmap robot_radius 0.12
  timeout 20 ros2 param set /local_costmap/local_costmap inflation_layer.inflation_radius 0.25" >/dev/null 2>&1 \
  && echo "  ok  chain_robot may turn in place; costmap radius 0.2 global / 0.12 local"
fi

if [ "$FROM" -le 2 ]; then
echo "[2/5] relays at their parking spots"
while read -r NS X Y YAW; do
  PARK_YAW=$YAW spawn "$NS" "$X" "$Y" < /dev/null     # docker exec would swallow the loop's input
done < <(python3 -c "
import yaml; c = yaml.safe_load(open('$NODES'))
for n in list(c)[1:-1]:
    print(n, *c[n]['park'], c[n].get('park_yaw', ${PARK_YAW:--1.5708}))")
fi

if [ "$FROM" -le 3 ]; then
echo "[3/5] relay brains"
for NS in "${RELAYS[@]}"; do "$HW/run_relay.sh" "$NS" 2>&1 | tail -n 1; done
fi

if [ "$FROM" -le 4 ]; then
echo "[4/5] Nav2 of every relay"
for NS in "${RELAYS[@]}"; do nav_up "/$NS"; done
fi

echo "[5/5] RSSI monitor, coordinator, explorer, logger"
"$ROOT/rssi/ros/run_rssi_node.sh" "$@" >/dev/null
"$ROOT/rssi/ros/run_exploration.sh" "$@" | tail -n 1
pkill -f "[l]ive_logger.sh" 2>/dev/null
RELAYS="${RELAYS[*]}" setsid nohup "$ROOT/rssi/ros/live_logger.sh" "$LOGCSV" "${LOG_PERIOD:-20}" >/dev/null 2>&1 < /dev/null &   # survives this shell
echo "logging ground truth to ${LOGCSV#$ROOT/}"
