#!/usr/bin/env bash
# ONE command for the whole chain_robot stack (was 7 terminals).
#
#   ./run_chain_stack.sh             brings up stages 1-6. Does NOT start mapping.
#   ./run_chain_stack.sh --explore   also start frontier exploration
#
# Exploration is OFF by default on purpose: start it yourself once everything
# else is up and you have eyes on the sim.
#
# Order matters and is why this script exists: each stage needs the previous one's
# topics/TF before it will configure. Nav2 in particular comes up on "Timed out
# waiting for transform" if /map and map->odom are not already there.
#
# Gazebo's window opens BEHIND VS Code at 576x320 and GNOME will not raise it --
# you get a "Gazebo is ready" notification instead. ALT+TAB to it.
set -uo pipefail
EXPLORE=0; [ "${1:-}" = "--explore" ] && EXPLORE=1

R() { docker exec -d safe_robot bash -ic "
  export GAZEBO_MODEL_PATH=/hw/models:\$GAZEBO_MODEL_PATH
  export GAZEBO_MODEL_DATABASE_URI=
  export GAZEBO_IP=127.0.0.1   # enp86s0 is UP with no IP; multicast sends there fail and flood the log
  $1 > $2 2>&1"; }
W() { local pat="$1" log="$2" name="$3" max="${4:-90}"
  for i in $(seq 1 "$max"); do
    docker exec safe_robot bash -c "grep -q '$pat' $log" 2>/dev/null && { echo "  ok  $name"; return 0; }
    sleep 2
  done; echo "  !!  $name did not report ready (see $log)"; return 1; }

echo "[1/7] gazebo + hospital + chain_robot"
R "ros2 launch chain_robot chain_robot2.launch.py world:=${WORLD_FILE:-/hw/hospital_chain.world} verbose:=true" /tmp/1_gazebo.log
W "Tracked robot plugin loaded successfully" /tmp/1_gazebo.log "gazebo (~60 s, 169 MB of meshes)" 120

echo "[2/7] FAST-LIO"
R "ros2 launch fast_lio mapping.launch.py rviz:=false" /tmp/2_fastlio.log
W "Initialize the map kdtree" /tmp/2_fastlio.log "fast_lio"

echo "[3/7] occupancy grid"
# Our copy of the package's launch with a hospital-sized map: theirs is 1.5 MB per /map,
# which nodes silently stop receiving (see occ_grid_hospital.yaml).
R "ros2 launch ${MAPPING_LAUNCH:-/hw/hospital_mapping.launch.xml}" /tmp/3_occ.log
W "mapped_cell_area" /tmp/3_occ.log "occ_grid"

echo "[4/7] chain robot controller"
R "ros2 launch chain_robot_planner chain_robot_controller.launch.py" /tmp/4_ctrl.log
W "initialized" /tmp/4_ctrl.log "controller"

echo "[5/7] RRT path planner  (NOT in the original guide - the controller needs /plan_rrt)"
R "ros2 launch chain_robot_planner path_planner_exploration.launch.py" /tmp/5_planner.log
W "Planner Initialized" /tmp/5_planner.log "path planner"

echo "[6/7] Nav2"
R "ros2 launch nav2_bringup navigation_exploration_launch.py" /tmp/6_nav2.log
sleep 45
# The lifecycle manager often logs "failed to send response ... change_state (timeout)"
# while still completing every transition, so check the nodes, not the log line.
docker exec safe_robot bash -ic 'for n in planner_server controller_server bt_navigator; do
  printf "  %-18s " $n; timeout 8 ros2 lifecycle get /$n 2>/dev/null || echo "?"; done' 2>/dev/null \
  | grep -viE "job control|ioctl"

if [ "$EXPLORE" = 1 ]; then
  echo "[7/7] frontier exploration"
  R "ros2 launch frontier_exploration classical_exploration.launch.py" /tmp/7_explore.log
  sleep 8; echo "  ok  exploration"
else
  echo "[7/7] exploration NOT started (default). When you are ready:"
  echo "      docker exec -d safe_robot bash -ic 'ros2 launch frontier_exploration classical_exploration.launch.py > /tmp/7_explore.log 2>&1'"
fi
echo
echo "logs: docker exec safe_robot tail -f /tmp/<N>_*.log"
echo "stop: ./stop_all.sh"
