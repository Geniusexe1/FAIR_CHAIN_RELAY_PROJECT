#!/usr/bin/env bash
# Stop everything on every NUC; run it on the big NUC. Leaves the containers themselves running.
set -uo pipefail
echo "[big NUC] safe_robot (gazebo + chain stack)..."
docker exec safe_robot bash -c 'for p in $(ps -eo pid,args --no-headers \
  | grep -vE "grep|ps -eo" \
  | grep -E "gzserver|gzclient|fastlio|rviz2|occ_grid|nav2_|frontier|chain_robot|path_planner|pointcloud_to_laser|static_transform|robot_state_pub|classical|bin/ros2|mapping|topic pub" \
  | awk "{print \$1}"); do kill -9 $p 2>/dev/null; done' 2>/dev/null || true
# static_transform_publisher too: a survivor keeps publishing an old map->relayN/odom and that
# relay's Nav2 then puts the robot off its costmap (every goal aborted)
while read -r ns target container; do
  case "$ns" in ''|\#*) continue ;; esac
  echo "[$target] $container ($ns brain)..."
  KILL="docker exec $container bash -c 'pkill -9 -f \"[n]av_robot|[n]av2_|[c]omponent_container|[t]f_relay|bin/[r]os2|[s]tatic_transform_publisher\"; true'"
  if [ "$target" = local ]; then bash -c "$KILL" 2>/dev/null; else
    ssh -n -o BatchMode=yes -o ConnectTimeout=5 "$target" "$KILL" 2>/dev/null || echo "  !! could not reach $target"; fi
done < "${RELAY_HOSTS:-$(dirname "$0")/relay_hosts.conf}"
sleep 4
echo "--- still alive in safe_robot (blank = clean) ---"
docker exec safe_robot bash -c 'ps -eo args --no-headers | grep -E "[g]zserver|[g]zclient|[f]astlio|[n]av2_" | grep -v defunct | cut -c1-45' 2>/dev/null || true
