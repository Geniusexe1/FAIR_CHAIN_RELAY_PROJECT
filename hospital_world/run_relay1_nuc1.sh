#!/usr/bin/env bash
# relay1's brain, on NUC1 but in the mesh_sim container (not safe_robot).
# safe_robot has nav2_bringup but NOT saye_bringup, so nav_robot.launch.py and
# send_waypoints.py live in `nuc1`. Both containers are --network host on domain
# 42, so they talk over DDS on the same machine.
# Gazebo and relay1's MODEL stay in safe_robot. This is decision-making only.
set -euo pipefail
NS="${1:-relay1}"
docker exec nuc1 /entrypoint.sh bash -c "
  export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=0
  timeout 20 ros2 topic list 2>/dev/null | grep -E '^/(clock|${NS}/)' \
    || echo 'NOT VISIBLE - is the sim up and ${NS} spawned?'"
# clear an earlier stack of this robot: a surviving static_transform_publisher keeps
# publishing an old map->${NS}/odom and Nav2 then localises the robot off its costmap
P="[${NS:0:1}]${NS:1}"     # [r]elay1: the pattern must not match this pkill's own shell
docker exec nuc1 bash -c "pkill -9 -f 'namespace:=$P|child-frame-id $P/odom|__ns:=/$P'; sleep 1; true"
docker exec -d nuc1 /entrypoint.sh bash -c "
  export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=0
  ros2 launch saye_bringup nav_robot.launch.py namespace:=${NS} static_map_odom:=true map_odom_x:=${MAP_ODOM_X:-0.0} map_odom_y:=${MAP_ODOM_Y:--12.0} \
    > /tmp/${NS}_nav.log 2>&1"
echo "[${NS}] Nav2 starting in container nuc1. tail: docker exec nuc1 tail -f /tmp/${NS}_nav.log"
