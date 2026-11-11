#!/usr/bin/env bash
# Start one relay's DECISION-MAKING on the NUC relay_hosts.conf gives it. Gazebo and the relay's
# MODEL stay on the big NUC: physics cannot be split. Here runs the Nav2 stack that consumes
# /<ns>/scan + /<ns>/odom and publishes /<ns>/cmd_vel back across the mesh.
#
#   hospital_world/run_relay.sh relay2
#
# Prereqs: the sim is up, <ns> is spawned, and this host has an ssh key on the target.
set -euo pipefail
HW="$(cd "$(dirname "$0")" && pwd)"
NS="${1:?usage: run_relay.sh <namespace>}"
read -r _ TARGET CONTAINER < <(awk -v n="$NS" '$1 == n' "${RELAY_HOSTS:-$HW/relay_hosts.conf}")
[ -n "${TARGET:-}" ] || { echo "$NS is not in relay_hosts.conf"; exit 1; }
P="[${NS:0:1}]${NS:1}"     # [r]elay2: the pattern must not match this pkill's own shell
if [ "$TARGET" = local ]; then ENV="ROS_LOCALHOST_ONLY=1"; else
  ENV="ROS_LOCALHOST_ONLY=0 FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/mesh_peers.xml"
  ssh -o BatchMode=yes -o ConnectTimeout=10 "$TARGET" "docker exec -i $CONTAINER tee /tmp/mesh_peers.xml >/dev/null" < "$HW/mesh_peers.xml"
  ssh -o BatchMode=yes -o ConnectTimeout=10 "$TARGET" "docker exec -i $CONTAINER tee /tmp/tf_relay.py >/dev/null" < "$HW/tf_relay.py"
fi
CMD="
  docker inspect -f '{{.State.Running}}' $CONTAINER 2>/dev/null | grep -q true || docker start $CONTAINER >/dev/null
  # a surviving static_transform_publisher keeps publishing an old map->$NS/odom, and Nav2 then
  # localises the robot off its costmap (every goal aborted)
  docker exec $CONTAINER bash -c \"pkill -9 -f 'namespace:=$P|child-frame-id $P/odom|__ns:=/$P|[t]f_relay'; sleep 1; true\"
  # Nav2 reads this relay's own TF (tf_relay.py), not the global topic: navigation_launch.py resolves
  # 'tf' to /<ns>/tf by itself once these two remaps are out
  L=/root/colcon_ws/install/saye_bringup/share/saye_bringup/launch/nav_robot.launch.py
  docker exec $CONTAINER sed -i -E 's/^( *)SetRemap\(src=.\/tf/\1# &/' \$L
  # and the map at the rate the mesh can carry (map_throttle.py on the chain's NUC)
  docker exec $CONTAINER bash -c \"grep -q map_mesh \$L || sed -i '/PushRosNamespace(ns),/a\            SetRemap(src=\x27/map\x27, dst=\x27/map_mesh\x27),' \$L\"
  # every relay runs the same Nav2 params: the launch reads this template, which had drifted apart between NUCs
  [ -n \"${RELAY_PARAMS:-}\" ] && docker exec -i $CONTAINER tee /root/colcon_ws/install/saye_bringup/share/saye_bringup/config/nav2_params.yaml.in >/dev/null < /tmp/${NS}_params.in
  docker exec -d $CONTAINER /entrypoint.sh bash -c \"ulimit -n 524288; export ROS_DOMAIN_ID=42 $ENV
    python3 /tmp/tf_relay.py $NS > /tmp/${NS}_tf.log 2>&1\"
  docker exec -d $CONTAINER /entrypoint.sh bash -c \"ulimit -n 524288; export ROS_DOMAIN_ID=42 $ENV
    ros2 launch saye_bringup nav_robot.launch.py namespace:=$NS static_map_odom:=true map_odom_x:=${MAP_ODOM_X:-0.0} map_odom_y:=${MAP_ODOM_Y:--12.0} > /tmp/${NS}_nav.log 2>&1\""
if [ -n "${RELAY_PARAMS:-}" ]; then
  if [ "$TARGET" = local ]; then cp "$RELAY_PARAMS" /tmp/${NS}_params.in
  else ssh -o BatchMode=yes -o ConnectTimeout=10 "$TARGET" "cat > /tmp/${NS}_params.in" < "$RELAY_PARAMS"; fi
fi
if [ "$TARGET" = local ]; then bash -c "$CMD"; else ssh -n -o BatchMode=yes -o ConnectTimeout=10 "$TARGET" "$CMD"; fi
echo "[$NS] Nav2 starting on $TARGET in $CONTAINER. tail: ssh $TARGET docker exec $CONTAINER tail -f /tmp/${NS}_nav.log"
