#!/usr/bin/env bash
# Spawn an Ackermann relay into the running hospital sim.
#
#   ./spawn_saye.sh <namespace> [x] [y] [z] [yaw]
#   ./spawn_saye.sh relay1  2.0 12.0
#   ./spawn_saye.sh relay2 -2.0 12.0 0.1 1.57
#
# Requires the sim to already be up (run_hospital.sh). Spawning is additive, so
# call it once per relay with a DIFFERENT namespace each time - the namespace
# becomes the Gazebo entity name, the ROS namespace and the TF frame prefix.
set -euo pipefail

NS="${1:?usage: spawn_saye.sh <namespace> [x] [y] [z] [yaw]}"
X="${2:-2.5}"; Y="${3:-12.0}"; Z="${4:-0.10}"; YAW="${5:-0.0}"

docker exec -i safe_robot bash -ic "
  export GAZEBO_MODEL_PATH=/hw/models:\$GAZEBO_MODEL_PATH
  export GAZEBO_MODEL_DATABASE_URI=
  export GAZEBO_IP=127.0.0.1   # enp86s0 is UP with no IP; multicast sends there fail and flood the log
  # /hw is mounted read-only, so the substituted copy goes to /tmp.
  sed 's/\\\${NS}/$NS/g' /hw/models/saye_classic/model.sdf > /tmp/saye_$NS.sdf
  ros2 run gazebo_ros spawn_entity.py \
      -file /tmp/saye_$NS.sdf \
      -entity $NS \
      -robot_namespace $NS \
      -x $X -y $Y -z $Z -Y $YAW
"

# Send one zero command. Until libgazebo_ros_ackermann_drive.so receives its first
# cmd_vel it has no target and the drive wheels free-run (measured 101 rad/s on a
# freshly spawned relay with nothing published). One latched zero parks it until
# Nav2 takes over.
docker exec safe_robot bash -ic "ros2 topic pub -1 /$NS/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.0}, angular: {z: 0.0}}'" >/dev/null 2>&1 || true

echo
echo "spawned '$NS' at ($X, $Y, $Z) yaw=$YAW"
echo "  drive it:  docker exec -it safe_robot bash -ic \"ros2 topic pub /$NS/cmd_vel geometry_msgs/Twist '{linear: {x: 0.5}, angular: {z: 0.2}}' -r 10\""
echo "  odom:      /$NS/odom       lidar: /$NS/points      imu: /$NS/imu"
echo "  cameras:   /$NS/camera_{front,rear,left,right}/image_raw, /$NS/rs_front/*"
