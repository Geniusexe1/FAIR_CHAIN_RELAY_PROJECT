#!/usr/bin/env bash
# Recreate the nuc1 container (relay1's Nav2 brain).
#
# --ipc host IS NOT OPTIONAL. Both this and safe_robot are --network host, so
# FastDDS sees them as same-host participants and prefers the SHARED MEMORY
# transport. With separate IPC namespaces their /dev/shm differ, SHM segments
# written by one are invisible to the other, and you get the worst failure mode
# there is: discovery succeeds over UDP (ros2 topic list / node list / topic
# info -v all look perfectly correct) while EVERY message is silently dropped.
# nuc2 is unaffected because it is a genuinely remote host and uses UDP.
set -euo pipefail
docker rm -f nuc1 >/dev/null 2>&1 || true
docker run -d --name nuc1 --network host --ipc host --privileged \
  --restart unless-stopped \
  -e ROS_DOMAIN_ID=42 -e ROBOT_NS=robot1 \
  -e DISPLAY="${DISPLAY:-:0}" -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --device /dev/dri \
  mesh_sim:humble sleep infinity
echo "nuc1 recreated. Verify the data path actually carries messages, not just topics:"
echo "  docker exec nuc1 /entrypoint.sh bash -c 'export ROS_DOMAIN_ID=42; ros2 topic hz /tf'"
