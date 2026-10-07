#!/usr/bin/env bash
# Recreate the safe_robot container with the AWS hospital world wired in.
# The image bakes code in (no bind mount of the workspace), so the patched
# launch file must be re-copied every time the container is recreated.
#
# NOTE: this machine has NO NVIDIA GPU (Intel iGPU only). Any --gpus all /
# --runtime=nvidia flag from the upstream docs WILL fail here.
set -euo pipefail
HW="$(cd "$(dirname "$0")" && pwd)"   # this script's own directory
IMG=gudwls124z/safe_robot:260826

xhost +local:docker >/dev/null
docker rm -f safe_robot >/dev/null 2>&1 || true
docker run -d --name safe_robot --network host --ipc host \
  -e DISPLAY="$DISPLAY" -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "$HW":/hw:ro \
  --device /dev/dri --privileged \
  "$IMG" sleep infinity

# Re-apply the ROS networking overrides (lost whenever the container is recreated).
# The image's .bashrc sets ROS_DOMAIN_ID=52 and ROS_LOCALHOST_ONLY=1. The latter
# confines DDS to loopback, so NUC2 can never see this machine no matter what.
# Appended at the END of .bashrc so they win over the image's values.
docker exec safe_robot bash -c 'grep -q "MESH SIM OVERRIDES" /root/.bashrc || cat >> /root/.bashrc <<EOF

# ---------------- MESH SIM OVERRIDES (appended, must stay LAST) --------------
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
EOF'

echo "safe_robot up. Launch the world with:"
echo "  docker exec -it safe_robot bash -ic 'ros2 launch chain_robot chain_robot2.launch.py'"
