#!/usr/bin/env bash
# Start relay2's DECISION-MAKING on NUC2. Gazebo stays on NUC1 - always.
#
# Split, mirroring mesh_sim_ws:
#   NUC1  gazebo + physics + chain_robot + relay1   (and relay2's MODEL)
#   NUC2  relay2's Nav2 / perception only
#
# relay2's model lives in NUC1's gzserver because physics cannot be split. What
# runs here is the stack that consumes /relay2/scan + /relay2/odom and publishes
# /relay2/cmd_vel back across the mesh.
#
# Prereqs: NUC1 up (run_hospital.sh) and relay2 spawned (spawn_saye.sh relay2 ...).
set -euo pipefail
source "$HOME/.fair_chain.env" 2>/dev/null || { echo "missing ~/.fair_chain.env (needs NUC2_SSH_PASSWORD)"; exit 1; }
NUC2=10.195.221.0     # mesh IP. 192.168.7.56 does not respond.
NS="${1:-relay2}"

sshpass_py() {
  python3 - "$1" <<'PY'
import paramiko, sys, os
cmd = sys.argv[1]
c = paramiko.SSHClient(); c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
c.connect('10.195.221.0', username='asl', password=os.environ['NUC2_SSH_PASSWORD'], timeout=20)
i, o, e = c.exec_command(cmd, timeout=120)
print(o.read().decode(), end=''); err = e.read().decode()
if err.strip(): print('ERR:', err[:500])
try: c.close()
except Exception: pass
PY
}

echo "[relay2] checking NUC1 topics are visible from NUC2 on domain 42..."
sshpass_py "docker exec nuc2 /entrypoint.sh bash -c \"
  export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=0
  timeout 25 ros2 topic list 2>/dev/null | grep -E '^/(clock|${NS}/)' || echo 'NOT VISIBLE - is NUC1 up and ${NS} spawned?'\""

echo "[relay2] starting Nav2 for ${NS} on NUC2..."
sshpass_py "docker exec -d nuc2 /entrypoint.sh bash -c \"
  export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=0
  ros2 launch saye_bringup nav_robot.launch.py namespace:=${NS} static_map_odom:=true map_odom_x:=${MAP_ODOM_X:-0.0} map_odom_y:=${MAP_ODOM_Y:--12.0} \
    > /tmp/${NS}_nav.log 2>&1\""
echo "[relay2] launched. tail it with:"
echo "  ssh asl@${NUC2} 'docker exec nuc2 tail -f /tmp/${NS}_nav.log'"
