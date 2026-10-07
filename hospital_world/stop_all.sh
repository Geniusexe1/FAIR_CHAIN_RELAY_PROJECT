#!/usr/bin/env bash
# Stop everything on both NUCs. Leaves the containers themselves running.
set -uo pipefail
source "$HOME/.fair_chain.env" 2>/dev/null || { echo "missing ~/.fair_chain.env (needs NUC2_SSH_PASSWORD)"; exit 1; }
echo "[NUC1] safe_robot (gazebo + chain stack)..."
docker exec safe_robot bash -c 'for p in $(ps -eo pid,args --no-headers \
  | grep -vE "grep|ps -eo" \
  | grep -E "gzserver|gzclient|fastlio|rviz2|occ_grid|nav2_|frontier|chain_robot|path_planner|pointcloud_to_laser|static_transform|robot_state_pub|classical|bin/ros2|mapping|topic pub" \
  | awk "{print \$1}"); do kill -9 $p 2>/dev/null; done' 2>/dev/null || true
echo "[NUC1] nuc1 (relay1 brain)..."
docker exec nuc1 bash -c 'pkill -9 -f "nav_robot|nav2_|bin/ros2" 2>/dev/null' 2>/dev/null || true
echo "[NUC2] nuc2 (relay2 brain)..."
python3 - <<'PY' 2>/dev/null || true
import paramiko, os
c=paramiko.SSHClient(); c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
c.connect('10.195.221.0',username='asl',password=os.environ['NUC2_SSH_PASSWORD'],timeout=15)
c.exec_command('docker exec nuc2 bash -c \'pkill -9 -f "nav_robot|nav2_|bin/ros2"\'')
try: c.close()
except Exception: pass
PY
sleep 4
echo "--- still alive on NUC1 (blank = clean) ---"
docker exec safe_robot bash -c 'ps -eo args --no-headers | grep -E "[g]zserver|[g]zclient|[f]astlio|[n]av2_" | grep -v defunct | cut -c1-45' 2>/dev/null || true
