#!/usr/bin/env bash
# rssi/ros/relay_bench.sh <config.yaml.in>... : the same relay legs under each Nav2 config. Run on the sim NUC.
set -uo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"; cd "$ROOT"; OUT=rssi/out/relay_bench; mkdir -p "$OUT"
export WORLD_FILE=/hw/real_env/real_env.world MAPPING_LAUNCH=/hw/real_env/real_env_mapping.launch.xml MAP_ODOM_X=-0.4 MAP_ODOM_Y=0.2 \
       NODES=rssi/ros/nodes_real_env.yaml PARK_YAW=-1.5708 STOP_AFTER=4
docker cp rssi/ros/relay_bench.py safe_robot:/tmp/relay_bench.py
FIRST=1
for CFG in "$@"; do
  L=$(basename "$CFG" .yaml.in); export RELAY_PARAMS="$ROOT/$CFG"
  echo "$(date +%T) === $L"
  if [ "$FIRST" = 1 ]; then hospital_world/stop_all.sh >/dev/null 2>&1; docker exec safe_robot pkill -9 gzserver; docker restart safe_robot >/dev/null; sleep 5; rssi/ros/run_full_live.sh bench 2>&1 | grep -E '!!|\[4/5\]|active' ; FIRST=0
    # the bench publishes the true map itself; chain_robot is only parked out of the way, in the lobby
    docker exec safe_robot pkill -f "[m]ap_throttle"
    docker exec safe_robot bash -c 'export GAZEBO_IP=127.0.0.1; gz model -m chain_robot -x 5.0 -y 7.5 -z 0.05'
  else
    # back to the parking spots, Nav2 restarted on the next config
    docker exec safe_robot bash -c 'export GAZEBO_IP=127.0.0.1; gz model -m relay1 -x -0.4 -y 2.6 -z 0.1 -Y -1.5708; gz model -m relay2 -x 0.5 -y 2.6 -z 0.1 -Y -1.5708'
    FROM=3 rssi/ros/run_full_live.sh bench 2>&1 | grep -E '!!|active'
  fi
  docker exec safe_robot bash -ic "export ROS_DOMAIN_ID=42 GAZEBO_IP=127.0.0.1; python3 /tmp/relay_bench.py /tmp/bench.jsonl $L /hw/real_env/gt/real_env_gt.npy /hw/real_env/gt/real_env_gt.json" 2>&1 | grep -E "^\{|BENCH_DONE|Traceback|Error" | tee -a "$OUT/$L.log"
  docker exec safe_robot cat /tmp/bench.jsonl > "$OUT/results.jsonl"
  while read -r ns target container; do case "$ns" in ''|\#*) continue ;; esac
    ssh -n -o BatchMode=yes "$target" "docker exec $container cat /tmp/${ns}_nav.log" > "$OUT/${L}_${ns}_nav.log" 2>/dev/null
  done < hospital_world/relay_hosts.conf
done
echo "$(date +%T) ALL_DONE"
