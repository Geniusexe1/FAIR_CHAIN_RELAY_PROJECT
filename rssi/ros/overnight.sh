#!/usr/bin/env bash
# Run a queue of recorded live runs unattended, on the big NUC. One line per run in the queue file:
#   <name> <world: unist|hospital> <P0> [extra coordinator args...]
#
#   DEADLINE="2026-11-05 07:30" setsid nohup rssi/ros/overnight.sh queue.txt > rssi/out/live_runs/overnight.log 2>&1 &
#
# Each run gets a fair share of the time left. It ends when the map is complete, when coverage has
# not grown for PLATEAU_MIN, when FAST-LIO has diverged from Gazebo, or at its share. Everything
# goes to rssi/out/live_runs/<name>*: csv, logs, map snapshots, Gazebo video, replay, plot, result.
set -uo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"; cd "$ROOT"
Q="${1:?queue file}"; OUT=rssi/out/live_runs; PY=.venv/bin/python; mkdir -p "$OUT"
PWD_REMOTE=${COORD_SSH:+FAIR_CHAIN_RELAY_PROJECT}; PWD_REMOTE=${PWD_REMOTE:-$ROOT}
END=$(date -d "${DEADLINE:?}" +%s); PLATEAU_MIN="${PLATEAU_MIN:-30}"; DONE_PCT="${DONE_PCT:-99.5}"

# on the GCS when COORD_SSH names one
G() { if [ -n "${COORD_SSH:-}" ]; then timeout 60 ssh -n -o BatchMode=yes -o ConnectTimeout=15 "$COORD_SSH" "$@"; else "$@"; fi; }
stop_run() {
  pkill -f "[r]un_full_live.sh"; pkill -f "[l]ive_logger.sh"; G docker rm -f fair_coord fair_rssi >/dev/null 2>&1
  docker exec safe_robot pkill -f "[f]air_explore_loop" 2>/dev/null
  hospital_world/stop_all.sh >/dev/null 2>&1
  docker restart safe_robot >/dev/null 2>&1; sleep 5
}
COV_ARGS=""; REPLAY_ARGS=""
coverage() { G "cd $PWD_REMOTE && $PY rssi/map_coverage.py $OUT/$1_maps --gt $2 --start $3 $4 $COV_ARGS" 2>/dev/null | awk '{print $3}' | tr -d '%'; }

mapfile -t QUEUE < <(grep -v '^#' "$Q" | grep .)
for i in "${!QUEUE[@]}"; do
  read -r NAME WORLD P0 EXTRA <<< "${QUEUE[$i]}"
  LEFT=$(( ${#QUEUE[@]} - i )); SHARE=$(( (END - $(date +%s)) / LEFT ))
  [ "$SHARE" -lt 1200 ] && { echo "$(date +%T) $NAME skipped: only ${SHARE}s left"; echo "$NAME result=skipped" >> "$OUT/overnight_results.txt"; continue; }
  echo "$(date +%T) === $NAME ($WORLD, P0 $P0, $EXTRA) share $((SHARE / 60)) min"
  stop_run
  rm -rf "$OUT/${NAME}_maps" "$OUT/$NAME.csv"; G rm -rf "$PWD_REMOTE/$OUT/${NAME}_maps"
  if [ "$WORLD" = unist ]; then
    export WORLD_FILE=/hw/unist/unist_indoor.world MAPPING_LAUNCH=/hw/unist/unist_mapping.launch.xml MAP_ODOM_X=0.5 MAP_ODOM_Y=6.0 \
           NODES=${UNIST_NODES:-rssi/ros/nodes_unist.yaml} PARK_YAW=1.5708
    GT=hospital_world/unist/gt/unist_gt; GCS="1.5 -6.1"; SX=-0.5; SY=-6
    ARGS="--config /fair/$NODES --gt $GT --map-offset -0.5 -6 --marker-offset 0.5 6"; COV_ARGS=""; REPLAY_ARGS=""
  elif [ "$WORLD" = real ]; then
    export WORLD_FILE=/hw/real_env/real_env.world MAPPING_LAUNCH=/hw/real_env/real_env_mapping.launch.xml MAP_ODOM_X=-4.4 MAP_ODOM_Y=-5.0 \
           NODES=rssi/ros/nodes_real_env.yaml PARK_YAW=1.5708
    GT=hospital_world/real_env/gt/real_env_gt; GCS="5.5 7.7"; SX=4.4; SY=5.0; COV_ARGS=""
    REPLAY_ARGS="--radio-gt hospital_world/real_env/gt/real_env_radio"
    ARGS="--config /fair/$NODES --gt $GT --radio-gt hospital_world/real_env/gt/real_env_radio --map-offset 4.4 5.0 --marker-offset -4.4 -5.0 --state-log /live_runs/${NAME}_relay_states.jsonl"
  else
    unset WORLD_FILE MAPPING_LAUNCH MAP_ODOM_X MAP_ODOM_Y NODES PARK_YAW
    GT=hospital_world/gt/hospital_gt; GCS="0 14"; SX=0; SY=12; ARGS=""; COV_ARGS=""; REPLAY_ARGS=""
  fi
  # A launch with any '!!' never explores (real_full_live8: Nav2 left unconfigured, 96 min at 0%):
  # tear it down and launch again, up to 3 times.
  LAUNCHED=0
  for TRY in 1 2 3; do
    T0=$(date +%s)
    # Gazebo's window appears partway through the launch: keep trying until the recorder starts
    ( for _ in $(seq 60); do rssi/ros/record_gazebo.sh start "$OUT/$NAME" 2>/dev/null | grep -q recording && break; sleep 10; done ) &
    REC=$!
    LOG_PERIOD=5 rssi/ros/run_full_live.sh "$NAME" --P0 "$P0" $ARGS --map-log "/live_runs/${NAME}_maps" $EXTRA > "$OUT/${NAME}_launch.log" 2>&1
    RC=$?
    wait "$REC"
    NBAD=$(grep -c '!!' "$OUT/${NAME}_launch.log")
    echo "$(date +%T) $NAME launched in $(( $(date +%s) - T0 )) s: $NBAD launch problems (try $TRY, exit $RC)"
    [ "$RC" -eq 0 ] && [ "$NBAD" -eq 0 ] && { LAUNCHED=1; break; }
    cp "$OUT/${NAME}_launch.log" "$OUT/${NAME}_launch_try$TRY.log"
    rssi/ros/record_gazebo.sh stop "$OUT/$NAME" > /dev/null 2>&1
    stop_run
    rm -rf "$OUT/${NAME}_maps" "$OUT/$NAME.csv"; G rm -rf "$PWD_REMOTE/$OUT/${NAME}_maps"
  done
  WHY=share; BEST=0; TBEST=$(date +%s); BAD=0; COV=0
  [ "$LAUNCHED" = 1 ] || WHY=launch_failed
  while [ "$LAUNCHED" = 1 ] && [ $(( $(date +%s) - T0 )) -lt "$SHARE" ]; do
    sleep 120
    COV=$(coverage "$NAME" "$GT" "$SX" "$SY"); COV=${COV:-0}
    if python3 -c "import sys; sys.exit(0 if $COV > $BEST + 0.2 else 1)"; then BEST=$COV; TBEST=$(date +%s); fi
    echo "$(date +%T) $NAME coverage $COV% $(tail -n 1 "$OUT/$NAME.csv" | cut -d, -f2)"
    python3 -c "import sys; sys.exit(0 if $COV >= $DONE_PCT else 1)" && { WHY=complete; break; }
    docker exec safe_robot pgrep -x gzserver >/dev/null || { WHY=gzserver_died; break; }
    [ $(( $(date +%s) - TBEST )) -gt $(( PLATEAU_MIN * 60 )) ] && { WHY=plateau; break; }
    # FAST-LIO against Gazebo, read together: the map is worthless once they disagree. The two reads are
    # seconds apart, so a robot driving at full speed shows up to ~2 m; only a sustained gap counts.
    D=$(docker exec safe_robot bash -ic "export ROS_DOMAIN_ID=42 GAZEBO_IP=127.0.0.1; timeout 15 ros2 topic echo --once /Odometry --field pose.pose.position; echo gz \$(timeout 6 gz model -m chain_robot -p)" 2>/dev/null \
        | awk -v sx="$SX" -v sy="$SY" '/^x:/{x=$2} /^y:/{y=$2} /^gz /{gx=$2; gy=$3} END{if (x!="" && gx!="") printf "%.1f", sqrt((x+sx-gx)^2+(y+sy-gy)^2)}')
    echo "$(date +%T) $NAME fastlio_vs_gazebo ${D:-?} m"
    if [ -n "$D" ] && python3 -c "import sys; sys.exit(0 if $D > 5.0 else 1)"; then BAD=$((BAD + 1)); else BAD=0; fi
    [ "$BAD" -ge 3 ] && { WHY=fastlio_diverged; break; }
    # how fast each relay brain answers across the mesh
    for r in $(python3 -c "import yaml, os; print(*list(yaml.safe_load(open(os.environ.get('NODES', 'rssi/ros/nodes.yaml'))))[1:-1])"); do
      docker exec safe_robot bash -ic "export ROS_DOMAIN_ID=42; timeout 6 ros2 topic hz /$r/odom 2>/dev/null | grep -m1 'average rate'" 2>/dev/null \
        | sed "s/^/$(date +%T) $r odom /" >> "$OUT/${NAME}_rates.log"
    done
  done
  # ---- save everything before anything is stopped
  G docker logs fair_coord > "$OUT/${NAME}_coordinator.log" 2>&1
  G docker logs fair_rssi > "$OUT/${NAME}_rssi.log" 2>&1
  [ -n "${COORD_SSH:-}" ] && rsync -a "$COORD_SSH:$PWD_REMOTE/$OUT/${NAME}_maps" "$OUT/"
  [ -n "${COORD_SSH:-}" ] && rsync -a "$COORD_SSH:$PWD_REMOTE/$OUT/${NAME}_relay_states.jsonl" "$OUT/" 2>/dev/null
  docker cp safe_robot:/tmp/contacts.jsonl "$OUT/${NAME}_contacts.jsonl" 2>/dev/null
  docker cp safe_robot:/tmp/contact_log.err "$OUT/${NAME}_contact_log.err" 2>/dev/null
  mkdir -p "$OUT/${NAME}_stacklogs"
  docker exec safe_robot bash -c 'cd /tmp; tar cf - 1_gazebo.log 2_fastlio.log 3_occ.log 4_ctrl.log 5_planner.log 6_nav2.log 7a_detector.log 7b_explore.log 2>/dev/null' | tar xf - -C "$OUT/${NAME}_stacklogs"
  while read -r ns target container; do
    case "$ns" in ''|\#*) continue ;; esac
    T="docker exec $container tail -c 3000000 /tmp/${ns}_nav.log"
    if [ "$target" = local ]; then bash -c "$T"; else ssh -n -o BatchMode=yes -o ConnectTimeout=10 "$target" "$T"; fi > "$OUT/${NAME}_stacklogs/${ns}_nav.log" 2>/dev/null
  done < "${RELAY_HOSTS:-hospital_world/relay_hosts.conf}"
  $PY rssi/map_coverage.py "$OUT/${NAME}_maps" --gt "$GT" --start "$SX" "$SY" --all $COV_ARGS > "$OUT/${NAME}_coverage.txt" 2>&1
  CROP="${CROP:-690:1232:270:68}" SPEEDUP=40 rssi/ros/record_gazebo.sh stop "$OUT/$NAME" > "$OUT/${NAME}_video.log" 2>&1
  stop_run
  FAIR_GT="$GT" FAIR_GCS="${GCS/ /,}" $PY rssi/plot_live_run.py "$OUT/$NAME.csv" "$OUT/${NAME}_coordinator.log" >> "$OUT/${NAME}_video.log" 2>&1
  $PY rssi/replay_live_run.py "$OUT/$NAME.csv" "$OUT/${NAME}_coordinator.log" --gt "$GT" --gcs $GCS --P0 "$P0" --maps "$OUT/${NAME}_maps" $REPLAY_ARGS $([ -s "$OUT/${NAME}_relay_states.jsonl" ] && echo --states "$OUT/${NAME}_relay_states.jsonl") >> "$OUT/${NAME}_video.log" 2>&1
  L=$(tail -n 1 "$OUT/${NAME}_coverage.txt" | awk '{print $3}')
  echo "$NAME result=$WHY coverage=$L wall_min=$(( ($(date +%s) - T0) / 60 )) world=$WORLD P0=$P0 extra=$EXTRA" | tee -a "$OUT/overnight_results.txt"
done
echo "$(date +%T) OVERNIGHT_DONE"
