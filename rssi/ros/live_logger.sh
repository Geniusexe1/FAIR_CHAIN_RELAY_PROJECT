#!/usr/bin/env bash
# Record a live exploration run from Gazebo ground truth, every PERIOD seconds:
#   time, sim RTF, robot poses (world), closest robot-robot distance, decisions so far
#
#   rssi/ros/live_logger.sh rssi/out/live_runs/<name>.csv [period_s]
#
# Poses come from Gazebo itself (gz model -p), not from odometry, so a collision or a
# robot pushed by another shows up as it really happened. Appends if the file exists.
set -uo pipefail
OUT="${1:?usage: live_logger.sh <out.csv> [period_s]}"; PERIOD="${2:-20}"
mkdir -p "$(dirname "$OUT")"
[ -s "$OUT" ] || echo "wall_time,rtf,head_x,head_y,relay1_x,relay1_y,relay2_x,relay2_y,min_sep_m,closest_pair,decisions,client_only,deploy,skip,waypoint_failures" > "$OUT"
while true; do
  P=$(docker exec safe_robot bash -c 'export GAZEBO_IP=127.0.0.1
    for m in chain_robot relay1 relay2; do echo "$m $(timeout 6 gz model -m $m -p 2>/dev/null)"; done
    echo "rtf $(timeout 6 gz stats -p 2>/dev/null | tail -n 1 | cut -d, -f1)"' 2>/dev/null)
  # counts, not the log itself: the log outgrows the 128 KB limit on one argument
  C=$(docker logs fair_coord 2>&1 | awk '/goal world/{g++} /CLIENT_ONLY/{c++} /DEPLOY/{d++} /SKIP/{s++} /did not reach/{f++}
        END {printf "%d,%d,%d,%d,%d", g, c, d, s, f}')
  python3 - "$P" "$C" >> "$OUT" <<'PY'
import sys, math, time, itertools
pose, counts = sys.argv[1], sys.argv[2]
xy, rtf = {}, ""
for line in pose.splitlines():
    f = line.split()
    if f and f[0] == "rtf":
        rtf = f[1] if len(f) > 1 else ""
    elif len(f) >= 3:
        xy[f[0]] = (float(f[1]), float(f[2]))
names = ["chain_robot", "relay1", "relay2"]
pairs = [(math.dist(xy[a], xy[b]), f"{a}-{b}") for a, b in itertools.combinations(names, 2) if a in xy and b in xy]
d, pair = min(pairs) if pairs else (float("nan"), "")
row = [time.strftime("%H:%M:%S"), rtf] + [f"{xy[n][i]:.2f}" if n in xy else "" for n in names for i in (0, 1)] + \
      [f"{d:.2f}", pair, counts]
print(",".join(map(str, row)))
PY
  sleep "$PERIOD"
done
