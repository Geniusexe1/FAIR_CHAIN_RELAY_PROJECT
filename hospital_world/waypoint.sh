#!/usr/bin/env bash
# Send waypoints to a relay. Routes to whichever machine owns that relay's brain.
#
#   ./waypoint.sh relay1 5.0 0.0                 one pair  -> NavigateToPose
#   ./waypoint.sh relay2 3.0 0.0  6.0 2.0        several   -> FollowWaypoints
#   ./waypoint.sh relay1 stop                    cancel + zero cmd_vel
#
# Coordinates are metres in the shared `map` frame. Orientation is ignored -
# Ackermann cars cannot rotate in place (CLAUDE.md invariant 6).
#
# NOTE on the frame: `map` is the CHAIN ROBOT's map origin, which is its spawn at
# world (0, 12) - not the world origin. So map (0,0) is world (0,12).
set -euo pipefail
source "$HOME/.fair_chain.env" 2>/dev/null || { echo "missing ~/.fair_chain.env (needs NUC2_SSH_PASSWORD)"; exit 1; }
NS="${1:?usage: waypoint.sh <relay1|relay2> <x y [x y ...]> | stop}"; shift

if [ "${1:-}" = "stop" ]; then CMD="stop_robot.py $NS"; else CMD="send_waypoints.py $NS $*"; fi
RUN="export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=0; ros2 run saye_bringup $CMD"

case "$NS" in
  relay1) docker exec -it nuc1 /entrypoint.sh bash -c "$RUN" ;;
  relay2) python3 - "$RUN" <<'PY'
import paramiko, sys, os
c = paramiko.SSHClient(); c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
c.connect('10.195.221.0', username='asl', password=os.environ['NUC2_SSH_PASSWORD'], timeout=20)
i, o, e = c.exec_command("docker exec nuc2 /entrypoint.sh bash -c '%s'" % sys.argv[1], timeout=300)
for line in iter(o.readline, ''): print(line, end='')
err = e.read().decode()
if err.strip(): print('ERR:', err[:400])
try: c.close()
except Exception: pass
PY
    ;;
  *) echo "unknown relay '$NS' (expected relay1 or relay2)"; exit 1 ;;
esac
