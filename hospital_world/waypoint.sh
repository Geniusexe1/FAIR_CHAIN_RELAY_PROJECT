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
HW="$(cd "$(dirname "$0")" && pwd)"
NS="${1:?usage: waypoint.sh <relayN> <x y [x y ...]> | stop}"; shift
read -r _ TARGET CONTAINER < <(awk -v n="$NS" '$1 == n' "$HW/relay_hosts.conf")
[ -n "${TARGET:-}" ] || { echo "$NS is not in relay_hosts.conf"; exit 1; }

if [ "${1:-}" = "stop" ]; then CMD="stop_robot.py $NS"; else CMD="send_waypoints.py $NS $*"; fi
RUN="export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=0; ros2 run saye_bringup $CMD"
ssh -o BatchMode=yes "$TARGET" "docker exec $CONTAINER /entrypoint.sh bash -c '$RUN'"
