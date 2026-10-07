#!/usr/bin/env bash
# KNOWN-GOOD: hospital world + chain_robot in Gazebo Classic 11.
# Verified working 2026-08-31.
#
# Both exports are REQUIRED:
#   GAZEBO_MODEL_PATH        - the 100 hospital models live in /hw/models. Without
#                              this Gazebo falls back to the online model database.
#   GAZEBO_MODEL_DATABASE_URI - models.gazebosim.org is RETIRED and never responds.
#                              Left set, Gazebo hangs forever on a missing model
#                              instead of erroring. Empty = fail fast, locally.
set -euo pipefail
docker exec -it safe_robot bash -ic '
export GAZEBO_MODEL_PATH=/hw/models:$GAZEBO_MODEL_PATH
export GAZEBO_MODEL_DATABASE_URI=
  export GAZEBO_IP=127.0.0.1   # enp86s0 is UP with no IP; multicast sends there fail and flood the log
ros2 launch chain_robot chain_robot2.launch.py world:=/hw/hospital_chain.world "$@"' -- "$@"
