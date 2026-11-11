#!/usr/bin/env bash
# Re-copy the port files from the sim tree and rewrite MANIFEST.md (source path + md5 of every copy).
#
#   sim2real_port/sync_from_sim.sh          # copy, then write the manifest
#   sim2real_port/sync_from_sim.sh --check  # copy nothing; list copies that differ from the sim tree
#
# The copies keep the sim tree's relative paths (rssi/, rssi/ros/, hospital_world/), because the code
# finds its modules and maps by them: exploration_coordinator.py imports from ../ and resolves --gt
# against ../../. Files under reference/ are for reading only (they are replaced in the port).
# PORTING_GUIDE.md, smoke_test.py, rajant_tools/ and this script live only here and are never overwritten.
set -euo pipefail
PORT="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(dirname "$PORT")"

# source (repo-relative)  ->  destination (relative to sim2real_port/)
FILES="
rssi/propagation.py                                   rssi/propagation.py
rssi/network.py                                       rssi/network.py
rssi/roadgraph.py                                     rssi/roadgraph.py
rssi/relay_planner.py                                 rssi/relay_planner.py
rssi/margin.py                                        rssi/margin.py
rssi/explore_sim.py                                   rssi/explore_sim.py
rssi/map_coverage.py                                  rssi/map_coverage.py
rssi/ros/exploration_coordinator.py                   rssi/ros/exploration_coordinator.py
rssi/ros/rssi_monitor.py                              rssi/ros/rssi_monitor.py
rssi/ros/nodes_real_env.yaml                          rssi/ros/nodes_real_env.yaml
rssi/ros/Dockerfile                                   rssi/ros/Dockerfile
rssi/ros/run_coordinator.sh                           rssi/ros/run_coordinator.sh
rssi/ros/run_rssi_node.sh                             rssi/ros/run_rssi_node.sh
hospital_world/mesh_peers.xml                         hospital_world/mesh_peers.xml
hospital_world/tf_relay.py                            hospital_world/tf_relay.py
hospital_world/map_throttle.py                        hospital_world/map_throttle.py
hospital_world/relay_hosts.conf                       hospital_world/relay_hosts.conf
hospital_world/real_env/gt/real_env_gt.npy            hospital_world/real_env/gt/real_env_gt.npy
hospital_world/real_env/gt/real_env_gt.json           hospital_world/real_env/gt/real_env_gt.json
hospital_world/real_env/gt/real_env_gt.png            hospital_world/real_env/gt/real_env_gt.png
hospital_world/real_env/gt/real_env_radio.npy         hospital_world/real_env/gt/real_env_radio.npy
hospital_world/real_env/gt/real_env_radio.json        hospital_world/real_env/gt/real_env_radio.json
hospital_world/tools/real_env_layout.py               hospital_world/tools/real_env_layout.py
mesh_sim_ws/src/rajant_mesh/package.xml               rajant_mesh/package.xml
mesh_sim_ws/src/rajant_mesh/setup.py                  rajant_mesh/setup.py
mesh_sim_ws/src/rajant_mesh/setup.cfg                 rajant_mesh/setup.cfg
mesh_sim_ws/src/rajant_mesh/resource/rajant_mesh      rajant_mesh/resource/rajant_mesh
mesh_sim_ws/src/rajant_mesh/launch/mesh_launch.py     rajant_mesh/launch/mesh_launch.py
mesh_sim_ws/src/rajant_mesh/rajant_mesh/__init__.py   rajant_mesh/rajant_mesh/__init__.py
mesh_sim_ws/src/rajant_mesh/rajant_mesh/mesh_matrix.py   rajant_mesh/rajant_mesh/mesh_matrix.py
mesh_sim_ws/src/rajant_mesh/rajant_mesh/weakest_link.py  rajant_mesh/rajant_mesh/weakest_link.py
hospital_world/run_relay.sh                           reference/ackermann_relay/run_relay.sh
hospital_world/relay_params/tuned_tol25.yaml.in       reference/ackermann_relay/tuned_tol25.yaml.in
rssi/ros/run_exploration.sh                           reference/chain_explorer/run_exploration.sh
rssi/ros/run_full_live.sh                             reference/sim_run/run_full_live.sh
rssi/ros/overnight.sh                                 reference/sim_run/overnight.sh
rssi/out/live_runs/real_full_live14_coordinator.log   reference/live14/coordinator.log
rssi/README.md                                        reference/rssi_README.md
hospital_world/CHECKPOINT.md                          reference/sim_relay_CHECKPOINT.md
rssi/out/live_runs/real_full_live14_maps/map_1794260316.npz   test_data/live14_final_map.npz
"

if [ "${1:-}" = --check ]; then
  rc=0
  while read -r src dst; do
    [ -n "$src" ] || continue
    cmp -s "$ROOT/$src" "$PORT/$dst" || { echo "DIFFERS  $dst  (sim: $src)"; rc=1; }
  done <<< "$FILES"
  [ $rc = 0 ] && echo "all copies match the sim tree"
  exit $rc
fi

{
  echo "# Manifest"
  echo
  echo "Written by \`sync_from_sim.sh\` on $(date +%F). Every file below is an unmodified copy of the sim-tree file"
  echo "named beside it. Run \`sim2real_port/sync_from_sim.sh --check\` to see which copies have drifted."
  echo
  echo "| File in sim2real_port/ | Copied from | md5 |"
  echo "|---|---|---|"
  while read -r src dst; do
    [ -n "$src" ] || continue
    mkdir -p "$PORT/$(dirname "$dst")"
    cp -p "$ROOT/$src" "$PORT/$dst"
    echo "| $dst | $src | $(md5sum "$PORT/$dst" | cut -c1-32) |"
  done <<< "$FILES"
} > "$PORT/MANIFEST.md"
echo "copied $(grep -c '^| [a-z]' "$PORT/MANIFEST.md") files; manifest: $PORT/MANIFEST.md"
