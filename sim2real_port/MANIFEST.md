# Manifest

Written by `sync_from_sim.sh` on 2026-11-11. Every file below is an unmodified copy of the sim-tree file
named beside it. Run `sim2real_port/sync_from_sim.sh --check` to see which copies have drifted.

| File in sim2real_port/ | Copied from | md5 |
|---|---|---|
| rssi/propagation.py | rssi/propagation.py | d7c69498fa63fdf9c4d7be9c6c52a22b |
| rssi/network.py | rssi/network.py | 6b783474c269da37e5106a2fcdd0a758 |
| rssi/roadgraph.py | rssi/roadgraph.py | 07672ef0de814ce64470b1fde72a8e62 |
| rssi/relay_planner.py | rssi/relay_planner.py | fd87c23b4998f630dcae7fc763296451 |
| rssi/margin.py | rssi/margin.py | 560a71a56d9fc533f87d9616aa4d5369 |
| rssi/explore_sim.py | rssi/explore_sim.py | 243007780985dc59bd4db65e19737f08 |
| rssi/map_coverage.py | rssi/map_coverage.py | 71b9bf447d2605d9f1d76a8633497375 |
| rssi/ros/exploration_coordinator.py | rssi/ros/exploration_coordinator.py | e16ed3c4d34422ea269f26ae0e19508d |
| rssi/ros/rssi_monitor.py | rssi/ros/rssi_monitor.py | 9f42579395998bca9ccfb93180d30288 |
| rssi/ros/nodes_real_env.yaml | rssi/ros/nodes_real_env.yaml | ae046a0349309fe3c7439e9caf347331 |
| rssi/ros/Dockerfile | rssi/ros/Dockerfile | 340c7a84e6232c18858883c7bbcab392 |
| rssi/ros/run_coordinator.sh | rssi/ros/run_coordinator.sh | efef8904f4ce41d47bd85ef6ffbbb99c |
| rssi/ros/run_rssi_node.sh | rssi/ros/run_rssi_node.sh | de389cf26bf5094f826eed5cf0f04581 |
| hospital_world/mesh_peers.xml | hospital_world/mesh_peers.xml | cb54c2606940bbde715a4380358278c2 |
| hospital_world/tf_relay.py | hospital_world/tf_relay.py | a843541025546555eb8684e4feb1f0c9 |
| hospital_world/map_throttle.py | hospital_world/map_throttle.py | d559c44cb1ee3cb5d8163fdf9979da2c |
| hospital_world/relay_hosts.conf | hospital_world/relay_hosts.conf | ff684366c0304d583e931d3c9bedec79 |
| hospital_world/real_env/gt/real_env_gt.npy | hospital_world/real_env/gt/real_env_gt.npy | d83efc3452d7619501dec69953f8d9aa |
| hospital_world/real_env/gt/real_env_gt.json | hospital_world/real_env/gt/real_env_gt.json | b0a910eebd451b9eac0123fe4ca91e18 |
| hospital_world/real_env/gt/real_env_gt.png | hospital_world/real_env/gt/real_env_gt.png | 8661876b809c47204f6afba3003b2c5d |
| hospital_world/real_env/gt/real_env_radio.npy | hospital_world/real_env/gt/real_env_radio.npy | 6e400bd6c638beee9d7aae5656742d51 |
| hospital_world/real_env/gt/real_env_radio.json | hospital_world/real_env/gt/real_env_radio.json | b0a910eebd451b9eac0123fe4ca91e18 |
| hospital_world/tools/real_env_layout.py | hospital_world/tools/real_env_layout.py | 83a1f06ba2f8d842bce26d6b0e75cc43 |
| rajant_mesh/package.xml | mesh_sim_ws/src/rajant_mesh/package.xml | 45f0126a199932a9cd00bd69b4f4e553 |
| rajant_mesh/setup.py | mesh_sim_ws/src/rajant_mesh/setup.py | ce0599e63c5a4fa6300a830e47bb56dd |
| rajant_mesh/setup.cfg | mesh_sim_ws/src/rajant_mesh/setup.cfg | 9cb8f8323ee54cccddf1049987578588 |
| rajant_mesh/resource/rajant_mesh | mesh_sim_ws/src/rajant_mesh/resource/rajant_mesh | d41d8cd98f00b204e9800998ecf8427e |
| rajant_mesh/launch/mesh_launch.py | mesh_sim_ws/src/rajant_mesh/launch/mesh_launch.py | 5500700fd2b867507a14fb557117c438 |
| rajant_mesh/rajant_mesh/__init__.py | mesh_sim_ws/src/rajant_mesh/rajant_mesh/__init__.py | d41d8cd98f00b204e9800998ecf8427e |
| rajant_mesh/rajant_mesh/mesh_matrix.py | mesh_sim_ws/src/rajant_mesh/rajant_mesh/mesh_matrix.py | cd881ffdde35bd5c64ae25604f3b5769 |
| rajant_mesh/rajant_mesh/weakest_link.py | mesh_sim_ws/src/rajant_mesh/rajant_mesh/weakest_link.py | 7c2a10745aa5527b153a89c33a6d3c99 |
| reference/ackermann_relay/run_relay.sh | hospital_world/run_relay.sh | 39da196cbfef67d778be3132ea02c0d0 |
| reference/ackermann_relay/tuned_tol25.yaml.in | hospital_world/relay_params/tuned_tol25.yaml.in | 702a26c598bf9fa849a5b8afc895b4d1 |
| reference/chain_explorer/run_exploration.sh | rssi/ros/run_exploration.sh | c9855de87589b211d4598f25d5c79e25 |
| reference/sim_run/run_full_live.sh | rssi/ros/run_full_live.sh | 4292227bac6b227c5d8b671f243d7433 |
| reference/sim_run/overnight.sh | rssi/ros/overnight.sh | bb405abace5552915369c116697580b2 |
| reference/live14/coordinator.log | rssi/out/live_runs/real_full_live14_coordinator.log | 0e5ec3dc596d7ecf756598b3b6cbbc81 |
| reference/rssi_README.md | rssi/README.md | 3b1aeaad3ba59138fc00341ca76c900e |
| reference/sim_relay_CHECKPOINT.md | hospital_world/CHECKPOINT.md | 425fbdec68eab70d7656fd3dd378394e |
| test_data/live14_final_map.npz | rssi/out/live_runs/real_full_live14_maps/map_1794260316.npz | 0b81cec88a74578de3628445db7d9265 |
