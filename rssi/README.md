# RSSI propagation model

Obstacle-aware signal model on the ground-truth occupancy grid
(`hospital_world/gt/hospital_gt.npy`).

    P(a,b) = P0 - 10 n log10(dE) - alpha log10(dbar + 1)      dbar = |dE - dS|

- `dE` straight-line distance, `dS` shortest free-space path around walls (fast marching)
- line of sight is ray-cast and forced to `dbar = 0` exactly
- cells with no free-space path to the transmitter (sealed rooms, outside) get no signal

## Setup

```bash
python3 -m venv --without-pip .venv
curl -fsSL https://bootstrap.pypa.io/get-pip.py | .venv/bin/python
.venv/bin/pip install numpy scipy matplotlib scikit-fmm
```

## Connectivity margin map

The heat map is a **client-to-GCS connectivity margin**, not "RSSI to the GCS". GCS and
relays are held where they are; the client is placed virtually at every cell, keeping its
**current** relay association (it never switches to whichever node is strongest there):

    margin(x) = min( client@x <-> attached relay,
                     weakest hop on the attached relay's current path to the GCS ) - eta

- every link uses its **weaker direction** (bidirectional connectivity)
- a strong link to the relay does not help if the relay's own upstream path is weak:
  the margin is capped at (weakest upstream hop - eta) everywhere
- **solid contour = zero margin**, the predicted boundary of the connected region
- **dashed contour = strong link**: end-to-end RSSI of `--strong` dBm (default -65, the paper's
  green band), i.e. margin = -65 - eta = **+20 dB**
- overlaid: the client's actual position and the current links (chain hops + client link),
  coloured by the paper's bands, labelled with their RSSI (dBm)
- colour bar: "Client-to-GCS connectivity margin (dB)"; grey = no propagation path at all
- code: `margin.py` (`margin_map`, `draw_margin`, `margin_checks`); examples: `out/v3_margin_strong-65dBm/margin_example*.png`

## Try it yourself

Place nodes **in chain order**: GCS first, client last. The client stays attached to the
node before it (or `--attach NAME`):

```bash
.venv/bin/python rssi/heatmap.py --node GCS 0 14 --node relay2 5 7 --node relay1 5 -1 --node head 5 -8 --noise
.venv/bin/python rssi/heatmap.py ...same... --attach relay2            # head on relay2 instead
.venv/bin/python rssi/heatmap.py ...same... --strong -70               # move the dashed contour (dBm)
.venv/bin/python rssi/heatmap.py ...same... --coverage                 # old best-RSSI map
```

- output: `rssi/out/user/<--name>.png` + a link table + a one-line margin summary
- placing a node in a wall or off the map is rejected, with the valid bounds

## Reproduce the phases

```bash
.venv/bin/python rssi/phase1_heatmap.py --gcs 0 12                 # single-GCS heatmap + 6 checks
.venv/bin/python rssi/phase2_topology.py                           # moving relays+head, 14 checks
.venv/bin/python rssi/phase2_topology.py --n 3.0 --tag stress_n3   # harsher indoor exponent
```

Positions are **world** coordinates (Gazebo frame). Model parameters: `--P0 --n --alpha`.
Noise: `--sigma-shadow --corr --sigma-fast`. Link threshold: `--threshold`.

## Communication-aware exploration (relay deployment)

chain_robot (the client) explores frontier by frontier until the map is covered; relays
are deployed so it keeps a link to the GCS. Method: Seo et al., RA-L 2025, Sec. IV-B.

For each next frontier (nearest first, as the explorer ranks them):

| situation | action |
|---|---|
| goal inside the client's **bounded region** for the current chain | client goes alone |
| outside | engage one more relay: **road graph** (Zhang-Suen skeleton), shortest path GCS -> goal split evenly, **virtual-force** refinement (eq. 12, beta 2, nu 1.5 dB, B_v 1000), **Hungarian** assignment (eq. 13-14), then **sequential MCTS** trajectories GCS-outwards, client last (eq. 16-17, t_f 5, C_p 1, 300 iterations) |
| outside even with every relay engaged and re-placed | **skip** it (the explorer moves to its next frontier) |

- **Bounded region** = where Pr[every hop >= eta] >= theta with the client's association
  fixed. Our links take the weaker of two noisy directions, so per direction the predicted
  RSSI must reach eta + sigma_n PHI^-1(sqrt(theta)) = **-81.7 dBm** (eta -85, sigma_n 3, theta 0.75).
- **Synchronised arrival**: every trajectory has t_f waypoints; in interval t each robot covers
  its own segment, the interval lasts as long as the slowest needs at full speed, the others
  slow down, and everyone reaches waypoint t together (`relay_planner.schedule`). On the
  robots the speeds go out as Nav2 speed limits and are re-balanced every 0.5 s from each
  robot's remaining distance.
- Engaged relays are **re-placed** for each new mission point (the paper). `--keep-deployed`
  instead never moves a relay once placed (cuts coverage; see results).
- **Safe moves only**: if MCTS cannot keep every step above the requirement it retries with
  more routes and finer steps (|J|, t_f) = (3,5) -> (5,5) -> (5,8) -> (6,12); a move that still
  cannot is not executed -- the goal is skipped.
- **Robots keep apart** (`d_sep` 0.8 m): routes avoid the other robots, MCTS rejects any step
  where two robots come closer than 0.8 m during the interval, and relay spots / goal / parked
  relays must be 0.8 m apart. Idle relays **park off the road graph**, west of the GCS
  (`nodes.yaml` `park`), not on the corridor in the client's way.
- **Known space only** (live): Nav2 plans only through what the robots' own map knows, so the
  coordinator turns `/map` into a known-free mask and routes / waypoints / relay spots stay in it
  (`RoadGraph.routes(allowed=...)`, `RelayPlanner.allowed`).
- Predictions use the GT map and the model's expectation with sigma = sigma_n; the paper's
  KF-GPR needs real RSSI readings to train, which the sim does not have.

Offline (the whole run on the GT map; frames, summary, events.csv, close_passes.csv per run):

```bash
.venv/bin/python rssi/explore_sim.py --P0 -30 --tag my_run            # current method
.venv/bin/python rssi/explore_sim.py --P0 -30 --d-sep 0 --tag no_avoid # robot avoidance off
.venv/bin/python rssi/explore_sim.py --P0 -30 --keep-deployed --tag stay_put
```

Results (`out/explore/`, 2 relays, client 0.7 m/s, relays 0.5 m/s, 8 m mapping range, P0 -30
unless noted). CR = simulated readings >= -85 dBm; "close" = samples with two robots < 0.5 m
apart (bodies are ~0.3 m, so a probable contact):

| run | what changed | coverage | CR | closest / close |
|---|---|---|---|---|
| `v0_*` | first attempt: relays never re-placed, per-direction requirement -83.0 dBm | 37% (P0 -60.8) | 75% | - |
| `v1_paperP0_replace` | paper re-placement, requirement -81.7 dBm | **100%** | 97.1% | - |
| `v1_paperP0_stayPut` | relays never re-placed | 97.9% | 98.0% | - |
| `v1_measuredP0_replace` / `_stayPut` | P0 -60.8 | 48.0% / 27.9% | 80.1% / 88.8% | - |
| `v2_paperP0_replace_safeMoves` | moves that can't keep every step are skipped | 99.3% | **99.4%** | - |
| `v3_paperP0_noAvoid_baseline` | v2 + old relay spawn in the corridor, no avoidance | 99.3% | 99.4% | 0.02 m / 1.7% |
| `v3_paperP0_avoidRobots` | + robot separation, relays parked off the road | 99.3% | 98.8% | 0.06 m / **0.1%** |
| `v4_paperP0_avoidContinuous` | + separation checked through each whole interval | 99.3% | 98.8% | 0.06 m / 0.2% |
| `v5_paperP0_closePassLog` | v4 + every close pass logged: all were the client passing a parked relay | 99.3% | 98.8% | 0.06 m / 0.2% |
| `v6_paperP0_legsAvoid` | sim legs avoid robots (did not help: the relay sat in the only passage) | 99.3% | 99.0% | 0.06 m / 0.2% |
| **`v7_paperP0_routeSeparation`** | separation sampled along each robot's ROUTE, not the straight line | **100%** | **98.7%** | **0.99 m / 0%** |

At the measured 5 GHz P0, two relays cannot cover the hospital, and with every hop near the
limit a 3-hop chain holds only ~0.75^3 of the time -- the per-hop theta is the paper's.
CR dips happen between waypoints: like the paper, the constraint is enforced at the discrete
steps only.

### Live

```bash
rssi/ros/run_full_live.sh run4 --P0 -30   # EVERYTHING: stack, relays parked, brains, Nav2 checks,
                                          # RSSI monitor, coordinator, explorer, ground-truth logger
docker logs -f fair_coord                 # every decision: CLIENT_ONLY / DEPLOY / SKIP / REJECTED
```

The explorer's `navigate_to_pose` action is remapped (its five hidden endpoints -- Humble does
not remap an action by its name) to `/coord/navigate_to_pose`; a rejected goal makes it try its
next-ranked frontier. What the coordinator does besides planning:

| | why |
|---|---|
| restarts a Nav2 stack (wait -> STARTUP -> RESET + STARTUP) when it rejects a goal | at RTF ~0.2 a node misses Nav2's 4 s bond heartbeat and the lifecycle manager shuts the whole stack down |
| aborts the explorer's goal when the client cannot follow its plan; rejects a goal after 2 failures | otherwise the explorer re-sends the same unreachable frontier forever |
| rejects goals within 0.6 m of a reached one or of the client | the explorer re-sends frontiers it cannot clear from where it stands |
| station keeping: an idle relay > 0.3 m from its spot is driven back (also during plans) | the relays creep ~5 mm/s when idle (drive plugin, `hospital_world/CHECKPOINT.md`) |
| re-publishes `/map` cropped to the explored area on `/coord/map` for the explorer + detector | some participants on this host fall back to UDP, where a large map does not arrive |
| plans only through known-free space | Nav2 cannot plan through unknown space |

`rssi/ros/live_logger.sh` records Gazebo ground-truth poses every 20 s (`out/live_runs/*.csv`);
`rssi/plot_live_run.py <csv> [coordinator.log]` draws them on the map (`*.png`).

Live runs (`out/live_runs/`; each has a ground-truth CSV, most a coordinator log and a plot):

| run | P0 | what it showed |
|---|---|---|
| run1, run2 | -30 | before the fixes: chain_robot pushed relay1 ~4 m; /map stopped arriving; Nav2 shut itself down |
| run3 | -30 | coordinator gates every goal; 5 goals reached, 12.6 m; closest robots 0.82 m; then chain_robot wedged 0.10 m from a wall (its start inside Nav2's 0.5 m inflation -> no plan, no recovery) |
| run4, run5 | -45 | **first live DEPLOY decisions**; relays stalled under the drive's speed deadband (fixed: speed floor + delayed start); two relays converged and every plan then failed (fixed: separation rule) |
| run6 | -45 | **2-relay chain executed**: GCS > relay1 > relay2 > head, relays on their targets (within ~0.2 m), client 69.5 m through both wings; the coordinator then crashed on a goal outside the map (fixed) and the unattended relays slid to 0.35 m apart |
| run7 | -45 | all fixes: 2-relay deployment, hops ~-55 dBm, see `run7_*` |

Open: chain_robot wedging itself against walls (its controller / costmap, unmodified), and the
relays' idle slide (drive plugin), which station keeping counters but cannot remove.

## Result folders (kept as a record, never overwritten)

| folder | what the phase-2 frames show |
|---|---|
| `out/v1_coverage/` | original: best RSSI from any node (before the mentor's revision) |
| `out/v2_margin_strong-18dB/` | connectivity margin, dashed contour at margin -18 dB |
| `out/v3_margin_strong-65dBm/` | **current**: connectivity margin, dashed contour at -65 dBm (margin +20 dB) |

Each holds `phase2_measuredP0/` (P0 -60.8) and `phase2_paperP0/` (P0 -30). New runs:
`phase2_topology.py --tag vN_<what>/phase2_measuredP0`.

## Files

| file | what |
|---|---|
| `propagation.py` | the model: grid, LOS, geodesic distance, eq. (1) |
| `network.py` | nodes, noise (shadowing + fast fading), links, connectivity |
| `phase1_heatmap.py` | Phase 1 heatmap + checks |
| `margin.py` | client-to-GCS connectivity margin map + its checks |
| `roadgraph.py` | road graph: Zhang-Suen skeleton of the free space, routes + alternatives |
| `relay_planner.py` | the paper's hierarchical relay planner + synchronised schedule |
| `explore_sim.py` | offline communication-aware exploration on the GT map |
| `ros/exploration_coordinator.py` | live: between the explorer and Nav2 (all robots) |
| `heatmap.py` | place your own nodes, render the margin map |
| `phase2_topology.py` | Phase 2 dynamic topology: every frame is a margin map (head on relay1) |
| `out/` | rendered results |

## Noise model

- **Shadowing** — log-normal, spatially correlated (`sigma_shadow` dB over `corr_m` m),
  one seeded field per transmitter, so a->b and b->a differ; a link's value is the
  **weaker** direction. The path-loss part is reciprocal.
- **Fast fading** — i.i.d. `N(0, sigma_fast)` per reading, per direction.

Maps show clean + shadowing (an expectation surface); link readings add fast fading.

## Parameters

| | value | source |
|---|---|---|
| P0 | **-60.8 dBm** | measured, see below |
| n | 2 | paper (Seo et al., RA-L 2025) Fig. 6c |
| alpha | 35 | paper Fig. 6c |
| sigma_n (per-reading noise) | 3 dB | paper Table I |
| threshold eta | -85 dBm | paper Table I |
| link colours | > -65 green, -65..-85 yellow, < -85 red | paper Fig. 9 |
| shadowing | 4 dB, 3 m correlation | ours (the paper's GP term has no published sigma) |

**P0, measured 2026-10-07:** Rajant Cardinal pair 1 m apart, antennas facing, 5 GHz ch161,
10 snapshots: -61.4 / -60.2 dBm per direction, sd < 1 dB. At 1 m, log10(d) = 0, so this is P0
directly. It is **31 dB below the paper's -30** on the same radio model, i.e. our TX power is
configured low; free-space range to -85 is ~16 m instead of ~280 m. Re-measure if the radios
are reconfigured. Every output also exists with the paper's P0 (`paperP0_*`).

At the measured P0 the two-relay chain cannot cover the 43.7 m Phase 2 route: the head
disconnects at ~30 m because ~14 m hops fall below -85 even with line of sight.

## Live ROS 2 node

Predicts every link from the robots' **current** positions in the sim.

```bash
rssi/ros/run_rssi_node.sh            # own container (fair_rssi), domain 42, --ipc host
docker logs -f fair_rssi             # link table every 5 s
```

From any container on domain 42:

| topic | type | content |
|---|---|---|
| `/rssi/chain` | Float32 | paper eq. (5): weakest hop of the fixed chain GCS > relay1 > relay2 > head |
| `/rssi/end_to_end/<node>` | Float32 | best route from that node to the GCS through any robots |
| `/rssi/link/<a>__<b>` | Float32 | one link, noisy |
| `/rssi/links` | String (JSON) | everything: positions, clean + noisy links, routes, chain |
| `/rssi/markers` | MarkerArray | links coloured by the paper's bands, for RViz (`map` frame) |
| `/rssi/margin/at_client` | Float32 | connectivity margin (dB) at the client's actual position |
| `/rssi/margin/image` | Image (rgb8) | the margin map (`ros2 run rqt_image_view rqt_image_view`) |

The margin map is also written to `rssi/out/live/margin_latest.png`. The client defaults to the
last node in `nodes.yaml` (head), attached to the node before it; `--client relay2 --attach relay1`
maps a relay instead (useful while FAST-LIO isn't running). Hop values are EMA-filtered link
estimates (`--est-alpha 0.1`). The map is re-rendered only when a node changes cell, a hop
estimate moves by more than `--est-change` (2 dB), or the topology changes — at most every
`--map-period` (2 s). Idle, that is about once a minute.

`rssi/ros/nodes.yaml` sets the nodes: the GCS is fixed at world (0, 14); relays come from
`/relayN/odom` (world frame); the head from FAST-LIO `/Odometry` + (0, 12). Change positions or
add nodes there, then restart. Options: `--no-noise`, `--P0`, `--n`, `--alpha`, `--threshold`,
`--rate`. Image: `rssi/ros/Dockerfile` = `mesh_sim:humble` + scikit-fmm.

Nothing drives the relays yet, so values only change by noise and by their slow idle creep
(~2 mm/s).

## Known model properties

- Every radio, including the GCS's Rajant, uses the same P0: the model assumes identical
  radios and antennas on every node.

- `dbar` measures detour **length**, not walls crossed: a link that grazes a corner is
  NLOS but costs almost nothing.
- Sealed rooms have no free-space path, so they get no signal at all. Real RF would
  penetrate the walls.
