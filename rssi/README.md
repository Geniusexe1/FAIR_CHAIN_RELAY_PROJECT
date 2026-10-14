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
