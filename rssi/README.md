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

## Try it yourself

Place any nodes, get a coverage heatmap + link table:

```bash
.venv/bin/python rssi/heatmap.py --node GCS 0 12
.venv/bin/python rssi/heatmap.py --node GCS 0 12 --node relay1 -6.4 -1 --node head -9 -28
.venv/bin/python rssi/heatmap.py --node GCS 0 12 --node head -9 -28 --noise --n 3 --per-node
```

- the **first** node is the GCS; names starting `relay`/`head` get those markers
- output: `rssi/out/user/<--name>.png` (`--per-node` adds one map per node)
- a node that can't reach the GCS is drawn with a **red** outline
- placing a node in a wall or off the map is rejected, with the valid bounds

## Reproduce the phases

```bash
.venv/bin/python rssi/phase1_heatmap.py --gcs 0 12                 # single-GCS heatmap + 6 checks
.venv/bin/python rssi/phase2_topology.py                           # moving GCS+relays+head, 7 checks
.venv/bin/python rssi/phase2_topology.py --n 3.0 --tag stress_n3   # harsher indoor exponent
```

Positions are **world** coordinates (Gazebo frame). Model parameters: `--P0 --n --alpha`.
Noise: `--sigma-shadow --corr --sigma-fast`. Link threshold: `--threshold`.

## Files

| file | what |
|---|---|
| `propagation.py` | the model: grid, LOS, geodesic distance, eq. (1) |
| `network.py` | nodes, noise (shadowing + fast fading), links, connectivity |
| `phase1_heatmap.py` | Phase 1 heatmap + checks |
| `phase2_topology.py` | Phase 2 dynamic topology + checks |
| `out/` | rendered results |

## Noise model

- **Shadowing** — log-normal, spatially correlated (`sigma_shadow` dB over `corr_m` m),
  one seeded field per transmitter, averaged both ways so links are symmetric.
- **Fast fading** — i.i.d. `N(0, sigma_fast)` per reading.

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

`rssi/ros/nodes.yaml` sets the nodes: the GCS is fixed at world (0, 14); relays come from
`/relayN/odom` (world frame); the head from FAST-LIO `/Odometry` + (0, 12). Change positions or
add nodes there, then restart. Options: `--no-noise`, `--P0`, `--n`, `--alpha`, `--threshold`,
`--rate`. Image: `rssi/ros/Dockerfile` = `mesh_sim:humble` + scikit-fmm.

Nothing drives the relays yet, so values only change by noise and by their slow idle creep
(~2 mm/s).

## Known model properties

- `dbar` measures detour **length**, not walls crossed: a link that grazes a corner is
  NLOS but costs almost nothing.
- Sealed rooms have no free-space path, so they get no signal at all. Real RF would
  penetrate the walls.
