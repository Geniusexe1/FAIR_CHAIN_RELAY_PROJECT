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

## Measured: Rajant AG1-5250M pair (2026-10-07)

`rajant_link_quality.py`, 3 snapshots, two radios powered:

| band | readings (dBm) | note |
|---|---|---|
| 5 GHz `wlan1` ch 161 | -67 -64 -64 / -63 -64 -63, mean **-64.2** | stable; the band InstaMesh routes over |
| 2.4 GHz `wlan0` ch 1 | -40 -46 -52 / -25 -32 -37 | +-12 dB, 15 dB asymmetric - unusable for calibration |

`P0 = RSSI + 10 n log10(d)` needs the separation `d` at measurement time: -64.2 dBm at 1 m,
-54.7 at 3 m (n=2), -44.2 at 10 m (n=2). **Pending: the actual separation.**

## Defaults are placeholders

`P0 = -40 dBm`, `n = 2.0`, `alpha = 20`, threshold `-85 dBm`. With `n = 2` the whole
hospital is in range and the relays are never needed; `n = 3` makes them necessary.
Replace with measured or paper values.

## Known model properties

- `dbar` measures detour **length**, not walls crossed: a link that grazes a corner is
  NLOS but costs almost nothing.
- Sealed rooms have no free-space path, so they get no signal at all. Real RF would
  penetrate the walls.
