#!/usr/bin/env python3
"""Offline check that the copied planner works on this machine. No ROS, no robots.

    python3 smoke_test.py            # prints one line per case, then PASS or FAIL
    python3 smoke_test.py --write    # rewrite smoke_test_expected.txt (only after a deliberate planner change)

It rebuilds what exploration_coordinator.py builds at run time: the radio model from the prior grid
(real_env_radio), the road graph from an explored map (the last /map snapshot of sim run live14), then asks
the planner for a few goals at P0 -57, the setting of that run. The planner is seeded, so the decisions and
relay targets must match smoke_test_expected.txt exactly. Needs numpy, scipy, scikit-fmm, scikit-image
(rssi/ros/Dockerfile has the versions).
"""
import math, os, sys
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "rssi"))
from propagation import GridMap, PropagationParams                      # noqa: E402
from network import NoiseParams                                         # noqa: E402
from roadgraph import RoadGraph                                         # noqa: E402
from relay_planner import Predictor, RelayPlanner, PlannerParams, Robot  # noqa: E402
from scipy.ndimage import binary_closing                                # noqa: E402

P0, GCS = -57.0, (5.5, 7.7)
gt = os.path.join(HERE, "hospital_world/real_env/gt/real_env_radio")
radio = GridMap.load(gt + ".npy", gt + ".json")

# the explored map, resampled onto the radio grid: exploration_coordinator.py _known_free() and _explored()
z = np.load(os.path.join(HERE, "test_data/live14_final_map.npz"))
live, res, o, off = z["grid"], float(z["res"]), z["origin"], z["off"]     # world = map + off
H, W = radio.shape
xs = radio.origin[0] + (np.arange(W) + 0.5) * radio.res - off[0]
ys = radio.origin[1] + (np.arange(H) + 0.5) * radio.res - off[1]
cc, rr = np.floor((xs - o[0]) / res).astype(int), np.floor((ys - o[1]) / res).astype(int)
ins = ((rr >= 0) & (rr < live.shape[0]))[:, None] & ((cc >= 0) & (cc < live.shape[1]))[None, :]
v = live[np.clip(rr, 0, live.shape[0] - 1)[:, None], np.clip(cc, 0, live.shape[1] - 1)[None, :]]
free, unknown = ins & (v >= 0) & (v < 50), ~ins | (v < 0)
free = free | (binary_closing(free, iterations=3) & unknown)
grid = GridMap(occ=(~free).astype(np.uint8), res=radio.res, origin=radio.origin)

pp = PlannerParams(sigma=3.0, d_sep=0.5, pass_clear=0.6)                 # the coordinator's defaults
pred = Predictor(radio, PropagationParams(P0=P0), NoiseParams(sigma_fast=3.0))
planner = RelayPlanner(grid, RoadGraph.build(grid, drop_outside=False), pred, GCS, pp, gcs_name="gcs")
planner.set_map(grid, RoadGraph.build(grid, drop_outside=False), allowed=free)

PARK = [(4.0, 7.4), (4.9, 7.4)]
CASES = [   # name, client xy, relay xys, goal (all world frame)
    ("lobby, relays parked", (4.4, 5.0), PARK, (2.4, 1.1)),
    ("corridor, relays parked", (2.4, 1.1), PARK, (-0.5, -2.9)),
    ("far corridor, relays parked", (2.4, -1.0), PARK, (2.4, -11.5)),
    ("south corridor, relay1 out", (2.4, -6.0), [(2.4, 0.5), (4.9, 7.4)], (2.4, -11.5)),
    ("south room far, both out", (-1.5, -14.2), [(2.4, -6.0), (2.4, -11.0)], (-6.2, -11.1)),
    ("south room, relays out", (1.0, -11.9), [(2.44, 0.57), (2.43, -1.25)], (-1.5, -14.2)),
    ("west classroom, relays out", (-2.0, -3.0), [(2.4, 0.5), (0.5, 0.3)], (-9.9, -4.3)),
]
out = [f"P0 {P0} req {pp.req:.2f} dBm"]
for name, cl, rl, goal in CASES:
    relays = [Robot(f"relay{i + 1}", xy, "relay", 0.5) for i, xy in enumerate(rl)]
    p = planner.plan(goal, Robot("head", cl, "client", 0.5), relays)
    end = p.traj["head"][-1] if "head" in p.traj else cl
    tg = " ".join(f"{n}({x:.1f},{y:.1f})" for n, (x, y) in sorted(p.targets.items()) if n != "head")
    out.append(f"{name:30s} {p.decision:12s} {' > '.join(p.chain):28s} client ends {math.dist(end, goal):.1f} m "
               f"from the goal | {tg or 'no relay moves'}")
    print(out[-1], flush=True)

exp = os.path.join(HERE, "smoke_test_expected.txt")
if "--write" in sys.argv:
    open(exp, "w").write("\n".join(out) + "\n")
    print("wrote", exp)
elif open(exp).read().splitlines() == out:
    print("PASS: matches smoke_test_expected.txt")
else:
    print("FAIL: differs from smoke_test_expected.txt")
    sys.exit(1)
