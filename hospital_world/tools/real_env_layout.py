#!/usr/bin/env python3
"""The real environment as a Gazebo world and ground-truth grids, from a floor plan measured off the scan.

    .venv/bin/python hospital_world/tools/real_env_layout.py hospital_world/real_env

Frame: the FAST-LIO scan (scans.pcd) turned by -0.8 deg so its walls lie on the axes: u = x cos t + y sin t,
v = -x sin t + y cos t, t = -0.8 deg. The origin is where the real robot started.

The building has straight walls and square rooms, so walls are measured lines (density peaks of the points
1.2 - 2.4 m above the local floor), not the cloud's outline. Column bumps on the walls and the hallway's niche
are left out. The double doors are clear openings (leaves swung right back). Desks were fitted at desktop
height (0.68 m) as one size, 0.6 x 1.6 m, in straight rows, with the panel and end frames the scan shows
under them; chairs are left out, but the chair side of each desk is closed.
"""
import json, os, sys
import numpy as np
from PIL import Image

T = 0.2                      # wall thickness (m); the east wall measures 1.10 - 1.30
RES = 0.05
TOP_H, TOP_T = 0.68, 0.04    # desktop height, thickness

# x0, x1, y0, y1
EAST_DOORS = [(-12.55, -11.0), (-0.4, 1.15)]     # small and large classroom, 1.55 m each
EAST = []
_y = -15.53
for _a, _b in EAST_DOORS:
    EAST.append((1.1, 1.3, _y, _a)); _y = _b
EAST.append((1.1, 1.3, _y, 6.75))
WALLS = EAST + [
    (-12.05, -11.85, -9.45, 3.67),     # large classroom: west
    (-12.05, 1.1, 3.47, 3.67),         # north
    (-12.05, 1.1, -9.45, -9.25),       # between the classrooms
    (-11.79, -11.59, -15.53, -9.45),   # small classroom: west
    (-11.79, 1.1, -15.53, -15.33),     # south
    (-9.7, 1.3, 6.55, 6.75),           # top hallway: south
    (-9.7, -9.5, 6.55, 8.51),          # west end
    (-9.7, 6.3, 8.31, 8.51),           # north
    (6.1, 6.3, 3.16, 8.51),            # lobby: east
    (1.1, 3.8, -15.64, -15.44),        # balcony corridor: south end
]
# The balcony edge over the first floor. Tall so the lidar cannot map floor beyond it; no wall to the radio.
EDGE = [(3.6, 3.8, -15.64, 3.36), (3.6, 6.3, 3.16, 3.36)]

DESK_ROWS = [      # large classroom: y of the row, x of each desk
    (2.22, (-8.19, -6.95, -5.44, -4.11, -2.70, -1.30)),
    (-0.01, (-8.25, -6.89, -5.58, -4.11, -3.02, -1.52)),
    (-2.44, (-8.05, -6.91, -5.40, -4.05, -2.84, -1.72)),
    (-5.28, (-8.03, -6.71, -5.46, -4.21, -3.06, -1.58)),
    (-7.30, (-8.00, -6.70, -5.60, -4.30, -3.10, -1.80)),   # last row of the scan, thinly covered: by eye
]
DESKS = [(xc - 0.3, xc + 0.3, y - 0.8, y + 0.8) for y, xs in DESK_ROWS for xc in xs]
TABLES = [    # solid to the floor: chairs stand around them in the scan, a robot does not go under
    (-10.5, -9.7, 1.0, 2.4),           # large classroom: lecturer's desk
    (-9.2, -8.0, -11.45, -9.9),        # small classroom: desks
    (-8.9, -7.85, -14.35, -12.8),
    (-11.59, -10.3, -9.9, -9.45),
    # the meeting table, a closed ring
    (-6.8, -1.1, -11.2, -10.8), (-6.8, -1.1, -13.6, -13.2), (-6.8, -6.2, -13.2, -11.2), (-1.6, -1.1, -13.2, -11.2),
]
CABINETS = [(-9.6, -8.1, -9.25, -8.8), (-11.85, -11.0, -9.25, -7.1), (-11.85, -11.3, 1.0, 1.6),     # large
            (-2.1, 0.8, -10.06, -9.45), (0.5, 1.1, -15.1, -13.66), (-11.59, -11.0, -15.33, -13.5)]   # small
CAB_H, WALL_H = 0.8, 2.6
# the walkable floor, as rectangles (everything else is solid to the planner)
ROOMS = [(-11.85, 1.1, -9.25, 3.47), (-11.59, 1.1, -15.33, -9.45), (1.3, 3.6, -15.44, 8.31), (3.6, 6.1, 3.36, 8.31),
         (-9.5, 1.3, 6.75, 8.31), (1.1, 1.3, -12.55, -11.0), (1.1, 1.3, -0.4, 1.15)]
BOUNDS = (-13.0, 7.5, -16.5, 9.5)
# In the lobby near the GCS (by the stairs): the robot sets out from the GCS. chain_robot's body trails 1.2 m behind
# this point, so it needs that much clear: inside a desk it was kicked 1 m and FAST-LIO diverged. Yaw stays 0: the
# odometry offset in nodes_real_env.yaml is a translation only.
SPAWN = (4.4, 5.0)


def box(name, x0, x1, y0, y1, z0, z1, mat):
    size = f"{x1 - x0:.3f} {y1 - y0:.3f} {z1 - z0:.3f}"
    pose = f"{(x0 + x1) / 2:.3f} {(y0 + y1) / 2:.3f} {(z0 + z1) / 2:.3f} 0 0 0"
    geom = f"<pose>{pose}</pose><geometry><box><size>{size}</size></box></geometry>"
    return (f"        <collision name='c_{name}'>{geom}</collision>\n"
            f"        <visual name='v_{name}'>{geom}<material><script><uri>file://media/materials/scripts/gazebo.material</uri>"
            f"<name>Gazebo/{mat}</name></script></material></visual>\n")


def desk(i, x0, x1, y0, y1):
    """As the scan shows them below the top: a panel the desk's whole length near its west edge and a
    frame across each end. The east side, where the scan has chairs, is closed too: chain_robot's 2D scan
    cannot see the top, so an open side reads as free floor and the explorer drove it in (real_full: stuck
    inside a desk at 34%)."""
    z = TOP_H - TOP_T
    return (box(f"d{i}", x0, x1, y0, y1, z, TOP_H, "Wood") + box(f"d{i}_p", x0 + 0.1, x0 + 0.15, y0, y1, 0, z, "Wood")
            + box(f"d{i}_s", x0, x1, y0, y0 + 0.05, 0, z, "Wood") + box(f"d{i}_n", x0, x1, y1 - 0.05, y1, 0, z, "Wood")
            + box(f"d{i}_e", x1 - 0.05, x1, y0, y1, 0, z, "Wood"))


def raster(rects, shape, x0, y0):
    g = np.zeros(shape, bool)
    for a, b, c, d in rects:
        g[int(round((c - y0) / RES)):int(round((d - y0) / RES)), int(round((a - x0) / RES)):int(round((b - x0) / RES))] = True
    return g


def main():
    out = sys.argv[1]
    body = ""
    for i, r in enumerate(WALLS + EDGE):
        body += box(f"w{i}", *r, 0, WALL_H, "Grey")
    for i, r in enumerate(CABINETS):
        body += box(f"c{i}", *r, 0, CAB_H, "Wood")
    for i, r in enumerate(TABLES):
        body += box(f"t{i}", *r, 0, TOP_H, "Wood")
    for i, r in enumerate(DESKS):
        body += desk(i, *r)
    tmpl = open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "../unist/unist_indoor.world")).read()
    head = tmpl[:tmpl.index("<model name='112Unit_8F'>")]
    robot = tmpl[tmpl.rindex("<include>"):tmpl.rindex("</include>") + len("</include>")].replace(
        "<pose>-0.5 -6 0.0 0 0 0</pose>", f"<pose>{SPAWN[0]} {SPAWN[1]} 0.0 0 0 0</pose>")
    world = head + "<model name='real_env'>\n      <static>1</static>\n      <link name='building'>\n" + body + f"""      </link>
    </model>
    <gui fullscreen='0'>
      <camera name='user_camera'>
        <pose>-2.9 -3.6 19 0 1.5707 1.5707</pose>
        <view_controller>orbit</view_controller>
        <projection_type>perspective</projection_type>
      </camera>
    </gui>
    {robot}
  </world>
</sdf>
"""
    open(os.path.join(out, "real_env.world"), "w").write(world)

    x0, x1, y0, y1 = BOUNDS
    shape = (int(round((y1 - y0) / RES)), int(round((x1 - x0) / RES)))
    walkable = raster(ROOMS, shape, x0, y0)
    furn = raster(DESKS + TABLES + CABINETS, shape, x0, y0)
    occ = (~walkable | furn).astype("u1")                # the planner keeps out from under desks too
    radio = raster(WALLS, shape, x0, y0).astype("u1")
    meta = dict(resolution=RES, origin_xy=[x0, y0], shape=list(shape), occupied_value=1, free_value=0,
                note="grid[row, col]; world_x = origin_xy[0] + (col+0.5)*res; world_y = origin_xy[1] + (row+0.5)*res")
    os.makedirs(os.path.join(out, "gt"), exist_ok=True)
    for name, g in (("real_env_gt", occ), ("real_env_radio", radio)):
        np.save(os.path.join(out, "gt", name + ".npy"), g)
        json.dump(meta, open(os.path.join(out, "gt", name + ".json"), "w"), indent=1)
    img = np.full(shape + (3,), 225, "u1")
    img[walkable] = 255
    img[furn] = (205, 160, 105)
    img[raster(WALLS, shape, x0, y0)] = (40, 40, 45)
    img[raster(EDGE, shape, x0, y0)] = (70, 120, 200)
    Image.fromarray(img[::-1]).save(os.path.join(out, "gt", "real_env_gt.png"))
    n = len(WALLS + EDGE) + len(CABINETS) + len(TABLES) + 5 * len(DESKS)
    print(f"{n} boxes ({len(WALLS + EDGE)} walls, {len(DESKS)} desks, {len(TABLES)} tables, {len(CABINETS)} cabinets); "
          f"free floor {(occ == 0).sum() * RES ** 2:.0f} m2")


if __name__ == "__main__":
    main()
