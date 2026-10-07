#!/usr/bin/env python3
"""Ground-truth 2D occupancy grid from the Gazebo world's COLLISION geometry.

This is true GT: it rasterizes the same collision meshes the physics engine uses,
not SLAM output. Run INSIDE the safe_robot container (needs numpy + trimesh, and
the world mounted at /hw).

    python3 gt_occupancy.py --world /hw/hospital_chain.world --models /hw/models \
        --out /tmp/hospital_gt --res 0.05 --zlo 0.05 --zhi 0.60

Writes <out>.npy (uint8: 1 = occupied, 0 = free), <out>.json (metadata for
mapping cell <-> world coords), and <out>.png (visual check).
"""
import argparse, json, os, math, re, sys
import xml.etree.ElementTree as ET
import numpy as np
import trimesh

FAILURES = []


def pose_to_T(txt):
    """SDF '<pose>x y z roll pitch yaw' -> 4x4 homogeneous matrix."""
    if txt is None:
        return np.eye(4)
    v = [float(x) for x in txt.split()]
    while len(v) < 6:
        v.append(0.0)
    x, y, z, R, P, Y = v[:6]
    cr, sr, cp, sp, cy, sy = math.cos(R), math.sin(R), math.cos(P), math.sin(P), math.cos(Y), math.sin(Y)
    # SDF is fixed-axis XYZ (roll, pitch, yaw) == Rz*Ry*Rx
    Rm = np.array([
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp,   cp*sr,            cp*cr],
    ])
    T = np.eye(4); T[:3, :3] = Rm; T[:3, 3] = [x, y, z]
    return T


def geom_to_mesh(geom, models_dir):
    """<geometry> -> trimesh in the collision's own frame, or None."""
    m = geom.find('mesh')
    if m is not None:
        uri = m.find('uri').text.strip()
        if not uri.startswith('model://'):
            return None
        rel = uri[len('model://'):]
        path = os.path.join(models_dir, rel)
        if not os.path.exists(path):
            return None
        try:
            mesh = trimesh.load(path, force='mesh', skip_materials=True)
        except Exception as e:
            # Loudly, not silently. A swallowed ImportError here (missing
            # pycollada) dropped EVERY .dae collision mesh -- including the
            # building walls -- and produced a GT grid of furniture floating in
            # empty space that still looked plausible at a glance.
            FAILURES.append((os.path.basename(path), f'{type(e).__name__}: {e}'))
            return None
        if not isinstance(mesh, trimesh.Trimesh) or mesh.faces.shape[0] == 0:
            FAILURES.append((os.path.basename(path), 'loaded but empty/non-mesh'))
            return None
        # COLLADA <unit meter="..."> -- 34 of the hospital's collision .dae files
        # declare centimetres. Gazebo honours this; trimesh does NOT, so without
        # it those meshes load 100x oversized (the walls alone spanned 5.6 km and
        # blew the grid up to 2514 x 5614 m for a 25 x 51 m building).
        if path.lower().endswith('.dae'):
            try:
                head = open(path, errors='ignore').read(4000)
                u = re.search(r'<unit[^>]*meter="([0-9.eE+-]+)"', head)
                if u:
                    f = float(u.group(1))
                    if abs(f - 1.0) > 1e-9:
                        mesh.apply_scale(f)
            except Exception as e:
                FAILURES.append((os.path.basename(path), f'unit parse: {e}'))
                return None
        s = m.find('scale')
        if s is not None:
            mesh.apply_scale([float(x) for x in s.text.split()])
        return mesh
    b = geom.find('box')
    if b is not None:
        return trimesh.creation.box(extents=[float(x) for x in b.find('size').text.split()])
    c = geom.find('cylinder')
    if c is not None:
        return trimesh.creation.cylinder(radius=float(c.find('radius').text),
                                         height=float(c.find('length').text))
    return None


def collect(world_path, models_dir, skip_substrings):
    """Yield world-frame collision meshes."""
    world = ET.parse(world_path).getroot().find('world')
    entries = []
    for mdl in world.findall('model'):
        inc = mdl.find('include/uri')
        if inc is None:
            continue
        entries.append((mdl.get('name'), inc.text.strip(), pose_to_T(
            mdl.find('pose').text if mdl.find('pose') is not None else None)))

    out, skipped = [], []
    for name, uri, T_world in entries:
        if any(s in uri or s in (name or '') for s in skip_substrings):
            skipped.append(name); continue
        sdf = os.path.join(models_dir, uri[len('model://'):], 'model.sdf')
        if not os.path.exists(sdf):
            continue
        try:
            root = ET.parse(sdf).getroot()
        except Exception:
            continue
        for mdl in root.iter('model'):
            T_m = T_world @ pose_to_T(mdl.find('pose').text if mdl.find('pose') is not None else None)
            for link in mdl.findall('link'):
                T_l = T_m @ pose_to_T(link.find('pose').text if link.find('pose') is not None else None)
                for col in link.findall('collision'):
                    T_c = T_l @ pose_to_T(col.find('pose').text if col.find('pose') is not None else None)
                    g = col.find('geometry')
                    if g is None:
                        continue
                    mesh = geom_to_mesh(g, models_dir)
                    if mesh is None:
                        continue
                    mesh = mesh.copy(); mesh.apply_transform(T_c)
                    out.append(mesh)
            break  # only the outermost <model>
    return out, skipped


def rasterize(meshes, res, zlo, zhi, pad=1.0):
    tris = []
    for m in meshes:
        v = m.vertices[m.faces]                      # (F,3,3)
        zmin, zmax = v[:, :, 2].min(1), v[:, :, 2].max(1)
        keep = (zmax >= zlo) & (zmin <= zhi)         # triangle crosses the slab
        if keep.any():
            tris.append(v[keep][:, :, :2])           # project to XY
    if not tris:
        raise SystemExit('no collision geometry intersected the z band')
    tris = np.concatenate(tris, 0)

    lo = tris.reshape(-1, 2).min(0) - pad
    hi = tris.reshape(-1, 2).max(0) + pad
    W = int(np.ceil((hi[0] - lo[0]) / res)); H = int(np.ceil((hi[1] - lo[1]) / res))
    grid = np.zeros((H, W), np.uint8)

    for t in tris:
        c = np.floor((t - lo) / res).astype(int)
        x0, y0 = np.clip(c.min(0), [0, 0], [W - 1, H - 1])
        x1, y1 = np.clip(c.max(0) + 1, [0, 0], [W, H])
        if x1 <= x0 or y1 <= y0:
            continue
        xs = (np.arange(x0, x1) + 0.5) * res + lo[0]
        ys = (np.arange(y0, y1) + 0.5) * res + lo[1]
        PX, PY = np.meshgrid(xs, ys)
        (ax, ay), (bx, by), (cx, cy) = t
        d = (by - cy) * (ax - cx) + (cx - bx) * (ay - cy)
        if abs(d) < 1e-12:                            # degenerate: stamp its bbox
            grid[y0:y1, x0:x1] = 1; continue
        l1 = ((by - cy) * (PX - cx) + (cx - bx) * (PY - cy)) / d
        l2 = ((cy - ay) * (PX - cx) + (ax - cx) * (PY - cy)) / d
        inside = (l1 >= -1e-9) & (l2 >= -1e-9) & (l1 + l2 <= 1 + 1e-9)
        grid[y0:y1, x0:x1] |= inside.astype(np.uint8)
    return grid, lo, res


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--world', required=True)
    ap.add_argument('--models', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--res', type=float, default=0.05)
    ap.add_argument('--zlo', type=float, default=0.05)
    ap.add_argument('--zhi', type=float, default=0.60)
    ap.add_argument('--skip', default='ceiling,floor_01_floor',
                    help='comma-separated substrings to exclude (floor/ceiling would fill everything)')
    a = ap.parse_args()

    skips = [s for s in a.skip.split(',') if s]
    meshes, skipped = collect(a.world, a.models, skips)
    print(f'  collision meshes loaded : {len(meshes)}')
    if FAILURES:
        print(f'  !! MESH LOAD FAILURES   : {len(FAILURES)}')
        for n, e in FAILURES[:10]:
            print(f'       {n}: {e}')
        sys.exit('ABORT: refusing to emit a GT grid with missing collision geometry')
    print(f'  models skipped          : {len(skipped)} -> {skipped}')

    grid, origin, res = rasterize(meshes, a.res, a.zlo, a.zhi)
    np.save(a.out + '.npy', grid)
    meta = dict(resolution=res, origin_xy=[float(origin[0]), float(origin[1])],
                shape=list(grid.shape), z_band=[a.zlo, a.zhi],
                occupied_value=1, free_value=0,
                note='grid[row, col]; world_x = origin_xy[0] + (col+0.5)*res; '
                     'world_y = origin_xy[1] + (row+0.5)*res')
    json.dump(meta, open(a.out + '.json', 'w'), indent=2)

    try:
        from PIL import Image
        Image.fromarray(np.flipud(grid * 255)).save(a.out + '.png')
    except Exception as e:
        print('  (png preview skipped:', e, ')')

    occ = int(grid.sum())
    print(f'  grid shape  : {grid.shape}  ({grid.shape[1]*res:.1f} x {grid.shape[0]*res:.1f} m)')
    print(f'  resolution  : {res} m/cell')
    print(f'  origin (xy) : ({origin[0]:.3f}, {origin[1]:.3f})')
    print(f'  occupied    : {occ} cells ({100*occ/grid.size:.1f}%)')
    print(f'  wrote {a.out}.npy / .json / .png')
