#!/usr/bin/env python3
"""Generate the AMCL map from the MJCF world — run on your machine, commit the result.

Geometry-agnostic: instead of slicing primitives analytically, this casts a
vertical ray down each grid cell and uses an odd-crossing (parity) test to
decide whether collision geometry occupies the cell at lidar height. Works
for boxes, cylinders, convex-decomposed meshes — anything MuJoCo can collide.
Overhangs (shelf stock, tree canopies) above lidar height are correctly free.

Run via `make map`, which dumps the Unity scene as MJCF, gates it through
lint_world.py, and feeds the dump here. Needs `mujoco` (pip) — no ROS.
"""

import argparse
import os
import sys

import mujoco
import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_OUT = os.path.join(REPO, "ground", "src", "spar_bringup", "maps")

RESOLUTION = 0.05
MARGIN = 1.0
EPS = 1e-6


def static_collision_geoms(model):
    """Geom ids that are part of the world. The map and the lint gate both
    ask the physics model "what can the robot hit that isn't the robot?" and
    this is the answer.

    Membership is MuJoCo's weld test, not "attached directly to worldbody".
    A prop that arrives inside a wrapper MjBody (how asset packs and grouped
    objects come in) is not a child of the world root, but it has no joint
    anywhere up its chain, so MuJoCo welds it to the world: body_weldid == 0.
    The robot and any mover DO have a joint in their chain, which starts a
    new weld group (weldid != 0), so they stay out of the map. Filtering on
    the parent body id instead would silently drop wrapped props: physics
    collides with them and the lidar sees them, but the map would not know
    them. lint_world.py imports this so the gate and the map can never
    disagree about what the world is.
    """
    ids = []
    for g in range(model.ngeom):
        if model.body_weldid[model.geom_bodyid[g]] != 0:
            continue  # robot or mover, not static world
        if model.geom_contype[g] == 0 and model.geom_conaffinity[g] == 0:
            continue  # visual-only
        if model.geom_type[g] == mujoco.mjtGeom.mjGEOM_PLANE:
            continue  # the floor doesn't occupy cells
        ids.append(g)
    return ids


def lidar_z(model, data):
    """World height of the lidar site. The map is a 2D slice of the world at
    exactly this height: only geometry the lidar can see belongs in the map
    AMCL matches scans against. Read from the model instead of hardcoded so
    a robot with a taller or shorter mast moves the slice automatically."""
    for s in range(model.nsite):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SITE, s) or ""
        if name.startswith("lidar"):
            return float(data.site_xpos[s][2])
    sys.exit("no lidar site in the model; the map height would be a guess")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("model", help="MJCF world file (dumped from Unity)")
    ap.add_argument("out_dir", nargs="?", default=DEFAULT_OUT, help="output directory")
    args = ap.parse_args()
    os.makedirs(args.out_dir, exist_ok=True)

    model = mujoco.MjModel.from_xml_path(args.model)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    solid = set(static_collision_geoms(model))
    if not solid:
        sys.exit("no static collision geometry found")
    scan_z = lidar_z(model, data)

    # World bounds from tight axis-aligned boxes around each solid geom
    # (rotate the geom-frame AABB into world axes). Finite floor planes also
    # count toward bounds — a sparse world is still as big as its floor —
    # but never toward occupancy.
    xs, ys, tops = [], [], []
    for g in range(model.ngeom):
        if (model.geom_type[g] == mujoco.mjtGeom.mjGEOM_PLANE
                and model.geom_size[g][0] > 0):
            x, y, _ = data.geom_xpos[g]
            sx, sy = model.geom_size[g][0], model.geom_size[g][1]
            xs += [x - sx, x + sx]
            ys += [y - sy, y + sy]
    for g in solid:
        center_l = model.geom_aabb[g, :3]
        half_l = model.geom_aabb[g, 3:]
        rot = data.geom_xmat[g].reshape(3, 3)
        center_w = data.geom_xpos[g] + rot @ center_l
        half_w = np.abs(rot) @ half_l
        xs += [center_w[0] - half_w[0] - MARGIN, center_w[0] + half_w[0] + MARGIN]
        ys += [center_w[1] - half_w[1] - MARGIN, center_w[1] + half_w[1] + MARGIN]
        tops.append(center_w[2] + half_w[2])
    min_x, max_x, min_y, max_y = min(xs), max(xs), min(ys), max(ys)
    z_top = max(tops) + 0.5
    w = int((max_x - min_x) / RESOLUTION)
    h = int((max_y - min_y) / RESOLUTION)
    grid = np.full((h, w), 254, dtype=np.uint8)  # free

    # Parity sweep per grid row and per geom: march a horizontal ray across
    # the world at lidar height, collecting each geom's surface crossings
    # SEPARATELY (mju_rayGeom / mj_rayHfield), and paint the union of the
    # inside intervals. Per-geom on purpose, twice over:
    #  - horizontal because MuJoCo's ray-cylinder test ignores the flat end
    #    caps, so vertical rays pass straight through cylinders;
    #  - separate rays per geom because a single scene-wide parity ray drops
    #    a crossing wherever two faces are exactly coplanar (butted walls,
    #    stacked crates) — only one geom gets reported for the tie, the
    #    other's toggle is lost, and the row floods to the map border.
    direction = np.array([1.0, 0.0, 0.0])
    occupied_cells = 0
    hfield = mujoco.mjtGeom.mjGEOM_HFIELD
    # world AABBs once per geom, for cheap row rejection and interval capping
    aabb = {}
    for g in solid:
        rot = data.geom_xmat[g].reshape(3, 3)
        c = data.geom_xpos[g] + rot @ model.geom_aabb[g, :3]
        hw = np.abs(rot) @ model.geom_aabb[g, 3:]
        aabb[g] = (c - hw, c + hw)
    for py in range(h):
        wy = min_y + (py + 0.5) * RESOLUTION
        row = grid[h - 1 - py]
        for g in solid:
            lo, hi = aabb[g]
            if not (lo[1] <= wy <= hi[1] and lo[2] <= scan_z <= hi[2]):
                continue
            crossings = []
            x = lo[0] - 0.1
            while x < hi[0] + 0.1:
                pnt = np.array([x, wy, scan_z])
                if model.geom_type[g] == hfield:
                    dist = mujoco.mj_rayHfield(model, data, g, pnt, direction)
                else:
                    dist = mujoco.mju_rayGeom(
                        data.geom_xpos[g], data.geom_xmat[g],
                        model.geom_size[g], pnt, direction,
                        model.geom_type[g])
                if dist < 0:
                    break
                x += dist
                crossings.append(x)
                x += EPS
            if len(crossings) % 2:  # tangent graze: close at the far side
                crossings.append(hi[0])
            for x0, x1 in zip(crossings[::2], crossings[1::2]):
                p0 = max(0, int((x0 - min_x) / RESOLUTION + 0.5))
                p1 = min(w, int((x1 - min_x) / RESOLUTION + 0.5))
                if p1 > p0:
                    occupied_cells += int((row[p0:p1] != 0).sum())
                    row[p0:p1] = 0

    name = os.path.splitext(os.path.basename(args.model))[0]
    pgm = os.path.join(args.out_dir, f"{name}.pgm")
    with open(pgm, "wb") as f:
        f.write(f"P5\n{w} {h}\n255\n".encode())
        f.write(grid.tobytes())
    with open(os.path.join(args.out_dir, f"{name}.yaml"), "w") as f:
        f.write(
            f"image: {name}.pgm\n"
            "mode: trinary\n"
            f"resolution: {RESOLUTION}\n"
            f"origin: [{min_x:.3f}, {min_y:.3f}, 0]\n"
            "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n"
        )
    print(f"wrote {pgm} ({w}x{h} @ {RESOLUTION}m, {occupied_cells} occupied cells)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
