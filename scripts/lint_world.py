#!/usr/bin/env python3
"""Validate a world MJCF (dumped from the Unity scene) before it ships.

Runs on the same dump that feeds rasterize_map.py, so bad collision geometry
is caught before the robot ever sees it.

Checks:
  1. stealth geoms  — every static collision geom must cross the 2D lidar
                      plane (at the height of the model's lidar site), or the
                      robot can hit what the lidar, and therefore the map and
                      costmaps, cannot see.
  2. dock zone      — the dock area must be clear of collision geometry, or
                      docking/recharge fails in ways that look like bugs.
  3. waypoints      — every route waypoint needs standoff from obstacles, or
                      Nav2 oscillates trying to reach an unreachable goal.
  4. geom budget    — a soft warning when the world grows past what the
                      render/raycast budget was sized for.

Usage: lint_world.py <world.xml> [autonomy_<world>.yaml]
Exits nonzero on any hard failure.
"""
import math
import os
import sys

import mujoco
import yaml

# The gate and the map must agree on what counts as world geometry and where
# the lidar plane sits, so both come from rasterize_map.
from rasterize_map import lidar_z, static_collision_geoms

DOCK_CLEAR_M = 1.2    # radius around the dock that must be collision-free
WAYPOINT_CLEAR_M = 0.6  # robot half-width + margin
GEOM_BUDGET = 300


def geom_z_extent(model, data, gid):
    """World-frame z half-extent of a primitive geom (exact for the primitive
    set the authoring discipline allows; bounding sphere for anything else)."""
    size = model.geom_size[gid]
    t = model.geom_type[gid]
    if t == mujoco.mjtGeom.mjGEOM_SPHERE:
        return size[0]
    R = data.geom_xmat[gid].reshape(3, 3)
    if t == mujoco.mjtGeom.mjGEOM_BOX:
        return sum(abs(R[2][i]) * size[i] for i in range(3))
    if t == mujoco.mjtGeom.mjGEOM_CYLINDER:
        return abs(R[2][2]) * size[1] + math.hypot(R[2][0], R[2][1]) * size[0]
    # capsule / other: bounding sphere (conservative; fine for a gate)
    return model.geom_rbound[gid]


def load_autonomy_yaml(path):
    """Pull dock pose and patrol waypoints out of the ros params yaml."""
    with open(path) as f:
        doc = yaml.safe_load(f)
    merged = {}
    for node in doc.values():  # {node_name: {ros__parameters: {...}}}
        if isinstance(node, dict):
            merged.update(node.get("ros__parameters", {}))
    dock = (merged.get("dock_x"), merged.get("dock_y"))
    wx = merged.get("waypoints_x", [])
    wy = merged.get("waypoints_y", [])
    return {
        "dock": dock if dock[0] is not None else None,
        "waypoints": list(zip(wx, wy)),
    }


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    model_path = sys.argv[1]
    cfg_path = sys.argv[2] if len(sys.argv) > 2 else None

    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    geoms = static_collision_geoms(model)
    z_lidar = lidar_z(model, data)
    failures, warnings = [], []

    def gname(gid):
        return mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, gid) or f"geom#{gid}"

    # 1. stealth geoms: static solids must cross the lidar plane.
    for gid in geoms:
        z = data.geom_xpos[gid][2]
        half = geom_z_extent(model, data, gid)
        top, bottom = z + half, z - half
        if top < z_lidar:
            failures.append(
                f"stealth obstacle: '{gname(gid)}' tops out at {top:.2f}m, "
                f"below the {z_lidar:.2f}m lidar plane — robot can hit what it can't see")
        elif bottom > z_lidar:
            warnings.append(
                f"overhang: '{gname(gid)}' starts at {bottom:.2f}m, above the "
                f"lidar plane — invisible to the 2D stack (ok if intended)")

    cfg = load_autonomy_yaml(cfg_path) if cfg_path else None

    # 2. dock zone clear.
    if cfg and cfg["dock"]:
        dx, dy = cfg["dock"]
        for gid in geoms:
            gx, gy = data.geom_xpos[gid][:2]
            if math.hypot(gx - dx, gy - dy) - model.geom_rbound[gid] < DOCK_CLEAR_M:
                failures.append(
                    f"dock zone: '{gname(gid)}' intrudes within {DOCK_CLEAR_M}m "
                    f"of the dock ({dx}, {dy})")

    # 3. waypoint standoff.
    if cfg:
        for i, (wx, wy) in enumerate(cfg["waypoints"]):
            for gid in geoms:
                gx, gy = data.geom_xpos[gid][:2]
                clear = math.hypot(gx - wx, gy - wy) - model.geom_rbound[gid]
                if clear < WAYPOINT_CLEAR_M:
                    failures.append(
                        f"waypoint {i} ({wx}, {wy}): only {max(clear, 0):.2f}m from "
                        f"'{gname(gid)}' (needs {WAYPOINT_CLEAR_M}m)")

    # 4. budget.
    if model.ngeom > GEOM_BUDGET:
        warnings.append(f"{model.ngeom} geoms exceeds the {GEOM_BUDGET} budget "
                        f"the render/raycast loop was sized for")

    for w in warnings:
        print(f"[lint] WARN: {w}")
    for f_ in failures:
        print(f"[lint] FAIL: {f_}")
    if failures:
        print(f"[lint] {len(failures)} failure(s) in {os.path.basename(model_path)}")
        return 1
    print(f"[lint] OK: {os.path.basename(model_path)} "
          f"({len(geoms)} static solids, {model.ngeom} geoms total)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
