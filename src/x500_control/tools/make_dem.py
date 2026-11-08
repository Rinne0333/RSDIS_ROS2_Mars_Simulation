#!/usr/bin/env python3
"""Bake the Martian surface mesh into a 1 m height grid in world ENU.

Gazebo places martian_surface with scale 100, pose (40 0 -25) and roll +90 deg,
so a glTF vertex (x, y, z) ends up at world (100x + 40, -100z, 100y - 25).
Sanity check: the grid gives 3.81 m at the PX4 spawn (8, -2), whose pose uses
z = 3.8, and 6.57 m at the Pathfinder (124, 4), whose world pose uses z = 6.8.

Usage: make_dem.py <martian_surface.glb> <out.csv> [resolution_m]
"""
import sys
import numpy as np
import trimesh
from scipy.spatial import cKDTree

glb, out = sys.argv[1], sys.argv[2]
res = float(sys.argv[3]) if len(sys.argv) > 3 else 1.0

scene = trimesh.load(glb)
mesh = scene.to_geometry() if isinstance(scene, trimesh.Scene) else scene
v = np.asarray(mesh.vertices)
w = np.stack([v[:, 0] * 100 + 40, -v[:, 2] * 100, v[:, 1] * 100 - 25], axis=1)

x0, y0 = np.floor(w[:, 0].min()) + 1, np.floor(w[:, 1].min()) + 1
nx = int((np.floor(w[:, 0].max()) - x0) / res)
ny = int((np.floor(w[:, 1].max()) - y0) / res)
grid = np.full((ny, nx), np.nan)
ix = np.rint((w[:, 0] - x0) / res).astype(int)
iy = np.rint((w[:, 1] - y0) / res).astype(int)
ok = (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny)
for a, b, z in zip(iy[ok], ix[ok], w[ok, 2]):
    if np.isnan(grid[a, b]) or z > grid[a, b]:
        grid[a, b] = z                      # keep the top surface
hole = np.isnan(grid)
if hole.any():
    idx = cKDTree(np.argwhere(~hole)).query(np.argwhere(hole))[1]
    grid[hole] = grid[~hole][idx]

with open(out, "w") as f:
    f.write(f"# x0 y0 res nx ny\n{x0} {y0} {res} {nx} {ny}\n")
    for row in grid:
        f.write(",".join(f"{z:.2f}" for z in row) + "\n")
print(f"{out}: {nx}x{ny} cells, x {x0}..{x0 + nx * res}, y {y0}..{y0 + ny * res}, "
      f"z {grid.min():.1f}..{grid.max():.1f}")
