#!/usr/bin/env python3
"""STEP 1 of the CAD -> URDF pipeline: measure the exported parts.

Before writing a single line of URDF you must know, for every part:
  * its units and bounding box (SolidWorks STL export is in millimetres)
  * where its joint axes (pivot holes) are, in the part's own coordinates
  * its mass properties

This script slices each STL at mid-thickness, finds circular holes and
circular end-arcs with a least-squares circle fit, and prints/saves a report.

Usage:  python3 tools/cad_to_urdf/step1_inspect_parts.py [--plot out.png]
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import trimesh
import yaml

ROOT = Path(__file__).resolve().parents[2]
STL_DIR = ROOT / "hardware" / "stl"

PARTS = {   # file -> role in the robot
    "base.STL": "base plate (fixed)",
    "base22.STL": "servo cradle (fixed, x3)",
    "connecting_bar.STL": "proximal link L1 (on servo horn, x3)",
    "middle_bar.STL": "distal link L2 (x3)",
    "end_effector.STL": "moving platform",
    "b_connection1.STL": "joint pin",
    "CIRCLE_SUP.STL": "bearing spacer",
    "servo dise v1.STL": "servo horn disc",
}


def fit_circle(pts: np.ndarray):
    """Kasa algebraic circle fit -> (cx, cy, r, rms residual)."""
    x, y = pts[:, 0], pts[:, 1]
    A = np.column_stack((2 * x, 2 * y, np.ones_like(x)))
    b = x**2 + y**2
    (cx, cy, c), *_ = np.linalg.lstsq(A, b, rcond=None)
    r = np.sqrt(c + cx**2 + cy**2)
    res = np.sqrt(np.mean((np.hypot(x - cx, y - cy) - r) ** 2))
    return float(cx), float(cy), float(r), float(res)


def inspect(path: Path):
    m = trimesh.load(path)
    k = int(np.argmin(m.extents))                 # thinnest axis = plate normal
    keep = [i for i in range(3) if i != k]
    sec = m.section(plane_origin=m.bounds.mean(0), plane_normal=np.eye(3)[k])
    loops = [l[:, keep] for l in sec.discrete] if sec is not None else []
    holes, outer = [], max(loops, key=lambda l: np.ptp(l[:, 0]) * np.ptp(l[:, 1])) if loops else None
    for l in loops:
        if l is outer:
            continue
        cx, cy, r, res = fit_circle(l)
        if res < 0.05 * r and r < 15:
            holes.append({"center_mm": [round(cx, 2), round(cy, 2)], "radius_mm": round(r, 2)})
    end_arcs = []
    allp = np.vstack(loops) if loops else np.zeros((0, 2))
    if len(allp) and np.ptp(allp[:, 0]) > 3 * np.ptp(allp[:, 1]):   # elongated part = a link
        # The link ends are circular arcs centred on the pivots. Use the points of
        # ALL section loops (a clamp slot can split an end into its own loop).
        # Keep only the convex-hull points: they belong to the outer outline, not to holes.
        from scipy.spatial import ConvexHull
        dense = np.vstack([np.linspace(a, b, 8, endpoint=False) for l in loops
                           for a, b in zip(l[:-1], l[1:])])
        allp = dense[ConvexHull(dense).vertices]
        xmin, xmax = allp[:, 0].min(), allp[:, 0].max()
        for sel in (allp[:, 0] < xmin + 5.0, allp[:, 0] > xmax - 5.0):
            cx, cy, r, res = fit_circle(allp[sel])
            end_arcs.append({"center_mm": [round(cx, 2), round(cy, 2)], "radius_mm": round(r, 2)})
    return {
        "units": "mm",
        "extents_mm": [round(float(v), 2) for v in m.extents],
        "bounds_min_mm": [round(float(v), 2) for v in m.bounds[0]],
        "plate_normal_axis": "xyz"[k],
        "watertight": bool(m.is_watertight),
        "volume_cm3": round(float(m.volume) / 1000.0, 3),
        "holes": holes,
        "end_arcs": end_arcs,
    }, (m, loops, keep)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--plot", help="save an annotated figure")
    ap.add_argument("--out", default=str(ROOT / "tools" / "cad_to_urdf" / "parts_report.yaml"))
    args = ap.parse_args()

    report, geo = {}, {}
    for name, role in PARTS.items():
        info, g = inspect(STL_DIR / name)
        info["role"] = role
        report[name] = info
        geo[name] = g
        print(f"\n{name:22s} [{role}]  extents {info['extents_mm']} mm  vol {info['volume_cm3']} cm3")
        for h in info["holes"]:
            print(f"    hole   c={h['center_mm']}  r={h['radius_mm']}")
        for a in info["end_arcs"]:
            print(f"    end-arc c={a['center_mm']}  r={a['radius_mm']}")
        if len(info["end_arcs"]) == 2:
            a, b = (np.array(e["center_mm"]) for e in info["end_arcs"])
            print(f"    -> pivot-to-pivot distance = {np.linalg.norm(b - a):.2f} mm")
    Path(args.out).write_text(yaml.safe_dump(report, sort_keys=False))
    print(f"\nreport written to {args.out}")

    if args.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, axs = plt.subplots(2, 4, figsize=(16, 7.5))
        for ax, (name, (m, loops, keep)) in zip(axs.flat, geo.items()):
            for l in loops:
                ax.plot(l[:, 0], l[:, 1], "k-", lw=0.8)
            for h in report[name]["holes"]:
                ax.add_patch(plt.Circle(h["center_mm"], h["radius_mm"], fill=False, ec="tab:red", lw=1.5))
                ax.plot(*h["center_mm"], "r+")
            for e in report[name]["end_arcs"]:
                ax.plot(*e["center_mm"], "bx", ms=10, mew=2)
            ax.set_title(f"{name}\n{report[name]['role']}", fontsize=9)
            ax.set_aspect("equal")
            ax.grid(alpha=0.3)
        fig.suptitle("Step 1 - measured pivots: red = fitted holes, blue x = fitted end-arc centres [mm]")
        fig.tight_layout()
        fig.savefig(args.plot, dpi=110)
        print(f"figure written to {args.plot}")


if __name__ == "__main__":
    main()
