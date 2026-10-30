"""Top-view drawing helpers shared by the scenarios and the learning figures."""

from __future__ import annotations

import numpy as np

LEG_COLORS = ("#1f77b4", "#2ca02c", "#9467bd")


def draw_robot(ax, kin, pose, ik=None, alpha=1.0, lw=3.0, show_base=True, color=None):
    """Draw the 3-RRR robot (top view, millimetres) and return the artists."""
    ik = ik or kin.ik(pose)
    B, A, C = kin.B * 1e3, ik.elbows * 1e3, ik.platform_joints * 1e3
    arts = []
    for i in range(3):
        c = color or LEG_COLORS[i]
        arts += ax.plot([B[i, 0], A[i, 0]], [B[i, 1], A[i, 1]], "-", color=c, lw=lw, alpha=alpha,
                        solid_capstyle="round")
        arts += ax.plot([A[i, 0], C[i, 0]], [A[i, 1], C[i, 1]], "-", color=c, lw=lw * 0.7, alpha=alpha,
                        solid_capstyle="round")
        arts += ax.plot(*A[i], "o", ms=4, color="k", alpha=alpha)
        if show_base:
            arts += ax.plot(*B[i], "s", ms=8, color="#444", alpha=alpha)
    tri = np.vstack((C, C[:1]))
    arts += ax.fill(tri[:, 0], tri[:, 1], color=color or "#d62728", alpha=0.35 * alpha)
    arts += ax.plot(tri[:, 0], tri[:, 1], "-", color=color or "#d62728", lw=2, alpha=alpha)
    p = np.asarray(pose[:2]) * 1e3
    d = 10 * np.array([np.cos(pose[2]), np.sin(pose[2])])
    arts += ax.plot(*p, "o", ms=4, color="k", alpha=alpha)
    arts.append(ax.annotate("", xy=p + d, xytext=p,
                            arrowprops=dict(arrowstyle="->", color="k", lw=1.2, alpha=alpha)))
    return arts


def setup_topview(ax, lim=105, title=None):
    ax.set_aspect("equal")
    ax.set_xlim(-lim, lim)
    ax.set_ylim(-lim, lim)
    ax.set_xlabel("x [mm]")
    ax.set_ylabel("y [mm]")
    ax.grid(alpha=0.25)
    ax.add_patch(__import__("matplotlib.pyplot").pyplot.Circle((0, 0), 100, fill=False, ec="#999", ls="--", lw=0.8))
    if title:
        ax.set_title(title, fontsize=10)
