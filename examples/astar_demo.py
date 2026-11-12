"""Offline demo of the A* planner in autopath_core.pathfinder (no ROS needed).

Builds a buoy channel with a static obstacle and a moving obstacle on a NED grid,
plans with NavDomain.a_star_search using the same weights as the vehicle
(OBJECT_WEIGHT_VALUE = 25, OBJECT_AREA_RADIUS = 4 in core.py), and saves
docs/astar_demo.png.

Run from the repository root:
    python examples/astar_demo.py
Requires numpy and matplotlib.
"""

from __future__ import annotations

import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

REPO_ROOT = Path(__file__).resolve().parent.parent

OBJECT_WEIGHT_VALUE = 25  # same values as src/autopath_core/core.py
OBJECT_AREA_RADIUS = 4

ROWS, COLS = 40, 70
START = (30, 3)
GOAL = (8, 66)

# Port (red) and starboard (green) buoys forming a channel that bends upward.
RED_BUOYS = [(14, 18), (12, 26), (10, 34), (8, 42), (6, 50)]
GREEN_BUOYS = [(24, 18), (22, 26), (20, 34), (18, 42), (16, 50)]
STATIC_OBSTACLES = [(26, 8), (27, 9), (15, 30)]  # one sits inside the channel
MOVING_OBSTACLES = [(13, 46)]


def build_domain():
    # autopath_core lives under src/ and is not installed outside a catkin workspace.
    sys.path.insert(0, str(REPO_ROOT / "src"))
    from autopath_core.pathfinder import NavDomain

    domain = NavDomain(ROWS, COLS)
    for row, col in RED_BUOYS:
        domain.updateMap(domain.Coordinate(row, col), "red_buoy", True,
                         OBJECT_WEIGHT_VALUE, OBJECT_AREA_RADIUS, is_repellor=True)
    for row, col in GREEN_BUOYS:
        domain.updateMap(domain.Coordinate(row, col), "green_buoy", True,
                         OBJECT_WEIGHT_VALUE, OBJECT_AREA_RADIUS, is_repellor=True)
    for row, col in STATIC_OBSTACLES:
        domain.updateMap(domain.Coordinate(row, col), "obstacle", True,
                         OBJECT_WEIGHT_VALUE, OBJECT_AREA_RADIUS, is_repellor=True)
    return domain


def corridor_midpoints() -> list[tuple[float, float]]:
    """Same rule as Commander._find_corridor: midpoint of each red/green pair."""
    return [((r[0] + g[0]) / 2.0, (r[1] + g[1]) / 2.0) for r, g in zip(RED_BUOYS, GREEN_BUOYS)]


def main() -> None:
    domain = build_domain()
    corridor = corridor_midpoints()
    path = domain.a_star_search(
        domain.Coordinate(*START),
        domain.Coordinate(*GOAL),
        corridor=corridor,
        moving_obstacles=[domain.Coordinate(r, c) for r, c in MOVING_OBSTACLES],
    )
    if not path:
        raise RuntimeError(f"A* found no path from {START} to {GOAL}")

    repel = np.array([[cell.repelval for cell in row] for row in domain.map])
    path_rc = np.array([(p.row, p.col) for p in path] + [GOAL])

    fig, ax = plt.subplots(figsize=(10, 5.6))
    im = ax.imshow(repel, cmap="Blues", origin="upper")
    fig.colorbar(im, ax=ax, fraction=0.025, pad=0.02, label="repulsive potential")
    ax.plot(path_rc[:, 1], path_rc[:, 0], color="#111", lw=2, label=f"A* path ({len(path_rc)} cells)")
    cor = np.array(corridor)
    ax.plot(cor[:, 1], cor[:, 0], "o--", color="#888", ms=4, lw=1, label="channel midpoints")
    ax.scatter([c for _, c in RED_BUOYS], [r for r, _ in RED_BUOYS], c="#d62728", s=60,
               edgecolors="k", label="red buoy (port)", zorder=3)
    ax.scatter([c for _, c in GREEN_BUOYS], [r for r, _ in GREEN_BUOYS], c="#2ca02c", s=60,
               edgecolors="k", label="green buoy (starboard)", zorder=3)
    ax.scatter([c for _, c in STATIC_OBSTACLES], [r for r, _ in STATIC_OBSTACLES], marker="s",
               c="#555", s=50, label="static obstacle", zorder=3)
    ax.scatter([c for _, c in MOVING_OBSTACLES], [r for r, _ in MOVING_OBSTACLES], marker="X",
               c="#ff7f0e", s=90, edgecolors="k", label="moving obstacle", zorder=3)
    ax.scatter(*START[::-1], marker="^", c="k", s=90, zorder=4)
    ax.scatter(*GOAL[::-1], marker="*", c="gold", s=220, edgecolors="k", zorder=4)
    ax.annotate("start", (START[1], START[0]), xytext=(4, 6), textcoords="offset points")
    ax.annotate("goal", (GOAL[1], GOAL[0]), xytext=(-24, 8), textcoords="offset points")
    ax.set_title("NavDomain.a_star_search through a buoy channel (grid cells, NED frame)")
    ax.set_xlabel("East (cells)")
    ax.set_ylabel("North (cells)")
    ax.legend(loc="lower right", fontsize=8, framealpha=0.9)
    fig.tight_layout()

    out = REPO_ROOT / "docs" / "astar_demo.png"
    out.parent.mkdir(exist_ok=True)
    fig.savefig(out, dpi=130)
    print(f"path length {len(path_rc)} cells -> {out}")


if __name__ == "__main__":
    main()
