import math
import time

import matplotlib.pyplot as plt
import numpy as np
import yaml
from matplotlib.patches import Polygon as MplPolygon
from shapely.geometry import Point
from shapely.ops import unary_union

CFG = yaml.safe_load(open("config.yaml").read())
AREA_BOUNDS = (
    CFG["area_bounds"]["x_min"],
    CFG["area_bounds"]["x_max"],
    CFG["area_bounds"]["y_min"],
    CFG["area_bounds"]["y_max"],
)
RADIUS = CFG["radius"]
N_ROBOTS = CFG["n_robots"]
ALLOC_TYPE = CFG.get("allocator_type", "balanced").lower()
START_BOUND_DIV = float(CFG.get("start_bound_div", 1))  # >1 shrinks start area

np.random.seed(42)


def shrink_bounds(bounds, div):
    if div <= 1:
        return bounds
    x_min, x_max, y_min, y_max = bounds
    w = x_max - x_min
    h = y_max - y_min
    new_w = w / div
    new_h = h / div
    x_min_new = x_min + 0.5 * (w - new_w)
    x_max_new = x_max - 0.5 * (w - new_w)
    y_min_new = y_min + 0.5 * (h - new_h)
    y_max_new = y_max - 0.5 * (h - new_h)
    return x_min_new, x_max_new, y_min_new, y_max_new


def sample_coverage_points(bounds, radius):
    x_min, x_max, y_min, y_max = bounds
    dx = radius * math.sqrt(3.0)
    dy = radius * 1.5
    centres = []
    y = y_min + radius / 2
    row = 0
    while y <= y_max - radius / 2 + 1e-9:
        shift = 0.0 if row % 2 == 0 else 0.5 * dx
        x = x_min + shift
        while x <= x_max + radius/2 + 1e-9:
            centres.append((x, y))
            x += dx
        y += dy
        row += 1
    return np.asarray(centres, float)


class BalancedKMeansAllocator:
    def __init__(self, starts, points, max_iter=25):
        self.starts = starts.astype(float)
        self.points = points.astype(float)
        self.k = len(starts)
        self.max_iter = max_iter
        self.assignments = {}

    def allocate(self):
        centroids = self.starts.copy()
        for _ in range(self.max_iter):
            dists = np.linalg.norm(self.points[:, None, :] - centroids[None, :, :], axis=2)
            lab = np.argmin(dists, axis=1)
            new = np.zeros_like(centroids)
            for i in range(self.k):
                mask = lab == i
                new[i] = self.points[mask].mean(0) if mask.any() else centroids[i]
            if np.allclose(new, centroids):
                break
            centroids = new
        self.assignments = {}
        for i in range(self.k):
            self.assignments[i] = self.points[lab == i]


class GreedyNearestAllocator:
    def __init__(self, starts, points):
        self.starts = np.array(starts)
        self.points = np.array(points)
        self.assignments = {}

    def allocate(self):
        dists = np.linalg.norm(self.points[:, None, :] - self.starts[None, :, :], axis=2)
        labels = np.argmin(dists, axis=1)
        groups = {i: [] for i in range(len(self.starts))}
        for pi, lbl in enumerate(labels):
            groups[int(lbl)].append(self.points[pi])
        self.assignments = {i: np.array(groups[i]) for i in groups}


def draw_solution(ax, starts, groups, radius, cmap):
    for r in range(len(starts)):
        start = starts[r]
        pts = groups[r]
        if len(pts) == 0:
            continue
        disks = [Point(x, y).buffer(radius, resolution=16) for x, y in pts]
        union = unary_union(disks)
        polys = list(union.geoms) if union.geom_type == "MultiPolygon" else [union]
        for poly in polys:
            ax.add_patch(
                MplPolygon(np.asarray(poly.exterior.coords), facecolor=cmap(r), edgecolor=None, alpha=0.25, zorder=1)
            )
        ax.plot(start[0], start[1], "o", color=cmap(r), markeredgecolor="k", ms=8, zorder=3)


def main():
    start_bounds = shrink_bounds(AREA_BOUNDS, START_BOUND_DIV)
    starts = np.random.uniform(
        [start_bounds[0], start_bounds[2]],
        [start_bounds[1], start_bounds[3]],
        size=(N_ROBOTS, 2),
    )
    points = sample_coverage_points(AREA_BOUNDS, RADIUS)
    if ALLOC_TYPE == "greedy":
        allocator = GreedyNearestAllocator(starts, points)
        tag = "greedy"
    else:
        allocator = BalancedKMeansAllocator(starts, points)
        tag = "balanced"
    allocator.allocate()
    fig, ax = plt.subplots(figsize=(8, 6))
    cmap = plt.cm.get_cmap("tab10", N_ROBOTS)
    draw_solution(ax, starts, allocator.assignments, RADIUS, cmap)
    ax.set_xlim(AREA_BOUNDS[0], AREA_BOUNDS[1])
    ax.set_ylim(AREA_BOUNDS[2], AREA_BOUNDS[3])
    ax.set_title(f"{tag} allocation")
    ax.set_aspect("equal")
    ax.grid(True)
    fig.tight_layout()
    fig.savefig(f"output/{tag}_allocator_{int(time.time())}.png", dpi=300)
    plt.close(fig)


if __name__ == "__main__":
    main()
