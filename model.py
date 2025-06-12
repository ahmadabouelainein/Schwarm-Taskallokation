
import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import Point
from shapely.ops import unary_union
from matplotlib.patches import Polygon as MplPolygon
import math
np.random.seed(0)
def sample_coverage_points(bounds, radius):
    """
    Sample waypoints so that circles of given radius cover the rectangular bounds.
    """
    x_min, x_max, y_min, y_max = bounds
    step = radius * math.sqrt(2)
    xs = np.arange(x_min + radius, x_max, step)
    ys = np.arange(y_min + radius, y_max, step)
    return np.array(np.meshgrid(xs, ys)).T.reshape(-1, 2)

def plan_tour(start, waypoints):
    """
    Nearest‐neighbor TSP heuristic from `start` through all `waypoints`.
    """
    pts = [tuple(p) for p in waypoints]
    tour = [tuple(start)]
    current = np.array(start)
    while pts:
        dists = [np.linalg.norm(np.array(p) - current) for p in pts]
        idx = int(np.argmin(dists))
        current = np.array(pts.pop(idx))
        tour.append(tuple(current))
    return np.array(tour)

class GreedyNearestAllocator:
    """
    Assign each waypoint to its nearest robot start (no balancing).
    """
    def __init__(self, starts, points):
        self.starts = np.array(starts)
        self.points = np.array(points)
        self.assignments = {}

    def allocate(self):
        dists = np.linalg.norm(
            self.points[:, None, :] - self.starts[None, :, :],
            axis=2
        )
        labels = np.argmin(dists, axis=1)
        groups = {i: [] for i in range(len(self.starts))}
        for pi, lbl in enumerate(labels):
            groups[lbl].append(self.points[pi])
        self.assignments = {i: np.array(groups[i]) for i in groups}

def draw_solution(ax, starts, groups, radius, cmap):
    """
    Draw coverage areas (unioned disks) + tours + start points.
    """
    for i, start in enumerate(starts):
        pts = groups[i]
        tour = plan_tour(start, pts)
        # merge coverage disks
        disks = [Point(x, y).buffer(radius, resolution=16) for x, y in tour]
        coverage_area = unary_union(disks)
        polys = [coverage_area] if coverage_area.geom_type == 'Polygon' else coverage_area.geoms
        for poly in polys:
            ax.add_patch(MplPolygon(
                np.array(poly.exterior.coords),
                facecolor=cmap(i), edgecolor=None,
                alpha=0.3, zorder=1
            ))
        # tour path and start marker
        ax.plot(tour[:,0], tour[:,1], '-', color=cmap(i), lw=1.5, zorder=2)
        ax.plot(start[0], start[1], 'o', color=cmap(i),
                markersize=8, markeredgecolor='k', zorder=3)

if __name__ == "__main__":
    # parameters
    area_bounds = (0, 100, 0, 50)
    n_robots    = 5
    radius      = 5.0

    # random starts + sample points
    rng    = np.random.RandomState(42)
    starts = rng.uniform([area_bounds[0], area_bounds[2]],
                         [area_bounds[1], area_bounds[3]],
                         size=(n_robots, 2))
    points = sample_coverage_points(area_bounds, radius)

    # initial greedy allocation
    allocator = GreedyNearestAllocator(starts, points)
    allocator.allocate()

    # visualize
    fig, ax = plt.subplots(figsize=(8,6))
    cmap = plt.cm.get_cmap('tab10', n_robots)
    draw_solution(ax, starts, allocator.assignments, radius, cmap)
    ax.set_xlim(area_bounds[0], area_bounds[1])
    ax.set_ylim(area_bounds[2], area_bounds[3])
    ax.set_aspect('equal'); ax.grid(True)
    ax.set_title("Initial GreedyNN Allocation")
    plt.savefig("output/initial.png", dpi=300)
    plt.close()

