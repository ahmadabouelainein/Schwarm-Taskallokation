# optimization.py

import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import Point
from shapely.ops import unary_union
import math
import random
import pandas as pd

from model import (
    sample_coverage_points,
    plan_tour,
    GreedyNearestAllocator,
    draw_solution
)

# ────────────────────────────────────────────────────────────────────────────────
def compute_path_turn(tour):
    """Return (path_length, total_turn_angle)."""
    segs = np.diff(tour, axis=0)
    path_length = np.linalg.norm(segs, axis=1).sum()
    total_angle = 0.0
    for i in range(1, len(segs)):
        v1, v2 = segs[i-1], segs[i]
        norm = np.linalg.norm(v1)*np.linalg.norm(v2)
        if norm == 0: continue
        cosang = max(min(np.dot(v1, v2)/norm, 1.0), -1.0)
        total_angle += math.acos(cosang)
    return path_length, total_angle

def genetic_optimize(starts, points, radius,
                     pop_size=30, generations=50,
                     turn_coef=2.0, elite_fraction=0.2, mut_rate=0.1):
    """
    GA that minimizes path+turn cost **plus** overlap-area penalty.
    """
    num_robots = len(starts)
    num_points = len(points)
    circle_area = math.pi * radius**2

    def random_ind():
        return [random.randrange(num_robots) for _ in range(num_points)]

    def evaluate(ind):
        # group waypoints
        groups = {i: [] for i in range(num_robots)}
        for pi, r in enumerate(ind):
            groups[r].append(points[pi])

        base_cost = 0.0
        coverage_areas = []
        self_overlaps = []

        # per‐robot path/turn + self‐overlap
        for i in range(num_robots):
            tour = plan_tour(starts[i], groups[i])
            pl, ang = compute_path_turn(tour)
            base_cost += pl + ang*turn_coef

            # build coverage polygon
            disks = [Point(x,y).buffer(radius, resolution=16) for x,y in tour]
            union = unary_union(disks)
            coverage_areas.append(union)

            # self‐overlap area
            self_overlaps.append(len(tour)*circle_area - union.area)

        # pairwise overlaps
        cross_overlap = 0.0
        for i in range(num_robots):
            for j in range(i+1, num_robots):
                cross_overlap += coverage_areas[i].intersection(coverage_areas[j]).area

        total_overlap = sum(self_overlaps) + cross_overlap
        return base_cost + total_overlap

    # init population
    pop = [random_ind() for _ in range(pop_size)]
    fitness = [evaluate(ind) for ind in pop]

    for _ in range(generations):
        elite_count = max(1, int(elite_fraction*pop_size))
        elites = sorted(range(pop_size), key=lambda i: fitness[i])[:elite_count]
        new_pop = [pop[i] for i in elites]

        # crossover + mutation
        while len(new_pop) < pop_size:
            p1, p2 = random.sample(new_pop, 2)
            cx = random.randrange(1, num_points-1)
            child = p1[:cx] + p2[cx:]
            if random.random() < mut_rate:
                child[random.randrange(num_points)] = random.randrange(num_robots)
            new_pop.append(child)

        pop = new_pop
        fitness = [evaluate(ind) for ind in pop]

    best_idx = min(range(pop_size), key=lambda i: fitness[i])
    return pop[best_idx], fitness[best_idx]

# ────────────────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    # problem setup
    area_bounds    = (0, 100, 0, 50)
    n_robots       = 5
    radius         = 5.0
    turn_coef      = 2.0

    rng    = np.random.RandomState(42)
    starts = rng.uniform([area_bounds[0], area_bounds[2]],
                         [area_bounds[1], area_bounds[3]],
                         size=(n_robots, 2))
    points = sample_coverage_points(area_bounds, radius)

    # -- Initial GreedyNN --
    greedy = GreedyNearestAllocator(starts, points)
    greedy.allocate()

    # -- GA Optimize --
    best_assign, best_cost = genetic_optimize(
        starts, points, radius,
        pop_size=250, generations=300,
        turn_coef=turn_coef,
        elite_fraction=0.2,
        mut_rate=0.05
    )

    # helpers to compute detailed overlap & cost
    def detailed_breakdown(starts, groups):
        circle_area = math.pi * radius**2
        records = []
        coverage_areas = []
        base_sum = 0.0
        self_sum  = 0.0

        for i in range(n_robots):
            tour = plan_tour(starts[i], groups[i])
            pl, ang = compute_path_turn(tour)
            base_sum += pl + ang*turn_coef

            disks = [Point(x,y).buffer(radius, resolution=16) for x,y in tour]
            union = unary_union(disks)
            coverage_areas.append(union)
            so = len(tour)*circle_area - union.area
            self_sum += so

            records.append({
                "Robot": f"R{i}",
                "Path Len": round(pl,2),
                "Turn Ang": round(ang,2),
                "Self‐Overlap": round(so,2)
            })

        cross_sum = 0.0
        for i in range(n_robots):
            for j in range(i+1, n_robots):
                cross_sum += coverage_areas[i].intersection(coverage_areas[j]).area

        total_overlap = self_sum + cross_sum
        total_cost    = base_sum + total_overlap
        return records, total_overlap, total_cost

    # build optimized groups
    opt_groups = {i: [] for i in range(n_robots)}
    for pi, r in enumerate(best_assign):
        opt_groups[r].append(points[pi])

    # -- Visualization --
    fig, (ax1, ax2) = plt.subplots(1,2, figsize=(16,6))
    cmap = plt.cm.get_cmap('tab10', n_robots)

    draw_solution(ax1, starts, greedy.assignments, radius, cmap)
    init_rec, init_ol, init_cost = detailed_breakdown(starts, greedy.assignments)
    ax1.set_title(f"Initial GreedyNN\nCost: {init_cost:.2f}")

    draw_solution(ax2, starts, opt_groups, radius, cmap)
    opt_rec, opt_ol, opt_cost = detailed_breakdown(starts, opt_groups)
    ax2.set_title(f"GA-Optimized\nCost: {opt_cost:.2f}")

    for ax in (ax1, ax2):
        ax.set_xlim(area_bounds[0], area_bounds[1])
        ax.set_ylim(area_bounds[2], area_bounds[3])
        ax.set_aspect('equal'); ax.grid(True)

    plt.tight_layout()
    plt.show()

    # -- Detailed Tables --
    print("\nInitial GreedyNN Detailed:\n")
    print(pd.DataFrame(init_rec).to_markdown(index=False))
    print(f"Pairwise Overlap + Self: {init_ol:.2f}\nTotal Cost: {init_cost:.2f}\n")

    print("\nGA-Optimized Detailed:\n")
    print(pd.DataFrame(opt_rec).to_markdown(index=False))
    print(f"Pairwise Overlap + Self: {opt_ol:.2f}\nTotal Cost: {opt_cost:.2f}")
