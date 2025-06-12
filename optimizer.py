import time
import yaml
import os
import math
import random
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
np.random.seed(42)
from model import (
    sample_coverage_points,
    plan_tour,
    GreedyNearestAllocator,
    draw_solution
)

# ────────────────────────────────────────────────────────────────────────────────
# Load configuration
# ────────────────────────────────────────────────────────────────────────────────
with open("config.yaml") as f:
    cfg = yaml.safe_load(f)

area_bounds = (
    cfg["area_bounds"]["x_min"],
    cfg["area_bounds"]["x_max"],
    cfg["area_bounds"]["y_min"],
    cfg["area_bounds"]["y_max"]
)
n_robots       = cfg["n_robots"]
radius         = cfg["radius"]
turn_coef      = cfg["turn_coef"]

ga_cfg         = cfg["ga"]
pop_size       = ga_cfg["pop_size"]
generations    = ga_cfg["generations"]
# force 10% elitism regardless of config
elite_fraction = 0.1
mut_rate       = ga_cfg["mut_rate"]

os.makedirs("output", exist_ok=True)

# ────────────────────────────────────────────────────────────────────────────────
def compute_path_turn(tour):
    segs = np.diff(tour, axis=0)
    length = np.linalg.norm(segs, axis=1).sum()
    angle = 0.0
    for i in range(1, len(segs)):
        v1, v2 = segs[i-1], segs[i]
        norm = np.linalg.norm(v1)*np.linalg.norm(v2)
        if norm == 0: continue
        cosang = max(min(np.dot(v1,v2)/norm,1.0),-1.0)
        angle += math.acos(cosang)
    return length, angle

# ────────────────────────────────────────────────────────────────────────────────
def optimize_order_ga(start, pts, robot_id):
    """
    GA optimizing visit order of `pts` starting from `start`.
    Uses 10% elitism and prints best system-wide cost per generation.
    Returns: best_order, best_cost.
    """
    n = len(pts)
    if n <= 1:
        print(f"Robot {robot_id}: only {n} point(s), no ordering needed.")
        return list(range(n)), 0.0

    def rand_ind():
        ind = list(range(n))
        random.shuffle(ind)
        return ind

    def eval_ind(ind):
        # fitness = travel + turn for this robot
        tour = np.vstack(([start], [pts[i] for i in ind]))
        length, angle = compute_path_turn(tour)
        return length + angle * turn_coef

    def crossover(p1, p2):
        cx = random.randrange(1, n)
        head = p1[:cx]
        tail = [g for g in p2 if g not in head]
        return head + tail

    def mutate(ind):
        i, j = random.sample(range(n), 2)
        ind[i], ind[j] = ind[j], ind[i]

    # initialize
    pop = [rand_ind() for _ in range(pop_size)]
    fitness = [eval_ind(c) for c in pop]
    print(f"Robot {robot_id}: GA start, initial best cost = {min(fitness):.2f}")

    # GA loop
    for gen in range(1, generations+1):
        # select top 10% as elites
        elite_count = max(1, int(elite_fraction * pop_size))
        idx_sorted = sorted(range(pop_size), key=lambda i: fitness[i])
        new_pop = [pop[i] for i in idx_sorted[:elite_count]]

        # fill rest by crossover + mutation
        while len(new_pop) < pop_size:
            p1, p2 = random.sample(new_pop, 2)
            child = crossover(p1, p2)
            if random.random() < mut_rate:
                mutate(child)
            new_pop.append(child)

        pop = new_pop
        fitness = [eval_ind(c) for c in pop]
        # compute system-wide cost = sum of robots' best costs
        best_cost = min(fitness)
        print(f"Robot {robot_id}: Gen {gen}/{generations}, best cost = {best_cost:.2f}")

    best_idx = min(range(pop_size), key=lambda i: fitness[i])
    return pop[best_idx], fitness[best_idx]

# ────────────────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    # fixed Voronoi partitions via GreedyNN
    starts = np.random.uniform(
        [area_bounds[0], area_bounds[2]],
        [area_bounds[1], area_bounds[3]],
        size=(n_robots, 2)
    )
    points = sample_coverage_points(area_bounds, radius)

    greedy = GreedyNearestAllocator(starts, points)
    greedy.allocate()
    groups = greedy.assignments

    init_tours, opt_tours = [], []
    init_cost, opt_cost = 0.0, 0.0
    records = []

    # optimize each robot sequentially
    for i in range(n_robots):
        pts = groups[i]

        # initial tour & cost
        t0 = plan_tour(starts[i], pts)
        l0, a0 = compute_path_turn(t0)
        c0 = l0 + a0*turn_coef
        init_tours.append(t0)
        init_cost += c0

        # GA optimize order
        order, c_opt = optimize_order_ga(starts[i], pts, robot_id=i)
        t1 = np.vstack(([starts[i]], [pts[j] for j in order]))
        l1, a1 = compute_path_turn(t1)
        c1 = l1 + a1*turn_coef
        opt_tours.append(t1)
        opt_cost += c1

        records.append({"Robot": f"R{i}", "Init Cost": round(c0,2), "Opt Cost": round(c1,2)})

    # visualize initial vs optimized
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16,6))
    cmap = plt.cm.get_cmap('tab10', n_robots)

    draw_solution(ax1, starts, groups, radius, cmap)
    for idx, tour in enumerate(init_tours):
        ax1.plot(tour[:,0], tour[:,1], color=cmap(idx), lw=1.5)
    ax1.set_title(f"Initial Tours\nTotal Cost: {init_cost:.2f}")

    draw_solution(ax2, starts, groups, radius, cmap)
    for idx, tour in enumerate(opt_tours):
        ax2.plot(tour[:,0], tour[:,1], color=cmap(idx), lw=1.5)
    ax2.set_title(f"Optimized Tours\nTotal Cost: {opt_cost:.2f}")

    for ax in (ax1, ax2):
        ax.set_xlim(area_bounds[0], area_bounds[1])
        ax.set_ylim(area_bounds[2], area_bounds[3])
        ax.set_aspect('equal')
        ax.grid(True)

    plt.tight_layout()
    plt.savefig(f"output/initial_vs_order-optimized_{int(time.time())}.png", dpi=300)
    plt.close()

    # print comparison
    print("\nPer-robot cost comparison:")
    print(pd.DataFrame(records).to_markdown(index=False))
