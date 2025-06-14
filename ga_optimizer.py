import os, time, random
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import yaml
from shapely.geometry import Point
from shapely.ops import unary_union
from tqdm.auto import tqdm

from allocator import (
    BalancedKMeansAllocator,
    GreedyNearestAllocator,
    sample_coverage_points,
    draw_solution,
    _shrink_bounds,
    AREA_BOUNDS,
    RADIUS,
    N_ROBOTS,
    ALLOC_TYPE,
    START_BOUND_DIV,
)

CFG = yaml.safe_load(Path("config.yaml").read_text())
TURN_COEF = CFG["turn_coef"]
REP_PEN  = CFG["repeat_penalty"]
GA_CFG   = CFG["ga"]
POP_SIZE = GA_CFG["pop_size"]
GENERATIONS = GA_CFG["generations"]
ELITE_FRAC = 0.10
MUT_RATE = GA_CFG["mut_rate"]

OUTPUT_DIR = Path("output"); OUTPUT_DIR.mkdir(exist_ok=True)
np.random.seed(42)


def plan_tour(start, wps):
    pts = [tuple(p) for p in wps]
    tour = [tuple(start)]
    cur = np.array(start)
    while pts:
        cur = np.array(pts.pop(int(np.argmin([np.linalg.norm(np.array(p) - cur) for p in pts]))))
        tour.append(tuple(cur))
    return np.array(tour)


def dist_ang(t):
    seg = np.diff(t, axis=0)
    d = float(np.linalg.norm(seg, axis=1).sum())
    a = 0.0
    for i in range(1, len(seg)):
        v1, v2 = seg[i - 1], seg[i]
        n1, n2 = np.linalg.norm(v1), np.linalg.norm(v2)
        if n1 == 0 or n2 == 0:
            continue
        a += float(np.arccos(np.clip(np.dot(v1, v2) / (n1 * n2), -1.0, 1.0)))
    return d, a


def overlap_single(t):
    disks = [Point(x, y).buffer(RADIUS, resolution=8) for x, y in t]
    union = unary_union(disks)
    return sum(d.area for d in disks) - union.area


def _rand_ind(n):
    ind = list(range(n)); random.shuffle(ind); return ind

def _crossover(p1, p2):
    cut = random.randrange(1, len(p1))
    head = p1[:cut]; tail = [g for g in p2 if g not in head]
    return head + tail

def _mutate(ind):
    i, j = random.sample(range(len(ind)), 2); ind[i], ind[j] = ind[j], ind[i]


def optimise_order_ga(start, pts):
    n = len(pts)
    if n <= 1:
        return list(range(n)), 0.0

    def cost(ind):
        trail = np.vstack(([start], [pts[i] for i in ind]))
        d, a = dist_ang(trail)
        o = overlap_single(trail)
        return d + a * TURN_COEF + o * REP_PEN

    pop = [_rand_ind(n) for _ in range(POP_SIZE)]
    fit = [cost(ind) for ind in pop]
    bar = tqdm(range(GENERATIONS), desc="GA", leave=False)
    for _ in bar:
        elite_n = max(1, int(ELITE_FRAC * POP_SIZE))
        elites = [pop[i] for i in np.argsort(fit)[:elite_n]]
        off = []
        while len(off) < POP_SIZE - elite_n:
            child = _crossover(*random.sample(elites, 2))
            if random.random() < MUT_RATE:
                _mutate(child)
            off.append(child)
        pop = elites + off
        fit = [cost(ind) for ind in pop]
        bar.set_postfix(best=f"{min(fit):.2f}")
    best = int(np.argmin(fit))
    return pop[best], fit[best]


def _vis(starts, groups, init_t, opt_t, metrics, tag):
    d0, a0, o0, d1, a1, o1 = metrics
    cmap = plt.cm.get_cmap("tab10", len(starts))
    fig, (ax0, ax1) = plt.subplots(1, 2, figsize=(13, 6), sharex=True, sharey=True)
    draw_solution(ax0, starts, groups, RADIUS, cmap)
    for i, t in enumerate(init_t):
        ax0.plot(t[:,0], t[:,1], color=cmap(i))
    draw_solution(ax1, starts, groups, RADIUS, cmap)
    for i, t in enumerate(opt_t):
        ax1.plot(t[:,0], t[:,1], color=cmap(i))
    ax0.set_title(f"Init D={d0:.1f} A={a0:.1f} O={o0:.1f}")
    ax1.set_title(f"GA   D={d1:.1f} A={a1:.1f} O={o1:.1f}")
    for ax in (ax0, ax1):
        ax.set_xlim(AREA_BOUNDS[0], AREA_BOUNDS[1])
        ax.set_ylim(AREA_BOUNDS[2], AREA_BOUNDS[3])
        ax.set_aspect('equal'); ax.grid(True)
    p = OUTPUT_DIR / f"{tag}_ga_{int(time.time())}.png"
    fig.tight_layout(); fig.savefig(p, dpi=300); plt.close(fig)
    print(f"[INFO] figure saved → {p}")


def main():
    sb = _shrink_bounds(AREA_BOUNDS, START_BOUND_DIV)
    starts = np.random.uniform([sb[0], sb[2]], [sb[1], sb[3]], (N_ROBOTS, 2))
    pts = sample_coverage_points(AREA_BOUNDS, RADIUS)
    if ALLOC_TYPE.startswith("greedy"):
        alloc = GreedyNearestAllocator(starts, pts); tag = "greedy"
    else:
        alloc = BalancedKMeansAllocator(starts, pts); tag = "balanced"
    alloc.allocate()

    init_t, opt_t = [], []
    d0 = a0 = o0 = d1 = a1 = o1 = 0.0
    for rid, wps in tqdm(alloc.assignments.items(), desc="Robots"):
        init = plan_tour(starts[rid], wps)
        d_i, a_i = dist_ang(init)
        o_i = overlap_single(init)
        init_t.append(init)
        d0 += d_i; a0 += a_i; o0 += o_i
        order, _ = optimise_order_ga(starts[rid], wps)
        trail = np.vstack(([starts[rid]], [wps[i] for i in order]))
        d_o, a_o = dist_ang(trail)
        o_o = overlap_single(trail)
        opt_t.append(trail)
        d1 += d_o; a1 += a_o; o1 += o_o
    _vis(starts, alloc.assignments, init_t, opt_t, (d0,a0,o0,d1,a1,o1), tag)
    df = pd.DataFrame({"Metric":["Distance","Turn","Overlap"], "Init":[d0,a0,o0], "GA":[d1,a1,o1]})
    print(df.to_markdown(index=False))


if __name__ == "__main__":
    main()
