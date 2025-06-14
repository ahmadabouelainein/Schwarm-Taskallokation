import os, time, random
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
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
INIT_TYPE = CFG.get("init_tour_type", "serpentine").lower()
K = 40
Path("output").mkdir(exist_ok=True)
np.random.seed(42)


def serpentine_idx(pts, tol):
    idx = np.lexsort((pts[:, 0], pts[:, 1]))
    rows, cur = [], [idx[0]]
    for p, n in zip(idx[:-1], idx[1:]):
        if abs(pts[n, 1] - pts[p, 1]) < tol:
            cur.append(n)
        else:
            rows.append(cur)
            cur = [n]
    rows.append(cur)
    for r in range(1, len(rows), 2):
        rows[r] = rows[r][::-1]
    flat = []
    for row in rows:
        flat.extend(row)
    return np.array(flat, int)


def nn_tour(start, pts):
    order, cur, pool = [], np.array(start), pts.tolist()
    while pool:
        nxt = min(pool, key=lambda p: np.linalg.norm(np.array(p) - cur))
        order.append(tuple(nxt))
        pool.remove(nxt)
        cur = np.array(nxt)
    return np.vstack(([start], order))


def build_dist(xy):
    diff = xy[:, None] - xy[None]
    return np.linalg.norm(diff, axis=2)


def build_cand(d, k):
    return [row.astype(int) for row in np.argsort(d, 1)[:, 1 : k + 1]]


def two_opt(order, d, cand):
    n, improved = len(order), True
    while improved:
        pos = {n: i for i, n in enumerate(order)}
        improved = False
        for i in range(n - 2):
            a, b = order[i], order[i + 1]
            for c in cand[a]:
                j = pos[c]
                if j - i <= 1 or j >= n - 1:
                    continue
                c_, d_ = order[j], order[j + 1]
                if d[a, b] + d[c_, d_] - d[a, c_] - d[b, d_] > 1e-9:
                    order[i + 1 : j + 1] = order[i + 1 : j + 1][::-1]
                    improved = True
                    break
            if improved:
                break


def dist_sum(t):
    return float(np.linalg.norm(np.diff(t, axis=0), axis=1).sum())


def turn_sum(t):
    seg = np.diff(t, axis=0)
    ang = 0.0
    for i in range(1, len(seg)):
        v1, v2 = seg[i - 1], seg[i]
        n1, n2 = np.linalg.norm(v1), np.linalg.norm(v2)
        if n1 == 0 or n2 == 0:
            continue
        ang += float(np.arccos(np.clip(np.dot(v1, v2) / (n1 * n2), -1.0, 1.0)))
    return ang


def overlap_area(paths):
    disks = []
    for p in paths:
        for x, y in p:
            disks.append(Point(x, y).buffer(RADIUS, resolution=8))
    union = unary_union(disks)
    sum_area = sum(d.area for d in disks)
    return sum_area - union.area


def main():
    start_bounds = _shrink_bounds(AREA_BOUNDS, START_BOUND_DIV)
    starts = np.random.uniform(
        [start_bounds[0], start_bounds[2]],
        [start_bounds[1], start_bounds[3]],
        (N_ROBOTS, 2),
    )
    lattice = sample_coverage_points(AREA_BOUNDS, RADIUS)
    if ALLOC_TYPE.startswith("greedy"):
        alloc = GreedyNearestAllocator(starts, lattice)
        tag = "greedy"
    else:
        alloc = BalancedKMeansAllocator(starts, lattice)
        tag = "balanced"
    alloc.allocate()

    init_paths, opt_paths = [], []
    init_d = opt_d = init_a = opt_a = 0.0

    for rid, pts in tqdm(alloc.assignments.items(), desc="Robots"):
        if INIT_TYPE.startswith("nn"):
            init_tour = nn_tour(starts[rid], pts)
        else:
            serp_idx = serpentine_idx(pts, tol=RADIUS * 0.75)
            init_tour = np.vstack(([starts[rid]], pts[serp_idx]))
        init_paths.append(init_tour)
        init_d += dist_sum(init_tour)
        init_a += turn_sum(init_tour)
        xy = init_tour
        dmat = build_dist(xy)
        cand = build_cand(dmat, K)
        order = list(range(len(xy)))
        two_opt(order, dmat, cand)
        opt_tour = xy[order]
        opt_paths.append(opt_tour)
        opt_d += dist_sum(opt_tour)
        opt_a += turn_sum(opt_tour)

    init_ov = overlap_area(init_paths)
    opt_ov = overlap_area(opt_paths)

    cmap = plt.cm.get_cmap("tab10", N_ROBOTS)
    fig, (ax0, ax1) = plt.subplots(1, 2, figsize=(12, 6), sharex=True, sharey=True)
    t0 = f"{INIT_TYPE.upper()} D={init_d:.1f} A={init_a:.1f} O={init_ov:.1f}"
    t1 = f"2-Opt D={opt_d:.1f} A={opt_a:.1f} O={opt_ov:.1f}"
    for ax, paths, ttl in ((ax0, init_paths, t0), (ax1, opt_paths, t1)):
        draw_solution(ax, starts, alloc.assignments, RADIUS, cmap)
        for r, p in enumerate(paths):
            ax.plot(p[:, 0], p[:, 1], "-", color=cmap(r), lw=1.3)
        ax.set_aspect("equal")
        ax.set_title(ttl)
        ax.grid(True, alpha=0.3)
    plt.tight_layout()
    out = Path("output") / f"{tag}_{INIT_TYPE}_2opt_{int(time.time())}.png"
    plt.savefig(out, dpi=300)
    plt.close()
    print(f"[INFO] figure saved → {out}")


if __name__ == "__main__":
    main()
