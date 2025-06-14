# Robotic Coverage and Tour Planning
> English · [zur **deutschen Version** scrollen](#robotische-abdeckung-und-tourenplanung)

This repository contains three lean Python scripts for multi-robot coverage inside a rectangle:

| Script                | Purpose                                                                                                                                                               |
| --------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **`allocator.py`**    | Samples coverage points, allocates them to robots using either a *balanced k-means* or a *greedy nearest* strategy (set via `allocator_type`), then visualises the discs. |
| **`two-opt.py`**      | Builds a serpentine scan path for each robot and refines it with **2-opt** (distance-only cost).                                                                      |
| **`ga_optimizer.py`** | Runs a simple genetic algorithm that minimises *distance + turning* cost for each tour and compares before/after costs.                                              |

All scripts read their parameters from **`config.yaml`** – *set it once, run everywhere*.

---

## Quick start

```bash
# install dependencies once
pip install -r requirements.txt

# create a coverage-allocation figure
python allocator.py

# 2-opt refinement
python two-opt.py

# GA (distance + turn) optimisation
python ga_optimizer.py
