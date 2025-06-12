# Robotic Coverage and Tour Planning

This repository provides Python scripts for planning coverage tours and optimizing robot routes in a 2D rectangular area. It includes:

* A **greedy allocation** approach for initial coverage planning.
* A **genetic algorithm** (GA) method to optimize the visitation order, minimizing travel distance and turning cost.

## Features

* **Grid‐based sampling** of coverage points to ensure full area coverage by disks of a given radius.
* **Greedy nearest‐neighbor allocation** of waypoints to robots (fixed Voronoi partitions).
* **Nearest‐neighbor TSP heuristic** for initial tour planning.
* **Genetic algorithm optimization** of tour sequences with turn‐aware cost.
* **Coverage union** via Shapely to merge overlapping sensing disks.
* **Matplotlib visualization** of coverage areas, tours, and robot start positions.
* **Pandas**-powered cost comparison output.

## Requirements

* Python 3.7+
* NumPy
* Matplotlib
* Shapely
* PyYAML
* Pandas

All dependencies and exact version requirements are listed in `requirements.txt`. You can install them in one step:

```bash
pip install -r requirements.txt
```

Alternatively, individually via:

```bash
pip install numpy matplotlib shapely pyyaml pandas
```

## Configuration

All parameters are set in `config.yaml`. Example:

```yaml
area_bounds:
  x_min: 0
  x_max: 100
  y_min: 0
  y_max: 50

n_robots: 5        # Number of robots
radius: 5.0        # Sensing/coverage radius
turn_coef: 1.0     # Weight for turning in GA cost

ga:
  pop_size: 50
  generations: 100
  # elite_fraction is forced to 0.1 in GA script
  mut_rate: 0.01
```

* **area\_bounds**: Defines the rectangular region to cover.
* **n\_robots**: Number of robots (and start positions).
* **radius**: Coverage radius of each robot (disks will be placed at waypoints).
* **turn\_coef**: Coefficient weighting turning angle in the GA cost function.
* **ga**: Genetic‐algorithm parameters for tour optimization.

## Scripts

### 1. Initial Greedy Allocation (`alloctor.py`)

Generates an initial allocation of coverage waypoints to each robot and visualizes the greedy tours.

**Usage**:

```bash
python alloctor.py
```

Produces:`output/initial_<timestamp>.png`

See [Initial Greedy Allocation](#scripts-1-initial-greedy-allocation-mainpy) above for details.

---

### 2. GA‐Based Tour Optimization (`optimizer.py`)

Improves the visitation order within each robot’s assigned waypoints to reduce travel length and turning angle costs.

**Usage**:

```bash
python optimizer.py
```

This script will:

1. Load parameters and bounds from `config.yaml`.
2. Sample random start positions for `n_robots` and generate coverage waypoints.
3. Allocate waypoints to robots using `GreedyNearestAllocator` (fixed Voronoi partition).
4. For each robot:

   * Compute the initial tour with `plan_tour()` and calculate its cost (`length + turn_coef × total_turn_angle`).
   * Run a GA (`optimize_order_ga`) with 10% elitism to find a better visit order.
   * Record initial and optimized costs.
5. Visualize side‐by‐side plots of initial vs. optimized tours, saving to `output/initial_vs_order-optimized_<timestamp>.png`.
6. Print a per‐robot cost comparison table via Pandas.

**Key Functions**:

* `compute_path_turn(tour)`:

  * Computes total path length and cumulative turning angle for a given ordered sequence of points.

* `optimize_order_ga(start, pts, robot_id)`:

  * Implements a simple GA to minimize `length + turn_coef × angle`.
  * Uses:

    * 10% elitism
    * Single‐point crossover
    * Swap mutation with probability `mut_rate`
  * Prints best cost per generation.
  * Returns the best ordering and its cost.

* `GreedyNearestAllocator` / `plan_tour` / `draw_solution`:

  * As described above in the initial allocation script.

## Outputs

* \`\`: Visualization of greedy allocation.
* \`\`: Comparison of initial vs. GA‐optimized tours.
* **Console**: Per‐robot cost comparison table.

## File Structure

```
├── allocator.py           # Greedy coverage allocation script
├── optimizer.py           # GA‐based route optimization script
├── config.yaml            # Parameter definitions
├── requirements.txt       # Python dependencies
├── Dockerfile             # Container image definition
├── docker-compose.yaml    # Service configuration for Docker Compose
├── entrypoint.sh          # Container entrypoint script
├── LICENSE                # Project license
├── output/                # Generated figures
└── README.md              # This documentation
```

## Docker Setup

This project leverages containerization to ensure a reproducible environment and seamless execution:

1. **Dockerfile**: Defines a lightweight Python 3.9 image. It installs all Python dependencies from `requirements.txt`, copies the project files, and configures an entrypoint script to initialize the workspace.

2. **Entrypoint script**: Automatically prepares the runtime directory (creating the `output` folder) before handing control to either a Bash shell (when no arguments are supplied) or any specified command. This allows you to run scripts directly as container commands or explore the environment interactively.

3. **Docker Compose file**: Orchestrates the single “swarm-explorer” service. It builds the image, mounts the project directory (for live code editing) and the output folder, and keeps the container in interactive mode. With this setup, you can easily launch the container, execute any of the Python entry points (`main.py`, `optimize_tours.py`, etc.), and have generated outputs persisted locally.

**How it works together:** When you run `docker compose up --build`, Docker Compose will:

* Build the custom image as defined in the Dockerfile.
* Mount your source code and `output` directory into `/ws` inside the container.
* Start the container in interactive mode, invoking the entrypoint script.

Within the container, you can then:

* Run `python main.py` or `python optimize_tours.py` to execute the coverage planning workflows.
* Omit command-line arguments to drop into a Bash shell for debugging or exploration.

This containerized setup ensures consistent dependencies, simplified setup on any machine, and straightforward retrieval of generated figures via the shared `output` volume.

## License

This project is released under the MIT License. Feel free to use and adapt the code for your own robotic coverage and routing needs.
