
For english version click [here](#robotic-coverage-and-tour-planning)

# Robotische Abdeckung und Tourenplanung

Dieses Projekt bietet zwei Python-Skripte zur Generierung und Optimierung von Abdeckungstouren für mehrere Roboter in einem 2D-Bereich.

* **Rasterbasierte Abtastung** sorgt für vollständige Abdeckung, indem Wegpunkte so platziert werden, dass Kreise mit einem gegebenen Radius das Gebiet überdecken.
* **Gierige Zuordnung** weist Wegpunkte dem jeweils nächstgelegenen Roboter zu, um eine erste Lösung zu erhalten.
* **Nearest-Neighbor-TSP-Heuristik** erstellt für jeden Roboter eine Anfangstour.
* **Genetische Algorithmus-Optimierung** verfeinert die Besuchsreihenfolge, um kombinierte Weglängen- und Kurvenkosten zu minimieren.
* **Visualisierung** vereinigt Abdeckungsregionen und zeichnet Touren mit Matplotlib.

## Anforderungen

Alle Abhängigkeiten sind in `requirements.txt` aufgelistet:

```bash
pip install -r requirements.txt
```

## Konfiguration

Bearbeiten Sie `config.yaml`, um Parameter festzulegen:

* **area\_bounds**: `[x_min, x_max, y_min, y_max]` definiert die Gebietsgrenzen.
* **n\_robots**: Anzahl der Roboter.
* **radius**: Abdeckungsradius jedes Roboters.
* **turn\_coef**: Gewichtung der Kurvenkosten im genetischen Algorithmus.
* **ga**:

  * `pop_size`: Populationsgröße.
  * `generations`: Anzahl der GA-Iterationen.
  * `mut_rate`: Mutationswahrscheinlichkeit.

## Skripte

### `allocator.py`

Führt aus:

1. Rasterbasierte Abtastung der Abdeckungspunkte.
2. Gierige Nearest-Neighbor-Zuordnung der Punkte zu Robotern.
3. Tourenplanung per Nearest-Neighbor-Heuristik.
4. Visualisierung der Anfangstouren und Speicherung als `output/initial_<timestamp>.png`.

> **Warum diese Berechnung?**
> Der Kurvenwinkel θᵢ wird aus dem Skalarprodukt aufeinanderfolgender Streckensegmente berechnet:
>
> θᵢ = arccos( ((pᵢ - pᵢ₋₁) · (pᵢ₋₁ - pᵢ₋₂)) / (||pᵢ - pᵢ₋₁|| \* ||pᵢ₋₁ - pᵢ₋₂||) ).
>
> Diese Formel misst die Richtungsänderung und bestraft scharfe Kurven, um glattere, effizientere Pfade zu fördern.

Ausführen mit:

```bash
python allocator.py
```

### `optimizer.py`

Optimiert jede Robotertour durch:

1. Berechnung der Anfangstourkosten: Weglänge plus Kurvenwinkel-Strafe.
2. Ausführung eines genetischen Algorithmus mit 10 % Elitismus, Ein-Punkt-Crossover und Tauschmutation.
3. Gegenüberstellung von Anfangs- und optimierten Touren in `output/initial_vs_order-optimized_<timestamp>.png`.
4. Ausgabe einer Tabelle mit Kostenvergleich pro Roboter.

#### Kostenfunktion

Für eine Tour \$T=(p\_0,\dots,p\_n)\$ gilt:

$$
C(T)=\sum_{i=1}^n ||p_i - p_{i-1}|| + \lambda \sum_{i=2}^n \theta_i
$$

wobei \$\theta\_i\$ der Kurvenwinkel am Punkt \$p\_{i-1}\$ ist und \$\lambda\$ (`turn_coef`) die Gewichtung darstellt.

Ausführen mit:

```bash
python optimizer.py
```

## Dateistruktur

```
├── allocator.py           # Greedy-Abdeckungszuordnung
├── optimizer.py           # GA-basierte Tourenoptimierung
├── config.yaml            # Parameterdefinitionen
├── requirements.txt       # Python-Abhängigkeiten
├── Dockerfile             # Definition des Container-Images
├── docker-compose.yaml    # Konfiguration für Docker Compose
├── entrypoint.sh          # Einstiegsskript für den Container
├── LICENSE                # Projektlizenz
├── output/                # Verzeichnis für generierte Grafiken
└── README.md              # Projektdokumentation
```

## Docker-Setup

Das Projekt nutzt Docker Compose (`docker-compose.yaml`), um eine reproduzierbare Umgebung bereitzustellen. Beim Ausführen von:

```bash
docker-compose up --build
```

wird:

1. Ein Python-3.9-Image mit allen Abhängigkeiten erstellt.
2. Der Quellcode und das `output/`-Verzeichnis im Container bereitgestellt.
3. Ein interaktives Terminal mit dem Einstiegsskript gestartet.

Im Container können Sie dann:

* `python allocator.py` oder `python optimizer.py` ausführen, um die Workflows zu starten.
* Keine Argumente übergeben, um eine Bash-Shell zu öffnen.

## Lizenz

Dieses Projekt steht unter der MIT-Lizenz. Nutzen und passen Sie den Code gerne für Ihre eigenen Anforderungen in der robotischen Abdeckung und Tourenplanung an.


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

### 1. Coverage Allocation (`allocator.py`)

Performs grid‐based sampling of coverage points, assigns them to robots using a greedy nearest‐neighbor allocator, and visualizes the initial tours.

**How to run:**

```bash
python allocator.py
```

This will generate a figure of the initial coverage allocation and save it under the `output/` directory.

---

### 2. Route Optimization (`optimizer.py`)

#### Cost Function

For a robot tour $ T=(p_0, p_1, \ldots, p_n) $ starting at $ p_0 $, the total cost is defined as:

$$
C(T) = \sum_{i=1}^{n} \lVert p_i - p_{i-1} \rVert + \ \lambda \sum_{i=2}^{n} \theta_i
$$

$$ \theta_i = \arccos\bigl( \frac{(p_i - p_{i-1}) \cdot (p_{i-1} - p_{i-2})}{\lVert p_i - p_{i-1} \rVert , \lVert p_{i-1} - p_{i-2} \rVert} \bigr)\ 
$$


where:
* $ \lVert p_i - p_{i-1} \rVert\ $ is the Euclidean distance between consecutive waypoints.
* The turning angle $ \theta_i$ is computed at $p_{i-1}$ from the dot product between consecutive path segments.
* $\lambda $ corresponds to the weighting in the GA.

**How to run:**

```bash
python optimizer.py
```

Process:

1. Load configuration and sample start positions.
2. Allocate coverage waypoints via the greedy allocator.
3. For each robot:

   * Compute the cost of the initial tour.
   * Run the GA optimizer to reduce combined distance and turning cost.
4. Produce a side‐by‐side visualization of initial vs. optimized tours in `output/`.
5. Print a per‐robot cost comparison table to the console.## Outputs

* **`output/initial_<timestamp>.png`**: Initial greedy coverage tours.
* **`output/initial_vs_order-optimized_<timestamp>.png`**: Comparison of tours before and after GA optimization.
* **Console**: Tabulated cost comparison for each robot.

## File Structure

```
├── config.yaml            # Parameter definitions
├── requirements.txt       # Project dependencies
├── main.py                # Initial greedy allocation entry point
├── optimize_tours.py      # GA‐based tour optimization script
├── coverage.py            # Sampling, allocation, routing, and visualization modules
├── model.py               # Core functions and classes (sample_coverage_points, plan_tour, etc.)
├── output/                # Generated figures
└── README.md              # Project documentation
```

## Docker Setup

This project leverages containerization to ensure a reproducible environment and seamless execution:

1. **Dockerfile**: Defines a lightweight Python 3.9 image. It installs all Python dependencies from `requirements.txt`, copies the project files, and configures an entrypoint script to initialize the workspace.

2. **Entrypoint script**: Automatically prepares the runtime directory (creating the `output` folder) before handing control to either a Bash shell (when no arguments are supplied) or any specified command. This allows you to run scripts directly as container commands or explore the environment interactively.

3. **Docker Compose file**: Orchestrates the single “swarm-explorer” service. It builds the image, mounts the project directory (for live code editing) and the output folder, and keeps the container in interactive mode. With this setup, you can easily launch the container, execute any of the Python entry points (`main.py`, `optimize_tours.py`, etc.), and have generated outputs persisted locally.

**How it works together:** When you run `docker-compose up --build`, Docker Compose will:

* Build the custom image as defined in the Dockerfile.
* Mount your source code and `output` directory into `/ws` inside the container.
* Start the container in interactive mode, invoking the entrypoint script.

Within the container, you can then:

* Run `python main.py` or `python optimize_tours.py` to execute the coverage planning workflows.
* Omit command-line arguments to drop into a Bash shell for debugging or exploration.

This containerized setup ensures consistent dependencies, simplified setup on any machine, and straightforward retrieval of generated figures via the shared `output` volume.

## License

This project is released under the MIT License. Feel free to use and adapt the code for your own robotic coverage and routing needs.
