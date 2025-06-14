# Robotic Coverage and Tour Planning

> English · [zur **deutschen Version** scrollen](#robotische-abdeckung-und-tourenplanung)

This repository contains three lean Python scripts for multi-robot coverage inside a rectangle:

| Script                | Purpose                                                                                                                                                                   |
| --------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **`allocator.py`**    | Samples coverage points, allocates them to robots using either a *balanced k-means* or a *greedy nearest* strategy (set via `allocator_type`), then visualises the discs. |
| **`two-opt.py`**      | Builds a serpentine scan path for each robot and refines it with **2-opt** (distance-only cost).                                                                          |
| **`ga_optimizer.py`** | Runs a simple genetic algorithm that minimises *distance + turning* cost for each tour and compares before/after costs.                                                   |

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
```

Images are written to **`output/`** with time-stamped names such as
`balanced_allocator_20250614_153045.png`.

---

## Configuration (`config.yaml`)

```yaml
area_bounds:          # rectangle to cover
  x_min: 0
  x_max: 100
  y_min: 0
  y_max: 50

n_robots: 7           # number of robots
radius: 2.0           # sensing radius

# Allocation options
allocator_type: balanced   # balanced or greedy
start_bound_div: 5         # 1 = full area, >1 = smaller centred start area

# Initial-tour options
init_tour_type: serpentine # serpentine or nn

# Cost-function parameters (GA script)
turn_coef: 2.0
repeat_penalty: 20.0

ga:                       # GA hyper-parameters
  pop_size: 1000
  generations: 150
  elite_fraction: 0.2
  mut_rate: 0.05
```

**Key options**

* **`allocator_type`** — `balanced` (k-means) or `greedy` (nearest neighbour) allocation.
* **`start_bound_div`** — shrink factor for the start rectangle (> 1 places starts nearer the centre).
* **`init_tour_type`** — `serpentine` (scan-line) or `nn` (nearest-neighbour) initial path.

---

## Algorithmic flow

| Stage               | Method                                                     | File              |
| ------------------- | ---------------------------------------------------------- | ----------------- |
| Coverage sampling   | Hexagonal grid covering the rectangle                      | `allocator.py`    |
| Allocation          | *Balanced* (iterative k-means) or *Greedy* (closest start) | `allocator.py`    |
| Tour initialisation | Serpentine scan-line                                       | `two-opt.py`      |
| Tour optimisation 1 | 2-opt with k-nearest list (`K = 40`)                       | `two-opt.py`      |
| Tour optimisation 2 | Genetic algorithm (distance + turn)                        | `ga_optimizer.py` |

All scripts honour `start_bound_div` via the shared `_shrink_bounds()` helper.

---

## Requirements

```text
numpy
matplotlib
shapely
pandas
PyYAML
tqdm
```

Install once:

```bash
pip install -r requirements.txt
```

---

## Folder layout

```
├── allocator.py          # Coverage allocation & visualisation
├── two-opt.py            # Serpentine + 2-opt refinement
├── ga_optimizer.py       # GA distance+turn optimisation
├── config.yaml           # All parameters
├── requirements.txt      # Python dependencies
├── docker-compose.yaml   # One-liner container orchestration (optional)
├── Dockerfile            # Minimal Python image with dependencies
├── entrypoint.sh         # Container start helper
├── output/               # Generated figures
└── README.md             # This file
```

---

## Docker / Docker Compose (optional)

A lightweight container image lets you avoid installing Python locally.

```bash
# build the image and start the service (detached)
docker compose up --build -d
```

What happens:

1. **Dockerfile** builds a slim Python image and installs the dependencies from `requirements.txt`.
2. **docker-compose.yaml** mounts the project folder and the `output/` volume for persistent figures.
3. The container launches **and immediately waits** (it does **not** drop you into a shell).

### Running commands inside the container

Use `docker exec` to open a Bash session or run individual scripts:

```bash
# attach an interactive shell
docker exec -it swarm-explorer bash

# inside the shell, for example:
python allocator.py && python two-opt.py
```

> **Tip (VS Code)** With the *Dev Containers* extension you can attach VS Code directly to the running container (“Attach to Running Container…”) and work as if it were your local environment.

Stop and remove the container with:

```bash
docker compose down
```

Figures remain in `output/` on the host.

---

## Technique comparison

### Allocation methods

| Method               | Core idea                                                                         | Strengths                        | Trade‑offs                                                                             |
| -------------------- | --------------------------------------------------------------------------------- | -------------------------------- | -------------------------------------------------------------------------------------- |
| **Balanced k‑means** | Iteratively moves centroids until each robot centre balances the assigned points. | Even workload, compact clusters. | Slightly slower (few dozen iterations); centroids can drift from real start positions. |
| **Greedy nearest**   | Assigns every waypoint to the geographically closest start.                       | Ultra‑fast, no iterations.       | One robot can receive many more points; cluster shape may be elongated.                |

**Rule of thumb** – use *balanced* when you need equal work per robot, *greedy* when speed matters and uneven loads are acceptable.

### Initial‑tour builders

| Builder                    | Principle                                                | When to use                                   |
| -------------------------- | -------------------------------------------------------- | --------------------------------------------- |
| **Serpentine**             | Lexicographic scan‑line; alternates direction every row. | Grid‑like layouts, predictable order.         |
| **NN (nearest‑neighbour)** | Greedily visits the closest unvisited waypoint.          | Irregular point clouds, shorter raw distance. |

### Tour‑optimisation pipelines

| Pipeline               | Optimisation steps                                                    | Cost used                | Typical runtime                                       | Best for                                   |
| ---------------------- | --------------------------------------------------------------------- | ------------------------ | ----------------------------------------------------- | ------------------------------------------ |
| **Serpentine + 2‑opt** | Scanline ordering → local 2‑opt swaps (distance only).                | Distance                 | ✧✧ Fast (sub‑second per robot)                        | Quick refinement, large swarms.            |
| **Genetic algorithm**  | Random population → crossover / mutation → elitism (distance + turns) | Distance + turning angle | ✧ Slower (seconds–minutes depending on `generations`) | High‑quality paths when turn cost matters. |

---

## Robotische Abdeckung und Tourenplanung

Dieser Abschnitt bietet eine deutschsprachige Zusammenfassung. Wenn Sie Englisch bevorzugen, scrollen Sie nach oben.

### Überblick

| Skript                | Zweck                                                                                                                             |
| --------------------- | --------------------------------------------------------------------------------------------------------------------------------- |
| **`allocator.py`**    | Erzeugt Abdeckungspunkte, weist sie Robotern zu (*balanced* = k-means, *greedy* = nächster Startpunkt) und speichert eine Grafik. |
| **`two-opt.py`**      | Erstellt aus den Punkten einen Serpentinenpfad pro Roboter und verbessert ihn mit 2-Opt (nur Distanzkosten).                      |
| **`ga_optimizer.py`** | Optimiert jeden Pfad mit einem genetischen Algorithmus (Distanz + Kurven) und vergleicht vorher/nachher.                          |

Alle Parameter liegen zentral in **`config.yaml`**.

---

### Schnellstart

```bash
pip install -r requirements.txt   # Abhängigkeiten installieren

python allocator.py               # Abdeckung + Visualisierung
python two-opt.py                 # 2-Opt-Verfeinerung
python ga_optimizer.py            # GA-Optimierung
```

Erzeugte Bilder landen unter **`output/`** (Datum + Uhrzeit im Dateinamen).

---

### Wichtige Konfigurationsfelder (`config.yaml`)

| Feld              | Bedeutung                                                                         |
| ----------------- | --------------------------------------------------------------------------------- |
| `allocator_type`  | `balanced` (ausgewogene k-means-Zuordnung) oder `greedy` (nächstgelegener Start). |
| `start_bound_div` | Verkleinert das Startareal. `1` = volle Fläche, `>1` = nur mittlerer Bereich.     |
| `radius`          | Abdeckungsradius pro Roboter (bestimmt die Gitterabtastung).                      |
| `turn_coef`       | Gewichtung der Kurvenkosten im GA-Kostenmaß.                                      |
| Block `ga:`       | Parameter des genetischen Algorithmus (Populationsgröße, Generationen …).         |

---

### Vergleich der Zuordnungsverfahren

| Verfahren            | Grundidee                                                                              | Stärken                                    | Nachteile                                                                  |
| -------------------- | -------------------------------------------------------------------------------------- | ------------------------------------------ | -------------------------------------------------------------------------- |
| **Balanced k-means** | Zentren werden iterativ verschoben, bis alle Roboter eine ähnliche Punktzahl besitzen. | Gleichmäßige Auslastung, kompakte Cluster. | Etwas langsamer; Schwerpunkte können von realen Startpositionen abweichen. |
| **Greedy nearest**   | Jeder Wegpunkt wird dem nächstgelegenen Start zugeordnet.                              | Sehr schnell, keine Iterationen.           | Ungleich verteilte Arbeit möglich; langgezogene Cluster.                   |

### Vergleich der Initialwege

| Methode        | Prinzip                                               | Empfehlung                       |
| -------------- | ----------------------------------------------------- | -------------------------------- |
| **Serpentine** | Zeilenweises Abfahren (jede zweite Zeile gespiegelt). | Gleichmäßige Gitterstrukturen.   |
| **NN**         | Greedy-Sprung zum nächstgelegenen unbesuchten Punkt.  | Unregelmäßige Punktverteilungen. |

### Vergleich der Tourenoptimierungen

| Pipeline                    | Schritte                                      | Kostenmaß              | Laufzeit        | Geeignet für                                     |
| --------------------------- | --------------------------------------------- | ---------------------- | --------------- | ------------------------------------------------ |
| **Serpentine + 2-Opt**      | Zeilenweise Sortierung → lokale 2-Opt-Tausche | Distanz                | ✧✧ Sehr schnell | Schnelle Verbesserungen, viele Roboter.          |
| **Genetischer Algorithmus** | Population → Crossover/Mutation → Elitismus   | Distanz + Kurvenwinkel | ✧ Langsamer     | Höhere Qualität, wenn Kurvenkosten wichtig sind. |

### Docker-Variante

Falls Sie nichts lokal installieren möchten, können Sie das Projekt containerisiert betreiben:

```bash
# Image bauen & Container (detached) starten
docker compose up --build -d

# Shell in den laufenden Container öffnen
docker exec -it swarm-explorer bash
```

Im Container können Sie dieselben Befehle ausführen (`python allocator.py` usw.).
Ergebnisse erscheinen dank des gemounteten Ordners **`output/`** direkt auf dem Host.

---

### Ordnerstruktur (Kurzfassung)

```
allocator.py       – Punktzuordnung & Plot
two-opt.py         – 2-Opt-Verbesserung
ga_optimizer.py    – Genetischer Algorithmus
config.yaml        – Zentrale Parameter
output/            – Erzeugte Abbildungen
```

Viel Erfolg bei Ihrer robotischen Abdeckung und Pfadplanung!
