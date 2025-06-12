# Schwarm-Taskallokation

## Ziel der Aufgabe

- Entwicklung eines Systems zur Taskallokation, das:

    - ein Gebiet (2D oder 3D) effizient aufteilt,

    - jedem Roboter eine Teilaufgabe (Gebietsteil) zuweist,

    - dabei flexibel bzgl. Schwarmgröße und Gebietsgrenzen bleibt.

## Annahmen

- Roboterpositionen sind bekannt (z. B. GPS, Motion Capture, oder internes Mapping).
- Roboter haben ähnliche Fähigkeiten (homogener Schwarm).
- Gebiet ist rechteckig (später erweiterbar auf beliebige Formen).
- Erkundung = vollständige Abdeckung


# Schwarm-Taskallokation

This repository implements a complete pipeline for **task‐allocating** a configurable swarm of ground or aerial robots to **explore** a 2D region. Developed as part of an interview exercise, it provides:

- **Algorithmic allocation** (Greedy & GA‐optimized)  
- **2D visualization** of coverage areas and robot paths  
- **Cost models** accounting for distance, turning effort, and overlap  
- **Dockerized** workflow for reproducible execution  

---

## Repository Structure
```
├── config.yaml
├── docker-compose.yaml
├── Dockerfile
├── entrypoint.sh
├── LICENSE
├── model.py
├── optimizer.py
├── README.md
└── requirements.txt
```

* **`config.yaml`**: Defines all user‐configurable parameters (area bounds, swarm size, penalties, GA settings).
* **`docker-compose.yaml`**: Orchestrates the Docker service, mounts volumes, and sets up the shared workspace.
* **`Dockerfile`**: Builds the Python environment and application image, installing dependencies and copying source code.
* **`entrypoint.sh`**: Container entrypoint script that initializes the `output/` folder and launches a shell or passed command.
* **`LICENSE`**: Open-source license governing usage and distribution of the code.
* **`model.py`**: Shared data structures and utility functions (sampling waypoints, TSP planner) used by both allocation and optimization scripts.
* **`optimizer.py`**: Implements the cost model and Genetic-Algorithm loop to improve task assignments, plus side-by-side comparison.
* **`README.md`**: Project overview, setup instructions, and usage guide.
* **`requirements.txt`**: Lists Python package dependencies (`numpy`, `matplotlib`, `shapely`, `pandas`, `PyYAML`).

---

## Installation & Setup

1. **Clone the repo**  
   ```bash
   git clone https://github.com/ahmadabouelainein/Schwarm-Taskallokation.git
   cd Schwarm-Taskallokation
2. **Build and run docker container**
    ```bash
    docker compose up --build -d
    ```
3. To run the python scripts

    - To directly run a script (optimizer.py for example):
    ``` 
    docker compose run --rm swarm-explorer python optimizer.py
    ```
    - Alternatively, To attach a terminal to the docker container and access the workspace
    ```
    docker-compose exec swarm-explorer bash
    ```
    In the attached terminal you can run:
    ```-
    python model.py
    ``` 
    or:
    ```
    python optimizer.py
    ``` 