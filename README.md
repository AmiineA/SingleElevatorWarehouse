# Elevator Control in an Automated Storage and Retrieval System (AS/RS)

This repository contains a simulation and control study of a **single-elevator Automated Storage and Retrieval System (AS/RS)**, developed as part of the **Operations Research curriculum at École polytechnique**.

The project was supervised by **Prof. Frédéric Meunier** and focuses on comparing several elevator control policies with respect to **average waiting time**.

---

## Project Overview

We consider a warehouse with:
- A single elevator serving `n` floors,
- Unit-capacity transport (one item at a time),
- Stochastic arrivals at each floor (Poisson processes),
- Random destination floors.

Each floor maintains a FIFO queue of pending transport requests.  
The objective is to design and compare **control policies** that decide:
- which floor to serve next when the elevator is idle,
- how to move when the system is empty (passive vs active idling),

with the goal of minimizing waiting time and system time.

---

## Implemented Control Policies

The following controllers are implemented and compared:

- **Naive controller**  
  Always serves the nearest non-empty floor.

- **Global FIFO controller**  
  Treats the system as a single global FIFO queue and always serves the oldest request.

- **Snapshot Dynamic Programming (DP)**  
  At each decision time, solves a deterministic dynamic program on the current system snapshot and executes the first optimal action.

- **Offline Dynamic Programming (tabulated DP)**  
  Uses a truncated state representation and memoization to approximate an optimal policy offline, reused across simulation runs.

---

## Repository Structure

```

├── simulation_EA.py            # Main simulation script
├── user_input.py               # Configuration and parameter handling
├── resultat.py                 # Result aggregation and statistics, computed by hand
├── plotting.py                 # Plotting utilities
├── controllers/                # Elevator control policies
│   ├── __init__.py
│   ├── naif_controller.py
│   ├── global_fifo_controller.py
│   ├── dp_snapshot_controller.py
│   ├── dp_offline_controller.py
│   └── utils.py
├── Simpy_basic_explanation.md  # Introductory SimPy explanation
└── README.md
```

---

## How to Run

To launch a simulation, please try running:

```bash
python simulation_EA.py
```

The script runs a full simulation and outputs:
- performance statistics (waiting time, system time),
- convergence traces,
- plots generated using Matplotlib.

You can also read Simpy_basic_explanation.md for an intuitive explanation of the Simpy functions used.
`plotting.py` contains different plotting functions that were used to compare different policies.

---

## Required Libraries

- Python >= 3.9
- simpy
- numpy
- matplotlib
- random
- math

Install dependencies with:

```bash
pip install simpy numpy matplotlib random math
```

---

## Notes on Code Style

This project was developed in an academic setting and under time constraints.

Many files mix French and English — apologies for the inconsistency.

---

## License

This repository is provided for academic and educational purposes.