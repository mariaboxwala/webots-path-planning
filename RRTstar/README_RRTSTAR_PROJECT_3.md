# Project 3: Dynamic RRT* with Display and Obstacle Supervision in Webots

This project implements a complete **RRT\***-based motion planning system in Webots, integrating:
- Replanning every _n_ timesteps
- Static + dynamic obstacle modeling via a **Supervisor**
- Real-time graph visualization using the Webots **Display** device
- **APF-based local control** for navigating between waypoints

---

## Folder Layout

```
.
├── 1_World_Demo/               # Simulation results (videos, trace images)
├── 2_World_Extended_Demo/     # Larger version of demo with more obstacles
├── controllers/               # Robot and supervisor Python controllers
│   ├── RRT_Controller.py      # Implements RRT* with APF integration
│   ├── Robot_Supervisor.py    # For 2 robots
│   ├── Robot_Supervisor_Extended.py # For 4 robots
│   └── robot_controller.py    # Obstacle robot avoidance logic
├── libraries/
├── plugins/
├── protos/
├── worlds/                    # Webots world files (.wbt)
└── README.md
```

---

## Overview

This project extends the RRT* (Rapidly-exploring Random Tree Star) algorithm to a **dynamic world**. The robot continuously replans its path using updated occupancy data from a **supervising controller**, then executes the path using a local **Artificial Potential Field (APF)** controller.

### Key Features
- **Dynamic replanning**: RRT* is rerun every _n_ timesteps
- **Display supervisor**: Shows current graph on the Webots ground
- **Dynamic obstacle avoidance**: Mobile robots simulate obstacle motion
- **Heuristic biasing**: Gaussian-sampled node biasing near goal

---

## Algorithms & Components

### RRT* Path Planning
- Samples random points in configuration space
- Connects them into a tree based on obstacle clearance and cost
- Replans dynamically using an updated occupancy grid

### Heuristic RRT*
- Adds **goal biasing** via Gaussian sampling
- Adjustable probability via `BIAS_PROB`
- Accelerates convergence in cluttered environments

### Supervisor Control
- Controls dynamic robots and floor display
- Publishes robot positions to a JSON file
- Reads and displays current RRT* graph (nodes + edges)

### APF Local Control
- Uses APF to move robot from current position to RRT* waypoint
- Combines **attractive and repulsive forces**
- Handles transitions and collision avoidance in real time

---

## Demos

- `1_World_Demo/` – Standard RRT* world with 2 moving obstacles
- `2_World_Extended_Demo/` – Larger arena with 4 mobile obstacles
- Each demo includes:
  - 📸 PNG graphs of each RRT* update
  - 🎥 MP4/WebM recordings of robot behavior

---

## Future Work

- Use **multi-threading** to separate path planning and control
- Increase update rate by optimizing RRT* parameters
- Improve APF waypoint transitions for smoother trajectories

---

## References

- [RosettaCode: A* Algorithm](https://rosettacode.org/wiki/A*_search_algorithm)
- Webots Documentation: https://cyberbotics.com/doc/

---

*Developed by Maria Boxwala and team — CMP494 Spring 2024.*
