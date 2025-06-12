# Project 2: Quadtree-Aided A* Planning and Hybrid APF Navigation in Webots

This project implements a hierarchical path-planning and reactive control system in Webots using:
- **Quadtree spatial decomposition**
- **A* search algorithm**
- **Artificial Potential Field (APF) controller**
- A final **hybrid controller** that fuses A* and APF to balance global planning and local reactivity

---

## 📁 Folder Structure

```
Project_2_Quadtree_AStar_APF/
├── Q1_Quadtree_AStar/        # Quadtree spatial breakdown and A* planner
├── Q2_APF_Waypoint_Navigation/ # Reactive controller using APF
├── Q3_Hybrid_Controller/     # Fusion of A* planner with APF waypoint controller
└── README.md
```

---

## 🧠 Q1: Quadtree Decomposition + A* Path Planning

- Decomposes a 2D Webots world into **quadtree levels** to efficiently represent free vs. obstructed space
- Waypoints are optimized by **merging adjacent, obstacle-free regions**
- A* search is run over this reduced waypoint graph
- Heuristic: Euclidean or **Chebyshev** (for safety-aware planning)
- Outputs a sequence of waypoints from **Start → Goal**
- 5 test scenarios verify different path selections

> ✅ Result: Efficient and safe path planning using hierarchical map abstraction.

---

## ⚙️ Q2: APF-Based Reactive Navigation

Implements a classic **Artificial Potential Field (APF)** strategy:
- **Attractive force** to goal
- **Repulsive force** from obstacles (via front and side sensors)
- Dynamic adjustment of orientation and speed based on force magnitude
- Uses IMU + GPS + 8 proximity sensors

> ✅ Result: Robot reliably navigates to goals while avoiding static and dynamic obstacles in real time.

---

## 🔀 Q3: Hybrid A* + APF Controller

This controller fuses:
1. **Deliberative path planning** using A* waypoints (from Q1)
2. **Local reactive control** via APF (from Q2)

- A* path is used to set **waypoints**
- APF controller guides the robot to each waypoint
- Transitions from one waypoint to the next upon arrival
- Handles dynamic changes in obstacle layout during runtime

### Advanced Variant:
- Adds **danger values** to each waypoint
- A* modified to incorporate average danger into move cost
- Uses **Chebyshev distance** for flexible and safe routing

> ✅ Result: Robot demonstrates smoother, safer, and more reliable navigation in dynamic, cluttered environments.

---

## ▶️ How to Run

1. Open Webots
2. Load any of the `Q1_`, `Q2_`, or `Q3_` world folders
3. Run the simulation from the controller script
4. Optionally plot or print path from terminal

---

## 🔍 References

- Rosetta Code A* Template: https://rosettacode.org/wiki/A*_search_algorithm
- Webots Documentation: https://www.cyberbotics.com/doc/

---

*Developed by Maria Boxwala and team — CMP494 Spring 2024.*
