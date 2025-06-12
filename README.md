# CMP494 Webots Robotics Projects

This repository contains a collection of robotics simulations and motion planning algorithms implemented using Webots, as part of CMP494 – Intelligent Autonomous Robotics, Spring 2024.

Each project demonstrates a key robotics concept, from reactive motion and path planning to FSM-based task automation and dynamic RRT* graph visualization.

---

## Project Structure

### 🔹 Project 1: Artificial Potential Fields (APF)
Path planning and motion control using attractive and repulsive forces.

- **P1:** Navigate to a static goal using APF
- **P2:** Add walls/barrels as obstacles, tune for speed vs safety
- **P3:** APF in a crowded warehouse, handles local minima

📄 See: [`README_APF_PROJECT_1.md`](./APF/README_APF_PROJECT_1.md)

---

### 🔹 Project 2: Quadtree + A* + Hybrid Navigation
Hierarchical map representation with global path planning and local control.

- **P1:** Quadtree map breakdown + A* waypoint graph
- **P2:** Reactive APF controller
- **P3:** A* waypoints + APF controller for hybrid navigation

📄 See: [`README_ASTAR_PROJECT_2.md`](./Astar/README_ASTAR_PROJECT_2.md)

---

### 🔹 Project 3: Dynamic RRT* with Display & Obstacles
An integrated RRT* planner with dynamic re-planning, graphical floor display, and real-time collision avoidance.

- **Dynamic RRT*** with APF control
- **Supervisor-controlled display and obstacle robots**
- **Heuristic biasing** for faster convergence

📄 See: [`README_RRTSTAR_PROJECT_3.md`](./RRTstar/README_RRTSTAR_PROJECT_3.md)

---

### 🔹 Project 4: FSM-Based Can Collection (HW3 Q3)
Finite State Machine (FSM) design for a multi-stage task.

- **Wanders**, detects and touches cans
- Transitions to **base-seeking mode**
- Terminates upon reaching base

📄 See: [`README_FSM_PROJECT_4.md`](./FSM/README_FSM_PROJECT_4.md)

---

*Authored by Maria Boxwala and team | Spring 2024 – American University of Sharjah*

