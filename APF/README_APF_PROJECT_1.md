# Project 1: Artificial Potential Field (APF) Navigation in Webots

This project explores three progressive implementations of the Artificial Potential Field (APF) navigation strategy using Webots. Each subfolder corresponds to one of three tasks, building from basic goal-seeking to obstacle avoidance and finally to navigating highly cluttered environments.

---

## 📁 Folder Structure

```
Project_1_APF/
├── Q1_Static_Goal/           # Basic APF with goal attraction only
├── Q2_Obstacle_Avoidance/   # APF with walls, barrels, speed vs safety tuning
├── Q3_Crowded_Warehouse/    # APF adapted to avoid local minima in cluttered scenes
└── README.md
```

---

## 🚗 Q1: Static Goal Navigation

This scenario implements a **pure attractive force APF**, where the robot navigates to a fixed goal using GPS and IMU data. Features include:

- Euclidean distance for attraction
- Dead zone for minor angular errors
- Drop-off function to reduce speed near goal
- Avoids oscillation near 180° yaw offset

**Start points tested:** Corners and random positions  
**Result:** Robot successfully reaches goal from any orientation

---

## 🧱 Q2: Obstacle Avoidance with Walls & Barrels

This task extends Q1 by adding **repulsive forces** from obstacles (walls, barrels) using 8 proximity sensors.

### Highlights:
- **k_orientation** and **k_distance** scale speed and turning
- Stops robot when inside a defined goal threshold radius
- Orientation deadzone avoids jitter
- Tunable for **maximum speed** or **maximum safety**
- Analysis of stuck scenarios and recovery strategies

**Files include:**  
- APF controller for speed tuning  
- APF controller for safety tuning  

---

## 🏭 Q3: Crowded Warehouse Navigation

This model tackles **local minima issues** in complex obstacle fields by **restricting repulsive force directions**.

### Adjustments:
- Only sensors ps0 and ps7 (front sides) contribute repulsion
- Other sensors’ directions set to 0 to reduce y-axis interference
- Improved stability in navigating narrow corridors

**Start points tested:** All four corners and adjacent to barrels  
**Result:** Robot successfully navigates through barrels that caused failure in earlier versions

---

## 🛠 How to Run

1. Open Webots
2. Navigate to one of the scenario folders (`Q1_Static_Goal/`, etc.)
3. Open the `.wbt` world file
4. Run the simulation

> Adjustments to constants (`k_orientation`, `k_distance`, etc.) can be made in the controller script to observe performance changes.

---

## 📚 References

- Webots Documentation: https://www.cyberbotics.com/doc/guide/epuck
- Course: CMP 494 – Intelligent Autonomous Robotics

---

*Developed by Maria Boxwala and team — CMP494 Spring 2024.*
