# FSM-Based Can Collection Task in Webots

This project implements a Finite State Machine (FSM) controller in Webots to simulate a robot that:
- **Detects and collects cans**
- **Returns to a base station**
- Transitions between behaviors based on **sensor input and task completion**

---

## Project Overview

A robot is equipped with proximity and camera sensors to explore an arena and locate multiple "can" objects. Once all cans are collected (touched), the robot transitions to seeking a fixed base station location. Upon reaching the base, the robot terminates its operation.

---

## Finite State Machine (FSM)

The robot's behavior is governed by a clear FSM with the following states:

| Current State        | Stimulus                | Next State              |
|----------------------|-------------------------|--------------------------|
| `Wander`             | Can detected by camera  | `Move forward`          |
| `Wander`             | Obstacle touched        | `Wander`                |
| `Wander`             | All cans found          | `Wander for Base`       |
| `Move forward`       | Obstacle touched        | `Wander`                |
| `Wander for Base`    | Obstacle touched        | `Wander for Base`       |
| `Wander for Base`    | Base detected           | `Move forward to Base`  |
| `Move forward to Base` | Obstacle touched       | `Stop` (Final State)    |

- **Start State**: `Wander`
- **End State**: `Stop`

---

## Implementation Highlights

- Uses a **Boolean array** to track which cans have been touched
- Implements **back-off + rotate behavior** on obstacle collision
- Employs **vision-based detection** to transition to "can detected" and "base detected" states
- Final transition to `Stop` occurs when base is touched after all cans have been collected

---

## Folder Layout

```
.
├── controllers/               # Contains FSM controller script
├── libraries/                 # (Optional) Any reused utility code
├── plugins/                   # Webots plugins if used
├── protos/                    # PROTO files for cans/base
├── worlds/                    # Webots world file with can/base objects
└── README_FSM_PROJECT_4.md
```

---

## ✅ Test Scenarios

Three tests were run with varying robot start orientations and obstacle layouts. All tests completed successfully with the robot:
- Identifying and touching all cans
- Transitioning to base-seeking behavior
- Reaching and stopping at the base

---

*Built by Maria Boxwala and team — CMP494 Spring 2024.*
