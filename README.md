# Multi-Arm UR10e Pick-and-Place
## ROS 2 Jazzy + MoveIt 2 + NVIDIA Isaac Sim (Omniverse) + Physical AI

A production-grade ROS 2, MoveIt 2, and NVIDIA Isaac Sim project featuring three independent UR10e manipulator arms equipped with Robotiq 2F-140 grippers performing an autonomous, coordinated sequential pick-and-place relay task across four physical stations (Station A → B → C → D).

---

## 📚 Documentation & Technical Guides
All in-depth technical guides, architecture documentation, and testing procedures are located in [`src/docs/`](src/docs/):

1. **[Comprehensive Project Study & Engineering Guide](src/docs/multi_arm_project_study_guide.md):** Complete end-to-end breakdown of how this project was built, packages and their roles, 8 key challenges & resolutions, phase progress, and pending items.
2. **[Hybrid Architecture Implementation Plan](src/docs/multi_arm_ur10e_implementation_plan.md):** Detailed multi-phase roadmap spanning baseline bringup to NVIDIA Cosmos foundation models.
3. **[Isaac Sim Multi-Arm Simulation Guide](src/docs/isaac_sim_guide.md):** Complete Isaac Sim 4.x/5.x/6.x setup, coordinate maps, gripper calibration, automated workcell spawning, and ROS 2 Jazzy bridge integration.
4. **[System Build & Architecture Guide](src/docs/system_build_and_architecture_guide.md):** Deep-dive explaining how each package, description, MoveIt 2 node, and Isaac Sim bridge was built.
5. **[Robot + Gripper Configuration Guide](src/docs/robot_gripper_config_guide.md):** Creating unified Xacro models, calibrated TCP links, MoveIt 2 Setup Assistant workflows, and multi-robot namespacing.
6. **[Isaac Sim Installation Guide](src/docs/isaac_sim_installation_guide.md):** Installing Isaac Sim on Ubuntu, NVIDIA drivers, Vulkan, FastDDS middleware, and ROS 2 bridge activation.
7. **[Step-by-Step Testing & Verification Guide](src/docs/testing_guide.md):** Modular testing instructions to verify kinematics, gripper states, action servers, and full relay execution.

---

## 📊 Implementation Status Summary

| Phase | Milestone / Title | Status | Key Deliverables & Artifacts |
| :---: | :--- | :---: | :--- |
| **P1** | **Single UR10e + MoveIt 2 Baseline Bringup** | `COMPLETED` | Clean ROS 2 Jazzy build, `ur_description`, `ur_moveit_config` verification |
| **P2** | **Composite Gripper (2F-140) + Action Server** | `COMPLETED` | `ur10e_robotiq.urdf.xacro`, `PickPlace.action`, `pick_place_server.py` |
| **P3** | **3× UR10e in Isaac Sim (Digital Twin)** | `COMPLETED` | `spawn_multi_ur10e.py`, `robot{1..3}_moveit_config`, OmniGraph `/clock` & bridges |
| **P4** | **Autonomous Relay Orchestration (A → B → C → D)** | `COMPLETED` | `task_manager.py` state machine, dynamic `/workpiece_marker`, S-curve solver |
| **P5** | **Synthetic Vision & GPU Object Detection** | `PENDING` | RTX Synthetic Cameras, `isaac_ros_yolov8`, dynamic 6D pose estimators |
| **P6** | **Multi-Arm Concurrency & Spatial Mutex** | `PENDING` | MoveIt `PlanningSceneWorld`, collision mutex for buffer stations B & C |
| **P7** | **Physical AI & NVIDIA Cosmos World Models** | `PENDING` | Omniverse Replicator domain randomization, Cosmos world model validation |

---

## 🛠️ Workspace Packages

- `src/multi_arm_description/`: Unified URDF/Xacro models (`multi_ur10e_workcell.urdf.xacro`, `ur10e_robotiq.urdf.xacro`) combining UR10e and Robotiq 2F-140 with calibrated TCP links and industrial workcell staging.
- `src/robot1_moveit_config/`, `robot2...`, `robot3...`: Independent MoveIt 2 configurations per robot with isolated namespaces (`/robot1`, `/robot2`, `/robot3`).
- `src/multi_arm_interfaces/`: Custom ROS 2 action interfaces (`PickPlace.action`).
- `src/multi_arm_control/`: Python nodes for `pick_place_server` and sequential state machine `task_manager`.
- `src/multi_arm_bringup/`: Centralized launch files (`multi_arm_simulation.launch.py`, `stations.yaml`).
- `src/ur_simulation/`: Procedural Isaac Sim 4.x/5.x/6.x Python workcell spawner and OmniGraph ActionGraph setup scripts (`spawn_multi_ur10e.py`).
- `src/docs/`: Comprehensive technical documentation.

---

## 📍 Workcell Coordinate Reference

| Component | Entity / Prim | Coordinates $(X, Y, Z)$ | Description |
| :--- | :--- | :--- | :--- |
| **Robot 1** | `/World/robot1` | $(0.00, 0.00, 0.20)$ | Base on 0.20m Pedestal (Cell 1) |
| **Robot 2** | `/World/robot2` | $(0.00, 1.60, 0.20)$ | Base on 0.20m Pedestal (Cell 2) |
| **Robot 3** | `/World/robot3` | $(0.00, 3.20, 0.20)$ | Base on 0.20m Pedestal (Cell 3) |
| **Station A** | `station_a_table` | $(0.70, 0.00, 0.20)$ | Source Table (Blue) |
| **Station B** | `station_b_table` | $(0.70, 0.80, 0.20)$ | Relay 1 Buffer Table (Orange) |
| **Station C** | `station_c_table` | $(0.70, 2.40, 0.20)$ | Relay 2 Buffer Table (Orange) |
| **Station D** | `station_d_table` | $(0.70, 3.20, 0.20)$ | Destination Table (Purple) |
| **Workpiece** | `Workpiece_Cube` | $(0.70, 0.00, 0.245)$ | $60\text{mm}$ Dynamic Cube ($0.15\text{kg}$) |

---

## 🚀 Quick Start Guide

### 1. Build the Workspace (ROS 2 Jazzy)
```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 2. Standalone RViz Workcell Simulation
```bash
# Terminal 1: Launch 3-Robot Workcell & Action Servers
ros2 launch multi_arm_bringup multi_arm_simulation.launch.py use_sim_time:=false

# Terminal 2: Run the Sequential Relay Task Manager (A -> B -> C -> D)
ros2 run multi_arm_control task_manager
```

### 3. NVIDIA Isaac Sim Physics Simulation
1. Export standalone URDF:
   ```bash
   xacro src/multi_arm_description/urdf/ur10e_robotiq.urdf.xacro > /tmp/ur10e_robotiq.urdf
   ```
2. In **Isaac Sim**, import `/tmp/ur10e_robotiq.urdf` via **Isaac Utils → Workflows → URDF Importer** to `/World/UR10e` (Check *Fix Base Link*).
3. Open **Window → Script Editor**, load `src/ur_simulation/scripts/spawn_multi_ur10e.py`, and click **Run**.
4. Press **PLAY (▶)** in Isaac Sim.
5. In your ROS 2 terminal:
   ```bash
   # Terminal 1: Launch Multi-Robot Stack with Sim Clock
   ros2 launch multi_arm_bringup multi_arm_simulation.launch.py use_sim_time:=true

   # Terminal 2: Execute Pick-and-Place State Machine
   ros2 run multi_arm_control task_manager --ros-args -p use_sim_time:=true
   ```

### 4. Gazebo Harmonic Simulation (ROS 2 Jazzy Native)
```bash
# Terminal 1: Launch 3-Robot Workcell in Gazebo Harmonic + MoveIt 2 + Action Servers
ros2 launch multi_arm_bringup multi_arm_gazebo.launch.py

# Terminal 2: Execute Pick-and-Place State Machine
ros2 run multi_arm_control task_manager --ros-args -p use_sim_time:=true
```
