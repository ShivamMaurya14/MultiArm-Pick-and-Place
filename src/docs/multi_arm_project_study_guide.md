# Multi-Arm UR10e Pick-and-Place: Comprehensive Project Study & Engineering Guide
## ROS 2 Jazzy Jalisco + MoveIt 2 + NVIDIA Isaac Sim (v4.x, v5.x, v6.0+)

---

## 1. Executive Summary & System Overview

This project implements an industrial **3× UR10e manipulator robotic workcell** equipped with **Robotiq 2F-140 parallel-jaw adaptive grippers** executing an autonomous, synchronized sequential pick-and-place relay task across four physical workstations:

$$\text{Station A (Source)} \xrightarrow{\text{Robot 1}} \text{Station B (Relay 1)} \xrightarrow{\text{Robot 2}} \text{Station C (Relay 2)} \xrightarrow{\text{Robot 3}} \text{Station D (Destination)}$$

The software architecture is designed on a **Hybrid Foundation**:
* **ROS 2 Jazzy Jalisco (Ubuntu 24.04 LTS) + MoveIt 2:** Handles deterministic kinematics, trajectory generation, S-curve joint interpolation, action server state transitions, and high-level state machine orchestration.
* **NVIDIA Isaac Sim (Omniverse):** Provides photorealistic GPU ray-tracing, PhysX 5 dynamic contact physics, friction-based grasping mechanics, and OmniGraph ROS 2 bridge communication (`/clock`, joint states, and joint commands).
* **RViz2:** Real-time kinematic and dynamic workpiece visualizer with live joint state and interactive marker tracking.

---

### 📊 Implementation Status Summary

| Phase | Milestone / Title | Status | Key Deliverables & Artifacts |
| :---: | :--- | :---: | :--- |
| **P1** | **Single UR10e + MoveIt 2 Baseline Bringup** | `COMPLETED` | Clean ROS 2 Jazzy build, `ur_description`, `ur_moveit_config` verification |
| **P2** | **Composite Gripper (2F-140) + Action Server** | `COMPLETED` | `ur10e_robotiq.urdf.xacro`, `PickPlace.action`, `pick_place_server.py` |
| **P3** | **3× UR10e in Isaac Sim (Digital Twin)** | `COMPLETED` | `spawn_multi_ur10e.py`, `robot{1..3}_moveit_config`, OmniGraph `/clock` & bridges |
| **P4** | **Autonomous Relay Orchestration (A → B → C → D)** | `COMPLETED` | `task_manager.py` state machine, dynamic `/workpiece_marker`, S-curve solver |
| **P5** | **Synthetic Vision & GPU Object Detection** | `PENDING` | RTX Synthetic Cameras, `isaac_ros_yolov8`, dynamic 6D pose estimators |
| **P6** | **Multi-Arm Concurrency & Spatial Mutex** | `PENDING` | MoveIt `PlanningSceneWorld`, collision mutex for buffer stations B & C |
| **P7** | **Physical AI & NVIDIA Cosmos World Models** | `PENDING` | Omniverse Replicator domain randomization, Cosmos world model validation |

```mermaid
graph TD
    subgraph IsaacSim ["NVIDIA Isaac Sim 4.x / 5.x / 6.0+ (Omniverse)"]
        Scene["Industrial Workcell USD (Floor, Pedestals, 4 Tables)"]
        Physics["PhysX 5 Dynamics (Contact, Gravity, Friction μ=1.2)"]
        CubeSim["Dynamic Workpiece Cube (0.06m x 0.06m x 0.05m, 0.15kg)"]
        SimClock["OmniGraph /clock Publisher"]
        OG_Bridge["OmniGraph ROS 2 Bridges (/robot{1..3}/joint_states & joint_commands)"]
    end

    subgraph ROS2_Stack ["ROS 2 Jazzy Control Stack"]
        TM["task_manager Node (A -> B -> C -> D Orchestrator)"]
        PPS1["/robot1/pick_place (Action Server)"]
        PPS2["/robot2/pick_place (Action Server)"]
        PPS3["/robot3/pick_place (Action Server)"]
        RSP["workcell_state_publisher (Unified TF Broadcaster)"]
    end

    subgraph Visualization ["Visualization Layer"]
        RViz["RViz2 (view_workcell.rviz)"]
        MarkerTopic["/workpiece_marker (Dynamic Workpiece Tracking)"]
    end

    SimClock -->|/clock| ROS2_Stack
    OG_Bridge -->|/robot{1..3}/joint_states| ROS2_Stack
    PPS1 & PPS2 & PPS3 -->|joint_commands| OG_Bridge
    
    TM -->|Goal: Pick A -> Place B| PPS1
    TM -->|Goal: Pick B -> Place C| PPS2
    TM -->|Goal: Pick C -> Place D| PPS3

    PPS1 & PPS2 & PPS3 -->|/joint_states| RSP
    RSP -->|/tf & /robot_description| RViz
    PPS1 & PPS2 & PPS3 -->|/workpiece_marker| RViz
```

---

## 2. Workspace Package Breakdown & Roles

The project is structured into modular, decoupled ROS 2 packages:

| Package | Language | Key Files / Resources | Technical Role & Description |
| :--- | :--- | :--- | :--- |
| **`multi_arm_description`** | Xacro / XML | `multi_ur10e_workcell.urdf.xacro`, `ur10e_robotiq.urdf.xacro`, `view_workcell.rviz` | Master robot workcell descriptions, unified mesh definitions, pedestal stands, table geometries, safety lines, and calibrated TCP frames. |
| **`multi_arm_interfaces`** | ROS 2 IDL | `action/PickPlace.action` | Custom ROS 2 action interfaces defining Goal (`action`, `station_name`, `grasp_width`), Feedback (`current_phase`, `progress_percent`), and Result (`success`, `message`, `execution_time`). |
| **`multi_arm_control`** | Python 3 | `pick_place_server.py`, `task_manager.py` | Core control nodes: Action Servers with analytical IK, S-curve trajectory generators, Robotiq 2F-140 finger mimic controllers, dynamic marker broadcasters, and the top-level relay state machine. |
| **`multi_arm_bringup`** | Python / YAML | `multi_arm_simulation.launch.py`, `stations.yaml` | Master launch orchestration uniting state publishers, namespaced MoveIt instances, action servers, and RViz2. |
| **`robot1_moveit_config`** | MoveIt 2 / SRDF | `robot1.srdf.xacro`, `ompl_planning.yaml`, `kinematics.yaml` | Isolated MoveIt 2 configuration and collision matrices for Robot 1 (`/robot1`). |
| **`robot2_moveit_config`** | MoveIt 2 / SRDF | `robot2.srdf.xacro`, `ompl_planning.yaml`, `kinematics.yaml` | Isolated MoveIt 2 configuration and collision matrices for Robot 2 (`/robot2`). |
| **`robot3_moveit_config`** | MoveIt 2 / SRDF | `robot3.srdf.xacro`, `ompl_planning.yaml`, `kinematics.yaml` | Isolated MoveIt 2 configuration and collision matrices for Robot 3 (`/robot3`). |
| **`ur_simulation`** | Python (Omniverse) | `scripts/spawn_multi_ur10e.py` | Procedural USD scene generator and OmniGraph ActionGraph ROS 2 bridge config for Isaac Sim 4.x/5.x/6.0+. |
| **`ur_description`** | Xacro / Meshes | `ur_macro.xacro`, visual & collision STL/DAE | Official Universal Robots kinematics, inertial models, and 3D visual/collision meshes for the UR10e arm. |
| **`robotiq_description`**| Xacro / Meshes | `robotiq_2f_140_macro.urdf.xacro`, `robotiq_2f_140.xacro` | Kinematic definition and meshes for the Robotiq 2F-140 adaptive gripper and mimic joint linkages. |

---

## 3. Workcell Coordinate Reference & Kinematic Calibration

### Spatial Coordinates Map
All coordinates are defined in meters in the unified global `world` frame:

```text
  [Station A: Source]         [Station B: Relay 1]        [Station C: Relay 2]        [Station D: Dropoff]
  X=0.70, Y=0.00, Z=0.25      X=0.70, Y=0.80, Z=0.25      X=0.70, Y=2.40, Z=0.25      X=0.70, Y=3.20, Z=0.25
        ▲                           ▲                           ▲                           ▲
        │                           │                           │                           │
  [Robot 1: Y=0.0, Z=0.20] ───────► [Robot 2: Y=1.6, Z=0.20] ───────► [Robot 3: Y=3.2, Z=0.20]
```

| Entity | Position $(X, Y, Z)$ | Target Tool Vector | Analytical Joint Solution $[q_1, q_2, q_3, q_4, q_5, q_6]$ |
| :--- | :--- | :--- | :--- |
| **Robot 1 Base** | $(0.00, 0.00, 0.20)$ | Upright Ready | `[-0.2514, -1.8000, 1.5000, -1.2700, -1.5708, 0.0000]` |
| **Robot 1 @ Station A (Pick)** | $(0.70, 0.00, 0.25)$ | $[0, 0, -1]$ | `[-0.2514, -1.3517, 2.0841, -2.3033, -1.5708, 0.0000]` |
| **Robot 1 @ Station B (Place)**| $(0.70, 0.80, 0.25)$ | $[0, 0, -1]$ | `[0.6874, -0.8380, 1.2705, -2.0033, -1.5708, 0.0000]` |
| **Robot 2 Base** | $(0.00, 1.60, 0.20)$ | Upright Ready | `[-0.2514, -1.8000, 1.5000, -1.2700, -1.5708, 0.0000]` |
| **Robot 2 @ Station B (Pick)** | $(0.70, 0.80, 0.25)$ | $[0, 0, -1]$ | `[-1.0165, -0.8380, 1.2705, -2.0033, -1.5708, 0.0000]` |
| **Robot 2 @ Station C (Place)**| $(0.70, 2.40, 0.25)$ | $[0, 0, -1]$ | `[0.6874, -0.8380, 1.2705, -2.0033, -1.5708, 0.0000]` |
| **Robot 3 Base** | $(0.00, 3.20, 0.20)$ | Upright Ready | `[-0.2514, -1.8000, 1.5000, -1.2700, -1.5708, 0.0000]` |
| **Robot 3 @ Station C (Pick)** | $(0.70, 2.40, 0.25)$ | $[0, 0, -1]$ | `[-1.0165, -0.8380, 1.2705, -2.0033, -1.5708, 0.0000]` |
| **Robot 3 @ Station D (Place)**| $(0.70, 3.20, 0.25)$ | $[0, 0, -1]$ | `[-0.2514, -1.3517, 2.0841, -2.3033, -1.5708, 0.0000]` |

### Robotiq 2F-140 Adaptive Gripper Calibration
* **Stroke:** $0\text{ to }140\text{ mm}$ ($0.0\text{ to }0.140\text{ m}$).
* **Joint Limits:** $0.0\text{ rad}$ (open, $140\text{mm}$) to $0.70\text{ rad}$ (fully closed, $0\text{mm}$).
* **Linear Grasp Angle Formulation for Cube Width $W$:**
  $$\theta_{\text{grasp}}(W) = 0.70 \times \left(1.0 - \frac{W}{0.140}\right)$$
  For the $60\text{mm}$ ($0.060\text{m}$) workpiece cube:
  $$\theta_{\text{grasp}}(0.060) = 0.70 \times \left(1.0 - \frac{0.060}{0.140}\right) = \mathbf{0.400\text{ rad}}$$
* **Active & Mimic Joints Broadcaster:**
  $$\text{Joints} = [\theta, -\theta, -\theta, -\theta, \theta, \theta]$$
  covering `finger_joint`, `right_outer_knuckle_joint`, `left_inner_knuckle_joint`, `right_inner_knuckle_joint`, `left_inner_finger_joint`, `right_inner_finger_joint`.

---

## 4. Engineering Challenges Encountered & Solutions Implemented

```
+----------------------------------------------------------------------------------------------------+
|                                KEY CHALLENGES & RESOLUTIONS TABLE                                  |
+----+------------------------------------+----------------------------------------------------------+
| #  | Problem / Symptom                  | Root Cause & Exact Fix Implemented                       |
+----+------------------------------------+----------------------------------------------------------+
| 1  | Hardware Driver Build Failures     | Container lacked physical serial/CAN hardware drivers.   |
|    |                                    | Fix: Added COLCON_IGNORE to physical driver packages     |
|    |                                    | (Universal_Robots_ROS2_Driver, robotiq_driver) to build  |
|    |                                    | pure simulation & control stack with zero errors.        |
+----+------------------------------------+----------------------------------------------------------+
| 2  | TF Tree Collisions in RViz         | Multiple robot_state_publisher nodes competing on /tf.   |
|    |                                    | Fix: Launched single unified workcell_state_publisher    |
|    |                                    | with launch_rsp:=false in sub-robot MoveIt launches.     |
+----+------------------------------------+----------------------------------------------------------+
| 3  | 180-Degree Wraparound Spins        | UR10e base_link_inertia introduces a pi rotation offset. |
|    |                                    | Fix: Calibrated pan angles (-0.2514 rad) and added       |
|    |                                    | modular shortest angular distance interpolation:         |
|    |                                    | diff = (target - start + pi) % (2*pi) - pi.              |
+----+------------------------------------+----------------------------------------------------------+
| 4  | Gripper Fingers Frozen Closed      | Incorrect 85mm joint names and missing S-curve easing.   |
|    |                                    | Fix: Replaced with real 2F-140 joint names + mimic array |
|    |                                    | and linear grasp formulation (0.400 rad for 60mm cube).  |
+----+------------------------------------+----------------------------------------------------------+
| 5  | Workpiece Cube Missing in RViz     | URDF static link was frozen; RViz display had QoS        |
|    |                                    | mismatch (Transient Local vs Volatile publisher).        |
|    |                                    | Fix: Converted to dynamic /workpiece_marker with         |
|    |                                    | Volatile QoS and robot-to-robot ownership handoffs.      |
+----+------------------------------------+----------------------------------------------------------+
| 6  | RViz ViewController Crash Popup    | Legacy ROS 1 syntax "Value: Orbit (rviz)" in YAML config.|
|    |                                    | Fix: Updated view_workcell.rviz to ROS 2 Jazzy Orbit     |
|    |                                    | view controller specification without legacy keys.       |
+----+------------------------------------+----------------------------------------------------------+
| 7  | Task Manager Waiting Indefinitely  | ActionServer was nested inside _handoff_callback.        |
|    |                                    | Fix: Moved ActionServer creation into __init__ method.   |
+----+------------------------------------+----------------------------------------------------------+
| 8  | Xacro Undefined Argument Error     | Root tag <robot name="$(arg name)"> failed without args. |
|    |                                    | Fix: Changed root tag to <robot name="ur10e_robotiq">.   |
+----+------------------------------------+----------------------------------------------------------+
```

---

## 5. Implementation Roadmap Status (Progress vs. Pending)

As defined in the project master implementation plan ([`multi_arm_ur10e_implementation_plan.md`](file:///Users/shivammaurya/Desktop/ros2_ws/nextup/multi_arm_ws/MultiArm-Pick-and-Place/src/docs/multi_arm_ur10e_implementation_plan.md)):

### ✅ COMPLETED PHASES

* [x] **Phase 1 — Baseline Single UR10e + MoveIt 2 Bringup:**
  * Clean workspace build on ROS 2 Jazzy Jalisco.
  * Verified 6-DOF UR10e kinematics, OMPL planning pipelines, and interactive marker execution.
* [x] **Phase 2 — Assembled Gripper & Pick/Place Server:**
  * Combined `ur_macro.xacro` with `robotiq_2f_140_macro.urdf.xacro` with calibrated TCP frame (`0.23m` Z-offset).
  * Implemented `PickPlace.action` custom interface and analytical S-curve action server with approach/reach/grasp/retreat phases.
* [x] **Phase 3 — Three UR10e Workcell in Isaac Sim (Omniverse):**
  * Automated procedural spawner `spawn_multi_ur10e.py` supporting Isaac Sim 4.x/5.x/6.0+.
  * Configured independent MoveIt 2 packages (`robot1_moveit_config`, `robot2_moveit_config`, `robot3_moveit_config`).
  * OmniGraph ROS 2 bridge action graphs for `/clock`, `/{namespace}/joint_states`, and `/{namespace}/joint_commands`.
* [x] **Phase 4 — A → B → C → D Sequential Multi-Arm Relay:**
  * Implemented `task_manager.py` state machine executing the 6-step handoff ($A \rightarrow B \rightarrow C \rightarrow D$).
  * Dynamic workpiece visual tracking in RViz and high-friction contact physics in Isaac Sim.

---

### ⏳ PENDING / FUTURE IMPLEMENTATION PHASES

* [ ] **Phase 5 — Synthetic Vision & GPU Perception (Isaac ROS):**
  * Add overhead/wrist RTX synthetic cameras in Isaac Sim publishing RGB-D point clouds.
  * Integrate `isaac_ros_yolov8` or TensorRT 6D pose estimators to dynamically detect workpiece coordinates instead of static YAML lookups.
* [ ] **Phase 6 — Multi-Arm Concurrency & Spatial Mutex Interlocks:**
  * Implement shared `PlanningSceneWorld` in MoveIt 2 for dynamic collision avoidance between moving robot arms.
  * Add spatial mutex volume reservation around intermediate buffer tables (Stations B and C) allowing multiple workpieces to traverse the workcell concurrently.
* [ ] **Phase 7 — Physical AI & NVIDIA Cosmos World Foundation Models:**
  * Domain randomization with Omniverse Replicator (lighting, textures, camera noise).
  * Video/physics edge-case validation using NVIDIA Cosmos foundation models.
  * GPU parallel reinforcement learning policy training via Isaac Lab.

---

## 6. Complete Execution Cheatsheet

### 1. Build Workspace
```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 2. Run in Standalone RViz Mode
```bash
# Terminal 1: Launch 3-Robot Workcell & Action Servers
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch multi_arm_bringup multi_arm_simulation.launch.py use_sim_time:=false

# Terminal 2: Run Relay State Machine
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run multi_arm_control task_manager
```

### 3. Run in NVIDIA Isaac Sim Mode
```bash
# Step A: Export URDF (in ROS 2 terminal)
source /opt/ros/jazzy/setup.bash
source install/setup.bash
xacro src/multi_arm_description/urdf/ur10e_robotiq.urdf.xacro > /tmp/ur10e_robotiq.urdf

# Step B: In Isaac Sim GUI
# 1. Isaac Utils -> Workflows -> URDF Importer -> Import /tmp/ur10e_robotiq.urdf to /World/UR10e (Fix Base Link: Checked)
# 2. Window -> Script Editor -> Open src/ur_simulation/scripts/spawn_multi_ur10e.py -> Click Run
# 3. Press PLAY (▶) in Isaac Sim

# Step C (Terminal 1): Launch ROS 2 Multi-Robot Stack with Sim Clock
ros2 launch multi_arm_bringup multi_arm_simulation.launch.py use_sim_time:=true

# Step D (Terminal 2): Run Relay State Machine
ros2 run multi_arm_control task_manager --ros-args -p use_sim_time:=true
```

### 4. Run in Gazebo Harmonic Mode (ROS 2 Jazzy Standard)
```bash
# Terminal 1: Launch 3-Robot Workcell in Gazebo Harmonic + MoveIt 2 + Action Servers
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch multi_arm_bringup multi_arm_gazebo.launch.py

# Terminal 2: Run Relay State Machine
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run multi_arm_control task_manager --ros-args -p use_sim_time:=true
```
