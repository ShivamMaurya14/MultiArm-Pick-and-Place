# Multi-Arm UR10e Pick-and-Place
## ROS 2 Jazzy + MoveIt 2 + NVIDIA Isaac Sim (Omniverse) + Physical AI

A production-grade robotics workspace featuring three independent Universal Robots UR10e manipulator arms equipped with Robotiq 2F-140 adaptive grippers performing an autonomous, coordinated sequential pick-and-place relay task across four physical workstations (Station A → B → C → D).

The platform supports **ROS 2 Jazzy Jalisco**, **MoveIt 2**, **Gazebo Harmonic**, and **NVIDIA Isaac Sim (v4.x, v5.x, v6.0+)**, featuring a GPU-accelerated **NVIDIA Cosmos Multi-Cell Spawner** capable of simulating **10 groups (30 UR10e arms, 10 platforms, 20 RTX cameras)** in real-time PhysX dynamics on consumer workstation hardware.

---

## 📚 Documentation & Technical Guides
All in-depth technical guides, architecture documentation, and testing procedures are located in [`src/docs/`](src/docs/):

1. **[Comprehensive Project Study & Engineering Guide](src/docs/multi_arm_project_study_guide.md):** Complete end-to-end breakdown of how this project was built, packages and their roles, 13 key challenges & resolutions, phase progress, and pending items.
2. **[Hybrid Architecture Implementation Plan](src/docs/multi_arm_ur10e_implementation_plan.md):** Detailed multi-phase roadmap spanning baseline bringup to NVIDIA Cosmos foundation models.
3. **[Isaac Sim Multi-Arm Simulation Guide](src/docs/isaac_sim_guide.md):** Complete Isaac Sim setup, URDF importer parameters, coordinate maps, gripper calibration, multi-cell procedural spawning, tactile sensing, and ROS 2 Jazzy bridge integration.
4. **[System Build & Architecture Guide](src/docs/system_build_and_architecture_guide.md):** Deep-dive explaining how each package, description, MoveIt 2 node, and Isaac Sim bridge was built.
5. **[Robot + Gripper Configuration Guide](src/docs/robot_gripper_config_guide.md):** Creating unified Xacro models, calibrated TCP links, MoveIt 2 Setup Assistant workflows, controller configurations, and multi-robot namespacing.
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
| **P7** | **Physical AI & NVIDIA Cosmos Multi-Cell Spawner** | `COMPLETED (7A)` | `spawn_multi_cell_cosmos.py`, 10 groups (30 robots), direct USDA loading, multi-link tactile & pinhole camera array |

---

## 🛠️ Workspace Packages

- `src/multi_arm_description/`: Unified URDF/Xacro models (`multi_ur10e_workcell.urdf.xacro`, `ur10e_robotiq.urdf.xacro`) combining UR10e and Robotiq 2F-140 with calibrated TCP links and industrial workcell staging.
- `src/robot1_moveit_config/`, `robot2...`, `robot3...`: Independent MoveIt 2 configurations per robot with isolated namespaces (`/robot1`, `/robot2`, `/robot3`).
- `src/multi_arm_interfaces/`: Custom ROS 2 action interfaces (`PickPlace.action`).
- `src/multi_arm_control/`: Python nodes for `pick_place_server` and sequential state machine `task_manager`.
- `src/multi_arm_bringup/`: Centralized launch files (`multi_arm_simulation.launch.py`, `stations.yaml`).
- `src/ur_simulation/`: Procedural Isaac Sim 4.x/5.x/6.x Python workcell spawner scripts (`spawn_multi_ur10e.py` for 3-arm relay; `spawn_multi_cell_cosmos.py` for parallel multi-cell foundation model dataset generation).
- `src/docs/`: Comprehensive technical documentation suite.

---

## 📍 Relay Workcell Coordinate Reference (3-Robot Relay)

```text
  [Station A: Source]         [Station B: Relay 1]        [Station C: Relay 2]        [Station D: Dropoff]
  X=0.70, Y=0.00, Z=0.25      X=0.70, Y=0.80, Z=0.25      X=0.70, Y=2.40, Z=0.25      X=0.70, Y=3.20, Z=0.25
        ▲                           ▲                           ▲                           ▲
        │                           │                           │                           │
  [Robot 1: Y=0.0, Z=0.20] ───────► [Robot 2: Y=1.6, Z=0.20] ───────► [Robot 3: Y=3.2, Z=0.20]
```

| Component | Entity / Prim | Coordinates $(X, Y, Z)$ | Description |
| :--- | :--- | :--- | :--- |
| **Robot 1** | `/World/robot1` | $(0.00, 0.00, 0.20)$ | Base on 0.20m Pedestal (Cell 1) |
| **Robot 2** | `/World/robot2` | $(0.00, 1.60, 0.20)$ | Base on 0.20m Pedestal (Cell 2) |
| **Robot 3** | `/World/robot3` | $(0.00, 3.20, 0.20)$ | Base on 0.20m Pedestal (Cell 3) |
| **Station A** | `station_a_table` | $(0.70, 0.00, 0.20)$ | Source Table (Blue) |
| **Station B** | `station_b_table` | $(0.70, 0.80, 0.20)$ | Relay 1 Buffer Table (Orange) |
| **Station C** | `station_c_table` | $(0.70, 2.40, 0.20)$ | Relay 2 Buffer Table (Orange) |
| **Station D** | `station_d_table` | $(0.70, 3.20, 0.20)$ | Destination Table (Purple) |
| **Workpiece** | `Workpiece_Cube` | $(0.70, 0.00, 0.245)$ | $60	ext{mm}$ Dynamic Cube ($0.15	ext{kg}$) |

---

## ⚡ NVIDIA Cosmos Massive Multi-Cell Spawner (30 Robots / 20 Cameras)

For large-scale dataset generation, reinforcement learning (Isaac Lab), and physical AI foundation models (NVIDIA Cosmos), `spawn_multi_cell_cosmos.py` procedurally constructs **10 tri-arm groups** in a $5 	imes 2$ grid:

- **Total Entities:** 30 UR10e arms + Robotiq 2F-140 grippers, 10 central tables, 10 dynamic workpieces, and 20 calibrated RTX cameras.
- **Direct USDA Instancing:** Bypasses GUI stage duplication by referencing `src/ur10e_robotiq/ur10e_robotiq.usda` directly from disk.
- **Radial Geometry ($R = 1.25	ext{m}$):** Arms are arranged in an equilateral triangle at $	heta = 90^\circ, 210^\circ, 330^\circ$ facing inward, providing ample collision clearance.
- **Physics Stabilization & Zero Jitter:**
  1. *Canonical Standby Posture:* Shoulder Pan: $0^\circ$, Lift: $-90^\circ$, Elbow: $+90^\circ$, Wrist 1: $-90^\circ$, Wrist 2: $-90^\circ$, Wrist 3: $0^\circ$.
  2. *Critically Damped Position Drives:* Stiffness $K_p = 5000.0$, Damping $K_d = 1000.0$, Max Force $F_{\max} = 10^6	ext{ N}$.
  3. *Frame-0 Initialization:* `JointStateAPI:angular` values are written on frame 0 to eliminate initial gravity drops.
  4. *Pedestal Collision Mesh Disabled:* Visual pedestals have no colliders (`has_collision = False`) to prevent micro-contact friction conflicts with `base_link_inertia`.
- **Tactile Sensing Array:** Attached `PhysxSchema.PhysxContactReportAPI` with `threshold = 0.0` across 7 rigid bodies per robot (`wrist_3_link`, `tool0`, `robotiq_140_base_link`, `left_inner_finger`, `right_inner_finger`, `left_inner_finger_pad`, `right_inner_finger_pad`).
- **Pinhole Camera Array:** 20 cameras total (`Camera_TopDown` at $Z=2.6	ext{m}$ and `Camera_Angled` at $Y=-2.4	ext{m}, Z=2.0	ext{m}$) configured with pinhole perspective optics ($fStop = 0.0$) ready for Omniverse Replicator dataset logging.
- **PhysX Unified Scene & Buffer Sizing:**
  * Aggregate Pairs: Dynamically scaled to **$3,500,000$ pairs** (`max(1048576, TOTAL_GROUPS * 350000)`).
  * GPU Contact Buffer: $2,097,152$ contacts | GPU Patch Buffer: $655,360$ patches | GPU Heap: $256	ext{ MB}$.
- **Hardware Telemetry (i7-14700F + 16GB RAM + RTX 5060 Ti 16GB):**
  * 3 Groups (9 Robots): **65.25 FPS** | 7.6 GiB RAM used | 901 MiB VRAM used.
  * 6 Groups (18 Robots): **63.91 FPS** | 7.2 GiB RAM used | 888 MiB VRAM used.
  * 10 Groups (30 Robots): **~40–50 FPS** | ~8.2 GiB RAM used | ~1.1 GiB VRAM used (100% stable; well under 16 GB limit).

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
#### Step 3.1: Export Standalone URDF directly into `src/`
```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
xacro src/multi_arm_description/urdf/ur10e_robotiq.urdf.xacro > src/multi_arm_description/urdf/ur10e_robotiq.urdf
```

#### Step 3.2: Exact Isaac Sim URDF Importer Settings
1. In Isaac Sim, open **Isaac Utils → Workflows → URDF Importer** (or **Tools → Robotics → URDF Importer**).
2. Configure settings:
   - **Input File:** `/home/arvr/ros2_ws/MultiArm-Pick-and-Place/src/multi_arm_description/urdf/ur10e_robotiq.urdf`
   - **Target Prim Path:** `/World/UR10e` (or save as `src/ur10e_robotiq/ur10e_robotiq.usda`)
   - **Fix Base Link:** `Checked` ✅ (Essential to anchor root to world)
   - **Joint Drive Type:** `Position`
   - **Default Drive Strength (Stiffness):** `5000.0` (Arm) / `1000.0` (Gripper)
   - **Default Damping:** `1000.0` (Arm) / `50.0` (Gripper)
   - **Colliders:** `Convex Decomposition`
   - **Self Collision:** `Unchecked` ⬜ (Avoids wrist/base mesh intersection locking)
   - **ROS Package Search Paths:** Add `/home/arvr/ros2_ws/MultiArm-Pick-and-Place/src`
3. Click **Import**.

#### Step 3.3: Run the 3-Robot Relay Scene
1. Open **Window → Script Editor**.
2. Load `src/ur_simulation/scripts/spawn_multi_ur10e.py` and click **Run**.
3. Press **PLAY (▶)** in Isaac Sim.
4. In your ROS 2 terminal:
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

### 5. NVIDIA Cosmos Massive Multi-Cell Spawner (30 Robots / 20 Cameras)
```bash
# 1. Launch Isaac Sim with a clean stage (File -> New)
# 2. Open Window -> Script Editor
# 3. Load src/ur_simulation/scripts/spawn_multi_cell_cosmos.py
# 4. Click Run (Configured with 10 groups, 30 UR10e robots, 20 RTX cameras in 5x2 grid)
# 5. Press PLAY (▶) to simulate all 30 robots, platforms, and perception arrays in GPU PhysX
```

---

## 📌 Important Engineering Guidelines & Constraints

> [!IMPORTANT]
> **Git Repository Guidelines:**
> - **NO GIT PUSH:** Do not push changes to remote git repositories without explicit user authorization. Local commits and working-tree modifications only.
> - **Author Metadata:** All commits must use `user.name = shivammaurya14` and `user.email = shivammaurya1432005@gmail.com`.

> [!TIP]
> **FastDDS XML Configuration:**
> To guarantee jitter-free, zero-packet-drop communication between MoveIt 2 and Isaac Sim OmniGraph bridges, set `FASTRTPS_DEFAULT_PROFILES_FILE=~/.ros/fastdds_racy.xml` with non-blocking UDP transports as documented in [`isaac_sim_installation_guide.md`](src/docs/isaac_sim_installation_guide.md).
