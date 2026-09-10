# Isaac Sim Multi-Arm Simulation Guide
## Target: ROS 2 Jazzy Jalisco + NVIDIA Isaac Sim (v4.x, v5.x, v6.0+)

This guide covers complete setup, automated scene generation, ROS 2 Jazzy bridge integration, and physics pick-and-place simulation.

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

## 1. Workcell Architecture & Coordinate Map

The multi-arm workcell features 3 UR10e manipulators with Robotiq 2F-140 grippers mounted on steel pedestals performing a sequential relay across 4 physical stations:

```text
  [Station A: Source]         [Station B: Relay 1]        [Station C: Relay 2]        [Station D: Dropoff]
  X=0.70, Y=0.00, Z=0.25      X=0.70, Y=0.80, Z=0.25      X=0.70, Y=2.40, Z=0.25      X=0.70, Y=3.20, Z=0.25
        ▲                           ▲                           ▲                           ▲
        │                           │                           │                           │
  [Robot 1: Y=0.0, Z=0.20] ───────► [Robot 2: Y=1.6, Z=0.20] ───────► [Robot 3: Y=3.2, Z=0.20]
```

### Table of Coordinates
| Component | Object / Frame | Position $(X, Y, Z)$ in meters | Function / Description |
| :--- | :--- | :--- | :--- |
| **Robot 1** | `/World/robot1` | $(0.00, 0.00, 0.20)$ | Base mounted on 0.20m Pedestal (Cell 1) |
| **Robot 2** | `/World/robot2` | $(0.00, 1.60, 0.20)$ | Base mounted on 0.20m Pedestal (Cell 2) |
| **Robot 3** | `/World/robot3` | $(0.00, 3.20, 0.20)$ | Base mounted on 0.20m Pedestal (Cell 3) |
| **Station A** | `station_a_table` | $(0.70, 0.00, 0.20)$ | Blue source table with target placement ring |
| **Station B** | `station_b_table` | $(0.70, 0.80, 0.20)$ | Orange buffer table for Robot 1 $\rightarrow$ 2 handoff |
| **Station C** | `station_c_table` | $(0.70, 2.40, 0.20)$ | Orange buffer table for Robot 2 $\rightarrow$ 3 handoff |
| **Station D** | `station_d_table` | $(0.70, 3.20, 0.20)$ | Purple final destination table |
| **Workpiece** | `Workpiece_Cube` | $(0.70, 0.00, 0.245)$ | $60\text{mm} \times 60\text{mm} \times 50\text{mm}$ Dynamic Cube ($0.15\text{kg}$) |

---

## 2. Robotiq 2F-140 Gripper Grasp Calibration

* **Stroke Range:** $0.0\text{m}$ to $0.140\text{m}$ ($140\text{mm}$).
* **Joint Limits:** $0.0\text{ rad}$ (fully open, $140\text{mm}$) to $0.70\text{ rad}$ (fully closed, $0\text{mm}$).
* **Linear Grasp Angle Formula:**
  $$\theta_{\text{grasp}} = 0.70 \times \left(1.0 - \frac{W_{\text{cube}}}{0.140}\right)$$
  For the $60\text{mm}$ ($0.060\text{m}$) workpiece cube:
  $$\theta_{\text{grasp}} = 0.70 \times \left(1.0 - \frac{0.060}{0.140}\right) = \mathbf{0.400\text{ rad}}$$
* **Approach / Release:** `0.0 rad` ($140\text{mm}$ opening width).
* **Hold / Transport:** `0.400 rad` (rubber pads rest firmly against cube sides).

---

## 3. Step-by-Step Simulation Setup in Isaac Sim

### Step A: Export Standalone URDF
In your ROS 2 Jazzy workspace terminal:
```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
xacro src/multi_arm_description/urdf/ur10e_robotiq.urdf.xacro > /tmp/ur10e_robotiq.urdf
```

### Step B: Import Robot Model into Isaac Sim (v4.x, v5.x, v6.0.1+)
1. Launch **Isaac Sim**.
2. Go to **Isaac Utils → Workflows → URDF Importer** *(or **Tools → Robotics → URDF Importer** in Kit 106+)*.
3. In the top mode selector, choose **`Import`** (Standard URDF Import mode).

#### ⚙️ Settings to Choose in the URDF Importer Window:

| Configuration Field | Recommended Value / Setting | Purpose |
| :--- | :--- | :--- |
| **Input File** | `/tmp/ur10e_robotiq.urdf` | Path to generated standalone URDF |
| **Target Prim Path / USD Output** | `/World/UR10e` | Imports the template robot prim under `/World/UR10e` |
| **Fix Base Link** | `Checked` ✅ | Anchors the base to prevent the robot from falling |
| **Drive Type** | `Position` | Enables position-controlled joint drives |
| **Colliders (Collision Mesh)** | `Convex Decomposition` *(or Convex Hull)* | Accurate physical collisions on arm & fingers |
| **Self Collision** | `Unchecked` ⬜ | Prevents internal link self-collision overhead |
| **Merge Fixed Joints** | `Checked` ✅ | Optimizes kinematics tree for fixed gripper links |
| **ROS Package Search Paths** | `.../MultiArm-Pick-and-Place/src` | Resolves `package://` meshes (`ur_description`, `robotiq_description`) |

4. Click the **`Import`** button at the bottom.
5. In the **Stage Tree** (top-right), select **`/World/UR10e`** and press **`F`** in the 3D viewport to center the camera on the robot.

---

### Step C: 🚀 Step to Spawn All 3 Robots & Full Workcell

*(Note: Importing URDF creates the single robot template `/World/UR10e`. To automatically instantiate all 3 robots, steel pedestals, 4 station tables, and dynamic workpiece, execute the procedural script):*

1. In Isaac Sim, open top menu: **Window → Script Editor**.
2. Open or paste [`src/ur_simulation/scripts/spawn_multi_ur10e.py`](file:///Users/shivammaurya/Desktop/ros2_ws/nextup/multi_arm_ws/MultiArm-Pick-and-Place/src/ur_simulation/scripts/spawn_multi_ur10e.py).
3. Click **Run** (or press `Ctrl + Enter`).
4. **What the Script Automatically Builds:**
   * **Physics Scene & Contact Materials:** Defines PhysX 5 gravity ($-9.81\text{ m/s}^2$) and high-friction contact material ($\mu_s=1.2, \mu_d=0.9$).
   * **Ground Platform & Demarcation Grid:** $2.4\text{m} \times 5.6\text{m}$ floor with yellow cell boundary lines.
   * **3 Steel Mounting Pedestals:** $0.20\text{m}$ elevated pedestals for `robot1` ($Y=0.0$), `robot2` ($Y=1.6$), and `robot3` ($Y=3.2$).
   * **4 Station Tables:** Station A (Blue Source), Station B (Orange Buffer 1), Station C (Orange Buffer 2), Station D (Purple Destination).
   * **Dynamic Workpiece Cube:** $60\text{mm} \times 60\text{mm} \times 50\text{mm}$ emerald green dynamic rigid-body cube ($0.15\text{kg}$) spawned at Station A $(0.70, 0.00, 0.245)$.
   * **Instantiates the 3 Robots:** Duplicates the template into `/World/robot1`, `/World/robot2`, and `/World/robot3` at pedestal heights ($Z=0.20\text{m}$).
   * **OmniGraph ROS 2 Bridges:** Configures `/clock`, `/{namespace}/joint_states`, and `/{namespace}/joint_commands` action graphs.
5. Press the **PLAY (▶)** button on the left toolbar in Isaac Sim.

---

## 4. Running the ROS 2 Pick & Place Control Stack

In your ROS 2 Jazzy terminal:

```bash
# Terminal 1: Launch Multi-Robot Coordination & Action Servers with Sim Clock
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch multi_arm_bringup multi_arm_simulation.launch.py use_sim_time:=true
```

```bash
# Terminal 2: Trigger Sequential Relay State Machine (A -> B -> C -> D)
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run multi_arm_control task_manager --ros-args -p use_sim_time:=true
```

---

## 5. Verification & Key Diagnostics

* **Simulation Clock Check:** `ros2 topic echo /clock` should publish simulation timestamps.
* **Joint State Topics:**
  * `ros2 topic echo /robot1/joint_states`
  * `ros2 topic echo /robot2/joint_states`
  * `ros2 topic echo /robot3/joint_states`
* **Action Servers Ready:**
  * `ros2 action list` should show `/robot1/pick_place`, `/robot2/pick_place`, `/robot3/pick_place`.
