# Multi-Arm UR10e Pick-and-Place

A ROS 2, MoveIt 2, and Isaac Sim project featuring three independent UR10e manipulator arms equipped with Robotiq 2F-140 grippers performing a sequential, coordinated pick-and-place task. 

## Overview
This repository contains a full simulation and control stack designed to orchestrate an object moving autonomously across three arms (Robot 1 → Robot 2 → Robot 3) between four physical stations (A → B → C → D). 

The system architecture utilizes independent namespaces for each arm (`/robot1`, `/robot2`, `/robot3`) allowing concurrent MoveIt 2 trajectory planning and execution, managed by a centralized global Task Manager node.

## Project Phases & Status

This project is structured into 7 core phases of development.

### ✅ Completed Phases
* **Phase 1: One UR10e + MoveIt 2 Bringup** 
  * Integrated official UR10e ROS 2 driver and descriptions.
* **Phase 2: Pick/Place Action Server**
  * Created custom `PickPlace.action` server wrapping MoveIt's `moveit_py` python API and `GripperCommand` for the Robotiq 2F-140.
  * Extracted physical stations to `stations.yaml`.
* **Phase 3: Three UR10e in Isaac Sim**
  * Built an automated Python script (`spawn_multi_ur10e.py`) to procedurally spawn the robots in Isaac Sim.
  * Created independent MoveIt configurations for each robot, completely isolating them via ROS 2 namespaces to prevent planning collisions.
* **Phase 4: A→B→C→D Sequential Coordination**
  * Implemented a `task_manager` node handling the full State Machine logic (Start → Robot1 Pick A → Robot1 Place B → Robot2 Pick B → ...).

### 🚧 Pending / Future Work
* **Phase 5: Camera + Object Detection**
  * Swap hardcoded station assumptions with real perception updates.
  * Integrate Isaac ROS GPU-accelerated pipelines (`isaac_ros_yolov8`) or ArUco markers to drive the Task Manager state machine using real-time synthetic camera feeds.
* **Phase 6: Dynamic Multi-Arm Coordination**
  * Transition from a strict sequential order to concurrent multi-object tracking.
  * Link MoveIt planning scenes or implement interlock zones to allow overlapping arm movements without physical collisions.
* **Phase 7: Physical AI / Cosmos Experiments**
  * Open R&D phase exploring Sim-to-Sim transfers, NVIDIA Cosmos world-foundation models for synthetic data generation, or learned multi-agent RL policies.

---

## Workspace Architecture

- `multi_arm_bringup/`: Centralized launch files (`multi_arm_isaac_sim.launch.py`, `pick_place_servers.launch.py`, `task_manager.launch.py`) and config definitions.
- `multi_arm_control/`: Contains the actual python nodes (`pick_place_server.py`, `task_manager.py`).
- `multi_arm_description/`: Contains the customized URDF connecting the UR10e arm to the Robotiq 2F-140 gripper.
- `multi_arm_interfaces/`: Defines custom ROS 2 actions (`PickPlace.action`).
- `robot1_moveit_config/`, `robot2...`, `robot3...`: Independent MoveIt configurations per robot.
- `ur_simulation/`: Contains Isaac Sim spawning scripts.

## Running the Simulation

1. **Build the Workspace (ROS 2 Humble):**
   ```bash
   colcon build --symlink-install
   source install/setup.bash
   ```
2. **Launch Isaac Sim Environment:**
   Run the procedural spawning script using Isaac Sim's python executable:
   ```bash
   ~/.local/share/ov/pkg/isaac_sim-2023.1.1/python.sh src/ur_simulation/scripts/spawn_multi_ur10e.py
   ```
   *Make sure to press PLAY in the Isaac Sim GUI to start the physics and ROS 2 clock!*
3. **Launch the MoveIt Instances:**
   ```bash
   ros2 launch multi_arm_bringup multi_arm_isaac_sim.launch.py
   ```
4. **Launch the Pick/Place Action Servers:**
   ```bash
   ros2 launch multi_arm_bringup pick_place_servers.launch.py
   ```
5. **Start the Task Manager Sequence:**
   ```bash
   ros2 launch multi_arm_bringup task_manager.launch.py
   ```
