# Isaac Sim Multi-Arm Simulation Guide

This guide explains how to bring up the simulation, how to modify robot starting positions, and what settings you need to check in Isaac Sim.

## 1. How to Setup and Run the Simulation

Because we already wrote a custom Isaac Sim python script (`spawn_multi_ur10e.py`), you do **not** need to manually drag-and-drop robots into the Isaac Sim GUI! The script handles spawning the 3 robots, namespacing them, and setting up the ROS 2 Action Graphs automatically.

**Steps to launch:**
1. Open a terminal on your Simulation PC.
2. Source your ROS 2 Humble environment:
   ```bash
   source /opt/ros/humble/setup.bash
   ```
3. Source this workspace:
   ```bash
   source ~/path/to/MultiArm-Pick-and-Place/install/setup.bash
   ```
4. Run the Isaac Sim python script. You must run this using Isaac Sim's bundled python executable (`python.sh`):
   ```bash
   ~/.local/share/ov/pkg/isaac_sim-2023.1.1/python.sh src/ur_simulation/scripts/spawn_multi_ur10e.py
   ```
   *(Note: Adjust the `isaac_sim-2023.1.1` path depending on exactly which version of Isaac Sim you installed).*

Once the script finishes loading, the Isaac Sim GUI will open, and you will see three UR10e robots spawned in the world! **Make sure you press the PLAY button** in the Isaac Sim GUI so the physics and ROS 2 bridges start running.

## 2. How to Modify Robot Positions

If you want to move the robots further apart or arrange them differently, you need to modify the Python script that spawns them.

1. Open `src/ur_simulation/scripts/spawn_multi_ur10e.py`
2. Scroll to the bottom where the robots are spawned:
   ```python
   # Robot 1
   spawn_robot("/World/Robot1", "/robot1", translation=np.array([0.0, 0.0, 0.0]))
   
   # Robot 2
   spawn_robot("/World/Robot2", "/robot2", translation=np.array([0.0, 1.5, 0.0]))
   
   # Robot 3
   spawn_robot("/World/Robot3", "/robot3", translation=np.array([0.0, -1.5, 0.0]))
   ```
3. Change the `translation=np.array([X, Y, Z])` values to move the bases of the robots. Note that Isaac Sim uses meters.
4. Save the file and restart the `spawn_multi_ur10e.py` script.

> [!IMPORTANT]
> If you move the robots, you must also update the Station coordinates! Open `src/multi_arm_bringup/config/stations.yaml` and update the X, Y, Z coordinates for Stations A, B, C, and D so they remain within reach of the new robot positions.

## 3. Other Settings to Check

- **Press Play:** The ROS 2 clock and joint states will NOT publish until you press the **Play** button in the left-hand toolbar of Isaac Sim.
- **Check Topics:** Open a new terminal, source ROS 2, and run `ros2 topic list`. You should see namespaced topics like `/robot1/joint_states`, `/robot2/joint_states`, etc.
- **Run MoveIt:** Once Isaac Sim is playing and topics are publishing, you can launch the MoveIt stack in another terminal:
  ```bash
  ros2 launch multi_arm_bringup multi_arm_isaac_sim.launch.py
  ```
