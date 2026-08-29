# Multi-Arm UR10e Pick-and-Place: Detailed Implementation Plan

This is a phase-by-phase build guide for the A→B→C→D multi-arm pipeline (3× UR10e, ROS 2, MoveIt 2, Isaac Sim). Each phase lists **what to build**, **which existing repo to reuse instead of writing from scratch**, **why that choice**, and **how to verify you're done** before moving on.

Workspace layout (create this once, up front):

```
multi_arm_ws/
└── src/
    ├── ur_robot_description/     # from UniversalRobots/Universal_Robots_ROS2_Description
    ├── ur_simulation/            # Isaac Sim launch/USD assets
    ├── robot1_moveit_config/     # generated via MoveIt Setup Assistant
    ├── robot2_moveit_config/
    ├── robot3_moveit_config/
    ├── multi_arm_bringup/        # top-level launch files
    ├── task_manager/             # your state machine node (custom, Phase 4)
    └── perception/                # camera + detection (Phase 5)
```

---

## Phase 1 — One UR10e + MoveIt 2

**Goal:** command an arbitrary target pose to a single simulated UR10e (using mock hardware) via MoveIt 2 and watch it plan + execute.

### Repos to reuse (do not write your own URDF or driver)
- **`UniversalRobots/Universal_Robots_ROS2_Driver`** — the official ROS 2 driver, supports CB3 and e-Series including UR10e, maintained by Universal Robots themselves. Branch per ROS distro (`humble`, `jazzy`, etc).
  `git clone -b humble https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git`
- **`UniversalRobots/Universal_Robots_ROS2_Description`** — official URDF/xacro for all UR models including UR10e. This is what feeds `ur_robot_description` in your tree.
- MoveIt config: the driver repo already ships a working `ur_moveit_config` demo — clone/inspect it first, then fork it three times for Phase 3 rather than hand-building SRDFs from zero.

### Why reuse instead of building from scratch
- The UR10e URDF (link lengths, inertials, collision meshes, joint limits) is precise, calibrated data — hand-rolling it risks small errors that cause bad planning or self-collision in the real arm.
- The official driver handles the real-time control loop, safety I/O (E-stop, safeguard stop, speed scaling) that a from-scratch driver would take weeks to get right and is safety-critical on hardware.

### Steps
1. `mkdir -p multi_arm_ws/src && cd multi_arm_ws`
2. Clone the driver and description packages into `src/`.
3. `rosdep install --from-paths src --ignore-src -r -y && colcon build --symlink-install`
4. Test with mock hardware (so you don't need a real robot or heavy URSim container):
   Terminal 1 (Driver):
   ```
   ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur10e use_mock_hardware:=true launch_rviz:=false
   ```
   Terminal 2 (MoveIt):
   ```
   ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur10e use_mock_hardware:=true launch_rviz:=true
   ```
5. In RViz, drag the interactive marker to a new pose, hit **Plan & Execute**.

### Exit criteria
Arm reaches an arbitrary commanded pose in RViz reliably, with no planning failures on typical reachable poses.

---

## Phase 2 — One UR10e + pick/place

**Goal:** one arm reliably executes approach → grasp → retreat → approach → release → retreat between two named stations.

### Repos/tools to reuse
- **MoveIt 2's `MoveGroupInterface` / `moveit_commander` (Python) or `moveit_cpp`** — don't write your own Cartesian planner; use MoveIt's built-in Cartesian path planning (`computeCartesianPath`) for the linear approach/retreat moves, and joint-space planning for the transit moves.
- **`arshadlab/pymoveit2`** (fork of `pymoveit2`) — a clean Python wrapper around MoveIt 2's action interfaces (`ex_pose_goal.py` style scripts) that's already namespace-aware — useful groundwork for Phase 3 where you'll need per-robot namespaces.
- Gripper: if you have a real gripper, look for its ROS 2 driver first (e.g. Robotiq 2F-140 has existing ROS 2 packages) rather than writing serial/Modbus code yourself. If mocked, a simple `std_srvs/SetBool`-style service is enough for now.

### Why reuse
- `computeCartesian Path` already handles waypoint interpolation and partial-path fallback — reimplementing this is a common source of subtle planning bugs (e.g. IK jumps).
- Namespace-aware client code (pymoveit2) saves you a rewrite in Phase 3 when you go from 1 to 3 arms.

### Steps
1. Define stations as a config, not hard-coded numbers (matches your own notes):
   ```
   STATION_A = (x1, y1, z1)
   STATION_B = (x2, y2, z2)
   ```
   Put this in a small YAML file under `multi_arm_bringup/config/stations.yaml` — load it at runtime, don't bake it into code.
2. Write a `pick_place` action server (custom `action` type: `PickPlace.action` with goal = target station name, feedback = phase, result = success/failure). Internally it calls MoveIt's Cartesian + joint planning.
3. Wire gripper open/close as a service call inside the pick/place sequence.
4. Test repeatedly (20+ cycles) for reliability before moving on — flaky pick/place here will compound badly across 3 arms later.

### Exit criteria
One arm picks from Station A, places at Station B, repeatably, with a clean success/failure result reported back.

---

## Phase 3 — Three UR10e in Isaac Sim

**Goal:** all three arms independently plan/execute inside the same Isaac Sim scene without interfering with each other.

### Repos/tools to reuse
- **NVIDIA Isaac Sim's official manipulator import tutorial** (Isaac Sim docs, "Setup a Manipulator") — walks through importing the UR10e URDF via the ROS 2 URDF importer into USD. Reuse this pipeline rather than hand-authoring USD robot files.
  ```
  git clone https://github.com/UniversalRobots/Universal_Robots_ROS2_Description.git
  git checkout humble   # match your ROS distro
  ros2 launch ur_description view_ur.launch.py ur_type:=ur10e
  # In Isaac Sim: File → Import from the ROS 2 URDF Node
  ```
- **`ainhoaarnaiz/ur10e_isaac_sim_ros2`** — a ready-made UR10e + Robotiq 2F-140 Isaac Sim + ROS 2 devcontainer stack (topic-based `ros2_control` bridge already wired). Good reference for how to wire the Isaac Sim ↔ ROS 2 bridge for a UR10e specifically, saving you the trial-and-error of setting up `ros2_control` hardware interfaces against Isaac Sim's simulated joints.
- **MoveIt's own dual/multi-arm tutorial** (`moveit2_tutorials`, "Dual Arms with MoveIt", using `dual_arm_panda_moveit_config` as the reference pattern) — even though it's Panda, not UR10e, it documents *exactly* the checklist you need for going from 1→N arms:
  - prefix every joint/link name per robot (`robot1_`, `robot2_`, `robot3_`)
  - prefix the `ros2_control` macro instantiation per robot
  - enumerate one controller set per robot in `ros2_controllers.yaml`
  - define one planning group + end-effector per robot in the SRDF
  - one kinematics solver entry per robot in `kinematics.yaml`
  - one moveit controller entry per robot in `moveit_controllers.yaml`
- **`arshadlab/multi_robot_arm`** — a working ROS 2 + Gazebo + MoveIt 2 example of 4 independently-namespaced arms (`/arm1`...`/arm4`) controlled via `pymoveit2`, with per-namespace `move_group` nodes. This is the closest existing open-source analogue to your exact target architecture (independent namespaced arms, one task manager driving them) — treat it as your primary structural reference even though it's Gazebo not Isaac Sim; the ROS-side namespacing/MoveIt config pattern transfers directly.

### Why reuse instead of building from scratch
- Multi-arm MoveIt configuration is finicky (name collisions between arms are the #1 source of bugs) — following an already-solved worked example (dual-arm Panda tutorial + `multi_robot_arm`'s namespacing) avoids re-discovering these problems from scratch.
- Getting Isaac Sim's `ros2_control` bridge correctly wired to real joint controllers is non-obvious; `ur10e_isaac_sim_ros2` already encodes correct settings (topic-based control vs. mock components) so you can diff against a known-good config when yours doesn't work.
- A `move_group` node cannot execute two trajectories at once — so each robot needs its **own** `move_group` in its **own** namespace. This is a documented gotcha (see MoveIt2 issue #2744) — worth knowing before you build, not after debugging it for a day.

### Steps
1. Import one UR10e into Isaac Sim following the official tutorial; confirm ROS 2 topics (`/robot_description`, joint states) show up in `rqt_graph`.
2. Duplicate the prim in Isaac Sim two more times, applying a unique namespace/prefix to each instance's ROS 2 bridge component (`/robot1/...`, `/robot2/...`, `/robot3/...`).
3. Fork `robot1_moveit_config` twice → `robot2_moveit_config`, `robot3_moveit_config`, renaming all prefixed joints/links/groups/controllers as per the dual-arm tutorial checklist.
4. Launch one `move_group` per robot, each in its own namespace, from a single `multi_arm_bringup` launch file.
5. Validate independently: command each arm from RViz (switch planning group per robot) or via `pymoveit2` with `--ros-args -r __ns:=/robot1`.

### Exit criteria
All three arms plan/execute independently in the same Isaac Sim scene, on separate `move_group` instances, with no name collisions and no interference.

---

## Phase 4 — A→B→C→D sequential coordination

**Goal:** an object moves autonomously across all three arms with no manual triggering, driven by a state machine.

### Repos/tools to reuse
- **`smacc2` or a plain `rclpy` state machine, or `py_trees_ros` / `BehaviorTree.CPP` (via `ros2 launch assembly_orchestrator ...` style projects)** — don't hand-roll ad hoc if/else state tracking in a single node if you expect Phase 6's dynamic coordination requirements; a behavior-tree or state-machine library gives you retries, timeouts, and parallel branches for free later. `py-trees-tree-viewer` (used in the `ur10_ros1_ros2` reference project) is a good way to visualize state during debugging.
- Reuse the **action server pattern from Phase 2** — expose it per robot as `/robot1/pick`, `/robot1/place`, `/robot2/pick`, `/robot2/place`, `/robot3/pick`, `/robot3/place`, matching your own topic tree sketch.

### Why action servers, not plain topics
Plain topics are fire-and-forget — you get no feedback and no clean success/failure signal. `action` servers give you goal/feedback/result semantics for free, which is exactly what a state machine needs to know when to advance ("robot1 finished placing at B, object is now at B, trigger robot2's pick").

### Steps
1. Build the state machine exactly as in your diagram:
   ```
   START → OBJECT_AT_A → ROBOT1_PICK → ROBOT1_PLACE_B → OBJECT_AT_B
         → ROBOT2_PICK → ROBOT2_PLACE_C → OBJECT_AT_C
         → ROBOT3_PICK → ROBOT3_PLACE_D → COMPLETE
   ```
2. `task_manager` node holds current state + calls the appropriate robot's action client on each transition; only advances state on a successful action result.
3. Add basic failure handling even at this stage: on action failure, transition to a `FAILED` state rather than hanging — this makes debugging Phase 3/4 integration issues much faster.

### Exit criteria
A single object autonomously traverses A→B→C→D across all three arms with zero manual intervention, and a failed pick/place is visibly reported rather than silently hanging.

---

## Phase 5 — Camera + object detection

**Goal:** replace hard-coded "object is at station X" assumptions with real perception-driven state updates.

### Repos/tools to reuse
- **`NVIDIA-ISAAC-ROS/isaac_ros_object_detection`** — GPU-accelerated ROS 2 packages: `isaac_ros_yolov8`, `isaac_ros_detectnet`, `isaac_ros_rtdetr`, `isaac_ros_grounding_dino`. Since you're already committed to Isaac Sim/Isaac ecosystem, this is the natural fit — it integrates with Isaac Sim's synthetic camera output directly, and YOLOv8 in particular is pre-trained on COCO's 80 classes so you can prototype without training your own model first.
- **NVIDIA "Isaac for Manipulation" reference workflow** — a full perception-driven pick-and-place reference (detection → pose estimation → cuMotion collision-free motion) built from these same Isaac ROS packages. Even if you don't adopt cuMotion yet, its architecture (detection node → pose estimation → planner) is the reference pattern to follow for how perception output should feed into your `task_manager`.
- If you want the absolute simplest possible starting point before committing to a DNN model: **ArUco marker detection** (`ros2 run aruco_ros` or OpenCV's `cv2.aruco`) is a well-trodden, dependency-light way to get station-arrival ground truth working quickly, then swap in YOLOv8 once the state-machine integration is proven.

### Why reuse instead of training your own detector immediately
- YOLOv8 pre-trained on COCO gets you *something* working with zero training data; you can fine-tune later once you know your target object's exact appearance.
- Isaac ROS's detection nodes are GPU-accelerated and designed to be dropped directly into a ROS 2 graph (`isaac_ros_examples.launch.py launch_fragments:=...`) — you avoid hand-writing TensorRT/ONNX inference glue code.

### Steps
1. Add a camera (Isaac Sim synthetic camera, or real RGB/RGB-D) publishing to a ROS 2 image topic, either overhead or per-station.
2. Stand up `isaac_ros_yolov8` (or ArUco, if starting simple) against that camera topic.
3. Add a small `perception` node that converts detections + known station geometry into `OBJECT_AT_A` / `OBJECT_AT_B` / etc. state updates, publishing them to `task_manager`.
4. Replace the hard-coded state assumptions in `task_manager` from Phase 4 with subscriptions to this perception output.

### Exit criteria
State transitions in `task_manager` (`OBJECT_AT_A`, `OBJECT_AT_B`, ...) are driven by real camera detections, not assumed/hard-coded.

---

## Phase 6 — Dynamic multi-arm coordination

**Goal:** handle multiple objects in flight simultaneously, with collision/workspace-overlap avoidance and failure recovery.

### Repos/tools to reuse
- **MoveIt 2's shared `PlanningSceneMonitor` / a common planning scene across the three `move_group` instances** — rather than building your own collision-checking layer, feed each robot's own workspace occupancy (or a simple interlock zone) into a shared planning scene so MoveIt's own collision checker catches overlap.
- **Isaac ROS "Manipulation Orchestration" behavior-tree framework** — explicitly built for this: parallel operations, retries, and error handling for multi-object pick-and-place. If you adopted `py_trees_ros`/behavior trees in Phase 4, this is a natural extension rather than a rewrite.
- If true concurrent motion planning across arms proves hard to coordinate safely, a simpler and very common interim pattern: **mutex/interlock zones** — before commanding a robot into a shared boundary region (e.g. near Station B, reachable by both robot1 and robot2), have `task_manager` grant exclusive access via a simple lock, so only one arm is ever moving through a shared zone at a time. This is far simpler than full dynamic collision avoidance and is a reasonable fallback if the full solution slips.

### Why reuse
- MoveIt's collision checking is already solid; the main new work is architectural (multiple objects/state tracked concurrently), not re-deriving collision geometry math.
- Behavior trees handle "retry this branch, run these two in parallel, but abort down this branch on failure" natively — implementing that logic in a flat state machine gets unwieldy fast.

### Steps
1. Extend `task_manager` (or its behavior tree) to track N independent objects, each with their own state.
2. Add interlock/shared-planning-scene collision protection at the A/B and B/C/D handoff boundaries.
3. Add explicit failure/retry paths: dropped object detection, blocked station (occupied when a new object arrives), pick failure with retry-N-times-then-fail.
4. Load-test with 2+ objects in the pipeline concurrently; watch for deadlocks (e.g. robot2 waiting on a station robot3 hasn't cleared yet).

### Exit criteria
System handles at least 2 objects in flight simultaneously with no collisions and no deadlocks, and recovers from at least one injected failure (e.g. forced pick failure) without hanging.

---

## Phase 7 — Physical AI / Cosmos experiments

**Goal:** open-ended R&D — define this concretely before starting.

This phase is intentionally not prescriptive in your notes ("Physical AI / Cosmos experiments"), and open-ended research phases sprawl badly without a target metric. Before starting, pick **one** concrete direction and a success metric, for example:

- **Sim-to-Sim transfer**: train/validate a perception or grasp policy purely in Isaac Sim, then measure success rate across variations of scene lighting or noise.
- **NVIDIA Cosmos world-foundation models** for synthetic data generation or policy pretraining — Cosmos is NVIDIA's generative world-model line, positioned for generating physically-plausible synthetic training data/video for robotics. If this is the direction, scope the first experiment narrowly (e.g. "does Cosmos-generated synthetic video improve YOLOv8's detection accuracy on Station D vs. Isaac-Sim-only synthetic data") rather than an open "explore Cosmos" task.
- **Learned coordination policy**: replace the hand-written state machine/behavior tree from Phase 6 with a learned multi-agent policy, benchmarked against the Phase 6 baseline's throughput/collision rate.

Because this phase is genuinely research-shaped, I'd suggest scoping it as its own planning exercise once Phases 1–6 are solid — happy to help draft that scope when you get there.

---

## Practical sequencing notes

- **Namespace everything from Phase 1**, even with a single robot — retrofitting `/robot1/...` prefixes onto a Phase 1–2 codebase that assumed a single unnamespaced robot is a bigger refactor than doing it up front.
- **Get Phase 1–2 solid using mock hardware before touching Isaac Sim.** Debugging MoveIt 2 + Isaac Sim + multi-arm simultaneously, with no known-good baseline, makes it hard to tell which layer a bug is in.
- **The station abstraction (image 1 in your notes) is worth implementing as a loaded YAML config from Phase 2**, not hard-coded constants — cheap now, expensive to retrofit once Phase 4/5/6 all depend on station names.
- Known gotcha worth internalizing before Phase 3: **one `move_group` cannot execute two trajectories concurrently** — this is exactly why you need one `move_group` per robot in Phase 3, not one shared `move_group` for all three arms.

## Reference repos (summary)

| Purpose | Repo |
|---|---|
| Official UR ROS 2 driver | `UniversalRobots/Universal_Robots_ROS2_Driver` |
| Official UR URDF/description | `UniversalRobots/Universal_Robots_ROS2_Description` |
| UR10e + Isaac Sim + ROS 2 bridge reference | `ainhoaarnaiz/ur10e_isaac_sim_ros2` |
| Multi-arm namespacing/MoveIt pattern (Gazebo) | `arshadlab/multi_robot_arm` + `arshadlab/pymoveit2` |
| Dual/multi-arm MoveIt config checklist | `moveit/moveit2_tutorials` — "Dual Arms with MoveIt" |
| GPU object detection (YOLOv8/RT-DETR/DetectNet) | `NVIDIA-ISAAC-ROS/isaac_ros_object_detection` |
| Full perception-driven pick-and-place reference | NVIDIA "Isaac for Manipulation" workflow |

All links reflect actively maintained repos as of September 2026 — check each repo's README for the branch matching your ROS 2 distro before cloning.










# Multi-Arm UR10e Pick-and-Place Task List

- `[/]` **Phase 1 — One UR10e + MoveIt 2**
  - `[x]` Clone `UniversalRobots/Universal_Robots_ROS2_Driver` into `src/` (branch `humble`)
  - `[x]` Verify `UniversalRobots/Universal_Robots_ROS2_Description` is in `src/` (branch `humble`)
  - `[/]` Run `rosdep install` and `colcon build`
  - `[ ]` Test with URSim and RViz (verify robot reaches arbitrary poses)
- `[ ]` **Phase 2 — One UR10e + pick/place**
  - `[ ]` Define `stations.yaml` in `multi_arm_bringup/config/`
  - `[ ]` Write a `pick_place` action server
  - `[ ]` Wire gripper open/close service call
  - `[ ]` Test pick/place sequence repeatedly
- `[ ]` **Phase 3 — Three UR10e in Isaac Sim**
  - `[ ]` Import UR10e into Isaac Sim
  - `[ ]` Duplicate prim two times with unique namespaces (`/robot1`, `/robot2`, `/robot3`)
  - `[ ]` Create `robot1_moveit_config`, `robot2_moveit_config`, `robot3_moveit_config` with correct namespacing
  - `[ ]` Launch one `move_group` per robot from `multi_arm_bringup`
  - `[ ]` Validate independent planning and execution
- `[ ]` **Phase 4 — A→B→C→D sequential coordination**
  - `[ ]` Build state machine / behavior tree
  - `[ ]` Create `task_manager` node
  - `[ ]` Implement failure handling
  - `[ ]` Verify object traverses A→B→C→D
- `[ ]` **Phase 5 — Camera + object detection**
  - `[ ]` Add camera topic
  - `[ ]` Integrate `isaac_ros_yolov8` or ArUco detection
  - `[ ]` Create `perception` node to publish state updates
  - `[ ]` Update `task_manager` to use perception output
- `[ ]` **Phase 6 — Dynamic multi-arm coordination**
  - `[ ]` Extend `task_manager` to track multiple objects
  - `[ ]` Add interlock/shared-planning-scene collision protection
  - `[ ]` Add explicit failure/retry paths
  - `[ ]` Load-test with multiple objects concurrently
- `[ ]` **Phase 7 — Physical AI / Cosmos experiments**
  - `[ ]` Define R&D scope
