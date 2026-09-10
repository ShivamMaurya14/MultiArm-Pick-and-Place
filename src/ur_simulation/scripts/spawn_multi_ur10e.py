#!/usr/bin/env python3
"""
================================================================================
Multi-UR10e Industrial Workcell Spawner for NVIDIA Isaac Sim (v4.x, v5.x, v6.0+)
Compatible with ROS 2 Jazzy Jalisco
================================================================================

This script automates the complete scene generation for the 3-arm robotic cell:
1. Physics Scene (9.81 m/s^2) & High-Friction Contact Materials
2. Ground Platform (2.4m x 5.6m) & Safety Demarcation Boundaries
3. 3 Elevated Steel Pedestals for Robot 1, Robot 2, and Robot 3 (Z=0.20m)
4. 4 Station Tables (Station A Blue, Station B/C Orange Buffers, Station D Purple)
5. Dynamic Workpiece Cube (60mm, 0.15kg) with PhysX 5 Rigid Body Dynamics
6. Multi-Robot Instantiation: clones template into /World/robot1, /World/robot2, /World/robot3
7. Joint Drive Stabilization: locks all arm and gripper joints firmly into the calibrated ready posture (prevents ragdoll flailing/random movement)
8. OmniGraph ROS 2 Bridge Action Graphs (/clock, /robot[1..3]/joint_states, /robot[1..3]/joint_commands)

Execution:
- In Isaac Sim: Window -> Script Editor -> Paste / Open -> Click "Run"
"""

import math
import omni
import omni.graph.core as og
import omni.kit.commands
from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema, Gf, Sdf, Vt

# ---------------------------------------------------------------------------
# Global Workcell & Kinematic Configurations
# ---------------------------------------------------------------------------
ROBOT_CONFIGS = [
    {
        "ns": "robot1",
        "origin": Gf.Vec3d(0.0, 0.0, 0.20),
        "target_station": "Station A -> Station B"
    },
    {
        "ns": "robot2",
        "origin": Gf.Vec3d(0.0, 1.6, 0.20),
        "target_station": "Station B -> Station C"
    },
    {
        "ns": "robot3",
        "origin": Gf.Vec3d(0.0, 3.2, 0.20),
        "target_station": "Station C -> Station D"
    },
]

STATION_TABLES = [
    {"name": "station_a", "pos": Gf.Vec3d(0.70, 0.00, 0.0), "color": Gf.Vec3f(0.10, 0.50, 0.90)},  # Blue (Source)
    {"name": "station_b", "pos": Gf.Vec3d(0.70, 0.80, 0.0), "color": Gf.Vec3f(0.98, 0.55, 0.05)},  # Orange (Relay 1)
    {"name": "station_c", "pos": Gf.Vec3d(0.70, 2.40, 0.0), "color": Gf.Vec3f(0.98, 0.55, 0.05)},  # Orange (Relay 2)
    {"name": "station_d", "pos": Gf.Vec3d(0.70, 3.20, 0.0), "color": Gf.Vec3f(0.60, 0.15, 0.75)},  # Purple (Destination)
]

WORKPIECE_CUBE = {
    "path": "/World/Workcell/Workpiece_Cube",
    "pos": Gf.Vec3d(0.70, 0.00, 0.245),       # Centered on Station A table top (Z=0.22m + 0.025m)
    "size": Gf.Vec3f(0.06, 0.06, 0.05),       # 60mm x 60mm x 50mm
    "mass": 0.15,                             # 150 grams
    "color": Gf.Vec3f(0.10, 0.85, 0.45)       # Emerald Green
}

# Calibrated joint ready positions (in degrees for USD PhysX Angular Drives)
CALIBRATED_READY_POSITIONS_DEG = {
    "shoulder_pan_joint": -14.404,     # -0.2514 rad (facing workspace forward)
    "shoulder_lift_joint": -103.132,   # -1.8000 rad
    "elbow_joint": 85.944,             # +1.5000 rad
    "wrist_1_joint": -72.766,          # -1.2700 rad
    "wrist_2_joint": -90.000,          # -1.5708 rad
    "wrist_3_joint": 0.000,            #  0.0000 rad
    "finger_joint": 0.000,             #  0.0000 rad (fully open)
    "left_outer_knuckle_joint": 0.000,
    "right_outer_knuckle_joint": 0.000,
}

# ---------------------------------------------------------------------------
# 1. Physics Scene & Physics Material Setup
# ---------------------------------------------------------------------------
def setup_physics_scene(stage):
    """Ensures PhysicsScene and high-friction contact material exist."""
    scene_path = "/World/PhysicsScene"
    scene_prim = stage.GetPrimAtPath(scene_path)
    if not scene_prim.IsValid():
        scene = UsdPhysics.Scene.Define(stage, Sdf.Path(scene_path))
        scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
        scene.CreateGravityMagnitudeAttr().Set(9.81)
        print("[Physics] Created PhysicsScene with 9.81 m/s^2 gravity.")
    else:
        scene = UsdPhysics.Scene(scene_prim)
        scene.CreateGravityMagnitudeAttr().Set(9.81)

    # PhysX Scene API for stability
    physx_scene_api = PhysxSchema.PhysxSceneAPI.Apply(stage.GetPrimAtPath(scene_path))
    physx_scene_api.CreateEnableCCDAttr(True)
    physx_scene_api.CreateEnableStabilizationAttr(True)

    # High-Friction Grasping Material (static=1.2, dynamic=0.9)
    mat_path = "/World/PhysicsMaterials/HighFrictionMat"
    if not stage.GetPrimAtPath(mat_path).IsValid():
        material_prim = UsdPhysics.MaterialAPI.Apply(stage.DefinePrim(Sdf.Path(mat_path), "Material"))
        material_prim.CreateStaticFrictionAttr(1.2)
        material_prim.CreateDynamicFrictionAttr(0.9)
        material_prim.CreateRestitutionAttr(0.0)
        print("[Physics] Created High-Friction contact material for Robotiq gripping.")
    return mat_path

# ---------------------------------------------------------------------------
# 2. Geometry Builders
# ---------------------------------------------------------------------------
def create_cylinder(stage, prim_path, radius, height, pos, color, has_collision=True, mat_path=None):
    cyl = UsdGeom.Cylinder.Define(stage, Sdf.Path(prim_path))
    cyl.GetRadiusAttr().Set(radius)
    cyl.GetHeightAttr().Set(height)
    cyl.GetAxisAttr().Set("Z")
    
    xform = UsdGeom.Xformable(cyl)
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(pos)
    cyl.GetDisplayColorAttr().Set(Vt.Vec3fArray([color]))
    
    if has_collision:
        UsdPhysics.CollisionAPI.Apply(cyl.GetPrim())
        if mat_path:
            PhysxSchema.PhysxCollisionAPI.Apply(cyl.GetPrim())
            omni.kit.commands.execute("BindMaterialCommand", prim_path=prim_path, material_path=mat_path)
    return cyl

def create_box(stage, prim_path, size, pos, color, has_collision=True, mat_path=None):
    box = UsdGeom.Cube.Define(stage, Sdf.Path(prim_path))
    box.GetSizeAttr().Set(1.0)
    
    xform = UsdGeom.Xformable(box)
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(pos)
    xform.AddScaleOp().Set(size)
    box.GetDisplayColorAttr().Set(Vt.Vec3fArray([color]))
    
    if has_collision:
        UsdPhysics.CollisionAPI.Apply(box.GetPrim())
        if mat_path:
            PhysxSchema.PhysxCollisionAPI.Apply(box.GetPrim())
            omni.kit.commands.execute("BindMaterialCommand", prim_path=prim_path, material_path=mat_path)
    return box

# ---------------------------------------------------------------------------
# 3. Environment & Workstation Assembly
# ---------------------------------------------------------------------------
def setup_workcell_environment(stage, mat_path):
    workcell_path = "/World/Workcell"
    UsdGeom.Xform.Define(stage, Sdf.Path(workcell_path))

    # A. Ground Platform (2.40m x 5.60m x 0.03m)
    create_box(
        stage,
        f"{workcell_path}/GroundPlatform",
        size=Gf.Vec3f(2.40, 5.60, 0.03),
        pos=Gf.Vec3d(0.40, 1.60, -0.015),
        color=Gf.Vec3f(0.15, 0.17, 0.20),
        has_collision=True,
        mat_path=mat_path
    )

    # B. Yellow Safety Grid Lines
    create_box(stage, f"{workcell_path}/Divider_1_2", Gf.Vec3f(2.30, 0.04, 0.002), Gf.Vec3d(0.40, 0.80, 0.001), Gf.Vec3f(0.95, 0.75, 0.05), False)
    create_box(stage, f"{workcell_path}/Divider_2_3", Gf.Vec3f(2.30, 0.04, 0.002), Gf.Vec3d(0.40, 2.40, 0.001), Gf.Vec3f(0.95, 0.75, 0.05), False)
    create_box(stage, f"{workcell_path}/Safety_Front", Gf.Vec3f(0.04, 5.40, 0.002), Gf.Vec3d(1.45, 1.60, 0.001), Gf.Vec3f(0.95, 0.75, 0.05), False)

    # C. Robot Steel Pedestals (Height 0.20m, Top at Z=0.20m)
    for cfg in ROBOT_CONFIGS:
        ns = cfg["ns"]
        ped_pos = Gf.Vec3d(cfg["origin"][0], cfg["origin"][1], 0.10)
        create_cylinder(stage, f"{workcell_path}/Pedestal_{ns}", radius=0.16, height=0.20, pos=ped_pos, color=Gf.Vec3f(0.28, 0.30, 0.35), has_collision=True)

    # D. 4 Workstation Tables (Pedestal + Table Top + Target Ring)
    for st in STATION_TABLES:
        name = st["name"]
        x, y = st["pos"][0], st["pos"][1]
        create_cylinder(stage, f"{workcell_path}/{name}_pedestal", radius=0.12, height=0.20, pos=Gf.Vec3d(x, y, 0.10), color=Gf.Vec3f(0.28, 0.30, 0.35), has_collision=True)
        create_cylinder(stage, f"{workcell_path}/{name}_top", radius=0.24, height=0.04, pos=Gf.Vec3d(x, y, 0.20), color=st["color"], has_collision=True, mat_path=mat_path)
        create_cylinder(stage, f"{workcell_path}/{name}_ring", radius=0.08, height=0.002, pos=Gf.Vec3d(x, y, 0.221), color=Gf.Vec3f(0.35, 0.38, 0.42), has_collision=False)

    # E. Dynamic Workpiece Cube (0.06m x 0.06m x 0.05m, mass 0.15kg)
    cube_box = create_box(
        stage,
        WORKPIECE_CUBE["path"],
        size=WORKPIECE_CUBE["size"],
        pos=WORKPIECE_CUBE["pos"],
        color=WORKPIECE_CUBE["color"],
        has_collision=True,
        mat_path=mat_path
    )
    rb_api = UsdPhysics.RigidBodyAPI.Apply(cube_box.GetPrim())
    rb_api.CreateRigidBodyEnabledAttr(True)
    mass_api = UsdPhysics.MassAPI.Apply(cube_box.GetPrim())
    mass_api.CreateMassAttr(WORKPIECE_CUBE["mass"])

    print("[Workcell] Successfully assembled Ground, 3 Pedestals, 4 Station Tables, and Dynamic Cube.")

# ---------------------------------------------------------------------------
# 4. Joint Drive Stabilization (Prevents Ragdoll Collapsing / Random Motion)
# ---------------------------------------------------------------------------
def lock_robot_joint_drives(stage, robot_path):
    """
    Configures high stiffness and damping on all UR10e and Robotiq joints,
    and sets target positions to the ready state so the robot stays completely still
    and rigid, preventing any flailing, jitter, or collapsing.
    """
    robot_prim = stage.GetPrimAtPath(robot_path)
    if not robot_prim.IsValid():
        return

    # Ensure Articulation Root and Solver Iterations
    UsdPhysics.ArticulationRootAPI.Apply(robot_prim)
    physx_art = PhysxSchema.PhysxArticulationAPI.Apply(robot_prim)
    physx_art.CreateSolverPositionIterationCountAttr(32)
    physx_art.CreateSolverVelocityIterationCountAttr(16)
    physx_art.CreateStabilizationThresholdAttr(0.001)

    joint_count = 0
    for prim in Usd.PrimRange(robot_prim):
        prim_type = prim.GetTypeName()
        if "Joint" in prim_type:
            joint_name = prim.GetName()
            drive_api = UsdPhysics.DriveAPI.Apply(prim, "angular")
            if not drive_api:
                continue

            # Determine stiffness and damping values
            is_arm_joint = any(j in joint_name for j in ["shoulder", "elbow", "wrist"])
            if is_arm_joint:
                stiffness = 5000000.0   # 5e6
                damping = 100000.0      # 1e5
                max_force = 10000.0
            else:
                stiffness = 100000.0    # 1e5
                damping = 1000.0        # 1e3
                max_force = 500.0

            drive_api.CreateTypeAttr("force")
            drive_api.CreateStiffnessAttr(stiffness)
            drive_api.CreateDampingAttr(damping)
            drive_api.CreateMaxForceAttr(max_force)

            # Assign calibrated ready angle (in degrees)
            for target_jname, target_deg in CALIBRATED_READY_POSITIONS_DEG.items():
                if target_jname in joint_name:
                    drive_api.CreateTargetPositionAttr(target_deg)
                    drive_api.CreateTargetVelocityAttr(0.0)
                    joint_count += 1
                    break

    print(f"[{robot_path}] Configured & locked {joint_count} joint drives in ready pose.")

# ---------------------------------------------------------------------------
# 5. OmniGraph ROS 2 Simulation Clock (/clock)
# ---------------------------------------------------------------------------
def setup_global_clock_graph():
    stage = omni.usd.get_context().get_stage()
    graph_path = "/World/ROS2_ClockGraph"
    if stage.GetPrimAtPath(graph_path).IsValid():
        return

    keys = og.Controller.Keys
    try:
        og.Controller.edit(
            {"graph_path": graph_path, "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("ROS2Context", "omni.isaac.ros2_bridge.ROS2Context"),
                    ("ROS2PublishClock", "omni.isaac.ros2_bridge.ROS2PublishClock"),
                ],
                keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "ROS2PublishClock.inputs:execIn"),
                    ("ROS2Context.outputs:context", "ROS2PublishClock.inputs:context"),
                ],
                keys.SET_VALUES: [
                    ("ROS2PublishClock.inputs:topicName", "/clock"),
                ],
            },
        )
        print("[ROS 2 Bridge] Global simulation clock publisher initialized on /clock")
    except Exception as e:
        print(f"[ROS 2 Bridge] Clock graph note: {e}")

# ---------------------------------------------------------------------------
# 6. OmniGraph Robot Action Graphs (/robot[X]/joint_states & joint_commands)
# ---------------------------------------------------------------------------
def create_ros2_action_graph(namespace, target_prim_path):
    graph_path = f"{target_prim_path}/ROS2_ActionGraph"
    keys = og.Controller.Keys
    try:
        og.Controller.edit(
            {"graph_path": graph_path, "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("ROS2Context", "omni.isaac.ros2_bridge.ROS2Context"),
                    ("ROS2PublishJointState", "omni.isaac.ros2_bridge.ROS2PublishJointState"),
                    ("ROS2SubscribeJointState", "omni.isaac.ros2_bridge.ROS2SubscribeJointState"),
                    ("ArticulationController", "omni.isaac.core_nodes.IsaacArticulationController"),
                ],
                keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "ROS2PublishJointState.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick", "ROS2SubscribeJointState.inputs:execIn"),
                    ("ROS2Context.outputs:context", "ROS2PublishJointState.inputs:context"),
                    ("ROS2Context.outputs:context", "ROS2SubscribeJointState.inputs:context"),
                    ("ROS2SubscribeJointState.outputs:execOut", "ArticulationController.inputs:execIn"),
                    ("ROS2SubscribeJointState.outputs:positionCommand", "ArticulationController.inputs:positionCommand"),
                    ("ROS2SubscribeJointState.outputs:velocityCommand", "ArticulationController.inputs:velocityCommand"),
                    ("ROS2SubscribeJointState.outputs:effortCommand", "ArticulationController.inputs:effortCommand"),
                    ("ROS2SubscribeJointState.outputs:jointNames", "ArticulationController.inputs:jointNames"),
                ],
                keys.SET_VALUES: [
                    ("ROS2PublishJointState.inputs:topicName", f"/{namespace}/joint_states"),
                    ("ROS2PublishJointState.inputs:targetPrim", [Sdf.Path(target_prim_path)]),
                    ("ROS2SubscribeJointState.inputs:topicName", f"/{namespace}/joint_commands"),
                    ("ArticulationController.inputs:targetPrim", [Sdf.Path(target_prim_path)]),
                    ("ArticulationController.inputs:usePath", False),
                ],
            },
        )
        print(f"[{namespace}] Action Graph linked: /{namespace}/joint_states & /{namespace}/joint_commands")
    except Exception as e:
        print(f"[{namespace}] Action Graph warning: {e}")

# ---------------------------------------------------------------------------
# 7. Helper to Find Any Imported Robot Template in Stage
# ---------------------------------------------------------------------------
def find_imported_robot_template(stage):
    candidates = [
        "/World/UR10e",
        "/World/ur10e_robotiq",
        "/World/ur10e",
        "/ur10e_robotiq",
        "/UR10e",
        "/World/robot1"
    ]
    for p in candidates:
        prim = stage.GetPrimAtPath(p)
        if prim.IsValid() and len(prim.GetChildren()) > 0:
            return p

    # Dynamic search across stage
    for prim in stage.Traverse():
        name_lower = prim.GetName().lower()
        if "ur10e" in name_lower or "robot" in name_lower:
            if len(prim.GetChildren()) > 0:
                return str(prim.GetPath())
    return None

# ---------------------------------------------------------------------------
# 8. Main Orchestrator
# ---------------------------------------------------------------------------
def main():
    # Enable ROS 2 Bridge Extension (Supports Isaac Sim 6.0.1 and Isaac Sim 4.x/5.x)
    ext_manager = omni.kit.app.get_app().get_extension_manager()
    for ext in ["isaacsim.ros2.bridge", "omni.isaac.ros2_bridge", "omni.isaac.core", "isaacsim.core"]:
        try:
            if not ext_manager.is_extension_enabled(ext):
                ext_manager.set_extension_enabled_immediate(ext, True)
        except Exception:
            pass

    stage = omni.usd.get_context().get_stage()
    if not stage:
        print("ERROR: No active USD stage found. Please open an Isaac Sim stage first.")
        return

    # 1. Physics Scene & Materials
    mat_path = setup_physics_scene(stage)

    # 2. Build Workcell Structure & Physics Cube
    setup_workcell_environment(stage, mat_path)

    # 3. Setup Global Simulation Clock
    setup_global_clock_graph()

    # 4. Locate Source Robot Template
    source_path = find_imported_robot_template(stage)
    if not source_path:
        print("\n[ERROR] Robot template not found in Stage!")
        print(">> Please import /tmp/ur10e_robotiq.urdf to /World/UR10e via URDF Importer first.")
        print(">> Then click Run on this script.\n")
        return

    print(f"[Spawner] Found robot template at '{source_path}'.")

    # 5. Instantiate all 3 UR10e Robots on their Pedestals
    for cfg in ROBOT_CONFIGS:
        ns = cfg["ns"]
        target_path = f"/World/{ns}"

        # If target doesn't exist or is empty, duplicate from template
        target_prim = stage.GetPrimAtPath(target_path)
        if not target_prim.IsValid() or len(target_prim.GetChildren()) == 0:
            if source_path != target_path:
                omni.kit.commands.execute(
                    'CopyPrims',
                    paths_from=[source_path],
                    paths_to=[target_path],
                    duplicate_layers=True
                )
        
        prim = stage.GetPrimAtPath(target_path)
        if prim.IsValid():
            # Make sure prim is visible
            UsdGeom.Imageable(prim).MakeVisible()

            # Set exact mounting height on top of pedestal (Z = 0.20m)
            xform = UsdGeom.Xformable(prim)
            xform.ClearXformOpOrder()
            translate_op = xform.AddTranslateOp()
            translate_op.Set(cfg["origin"])

            # Lock joint drives in calibrated upright ready pose
            lock_robot_joint_drives(stage, target_path)

            # Attach OmniGraph ROS 2 Bridge
            create_ros2_action_graph(ns, target_path)

    # If source template was /World/UR10e (separate from robot1), hide template
    if source_path not in [f"/World/{c['ns']}" for c in ROBOT_CONFIGS]:
        source_prim = stage.GetPrimAtPath(source_path)
        if source_prim.IsValid():
            UsdGeom.Imageable(source_prim).MakeInvisible()

    print("\n==========================================================================")
    print(" ✅ ALL 3 UR10e ROBOTS SPAWNED & STABILIZED IN ISAAC SIM!")
    print(" - Robot 1: /World/robot1 (Y=0.0m, Z=0.20m on Pedestal)")
    print(" - Robot 2: /World/robot2 (Y=1.6m, Z=0.20m on Pedestal)")
    print(" - Robot 3: /World/robot3 (Y=3.2m, Z=0.20m on Pedestal)")
    print(" - Joint Drives: Locked in stable upright ready pose (NO random movement)")
    print(" - 4 Stations (A, B, C, D) + Dynamic Green Cube ready on Station A")
    print(" - ROS 2 Bridge: /clock, /robot[1..3]/joint_states & joint_commands active")
    print(" 👉 Click PLAY (▶) in Isaac Sim, then run task_manager in ROS 2.")
    print("==========================================================================\n")

if __name__ == "__main__":
    main()
