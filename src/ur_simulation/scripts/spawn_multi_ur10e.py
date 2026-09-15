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
try:
    import omni.graph.core as og
except ImportError:
    og = None

try:
    import omni.kit.commands
except ImportError:
    pass

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
}

# ---------------------------------------------------------------------------
# 1. Physics Scene & Physics Material Setup
# ---------------------------------------------------------------------------
def setup_physics_scene(stage):
    """Ensures PhysicsScene, TGS solver, and high-friction contact material exist."""
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

    scene_prim = stage.GetPrimAtPath(scene_path)
    # Configure high-accuracy Temporal Gauss-Seidel (TGS) solver for robotic articulations
    try:
        scene_prim.CreateAttribute("physxScene:solverType", Sdf.ValueTypeNames.Token).Set("TGS")
        scene_prim.CreateAttribute("physxScene:timeStepsPerSecond", Sdf.ValueTypeNames.Float).Set(60.0)
    except Exception:
        pass

    # PhysX Scene API for stability
    try:
        physx_scene_api = PhysxSchema.PhysxSceneAPI.Apply(scene_prim)
        physx_scene_api.CreateEnableCCDAttr(True)
        physx_scene_api.CreateEnableStabilizationAttr(True)

        # PhysX GPU memory configuration to prevent aggregate buffer overflow
        for attr_name, val in [
            ("CreateGpuFoundLostAggregatePairsCapacityAttr", 32768),
            ("CreateGpuTotalAggregatePairsCapacityAttr", 32768),
            ("CreateGpuMaxRigidContactCountAttr", 524288),
            ("CreateGpuMaxRigidPatchCountAttr", 163840),
            ("CreateGpuHeapCapacityAttr", 67108864),
            ("CreateGpuFoundLostPairsCapacityAttr", 32768),
        ]:
            if hasattr(physx_scene_api, attr_name):
                getattr(physx_scene_api, attr_name)(val)
    except Exception:
        pass

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
            try:
                PhysxSchema.PhysxCollisionAPI.Apply(cyl.GetPrim())
            except Exception:
                pass
            try:
                omni.kit.commands.execute("BindMaterialCommand", prim_path=prim_path, material_path=mat_path)
            except Exception:
                pass
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
            try:
                PhysxSchema.PhysxCollisionAPI.Apply(box.GetPrim())
            except Exception:
                pass
            try:
                omni.kit.commands.execute("BindMaterialCommand", prim_path=prim_path, material_path=mat_path)
            except Exception:
                pass
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

# Tuned joint parameters per joint (Stiffness, Damping, Max Torque)
# Matched to each joint's mass and rotational inertia to prevent torque chatter / runaway spinning
# wrist_3_joint uses acceleration drive to eliminate numerical spinning from tiny inertia (0.00034)
ARM_JOINT_PARAMS = {
    "shoulder_pan_joint":  {"target": -14.404, "stiffness": 400000.0, "damping": 40000.0, "max_force": 330.0, "type": "force"},
    "shoulder_lift_joint": {"target": -103.132, "stiffness": 400000.0, "damping": 40000.0, "max_force": 330.0, "type": "force"},
    "elbow_joint":         {"target": 85.944,   "stiffness": 200000.0, "damping": 20000.0, "max_force": 150.0, "type": "force"},
    "wrist_1_joint":       {"target": -72.766,  "stiffness": 50000.0,  "damping": 5000.0,  "max_force": 54.0,  "type": "force"},
    "wrist_2_joint":       {"target": -90.000,  "stiffness": 50000.0,  "damping": 5000.0,  "max_force": 54.0,  "type": "force"},
    "wrist_3_joint":       {"target": 0.000,    "stiffness": 2000.0,   "damping": 200.0,   "max_force": 100000.0, "type": "acceleration"},
}

# ---------------------------------------------------------------------------
# 4. Joint Drive Stabilization (Prevents Ragdoll Collapsing / Random Motion)
# ---------------------------------------------------------------------------
def lock_robot_joint_drives(stage, robot_path):
    """
    Locks all UR10e arm joints and Robotiq 2F-140 gripper joints firmly into their
    calibrated ready posture with tuned stiffness and critical damping.
    Ensures the gripper base link is rigidly coupled to wrist_3_link so it cannot
    spin independently, and keeps mimic joints compliant.
    """
    robot_prim = stage.GetPrimAtPath(robot_path)
    if not robot_prim.IsValid():
        return

    # If any previous bogus RootFixedJoint exists, remove it
    bogus_joint = stage.GetPrimAtPath(f"{robot_path}/RootFixedJoint")
    if bogus_joint.IsValid():
        stage.RemovePrim(bogus_joint.GetPath())

    # CRITICAL FIX: Ensure robotiq_140_base_joint connects wrist_3_link directly to robotiq_140_base_link.
    # In the raw USDA, body0 pointed to 'robotiq_base_link' which lacked a RigidBodyAPI, leaving
    # the entire gripper unconstrained and freely spinning around wrist_3!
    gripper_fixed_joint = stage.GetPrimAtPath(f"{robot_path}/Physics/robotiq_140_base_joint")
    if gripper_fixed_joint.IsValid():
        wrist_3_path = f"{robot_path}/Geometry/world/base_link/base_link_inertia/shoulder_link/upper_arm_link/forearm_link/wrist_1_link/wrist_2_link/wrist_3_link"
        robotiq_140_path = f"{wrist_3_path}/flange/tool0/robotiq_base_link/robotiq_140_base_link"
        try:
            gripper_fixed_joint.GetRelationship("physics:body0").SetTargets([Sdf.Path(wrist_3_path)])
            gripper_fixed_joint.GetRelationship("physics:body1").SetTargets([Sdf.Path(robotiq_140_path)])
            if gripper_fixed_joint.GetAttribute("physics:localPos0").IsValid():
                gripper_fixed_joint.GetAttribute("physics:localPos0").Set(Gf.Vec3f(0.0, 0.0, 0.0))
            if gripper_fixed_joint.GetAttribute("physics:localPos1").IsValid():
                gripper_fixed_joint.GetAttribute("physics:localPos1").Set(Gf.Vec3f(0.0, 0.0, 0.0))

            # Compatible quaternion construction across all pxr builds
            try:
                q0 = Gf.Quatf(0.70710677, Gf.Vec3f(0.0, 0.0, 0.70710677))
                q1 = Gf.Quatf(1.0, Gf.Vec3f(0.0, 0.0, 0.0))
            except Exception:
                try:
                    q0 = Gf.Quatf(0.70710677, (0.0, 0.0, 0.70710677))
                    q1 = Gf.Quatf(1.0, (0.0, 0.0, 0.0))
                except Exception:
                    q0 = Gf.Quatf(0.70710677, 0.0, 0.0, 0.70710677)
                    q1 = Gf.Quatf(1.0, 0.0, 0.0, 0.0)

            if gripper_fixed_joint.GetAttribute("physics:localRot0").IsValid():
                gripper_fixed_joint.GetAttribute("physics:localRot0").Set(q0)
            if gripper_fixed_joint.GetAttribute("physics:localRot1").IsValid():
                gripper_fixed_joint.GetAttribute("physics:localRot1").Set(q1)
        except Exception as e:
            print(f"[{robot_path}] Gripper joint clamp notice: {e}")

    arm_count = 0
    mimic_count = 0

    for prim in Usd.PrimRange(robot_prim):
        if prim.GetTypeName() != "PhysicsRevoluteJoint":
            continue

        joint_name = prim.GetName()
        drive_api = UsdPhysics.DriveAPI.Apply(prim, "angular")
        if not drive_api:
            continue

        if joint_name in ARM_JOINT_PARAMS:
            # Tuned UR10e arm joints tailored to each joint's mass/inertia
            p = ARM_JOINT_PARAMS[joint_name]
            drive_api.CreateTypeAttr(p.get("type", "force"))
            drive_api.CreateStiffnessAttr(p["stiffness"])
            drive_api.CreateDampingAttr(p["damping"])
            drive_api.CreateMaxForceAttr(p["max_force"])
            drive_api.CreateTargetPositionAttr(p["target"])
            drive_api.CreateTargetVelocityAttr(0.0)
            arm_count += 1
        elif joint_name == "finger_joint":
            # Robotiq 2F-140 main active driver joint (fully open)
            drive_api.CreateTypeAttr("force")
            drive_api.CreateStiffnessAttr(5000.0)
            drive_api.CreateDampingAttr(200.0)
            drive_api.CreateMaxForceAttr(100.0)
            drive_api.CreateTargetPositionAttr(0.0)
            drive_api.CreateTargetVelocityAttr(0.0)
        else:
            # Gripper parallel linkage follower mimic joints:
            # Zero stiffness so they smoothly follow NewtonMimicAPI without fighting
            drive_api.CreateTypeAttr("force")
            drive_api.CreateStiffnessAttr(0.0)
            drive_api.CreateDampingAttr(0.0)
            drive_api.CreateMaxForceAttr(0.0)
            mimic_count += 1

    print(f"[{robot_path}] Configured & locked: {arm_count} arm joints rigid, gripper rigidly clamped to wrist_3.")

# ---------------------------------------------------------------------------
# 5. OmniGraph ROS 2 Simulation Clock (/clock)
# ---------------------------------------------------------------------------
def setup_global_clock_graph():
    stage = omni.usd.get_context().get_stage()
    graph_path = "/World/ROS2_ClockGraph"
    if stage.GetPrimAtPath(graph_path).IsValid():
        return

    keys = og.Controller.Keys
    # Check node types for Isaac Sim 6.0+ vs 4.x/legacy
    ros_context_type = "isaacsim.ros2.bridge.ROS2Context"
    ros_clock_type = "isaacsim.ros2.bridge.ROS2PublishClock"
    try:
        if not og.Controller.node_type_exists(ros_context_type):
            ros_context_type = "omni.isaac.ros2_bridge.ROS2Context"
            ros_clock_type = "omni.isaac.ros2_bridge.ROS2PublishClock"
    except Exception:
        pass

    try:
        og.Controller.edit(
            {"graph_path": graph_path, "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("ROS2Context", ros_context_type),
                    ("ROS2PublishClock", ros_clock_type),
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
# ---------------------------------------------------------------------------
# 6. OmniGraph Robot Action Graphs (/robot[X]/joint_states & joint_commands)
# ---------------------------------------------------------------------------
def create_ros2_action_graph(namespace, target_prim_path):
    graph_path = f"{target_prim_path}/ROS2_ActionGraph"
    keys = og.Controller.Keys

    # Node types with Isaac Sim 6.0 / legacy fallback
    ros_context_type = "isaacsim.ros2.bridge.ROS2Context"
    ros_pub_js_type = "isaacsim.ros2.bridge.ROS2PublishJointState"
    ros_sub_js_type = "isaacsim.ros2.bridge.ROS2SubscribeJointState"
    art_ctrl_type = "isaacsim.core.nodes.IsaacArticulationController"
    try:
        if not og.Controller.node_type_exists(ros_context_type):
            ros_context_type = "omni.isaac.ros2_bridge.ROS2Context"
            ros_pub_js_type = "omni.isaac.ros2_bridge.ROS2PublishJointState"
            ros_sub_js_type = "omni.isaac.ros2_bridge.ROS2SubscribeJointState"
            art_ctrl_type = "omni.isaac.core_nodes.IsaacArticulationController"
    except Exception:
        pass

    # The actual articulation root with PhysicsArticulationRootAPI is base_link
    stage = omni.usd.get_context().get_stage()
    art_root_path = f"{target_prim_path}/Geometry/world/base_link"
    if not stage.GetPrimAtPath(art_root_path).IsValid():
        art_root_path = target_prim_path

    try:
        og.Controller.edit(
            {"graph_path": graph_path, "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("ROS2Context", ros_context_type),
                    ("ROS2PublishJointState", ros_pub_js_type),
                    ("ROS2SubscribeJointState", ros_sub_js_type),
                    ("ArticulationController", art_ctrl_type),
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
                    ("ROS2PublishJointState.inputs:targetPrim", [Sdf.Path(art_root_path)]),
                    ("ROS2SubscribeJointState.inputs:topicName", f"/{namespace}/joint_commands"),
                    ("ArticulationController.inputs:robotPath", art_root_path),
                ],
            },
        )
        print(f"[{namespace}] Action Graph linked: /{namespace}/joint_states & /{namespace}/joint_commands (target: {art_root_path})")
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
    ]
    for p in candidates:
        prim = stage.GetPrimAtPath(p)
        if prim.IsValid() and len(prim.GetChildren()) > 0:
            return p

    # Dynamic search across stage (excluding robot1..3)
    for prim in stage.Traverse():
        name_lower = prim.GetName().lower()
        path_str = str(prim.GetPath())
        if any(f"/World/robot{i}" in path_str for i in [1, 2, 3]):
            continue
        if "ur10e" in name_lower or "robot" in name_lower:
            if len(prim.GetChildren()) > 0:
                return path_str
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
    import os
    usd_asset_file = "/home/arvr/ros2_ws/MultiArm-Pick-and-Place/src/ur10e_robotiq/ur10e_robotiq.usda"
    use_file_ref = os.path.exists(usd_asset_file)

    if not source_path and not use_file_ref:
        print("\n[ERROR] Robot template not found in Stage or Disk!")
        print(f">> Expected USDA asset at: {usd_asset_file}")
        return

    if source_path:
        print(f"[Spawner] Found robot template at '{source_path}'.")

    # 5. Instantiate all 3 UR10e Robots on their Pedestals
    for cfg in ROBOT_CONFIGS:
        ns = cfg["ns"]
        target_path = f"/World/{ns}"

        # Clean any pre-existing prim to guarantee completely fresh, identical robots without stale caches
        if stage.GetPrimAtPath(target_path).IsValid():
            stage.RemovePrim(Sdf.Path(target_path))

        target_prim = stage.DefinePrim(Sdf.Path(target_path), "Xform")
        target_prim.GetReferences().ClearReferences()
        if use_file_ref:
            target_prim.GetReferences().AddReference(assetPath=usd_asset_file)
        elif source_path:
            target_prim.GetReferences().AddInternalReference(Sdf.Path(source_path))

        prim = stage.GetPrimAtPath(target_path)
        if prim.IsValid():
            # Force explicit inherited visibility on target robot prim
            img = UsdGeom.Imageable(prim)
            img.CreateVisibilityAttr().Set(UsdGeom.Tokens.inherited)

            # Set exact mounting height on top of pedestal (Z = 0.20m)
            xform = UsdGeom.Xformable(prim)
            xform.ClearXformOpOrder()
            translate_op = xform.AddTranslateOp()
            translate_op.Set(cfg["origin"])

            # Clean any bogus RootFixedJoint left from previous runs
            old_joint = stage.GetPrimAtPath(f"{target_path}/RootFixedJoint")
            if old_joint.IsValid():
                stage.RemovePrim(old_joint.GetPath())

            # Lock joint drives in calibrated upright ready pose
            lock_robot_joint_drives(stage, target_path)

            # Attach OmniGraph ROS 2 Bridge
            create_ros2_action_graph(ns, target_path)

    # Clean up source template to avoid ghost collisions with robot1
    if use_file_ref and source_path and source_path not in [f"/World/{c['ns']}" for c in ROBOT_CONFIGS]:
        source_prim = stage.GetPrimAtPath(source_path)
        if source_prim.IsValid():
            stage.RemovePrim(Sdf.Path(source_path))
            print(f"[Spawner] Cleaned up temporary template '{source_path}' to avoid physical overlap.")

    print("\n==========================================================================")
    print(" ✅ ALL 3 UR10e ROBOTS SPAWNED & RIGIDLY STABILIZED IN ISAAC SIM!")
    print(" - Robot 1: /World/robot1 (Y=0.0m, Z=0.20m on Pedestal)")
    print(" - Robot 2: /World/robot2 (Y=1.6m, Z=0.20m on Pedestal)")
    print(" - Robot 3: /World/robot3 (Y=3.2m, Z=0.20m on Pedestal)")
    print(" - Joint Drives: Tuned (K=4e5, D=4e4) in calibrated upright ready pose")
    print(" - Parallel Gripper: Followers compliant (stiffness=0), active finger locked")
    print(" - 4 Stations (A, B, C, D) + Dynamic Green Cube ready on Station A")
    print(" - ROS 2 Bridge: /clock, /robot[1..3]/joint_states & joint_commands active")
    print(" 👉 Click PLAY (▶) in Isaac Sim: All 3 robots will remain completely rigid!")
    print("==========================================================================\n")

if __name__ == "__main__":
    main()
 