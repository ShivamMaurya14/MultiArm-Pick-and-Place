#!/usr/bin/env python3
"""
================================================================================
Multi-UR10e Industrial Workcell Spawner for NVIDIA Isaac Sim (v4.x, v5.x, v6.0+)
Compatible with ROS 2 Jazzy Jalisco
================================================================================

This script automates the complete scene generation for the 3-arm robotic cell:
1. Physics Scene & Environment Lighting
2. Ground Platform & Safety Demarcation Boundaries
3. Steel Mounting Pedestals for Robot 1, Robot 2, and Robot 3
4. High-Friction Industrial Workstation Tables (Stations A, B, C, D)
5. Dynamic Workpiece Cube with Physics Colliders & High-Friction Contact Material
6. Multi-Robot Instantiation at Pedestal Heights (Z=0.20m)
7. OmniGraph ROS 2 Bridge Action Graphs (/clock, /robot[1..3]/joint_states, /robot[1..3]/joint_commands)

Execution Methods:
- In Isaac Sim: Window -> Script Editor -> Paste / Open -> Click "Run"
- Standalone: ./python.sh src/ur_simulation/scripts/spawn_multi_ur10e.py
"""

import math
import omni
import omni.graph.core as og
import omni.kit.commands
from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema, Gf, Sdf, Vt

# ---------------------------------------------------------------------------
# Global Workcell Configuration (Calibrated to URDF World Coordinates)
# ---------------------------------------------------------------------------
SOURCE_PRIM_PATH = "/World/UR10e"

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

# ---------------------------------------------------------------------------
# 1. Physics Scene & Physics Material Setup
# ---------------------------------------------------------------------------
def setup_physics_scene(stage):
    """Ensures PhysicsScene and high-friction contact material exist."""
    scene_path = "/World/PhysicsScene"
    if not stage.GetPrimAtPath(scene_path).IsValid():
        scene = UsdPhysics.Scene.Define(stage, Sdf.Path(scene_path))
        scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
        scene.CreateGravityMagnitudeAttr().Set(9.81)
        print("[Physics] Created PhysicsScene with 9.81 m/s^2 gravity.")

    # Create High-Friction Grasping Material (static=1.2, dynamic=0.9)
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
        # Pedestal stand
        create_cylinder(stage, f"{workcell_path}/{name}_pedestal", radius=0.12, height=0.20, pos=Gf.Vec3d(x, y, 0.10), color=Gf.Vec3f(0.28, 0.30, 0.35), has_collision=True)
        # Circular table top (surface at Z=0.22m)
        create_cylinder(stage, f"{workcell_path}/{name}_top", radius=0.24, height=0.04, pos=Gf.Vec3d(x, y, 0.20), color=st["color"], has_collision=True, mat_path=mat_path)
        # Placement ring indicator
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
    # Enable Rigid Body Dynamics
    rb_api = UsdPhysics.RigidBodyAPI.Apply(cube_box.GetPrim())
    rb_api.CreateRigidBodyEnabledAttr(True)
    mass_api = UsdPhysics.MassAPI.Apply(cube_box.GetPrim())
    mass_api.CreateMassAttr(WORKPIECE_CUBE["mass"])

    print("[Workcell] Successfully assembled Ground, 3 Pedestals, 4 Station Tables, and Dynamic Cube.")

# ---------------------------------------------------------------------------
# 4. OmniGraph ROS 2 Simulation Clock (/clock)
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
# 5. OmniGraph Robot Action Graphs (/robot[X]/joint_states & joint_commands)
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
# 6. Main Orchestrator
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
    global SOURCE_PRIM_PATH
    source_prim = stage.GetPrimAtPath(SOURCE_PRIM_PATH)
    if not source_prim.IsValid():
        alt_paths = ["/World/ur10e_robotiq", "/World/ur10e", "/UR10e"]
        for p in alt_paths:
            if stage.GetPrimAtPath(p).IsValid():
                SOURCE_PRIM_PATH = p
                source_prim = stage.GetPrimAtPath(p)
                break

    if not source_prim or not source_prim.IsValid():
        print(f"\n[INFO] Robot template not found at '{SOURCE_PRIM_PATH}'.")
        print(">> To import robot: Use 'Isaac Utils -> Workflows -> URDF Importer', select '/tmp/ur10e_robotiq.urdf', and import to '/World/UR10e'.")
        print(">> Then re-run this script to instantiate Robot 1, 2, and 3.\n")
        return

    # 5. Instantiate & Configure the 3 UR10e Robots on Pedestals
    for cfg in ROBOT_CONFIGS:
        ns = cfg["ns"]
        target_path = f"/World/{ns}"
        
        # Duplicate robot if not present
        if not stage.GetPrimAtPath(target_path).IsValid():
            omni.kit.commands.execute(
                'CopyPrims',
                paths_from=[SOURCE_PRIM_PATH],
                paths_to=[target_path],
                duplicate_layers=True
            )
        
        # Set exact mounting height on top of pedestal (Z = 0.20m)
        prim = stage.GetPrimAtPath(target_path)
        if prim.IsValid():
            xform = UsdGeom.Xformable(prim)
            xform.ClearXformOpOrder()
            translate_op = xform.AddTranslateOp()
            translate_op.Set(cfg["origin"])
            
            # Ensure Articulation Root API is applied
            if not prim.HasAPI(UsdPhysics.ArticulationRootAPI):
                UsdPhysics.ArticulationRootAPI.Apply(prim)
            
            # Attach OmniGraph ROS 2 Bridge
            create_ros2_action_graph(ns, target_path)

    # Hide the source template
    UsdGeom.Imageable(source_prim).MakeInvisible()

    print("\n==========================================================================")
    print(" INDUSTRIAL WORKCELL & MULTI-UR10e SETUP COMPLETED IN ISAAC SIM!")
    print(" - 3 Robots positioned at Pedestal Heights (Z=0.20m): Y=0.0, Y=1.6, Y=3.2")
    print(" - 4 Stations (A, B, C, D) with calibrated target rings & colors")
    print(" - Dynamic Workpiece Cube placed at Station A (0.70, 0.00, 0.245)")
    print(" - High-Friction contact material applied for slip-free grasping")
    print(" - ROS 2 Bridge active on /clock, /robot[1..3]/joint_states")
    print(" Click PLAY (▶) in Isaac Sim, then run task_manager in ROS 2.")
    print("==========================================================================\n")

if __name__ == "__main__":
    main()
