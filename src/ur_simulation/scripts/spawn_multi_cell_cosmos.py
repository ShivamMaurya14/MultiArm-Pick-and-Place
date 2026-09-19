#!/usr/bin/env python3
"""
================================================================================
Massive Multi-Cell Parallel Workcell Spawner for NVIDIA Isaac Sim (v4.x, v5.x, v6.0+)
Target: Multi-Cell Triangular UR10e Tri-Arm Groups with RTX Cameras & Force Sensors
Foundation for NVIDIA Cosmos World Models & Physical AI Policy Scaling
================================================================================

Architecture Overview:
- Parallel Autonomous Robotic Groups (arranged in a spatial grid)
- Each Group contains:
  1. 3x UR10e Manipulators with Robotiq 2F-140 Grippers in an inward-facing Triangular Layout (120 deg apart)
  2. 3x Steel Elevated Mounting Pedestals (Z=0.20m)
  3. 1x Central Shared Interaction Platform / Table (Z=0.22m, Radius=0.45m)
  4. 1x Dynamic Workpiece Cube (PhysX 5 Rigid Body, 60mm, 0.15kg) in center of each group
  5. 2x RTX High-Fidelity Synthetic Cameras per group (Top-Down & Angled 45-deg) for Cosmos video generation
  6. PhysX Force/Torque Sensors attached to each robot's tool0/wrist for tactile feedback
  7. High-Stiffness Acceleration Drives locking all robots in ready posture (prevents sagging/flailing)
"""

import math
import os
import omni
try:
    import omni.graph.core as og
except ImportError:
    og = None

try:
    import omni.kit.commands
except ImportError:
    pass

from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema, Sdf, Gf

# ---------------------------------------------------------------------------
# Global Grid & Triangular Cell Configuration
# ---------------------------------------------------------------------------
TOTAL_GROUPS = 10          # Scaled to 10 parallel groups (30 UR10e robots, 20 RTX cameras)
GRID_COLS = 5              # Balanced 5x2 spatial grid (5 columns x 2 rows)
CELL_SPACING_X = 5.0       # 5.0 meters between cell centers along X
CELL_SPACING_Y = 5.0       # 5.0 meters between cell centers along Y
TRIANGLE_RADIUS = 1.25     # Clean breathing room from central platform (meters)

# Canonical industrial standby pose (avoids arm-to-arm and arm-to-table collisions)
READY_POSE_DEG = {
    "shoulder_pan_joint": 0.0,
    "shoulder_lift_joint": -90.0,
    "elbow_joint": 90.0,
    "wrist_1_joint": -90.0,
    "wrist_2_joint": -90.0,
    "wrist_3_joint": 0.0,
}

# ---------------------------------------------------------------------------
# 1. Physics Scene & High-Friction Contact Materials
# ---------------------------------------------------------------------------
def setup_physics_scene(stage):
    # Detect all PhysicsScene prims across the stage (e.g. /PhysicsScene vs /World/PhysicsScene)
    scenes = [p for p in stage.Traverse() if p.GetTypeName() == "PhysicsScene"]
    if scenes:
        primary_scene = scenes[0]
        scene_prim = primary_scene
        scene_path = str(scene_prim.GetPath())
        # Remove any secondary duplicate PhysicsScene to prevent PhysX conflict
        for extra in scenes[1:]:
            print(f"[Physics] Removing duplicate PhysicsScene at '{extra.GetPath()}'.")
            stage.RemovePrim(extra.GetPath())
    else:
        scene_path = "/World/PhysicsScene"
        scene = UsdPhysics.Scene.Define(stage, Sdf.Path(scene_path))
        scene_prim = scene.GetPrim()

    scene = UsdPhysics.Scene(scene_prim)
    scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
    scene.CreateGravityMagnitudeAttr().Set(9.81)

    physx_scene_api = PhysxSchema.PhysxSceneAPI.Apply(scene_prim)
    physx_scene_api.CreateEnableCCDAttr(True)
    physx_scene_api.CreateEnableStabilizationAttr(True)
    physx_scene_api.CreateBroadphaseTypeAttr("GPU")

    # Configure Temporal Gauss-Seidel (TGS) solver with 60 FPS physics
    try:
        scene_prim.CreateAttribute("physxScene:solverType", Sdf.ValueTypeNames.Token).Set("TGS")
        scene_prim.CreateAttribute("physxScene:timeStepsPerSecond", Sdf.ValueTypeNames.Float).Set(60.0)
    except Exception:
        pass

    # Scale PhysX GPU aggregate buffers to eliminate all 'foundLostAggregatePairsCapacity' warnings
    agg_pairs = max(1048576, TOTAL_GROUPS * 350000)
    for attr_name, val in [
        ("CreateGpuFoundLostAggregatePairsCapacityAttr", agg_pairs),
        ("CreateGpuTotalAggregatePairsCapacityAttr", agg_pairs),
        ("CreateGpuMaxRigidContactCountAttr", 2097152),
        ("CreateGpuMaxRigidPatchCountAttr", 655360),
        ("CreateGpuHeapCapacityAttr", 268435456),   # 256 MB heap
        ("CreateGpuFoundLostPairsCapacityAttr", 262144),
    ]:
        if hasattr(physx_scene_api, attr_name):
            try:
                getattr(physx_scene_api, attr_name)(val)
            except Exception:
                pass

    try:
        scene_prim.CreateAttribute("physxScene:gpuFoundLostAggregatePairsCapacity", Sdf.ValueTypeNames.Int).Set(agg_pairs)
        scene_prim.CreateAttribute("physxScene:gpuTotalAggregatePairsCapacity", Sdf.ValueTypeNames.Int).Set(agg_pairs)
    except Exception:
        pass

    print(f"[Physics] Configured primary PhysicsScene at '{scene_path}' (AggregatePairs={agg_pairs}).")

    # High-Friction Grasping Material
    mat_path = "/World/PhysicsMaterials/CosmosHighFrictionMat"
    if not stage.GetPrimAtPath(mat_path).IsValid():
        material_prim = UsdPhysics.MaterialAPI.Apply(stage.DefinePrim(Sdf.Path(mat_path), "Material"))
        material_prim.CreateStaticFrictionAttr(1.2)
        material_prim.CreateDynamicFrictionAttr(0.9)
        material_prim.CreateRestitutionAttr(0.0)
    return mat_path

# ---------------------------------------------------------------------------
# 2. Geometry Builders
# ---------------------------------------------------------------------------
def create_cylinder(stage, prim_path, radius, height, pos, color, has_collision=True, mat_path=None):
    cylinder = UsdGeom.Cylinder.Define(stage, Sdf.Path(prim_path))
    cylinder.CreateRadiusAttr(radius)
    cylinder.CreateHeightAttr(height)
    cylinder.CreateAxisAttr("Z")
    cylinder.CreateDisplayColorAttr([color])

    xform = UsdGeom.Xformable(cylinder)
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(pos)

    prim = cylinder.GetPrim()
    if has_collision:
        UsdPhysics.CollisionAPI.Apply(prim)
        if mat_path:
            PhysxSchema.PhysxMaterialAPI.Apply(prim)
            mat_rel = prim.CreateRelationship("material:binding")
            mat_rel.SetTargets([Sdf.Path(mat_path)])
    return cylinder

def create_box(stage, prim_path, size, pos, color, has_collision=True, mat_path=None):
    cube = UsdGeom.Cube.Define(stage, Sdf.Path(prim_path))
    cube.CreateSizeAttr(1.0)
    cube.CreateDisplayColorAttr([color])

    xform = UsdGeom.Xformable(cube)
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(pos)
    xform.AddScaleOp().Set(size)

    prim = cube.GetPrim()
    if has_collision:
        UsdPhysics.CollisionAPI.Apply(prim)
        if mat_path:
            PhysxSchema.PhysxMaterialAPI.Apply(prim)
            mat_rel = prim.CreateRelationship("material:binding")
            mat_rel.SetTargets([Sdf.Path(mat_path)])
    return cube

# ---------------------------------------------------------------------------
# 3. Perception Setup (2 RTX Synthetic Cameras per Group for Cosmos)
# ---------------------------------------------------------------------------
def create_group_cameras(stage, group_path, center_x, center_y):
    # A. Top-Down Camera (Pinhole perspective looking straight down at table for segmentation & tracking)
    top_cam_path = f"{group_path}/Camera_TopDown"
    top_cam = UsdGeom.Camera.Define(stage, Sdf.Path(top_cam_path))
    top_cam.CreateProjectionAttr("perspective")
    top_cam.CreateFocalLengthAttr(24.0)
    top_cam.CreateHorizontalApertureAttr(20.955)
    top_cam.CreateVerticalApertureAttr(15.29)
    top_cam.CreateClippingRangeAttr(Gf.Vec2f(0.05, 100.0))
    top_cam.CreateFStopAttr(0.0)             # Zero f-stop = pinhole lens without depth-of-field blur
    top_cam.CreateFocusDistanceAttr(2.40)

    top_xform = UsdGeom.Xformable(top_cam)
    top_xform.ClearXformOpOrder()
    top_xform.AddTranslateOp().Set(Gf.Vec3d(center_x, center_y, 2.40))
    top_xform.AddRotateXYZOp().Set(Gf.Vec3f(0.0, 0.0, 0.0))

    # B. Angled 45-degree Perspective Camera (3D bounding boxes & depth perception)
    angled_cam_path = f"{group_path}/Camera_Angled"
    ang_cam = UsdGeom.Camera.Define(stage, Sdf.Path(angled_cam_path))
    ang_cam.CreateProjectionAttr("perspective")
    ang_cam.CreateFocalLengthAttr(28.0)
    ang_cam.CreateHorizontalApertureAttr(20.955)
    ang_cam.CreateVerticalApertureAttr(15.29)
    ang_cam.CreateClippingRangeAttr(Gf.Vec2f(0.05, 100.0))
    ang_cam.CreateFStopAttr(0.0)
    ang_cam.CreateFocusDistanceAttr(2.90)

    ang_xform = UsdGeom.Xformable(ang_cam)
    ang_xform.ClearXformOpOrder()
    ang_xform.AddTranslateOp().Set(Gf.Vec3d(center_x + 1.80, center_y - 1.80, 1.60))
    ang_xform.AddRotateXYZOp().Set(Gf.Vec3f(45.0, 0.0, 45.0))

# ---------------------------------------------------------------------------
# 4. Joint Drive Stabilization & Force Sensing
# ---------------------------------------------------------------------------
def configure_robot_physics_and_sensors(stage, robot_path):
    """
    Locks joint drives with tuned stiffness/damping (preventing flailing)
    and attaches PhysX Force/Torque sensors to the wrist/tool0 link.
    Follower mimic joints on Robotiq gripper have zero active stiffness.
    """
    robot_prim = stage.GetPrimAtPath(robot_path)
    if not robot_prim.IsValid():
        return

    # Ensure robotiq_140_base_joint connects wrist_3_link directly to robotiq_140_base_link.
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
        except Exception:
            pass

    # 1. Lock Joint Drives with Acceleration Control & Initial States
    for prim in Usd.PrimRange(robot_prim):
        if prim.GetTypeName() != "PhysicsRevoluteJoint":
            continue

        joint_name = prim.GetName()
        drive_api = UsdPhysics.DriveAPI.Apply(prim, "angular")
        if not drive_api:
            continue

        if joint_name in READY_POSE_DEG:
            target_deg = READY_POSE_DEG[joint_name]
            # ACCELERATION DRIVE: Invariant to link mass, critically damped to prevent twitching/oscillation
            drive_api.CreateTypeAttr("acceleration")
            drive_api.CreateStiffnessAttr(5000.0)
            drive_api.CreateDampingAttr(1000.0)
            drive_api.CreateMaxForceAttr(1000000.0)
            drive_api.CreateTargetPositionAttr(target_deg)
            drive_api.CreateTargetVelocityAttr(0.0)

            # INITIAL JOINT STATE: Spawns the robot ALREADY at the ready angle.
            # Eliminates sudden 100-degree dynamic swings and momentum bounce when simulation starts!
            try:
                state_api = UsdPhysics.JointStateAPI.Apply(prim, "angular")
                state_api.CreatePositionAttr(target_deg)
                state_api.CreateVelocityAttr(0.0)
            except Exception:
                pass
        elif joint_name == "finger_joint":
            drive_api.CreateTypeAttr("acceleration")
            drive_api.CreateStiffnessAttr(2000.0)
            drive_api.CreateDampingAttr(200.0)
            drive_api.CreateMaxForceAttr(100000.0)
            drive_api.CreateTargetPositionAttr(0.0)
            drive_api.CreateTargetVelocityAttr(0.0)
            try:
                state_api = UsdPhysics.JointStateAPI.Apply(prim, "angular")
                state_api.CreatePositionAttr(0.0)
                state_api.CreateVelocityAttr(0.0)
            except Exception:
                pass
        else:
            # Gripper parallel linkage follower mimic joints (slight damping for stability)
            drive_api.CreateTypeAttr("acceleration")
            drive_api.CreateStiffnessAttr(0.0)
            drive_api.CreateDampingAttr(10.0)
            drive_api.CreateMaxForceAttr(10000.0)

    # 2. Attach PhysX Contact & Force-Torque Sensors to Wrist & Gripper Fingers
    # Enables full tactile contact reporting on both the wrist and fingertip grasp pads
    sensor_link_names = {
        "wrist_3_link", "tool0", "robotiq_140_base_link",
        "left_inner_finger", "right_inner_finger",
        "left_inner_finger_pad", "right_inner_finger_pad"
    }
    for prim in Usd.PrimRange(robot_prim):
        if prim.GetName() in sensor_link_names:
            try:
                sensor_api = PhysxSchema.PhysxContactReportAPI.Apply(prim)
                sensor_api.CreateThresholdAttr(0.0)  # Report all contact forces / torques (>= 0.0 N)
            except Exception:
                pass

# ---------------------------------------------------------------------------
# 5. Spawning Triangular Tri-Arm Groups
# ---------------------------------------------------------------------------
def spawn_triangular_cell(stage, group_idx, center_x, center_y, source_robot_path, mat_path, usd_asset_file=None):
    group_name = f"Group_{group_idx:02d}"
    group_path = f"/World/Cosmos_Grid/{group_name}"
    UsdGeom.Xform.Define(stage, Sdf.Path(group_path))

    # A. Central Interaction Platform
    create_cylinder(
        stage,
        f"{group_path}/CenterTable_Pedestal",
        radius=0.18,
        height=0.18,
        pos=Gf.Vec3d(center_x, center_y, 0.09),
        color=Gf.Vec3f(0.20, 0.22, 0.25),
        has_collision=True
    )
    create_cylinder(
        stage,
        f"{group_path}/CenterTable_Top",
        radius=0.45,
        height=0.04,
        pos=Gf.Vec3d(center_x, center_y, 0.20),
        color=Gf.Vec3f(0.20, 0.55, 0.85),  # Tech Cyan Blue
        has_collision=True,
        mat_path=mat_path
    )

    # B. Central Dynamic Workpiece Box (60mm x 60mm x 50mm, 0.15kg)
    box_path = f"{group_path}/Workpiece_Cube"
    cube_box = create_box(
        stage,
        box_path,
        size=Gf.Vec3f(0.06, 0.06, 0.05),
        pos=Gf.Vec3d(center_x, center_y, 0.245),
        color=Gf.Vec3f(0.10, 0.85, 0.45),  # Emerald Green
        has_collision=True,
        mat_path=mat_path
    )
    rb_api = UsdPhysics.RigidBodyAPI.Apply(cube_box.GetPrim())
    rb_api.CreateRigidBodyEnabledAttr(True)
    mass_api = UsdPhysics.MassAPI.Apply(cube_box.GetPrim())
    mass_api.CreateMassAttr(0.15)

    use_file_ref = usd_asset_file is not None and os.path.exists(usd_asset_file)

    # C. 3 Robots placed at 120-degree Triangular Positions facing center
    angles_deg = [0.0, 120.0, 240.0]
    for arm_idx, angle_deg in enumerate(angles_deg, start=1):
        rad = math.radians(angle_deg)
        rx = center_x + TRIANGLE_RADIUS * math.cos(rad)
        ry = center_y + TRIANGLE_RADIUS * math.sin(rad)
        rz = 0.20  # Mounted on top of 0.20m pedestal
        yaw_deg = angle_deg + 180.0

        # Pedestal (visual stand under fixed base - collision disabled to prevent base fighting)
        create_cylinder(
            stage,
            f"{group_path}/Pedestal_Arm{arm_idx}",
            radius=0.16,
            height=0.20,
            pos=Gf.Vec3d(rx, ry, 0.10),
            color=Gf.Vec3f(0.35, 0.38, 0.42),
            has_collision=False
        )

        robot_path = f"{group_path}/Robot_{arm_idx}"
        
        # Clean any pre-existing prim to guarantee completely fresh, identical robots without stale caches
        if stage.GetPrimAtPath(robot_path).IsValid():
            stage.RemovePrim(Sdf.Path(robot_path))

        target_prim = stage.DefinePrim(Sdf.Path(robot_path), "Xform")
        target_prim.GetReferences().ClearReferences()
        if use_file_ref:
            target_prim.GetReferences().AddReference(assetPath=usd_asset_file)
        elif source_robot_path:
            target_prim.GetReferences().AddInternalReference(Sdf.Path(source_robot_path))

        prim = stage.GetPrimAtPath(robot_path)
        if prim.IsValid():
            xform = UsdGeom.Xformable(prim)
            xform.ClearXformOpOrder()
            xform.AddTranslateOp().Set(Gf.Vec3d(rx, ry, rz))
            xform.AddRotateXYZOp().Set(Gf.Vec3f(0.0, 0.0, yaw_deg))

            # Remove any bogus RootFixedJoint left from previous runs
            old_joint = stage.GetPrimAtPath(f"{robot_path}/RootFixedJoint")
            if old_joint.IsValid():
                stage.RemovePrim(old_joint.GetPath())

            # Configure joint stiffness, damping, ready angles, and force sensor
            configure_robot_physics_and_sensors(stage, robot_path)

    # D. Perception (Synthetic Cameras)
    create_group_cameras(stage, group_path, center_x, center_y)

def find_source_template(stage):
    candidates = [
        "/World/UR10e",
        "/World/ur10e_robotiq",
        "/ur10e_robotiq",
        "/World/ur10e",
        "/ur10e_robotiq",
        "/UR10e",
    ]
    for p in candidates:
        prim = stage.GetPrimAtPath(p)
        if prim.IsValid() and len(prim.GetChildren()) > 0:
            return p

    for prim in stage.Traverse():
        name_lower = prim.GetName().lower()
        path_str = str(prim.GetPath())
        if path_str.startswith("/World/Cosmos_Grid") or "/robot" in path_str.lower():
            continue
        if "ur10e" in name_lower or "robot" in name_lower:
            if len(prim.GetChildren()) > 0:
                return path_str
    return None

# ---------------------------------------------------------------------------
# 6. Main Orchestrator
# ---------------------------------------------------------------------------
def main():
    print("\n==========================================================================")
    print(" 🚀 INITIALIZING MASSIVE MULTI-CELL UR10e WORKCELL SPAWNER FOR COSMOS AI...")
    print("==========================================================================")

    import omni.usd
    stage = omni.usd.get_context().get_stage()
    if not stage:
        print("[ERROR] Failed to get active USD Stage from Isaac Sim!")
        return

    # 1. Physics Scene & Friction Materials
    mat_path = setup_physics_scene(stage)

    # 2. Locate USD Asset on Disk or Source Template in Stage
    candidate_asset_paths = [
        "/home/arvr/Desktop/ros2_ws/MultiArm-Pick-and-Place/src/ur10e_robotiq/ur10e_robotiq.usda",
        "/home/arvr/ros2_ws/MultiArm-Pick-and-Place/src/ur10e_robotiq/ur10e_robotiq.usda",
    ]
    usd_asset_file = next((p for p in candidate_asset_paths if os.path.exists(p)), None)
    use_file_ref = usd_asset_file is not None

    source_path = find_source_template(stage)
    if not source_path and not use_file_ref:
        print("\n[ERROR] Robot template not found in Stage or Disk!")
        print(f">> Expected USDA asset at: {candidate_asset_paths[0]}")
        return

    if use_file_ref:
        print(f"[Cosmos Spawner] Loading clean USD asset from: {usd_asset_file}")
    elif source_path:
        print(f"[Cosmos Spawner] Using template robot at '{source_path}'.")

    # 3. Clean Reset of Grid Parent Xform & Mega Ground Plane
    grid_root = "/World/Cosmos_Grid"
    old_grid = stage.GetPrimAtPath(grid_root)
    if old_grid.IsValid():
        print(f"[Cosmos Spawner] Resetting previous '{grid_root}' stage elements...")
        stage.RemovePrim(old_grid.GetPath())
    UsdGeom.Xform.Define(stage, Sdf.Path(grid_root))

    # Compute dynamic grid bounds based on TOTAL_GROUPS and GRID_COLS
    num_rows = (TOTAL_GROUPS + GRID_COLS - 1) // GRID_COLS
    grid_width = GRID_COLS * CELL_SPACING_X + 6.0
    grid_length = num_rows * CELL_SPACING_Y + 6.0
    grid_center_x = (GRID_COLS - 1) * CELL_SPACING_X / 2.0
    grid_center_y = (num_rows - 1) * CELL_SPACING_Y / 2.0

    create_box(
        stage,
        f"{grid_root}/MegaGroundPlatform",
        size=Gf.Vec3f(grid_width, grid_length, 0.05),
        pos=Gf.Vec3d(grid_center_x, grid_center_y, -0.025),
        color=Gf.Vec3f(0.12, 0.13, 0.15),  # Industrial Dark Charcoal
        has_collision=True,
        mat_path=mat_path
    )

    # 4. Spawn Parallel Triangular Groups
    print(f"\n[Cosmos Spawner] Spawning {TOTAL_GROUPS} triangular groups (Total: {TOTAL_GROUPS * 3} UR10e robots, {TOTAL_GROUPS * 2} RTX Cameras)...")
    for group_idx in range(TOTAL_GROUPS):
        col = group_idx % GRID_COLS
        row = group_idx // GRID_COLS
        cx = col * CELL_SPACING_X
        cy = row * CELL_SPACING_Y

        spawn_triangular_cell(stage, group_idx + 1, cx, cy, source_path, mat_path, usd_asset_file)

    # Move template far away and hide it so its colliders never interfere with the groups
    if source_path:
        template_prim = stage.GetPrimAtPath(source_path)
        if template_prim.IsValid():
            src_xform = UsdGeom.Xformable(template_prim)
            src_xform.ClearXformOpOrder()
            src_xform.AddTranslateOp().Set(Gf.Vec3d(100.0, 100.0, -100.0))
            UsdGeom.Imageable(template_prim).CreateVisibilityAttr().Set(UsdGeom.Tokens.invisible)

    print("\n==========================================================================")
    print(f" 🚀 {TOTAL_GROUPS} TRIANGULAR TRI-ARM GROUPS SPAWNED SUCCESSFULLY IN ISAAC SIM!")
    print(f" - Total Robots: {TOTAL_GROUPS * 3} UR10e Manipulators with Robotiq 2F-140 Grippers")
    print(f" - Layout: 120-degree Triangular Tri-Arm Cells facing central platforms")
    print(f" - Total Workpieces: {TOTAL_GROUPS} Dynamic Green Cubes (PhysX 5 Dynamics)")
    print(f" - Total RTX Cameras: {TOTAL_GROUPS * 2} Cameras (Top-Down + 45-deg Angled for Cosmos)")
    print(f" - Sensors: Contact Force/Torque reporting enabled on all {TOTAL_GROUPS * 3} robot wrists")
    print(f" - Stability: High-stiffness position drives locked in ready posture (NO flailing)")
    print(" Press PLAY (▶) in Isaac Sim to begin GPU-accelerated Physical AI simulation!")
    print("==========================================================================\n")

if __name__ == "__main__":
    main()
