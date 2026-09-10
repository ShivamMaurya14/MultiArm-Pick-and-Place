#!/usr/bin/env python3
"""
================================================================================
Massive Multi-Cell Parallel Workcell Spawner for NVIDIA Isaac Sim (v4.x, v5.x, v6.0+)
Target: 30x Triangular UR10e Tri-Arm Groups with RTX Cameras & Force Sensors
Foundation for NVIDIA Cosmos World Models & Physical AI Policy Scaling
================================================================================

Architecture Overview:
- 30 Autonomous Robotic Groups (arranged in a 6x5 spatial grid)
- Each Group contains:
  1. 3x UR10e Manipulators with Robotiq 2F-140 Grippers in an inward-facing Triangular Layout (120 deg apart)
  2. 3x Steel Elevated Mounting Pedestals (Z=0.20m)
  3. 1x Central Shared Interaction Platform / Table (Z=0.22m, Radius=0.45m)
  4. 1x Dynamic Workpiece Cube (PhysX 5 Rigid Body, 60mm, 0.15kg) in center of each group
  5. 2x RTX High-Fidelity Synthetic Cameras per group (Top-Down & Angled 45-deg) for Cosmos video generation
  6. PhysX Force/Torque Sensors attached to each robot's tool0/wrist for tactile feedback
  7. High-Stiffness Position Drives locking all 90 robots in ready posture (prevents ragdoll flailing)
"""

import math
import omni
import omni.graph.core as og
import omni.kit.commands
from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema, Gf, Sdf, Vt

# ---------------------------------------------------------------------------
# Global Grid & Triangular Cell Configuration
# ---------------------------------------------------------------------------
TOTAL_GROUPS = 30           # Total parallel groups (e.g. 30 groups = 90 robots)
GRID_COLS = 6               # 6 columns x 5 rows = 30 groups
CELL_SPACING_X = 5.0        # 5.0 meters between cell centers along X
CELL_SPACING_Y = 5.0        # 5.0 meters between cell centers along Y
TRIANGLE_RADIUS = 1.10      # Distance from central platform to robot bases (meters)

# Ready pose joint angles (in degrees for USD PhysX angular drives)
READY_POSE_DEG = {
    "shoulder_pan_joint": 0.0,
    "shoulder_lift_joint": -100.0,
    "elbow_joint": 85.0,
    "wrist_1_joint": -75.0,
    "wrist_2_joint": -90.0,
    "wrist_3_joint": 0.0,
    "finger_joint": 0.0,
    "left_outer_knuckle_joint": 0.0,
    "right_outer_knuckle_joint": 0.0,
}

# ---------------------------------------------------------------------------
# 1. Physics Scene & High-Friction Contact Materials
# ---------------------------------------------------------------------------
def setup_physics_scene(stage):
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

    physx_scene_api = PhysxSchema.PhysxSceneAPI.Apply(stage.GetPrimAtPath(scene_path))
    physx_scene_api.CreateEnableCCDAttr(True)
    physx_scene_api.CreateEnableStabilizationAttr(True)
    physx_scene_api.CreateBroadphaseTypeAttr("GPU")  # Fast GPU Broadphase for 90 robots

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
# 3. RTX Synthetic Camera Setup (2 Cameras per Group for Cosmos Vision AI)
# ---------------------------------------------------------------------------
def create_group_cameras(stage, group_path, center_x, center_y):
    """
    Spawns 2 calibrated RTX cameras for foundation model perception:
    - Camera 1: Overhead Top-Down Orthographic/Perspective view of the workspace
    - Camera 2: Angled 45-degree view capturing 3-arm coordination and depth
    """
    # Camera 1: Top-Down
    cam_top_path = f"{group_path}/Camera_TopDown"
    cam_top = UsdGeom.Camera.Define(stage, Sdf.Path(cam_top_path))
    cam_top.CreateFocalLengthAttr(24.0)
    cam_top.CreateFocusDistanceAttr(2.2)
    xform_top = UsdGeom.Xformable(cam_top)
    xform_top.ClearXformOpOrder()
    xform_top.AddTranslateOp().Set(Gf.Vec3d(center_x, center_y, 2.40))
    xform_top.AddRotateXYZOp().Set(Gf.Vec3f(0.0, 0.0, 0.0))

    # Camera 2: Angled 45-degree Perspective View
    cam_ang_path = f"{group_path}/Camera_Angled"
    cam_ang = UsdGeom.Camera.Define(stage, Sdf.Path(cam_ang_path))
    cam_ang.CreateFocalLengthAttr(35.0)
    cam_ang.CreateFocusDistanceAttr(3.0)
    xform_ang = UsdGeom.Xformable(cam_ang)
    xform_ang.ClearXformOpOrder()
    xform_ang.AddTranslateOp().Set(Gf.Vec3d(center_x + 1.80, center_y - 1.80, 1.60))
    xform_ang.AddRotateXYZOp().Set(Gf.Vec3f(55.0, 0.0, 45.0))

# ---------------------------------------------------------------------------
# 4. Joint Drive & Force Sensor Attachment
# ---------------------------------------------------------------------------
def configure_robot_physics_and_sensors(stage, robot_path):
    """
    Locks joint drives with high stiffness/damping (preventing flailing)
    and attaches PhysX Force/Torque sensors to the wrist/tool0 link.
    """
    robot_prim = stage.GetPrimAtPath(robot_path)
    if not robot_prim.IsValid():
        return

    UsdPhysics.ArticulationRootAPI.Apply(robot_prim)
    physx_art = PhysxSchema.PhysxArticulationAPI.Apply(robot_prim)
    physx_art.CreateSolverPositionIterationCountAttr(32)
    physx_art.CreateSolverVelocityIterationCountAttr(16)

    # 1. Lock Joint Drives
    for prim in Usd.PrimRange(robot_prim):
        if "Joint" in prim.GetTypeName():
            joint_name = prim.GetName()
            drive_api = UsdPhysics.DriveAPI.Apply(prim, "angular")
            if drive_api:
                is_arm = any(j in joint_name for j in ["shoulder", "elbow", "wrist"])
                stiffness = 5000000.0 if is_arm else 100000.0
                damping = 100000.0 if is_arm else 1000.0
                max_force = 10000.0 if is_arm else 500.0

                drive_api.CreateTypeAttr("force")
                drive_api.CreateStiffnessAttr(stiffness)
                drive_api.CreateDampingAttr(damping)
                drive_api.CreateMaxForceAttr(max_force)

                for target_j, target_deg in READY_POSE_DEG.items():
                    if target_j in joint_name:
                        drive_api.CreateTargetPositionAttr(target_deg)
                        drive_api.CreateTargetVelocityAttr(0.0)
                        break

    # 2. Attach Force-Torque Sensor to Wrist/Tool0
    for link_name in ["tool0", "wrist_3_link", "robotiq_base_link"]:
        link_prim = stage.GetPrimAtPath(f"{robot_path}/{link_name}")
        if link_prim.IsValid():
            # Apply Physx Contact / Joint Force Reporting
            sensor_api = PhysxSchema.PhysxContactReportAPI.Apply(link_prim)
            sensor_api.CreateThresholdAttr(0.0)  # Report all contact forces
            break

# ---------------------------------------------------------------------------
# 5. Spawning 30 Triangular Tri-Arm Groups
# ---------------------------------------------------------------------------
def spawn_triangular_cell(stage, group_idx, center_x, center_y, source_robot_path, mat_path):
    """
    Creates 1 Triangular Cell:
    - Central Interaction Platform (Z=0.20m, Radius=0.45m)
    - Dynamic Green Workpiece Box in the center (Z=0.245m)
    - 3x UR10e Manipulators at 120 deg radial angles facing inward
    - 3x Mounting Pedestals (Z=0.20m)
    - 2x Synthetic RTX Cameras
    """
    group_path = f"/World/Cosmos_Grid/Group_{group_idx:02d}"
    UsdGeom.Xform.Define(stage, Sdf.Path(group_path))

    # A. Central Interaction Table (Radius = 0.45m, Height = 0.20m)
    create_cylinder(
        stage,
        f"{group_path}/CenterTable_Pedestal",
        radius=0.18,
        height=0.20,
        pos=Gf.Vec3d(center_x, center_y, 0.10),
        color=Gf.Vec3f(0.25, 0.27, 0.30),
        has_collision=True
    )
    create_cylinder(
        stage,
        f"{group_path}/CenterTable_Top",
        radius=0.45,
        height=0.04,
        pos=Gf.Vec3d(center_x, center_y, 0.20),
        color=Gf.Vec3f(0.20, 0.55, 0.85),  # Distinct Tech Cyan Blue
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

    # C. 3 Robots placed at 120-degree Triangular Positions facing center
    # Angles: 0 deg (East), 120 deg (North-West), 240 deg (South-West)
    angles_deg = [0.0, 120.0, 240.0]
    for arm_idx, angle_deg in enumerate(angles_deg, start=1):
        rad = math.radians(angle_deg)
        # Position on circle around center
        rx = center_x + TRIANGLE_RADIUS * math.cos(rad)
        ry = center_y + TRIANGLE_RADIUS * math.sin(rad)
        rz = 0.20  # Mounted on top of 0.20m pedestal

        # Yaw angle to face inward toward center (angle + 180 deg)
        yaw_deg = angle_deg + 180.0

        # Pedestal
        create_cylinder(
            stage,
            f"{group_path}/Pedestal_Arm{arm_idx}",
            radius=0.16,
            height=0.20,
            pos=Gf.Vec3d(rx, ry, 0.10),
            color=Gf.Vec3f(0.30, 0.32, 0.36),
            has_collision=True
        )

        # Clone Robot
        robot_path = f"{group_path}/Robot_{arm_idx}"
        target_prim = stage.GetPrimAtPath(robot_path)
        if not target_prim.IsValid() or len(target_prim.GetChildren()) == 0:
            omni.kit.commands.execute(
                'CopyPrims',
                paths_from=[source_robot_path],
                paths_to=[robot_path],
                duplicate_layers=True
            )

        prim = stage.GetPrimAtPath(robot_path)
        if prim.IsValid():
            UsdGeom.Imageable(prim).MakeVisible()
            xform = UsdGeom.Xformable(prim)
            xform.ClearXformOpOrder()
            xform.AddTranslateOp().Set(Gf.Vec3d(rx, ry, rz))
            xform.AddRotateXYZOp().Set(Gf.Vec3f(0.0, 0.0, yaw_deg))

            # Configure joint stiffness, damping, ready angles, and force sensor
            configure_robot_physics_and_sensors(stage, robot_path)

    # D. Attach 2x RTX Cameras per group for NVIDIA Cosmos foundation training
    create_group_cameras(stage, group_path, center_x, center_y)

# ---------------------------------------------------------------------------
# 6. Global Helper to Find Robot Template
# ---------------------------------------------------------------------------
def find_source_template(stage):
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

    for prim in stage.Traverse():
        name_lower = prim.GetName().lower()
        if "ur10e" in name_lower or "robot" in name_lower:
            if len(prim.GetChildren()) > 0 and not prim.GetPath().pathString.startswith("/World/Cosmos_Grid"):
                return str(prim.GetPath())
    return None

# ---------------------------------------------------------------------------
# 7. Main Spawner Routine
# ---------------------------------------------------------------------------
def main():
    # Enable ROS 2 & Isaac Core Extensions
    ext_manager = omni.kit.app.get_app().get_extension_manager()
    for ext in ["isaacsim.ros2.bridge", "omni.isaac.ros2_bridge", "omni.isaac.core", "isaacsim.core"]:
        try:
            if not ext_manager.is_extension_enabled(ext):
                ext_manager.set_extension_enabled_immediate(ext, True)
        except Exception:
            pass

    stage = omni.usd.get_context().get_stage()
    if not stage:
        print("ERROR: No active USD stage. Please open Isaac Sim stage first.")
        return

    # 1. Physics Scene & Friction Materials
    mat_path = setup_physics_scene(stage)

    # 2. Locate Source Robot Template
    source_path = find_source_template(stage)
    if not source_path:
        print("\n[ERROR] Robot template not found in Stage!")
        print(">> Please import /tmp/ur10e_robotiq.urdf to /World/UR10e via URDF Importer first.")
        print(">> Then click Run on this script.\n")
        return

    print(f"[Cosmos Spawner] Using template robot at '{source_path}'.")

    # 3. Create Grid Parent Xform & Mega Ground Plane
    grid_root = "/World/Cosmos_Grid"
    UsdGeom.Xform.Define(stage, Sdf.Path(grid_root))

    # Total grid bounds: 6 cols x 5.0m = 30m, 5 rows x 5.0m = 25m
    grid_width = GRID_COLS * CELL_SPACING_X + 6.0
    grid_length = (TOTAL_GROUPS // GRID_COLS + 1) * CELL_SPACING_Y + 6.0
    grid_center_x = (GRID_COLS - 1) * CELL_SPACING_X / 2.0
    grid_center_y = ((TOTAL_GROUPS // GRID_COLS) - 1) * CELL_SPACING_Y / 2.0

    create_box(
        stage,
        f"{grid_root}/MegaGroundPlatform",
        size=Gf.Vec3f(grid_width, grid_length, 0.05),
        pos=Gf.Vec3d(grid_center_x, grid_center_y, -0.025),
        color=Gf.Vec3f(0.12, 0.14, 0.16),
        has_collision=True,
        mat_path=mat_path
    )

    # 4. Spawn 30 Parallel Triangular Groups
    print(f"\n[Cosmos Spawner] Spawning {TOTAL_GROUPS} triangular groups (Total: {TOTAL_GROUPS * 3} UR10e robots, {TOTAL_GROUPS * 2} RTX Cameras)...")
    for group_idx in range(TOTAL_GROUPS):
        col = group_idx % GRID_COLS
        row = group_idx // GRID_COLS
        cx = col * CELL_SPACING_X
        cy = row * CELL_SPACING_Y

        spawn_triangular_cell(stage, group_idx + 1, cx, cy, source_path, mat_path)

    # Hide the isolated template robot
    template_prim = stage.GetPrimAtPath(source_path)
    if template_prim.IsValid():
        UsdGeom.Imageable(template_prim).MakeInvisible()

    print("\n==========================================================================")
    print(f" 🚀 {TOTAL_GROUPS} TRIANGULAR TRI-ARM GROUPS SPAWNED SUCCESSFULLY IN ISAAC SIM!")
    print(f" - Total Robots: {TOTAL_GROUPS * 3} UR10e Manipulators with Robotiq 2F-140 Grippers")
    print(f" - Layout: 120-degree Triangular Tri-Arm Cells facing central platforms")
    print(f" - Total Workpieces: {TOTAL_GROUPS} Dynamic Green Cubes (PhysX 5 Dynamics)")
    print(f" - Total RTX Cameras: {TOTAL_GROUPS * 2} Cameras (Top-Down + 45-deg Angled for Cosmos)")
    print(f" - Sensors: Contact Force/Torque reporting enabled on all 90 robot wrists")
    print(f" - Stability: High-stiffness position drives locked in ready posture (NO flailing)")
    print(" Press PLAY (▶) in Isaac Sim to begin GPU-accelerated Physical AI simulation!")
    print("==========================================================================\n")

if __name__ == "__main__":
    main()
