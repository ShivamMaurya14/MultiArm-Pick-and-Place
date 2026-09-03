# This script is designed to be run in Isaac Sim's Script Editor or via standalone python.
# It programmatically duplicates an existing imported UR10e prim and sets up ROS 2 action graphs.

import omni
import omni.graph.core as og
from pxr import Usd, UsdGeom, Gf
import omni.kit.commands

# Configuration
# Assuming you have already imported ONE UR10e via the ROS 2 URDF Importer to this path:
SOURCE_PRIM_PATH = "/World/UR10e"
ROBOT_NAMESPACES = ["robot1", "robot2", "robot3"]
OFFSETS = [
    Gf.Vec3d(0.0, 0.0, 0.0),
    Gf.Vec3d(0.0, 1.5, 0.0),
    Gf.Vec3d(0.0, 3.0, 0.0)
]

stage = omni.usd.get_context().get_stage()

def create_ros2_action_graph(namespace, target_prim_path):
    graph_path = f"{target_prim_path}/ROS2_ActionGraph"
    
    # Create the action graph for ROS 2 Control (topic based)
    keys = og.Controller.Keys
    try:
        (graph, nodes, _, _) = og.Controller.edit(
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
                    ("ROS2PublishJointState.inputs:targetPrim", [target_prim_path]),
                    ("ROS2SubscribeJointState.inputs:topicName", f"/{namespace}/joint_commands"),
                    ("ArticulationController.inputs:targetPrim", [target_prim_path]),
                ],
            },
        )
        print(f"[{namespace}] Created ROS 2 Action Graph at {graph_path}")
    except Exception as e:
        print(f"[{namespace}] Failed to create Action Graph: {e}")

def main():
    if not stage.GetPrimAtPath(SOURCE_PRIM_PATH):
        print(f"ERROR: Could not find source prim at {SOURCE_PRIM_PATH}.")
        print("Please import a single UR10e using the URDF Importer first.")
        return

    for i, namespace in enumerate(ROBOT_NAMESPACES):
        target_path = f"/World/{namespace}"
        
        # 1. Duplicate the prim
        if not stage.GetPrimAtPath(target_path):
            omni.kit.commands.execute('CopyPrims',
                paths_from=[SOURCE_PRIM_PATH],
                paths_to=[target_path])
            print(f"[{namespace}] Duplicated prim to {target_path}")
        
        # 2. Offset position to avoid overlap
        prim = stage.GetPrimAtPath(target_path)
        xform = UsdGeom.Xformable(prim)
        xform_ops = xform.GetOrderedXformOps()
        translate_op = None
        for op in xform_ops:
            if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                translate_op = op
                break
        if not translate_op:
            translate_op = xform.AddTranslateOp()
        
        translate_op.Set(OFFSETS[i])
        
        # 3. Setup ROS 2 Bridge Action Graph
        create_ros2_action_graph(namespace, target_path)

if __name__ == "__main__":
    main()
