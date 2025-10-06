# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from isaaclab.assets import RigidObjectCfg, DeformableObjectCfg, RigidObjectCollectionCfg
from isaaclab.sensors import FrameTransformerCfg
from isaaclab.sensors.frame_transformer.frame_transformer_cfg import OffsetCfg
from isaaclab.sim.schemas.schemas_cfg import RigidBodyPropertiesCfg, DeformableBodyPropertiesCfg
from isaaclab.sim.spawners.from_files.from_files_cfg import UsdFileCfg
from isaaclab.utils import configclass
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR
import isaacsim.core.utils.prims as prim_utils
import isaaclab.sim as sim_utils

from . import mdp
from .pick_and_place_env_cfg import PickAndPlaceEnvCfg, RecorderCfg_Inference, TerminationsCfg_Inference, EventCfg_Inference, TABLE_OFFSET, TABLE_HEIGHT

##
# Pre-defined configs
##
from isaaclab.markers.config import FRAME_MARKER_CFG  # isort: skip
from isaaclab_tasks.manager_based.tng_ur5.tng_assets.ur5.ur5 import ARM_JOINTS, GRIPPER_JOINTS, UR5_RECORD_CFG, UR5_INFERENCE_CFG  # isort: skip
from isaaclab.sensors import CameraCfg
import isaaclab.sim as sim_utils

CUBE_BASE_SCALE = 0.04

@configclass
class UR5CubePickAndPlaceEnvCfg(PickAndPlaceEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        # Set UR5 as robot
        self.scene.robot = UR5_RECORD_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

        self.scene.camera_wrist = CameraCfg(
            prim_path="{ENV_REGEX_NS}/Robot/wrist_3_link/camera_wrist",
            update_period=0,
            height=512,
            width=512,
            data_types=["rgb"],
            spawn=sim_utils.PinholeCameraCfg(
                focal_length=35.0,
                focus_distance=400.0,
                horizontal_aperture=20.955,
                clipping_range=(0.1, 1.0e5)
            ),
            offset=CameraCfg.OffsetCfg(
                rot=(0.9681, -0.2504, 0.0, 0.0),
                pos=(0.0, -0.11, 0.035)
            ),
        )

        self.scene.camera_global_main = CameraCfg(
            prim_path="{ENV_REGEX_NS}/Robot/world/camera_global_main",
            update_period=0,
            height=512,
            width=512,
            data_types=[
                "rgb",
            ],
            spawn=sim_utils.PinholeCameraCfg(
                focal_length=30.0,
                focus_distance=400.0,
                horizontal_aperture=20.955,
                clipping_range=(0.1, 1.0e5)
            ),
            offset=CameraCfg.OffsetCfg(
                pos=(1.5, 0.0, 0.8),
                rot=(0.32651, -0.62721, -0.62721, 0.32651),
            ),
        )


        self.scene.camera_global_secondary = CameraCfg(
            prim_path="{ENV_REGEX_NS}/Robot/world/camera_global_secondary",
            update_period=0,
            height=512,
            width=512,
            data_types=[
                "rgb",
            ],
            spawn=sim_utils.PinholeCameraCfg(
                focal_length=30.0,
                focus_distance=400.0,
                horizontal_aperture=20.955,
                clipping_range=(0.1, 1.0e5)
            ),
            offset=CameraCfg.OffsetCfg(
                pos=(0.5, 1.08, 0.6),
                rot=(0.0, -0.0, -0.86603, 0.5),
            ),
        )

        # Set actions for the specific robot type (UR5)
        self.actions.arm_action = mdp.JointPositionActionCfg(
            asset_name="robot", joint_names=ARM_JOINTS, scale=1.0, use_default_offset=False
        )

        # Set gripper actions for each joint in GRIPPER_JOINTS
        self.actions.gripper_action = mdp.JointPositionActionCfg(
            asset_name="robot", joint_names=GRIPPER_JOINTS, scale=1.0, use_default_offset=False
        )

        # Set Cube as object
        self.scene.object = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/Object",
            spawn=sim_utils.CuboidCfg(
                size=(CUBE_BASE_SCALE, CUBE_BASE_SCALE, CUBE_BASE_SCALE),
                rigid_props=RigidBodyPropertiesCfg(
                    solver_position_iteration_count=16,
                    solver_velocity_iteration_count=1,
                    max_angular_velocity=1000.0,
                    max_linear_velocity=1000.0,
                    max_depenetration_velocity=5.0,
                    disable_gravity=False,
                ),
                mass_props=sim_utils.MassPropertiesCfg(mass=0.1),
                collision_props=sim_utils.CollisionPropertiesCfg(),
                visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.0, 0.0, 0.5), metallic=0.2),
            ),
            init_state=RigidObjectCfg.InitialStateCfg(pos=[0.5, 0, TABLE_OFFSET + 0.03], rot=[0, -0.707, 0.707, 0]),
        )


        self.scene.target_object = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/Target",
            spawn=sim_utils.CuboidCfg(
                size=(0.07, 0.07, 0.01),
                rigid_props=sim_utils.RigidBodyPropertiesCfg(),
                mass_props=sim_utils.MassPropertiesCfg(mass=1.0),
                collision_props=sim_utils.CollisionPropertiesCfg(),
                visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.0, 0.0, 0.0), metallic=0.2),
            ),
            init_state=RigidObjectCfg.InitialStateCfg(pos=(0.5, 0, TABLE_OFFSET + 0.03), rot=(0, -0.707, 0.707, 0)),
        )

        # Create distractor objects using RigidObjectCollectionCfg
        distractor_objects_dict = {}
        for i in range(3):
            distractor_objects_dict[f"distractor_object_{i+1}"] = RigidObjectCfg(
                prim_path="{ENV_REGEX_NS}/DistractorObject" + str(i + 1),
                spawn=sim_utils.CuboidCfg(
                    size=(CUBE_BASE_SCALE, CUBE_BASE_SCALE, CUBE_BASE_SCALE),
                    rigid_props=RigidBodyPropertiesCfg(
                        disable_gravity=False,
                    ),
                    mass_props=sim_utils.MassPropertiesCfg(mass=0.1),
                    collision_props=sim_utils.CollisionPropertiesCfg(),
                    visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.5, 0.5, 0.5), metallic=0.2),
                ),
                init_state=RigidObjectCfg.InitialStateCfg(pos=(0.4, 0 + i * 0.1, TABLE_OFFSET - TABLE_HEIGHT + 0.03), rot=(0, -0.707, 0.707, 0)),
            )

        self.scene.distractor_objects = RigidObjectCollectionCfg(rigid_objects=distractor_objects_dict)

        # Create distractor targets using RigidObjectCollectionCfg
        distractor_targets_dict = {}
        for i in range(3):
            distractor_targets_dict[f"distractor_target_{i+1}"] = RigidObjectCfg(
                prim_path="{ENV_REGEX_NS}/DistractorTarget" + str(i + 1),
                spawn=sim_utils.CuboidCfg(
                    size=(0.07, 0.07, 0.01),
                    rigid_props=sim_utils.RigidBodyPropertiesCfg(
                        disable_gravity=False,
                    ),
                    mass_props=sim_utils.MassPropertiesCfg(mass=1.0),
                    collision_props=sim_utils.CollisionPropertiesCfg(),
                    visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.5, 0.5, 0.5), metallic=0.2),
                ),
                init_state=RigidObjectCfg.InitialStateCfg(pos=(0.6, 0 + i * 0.1, TABLE_OFFSET - TABLE_HEIGHT + 0.03), rot=(0, -0.707, 0.707, 0)),
            )

        self.scene.distractor_targets = RigidObjectCollectionCfg(rigid_objects=distractor_targets_dict)

        # Listens to the required transforms
        marker_cfg = FRAME_MARKER_CFG.copy()
        marker_cfg.markers["frame"].scale = (0.1, 0.1, 0.1)
        marker_cfg.prim_path = "/Visuals/FrameTransformer"
        self.scene.ee_frame = FrameTransformerCfg(
            prim_path="{ENV_REGEX_NS}/Robot/world",
            debug_vis=False,
            visualizer_cfg=marker_cfg,
            target_frames=[
                FrameTransformerCfg.FrameCfg(
                    prim_path="{ENV_REGEX_NS}/Robot/wrist_3_link",
                    name="end_effector",
                    offset=OffsetCfg(
                        pos=(0.0, 0.0, 0.0),
                    ),
                ),
            ],
        )


@configclass
class UR5CubePickAndPlaceEnvCfg_PLAY(UR5CubePickAndPlaceEnvCfg):
    recorders = RecorderCfg_Inference()
    events = EventCfg_Inference()
    terminations = TerminationsCfg_Inference()
    def __post_init__(self):
        # post init of parent
        super().__post_init__()
        # make a smaller scene for play
        self.scene.robot = UR5_INFERENCE_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
        self.scene.num_envs = 1
        self.scene.env_spacing = 5
        self.episode_length_s = 40.0

