# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from isaaclab.assets import RigidObjectCfg
from isaaclab.sensors import FrameTransformerCfg
from isaaclab.sensors.frame_transformer.frame_transformer_cfg import OffsetCfg
from isaaclab.sim.schemas.schemas_cfg import RigidBodyPropertiesCfg
from isaaclab.sim.spawners.from_files.from_files_cfg import UsdFileCfg
from isaaclab.utils import configclass
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR

from isaaclab_tasks.manager_based.manipulation.lift import mdp
from .lift_env_cfg import LiftEnvCfg

##
# Pre-defined configs
##
from isaaclab.markers.config import FRAME_MARKER_CFG  # isort: skip
from isaaclab_tasks.manager_based.tng_ur5.tng_assets.ur5.ur5 import UR5_CFG, ARM_JOINTS, GRIPPER_JOINTS  # isort: skip
from isaaclab.sensors import CameraCfg
import isaaclab.sim as sim_utils

@configclass
class UR5CubeLiftEnvCfg(LiftEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        # Set UR5 as robot
        self.scene.robot = UR5_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

        self.scene.camera_wrist = CameraCfg(
            prim_path="{ENV_REGEX_NS}/Robot/wrist_3_link/camera_wrist",
            update_period=0.1,
            height=256,
            width=256,
            data_types=["rgb"],
            spawn=sim_utils.PinholeCameraCfg(
                focal_length=24.0,
                focus_distance=400.0,
                horizontal_aperture=20.955,
                clipping_range=(0.1, 1.0e5)
            ),
            offset=CameraCfg.OffsetCfg(
                pos=(0.0, -0.11, 0.043),
                rot=(0.96593, -0.25882, 0.0, 0.0),
            ),
        )

        self.scene.camera_global = CameraCfg(
            prim_path="{ENV_REGEX_NS}/Robot/world/camera_global",
            update_period=0,
            height=256,
            width=256,
            data_types=[
                "rgb",
            ],
            spawn=sim_utils.PinholeCameraCfg(
                focal_length=24.0,
                focus_distance=400.0,
                horizontal_aperture=20.955,
                clipping_range=(0.1, 1.0e5)
            ),
            offset=CameraCfg.OffsetCfg(
                pos=(2.19, 0.0, 1.5),
                rot=(0.32651, -0.62721, -0.62721, 0.32651),
            ),
        )

        # Set actions for the specific robot type (UR5)
        self.actions.arm_action = mdp.JointPositionActionCfg(
            asset_name="robot", joint_names=ARM_JOINTS, scale=0.5, use_default_offset=True
        )
        # Set gripper actions for each joint in GRIPPER_JOINTS
        self.actions.gripper_action = mdp.BinaryJointPositionActionCfg(
            asset_name="robot",
            joint_names=GRIPPER_JOINTS,
            open_command_expr={joint: 0.0 for joint in GRIPPER_JOINTS},
            close_command_expr={joint: 0.0215 for joint in GRIPPER_JOINTS},
        )
        # Set the body name for the end effector
        self.commands.object_pose.body_name = "wrist_3_link"

        # Set Cube as object
        self.scene.object = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/Object",
            init_state=RigidObjectCfg.InitialStateCfg(pos=[0.5, 0, 0.055], rot=[1, 0, 0, 0]),
            spawn=UsdFileCfg(
                usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Blocks/DexCube/dex_cube_instanceable.usd",
                scale=(0.8, 0.8, 0.8),
                rigid_props=RigidBodyPropertiesCfg(
                    solver_position_iteration_count=16,
                    solver_velocity_iteration_count=1,
                    max_angular_velocity=1000.0,
                    max_linear_velocity=1000.0,
                    max_depenetration_velocity=5.0,
                    disable_gravity=False,
                ),
            ),
        )

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
                        pos=[0.0, 0.0, 0.18],
                    ),
                ),
            ],
        )


@configclass
class UR5CubeLiftEnvCfg_PLAY(UR5CubeLiftEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()
        # make a smaller scene for play
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        # disable randomization for play
        self.observations.policy.enable_corruption = False
