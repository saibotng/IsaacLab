# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause
from isaaclab.controllers.differential_ik_cfg import DifferentialIKControllerCfg
from isaaclab.envs.mdp.actions.actions_cfg import DifferentialInverseKinematicsActionCfg
from isaaclab.utils import configclass

from . import joint_pos_env_cfg
from .pick_and_place_env_cfg import RecorderCfg_Inference, TerminationsCfg_Inference, EventCfg_Inference
from . import mdp

##
# Pre-defined configs
##
from isaaclab_tasks.manager_based.tng_ur5.tng_assets.ur5.ur5 import UR5_RECORD_CFG, UR5_INFERENCE_CFG, ARM_JOINTS, GRIPPER_JOINTS  # isort: skip


@configclass
class UR5CubePickAndPlaceEnvCfg(joint_pos_env_cfg.UR5CubePickAndPlaceEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        # Set UR5 as robot
        # We switch here to a stiffer PD controller for IK tracking to be better.
        self.scene.robot = UR5_RECORD_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

        # Set actions for the specific robot type (UR5)
        self.actions.arm_action = DifferentialInverseKinematicsActionCfg(
            asset_name="robot",
            joint_names=ARM_JOINTS,
            body_name="wrist_3_link",
            controller=DifferentialIKControllerCfg(command_type="pose", use_relative_mode=True, ik_method="dls"),
            scale=0.5,
            body_offset=DifferentialInverseKinematicsActionCfg.OffsetCfg(pos=[0.0, 0.0, 0.18]),
        )
        self.actions.gripper_action = mdp.BinaryJointPositionActionCfg(
            asset_name="robot",
            joint_names=GRIPPER_JOINTS,
            open_command_expr={joint: 0.0 for joint in GRIPPER_JOINTS},
            close_command_expr={joint: 0.025 for joint in GRIPPER_JOINTS},
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

