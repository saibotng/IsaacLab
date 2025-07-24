# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from isaaclab.controllers.differential_ik_cfg import DifferentialIKControllerCfg
from isaaclab.envs.mdp.actions.actions_cfg import DifferentialInverseKinematicsActionCfg
from isaaclab.utils import configclass


from . import joint_pos_env_cfg

##
# Pre-defined configs
##
from isaaclab_tasks.manager_based.tng_ur5.tng_assets.ur5.ur5 import UR5_HIGH_PD_CFG, ARM_JOINTS  # isort: skip
##
# Rigid object lift environment.
##


@configclass
class UR5CubeLiftEnvCfg(joint_pos_env_cfg.UR5CubeLiftEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        # Set UR5 as robot
        # We switch here to a stiffer PD controller for IK tracking to be better.
        self.scene.robot = UR5_HIGH_PD_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

        # Set actions for the specific robot type (UR5)
        self.actions.arm_action = DifferentialInverseKinematicsActionCfg(
            asset_name="robot",
            joint_names=ARM_JOINTS,
            body_name="wrist_3_link",
            controller=DifferentialIKControllerCfg(
                command_type="pose", use_relative_mode=False, ik_method="dls"
            ),
            body_offset=DifferentialInverseKinematicsActionCfg.OffsetCfg(
                pos=[0.0, 0.0, 0.18]
            ),
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
