# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from __future__ import annotations

import torch
from typing import TYPE_CHECKING

from isaaclab.assets import RigidObject
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils.math import subtract_frame_transforms
from isaaclab.sensors import FrameTransformer
from isaaclab_tasks.manager_based.tng_ur5.tng_assets.ur5.ur5 import ARM_JOINTS, GRIPPER_JOINTS
from isaaclab.assets import Articulation
from isaacsim.core.prims.impl.xform_prim import XFormPrim

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def object_reached_goal(
    env: ManagerBasedRLEnv,
    threshold: float = 0.05,
    target_cfg: SceneEntityCfg = SceneEntityCfg("target_object"),
    object_cfg: SceneEntityCfg = SceneEntityCfg("object"),
) -> torch.Tensor:

    object: RigidObject = env.scene[object_cfg.name]
    target: RigidObject = env.scene[target_cfg.name]
    distance = torch.norm(target.data.root_pos_w[:, :3] - object.data.root_pos_w[:, :3], dim=1)
    return distance < threshold

def object_lifted(
    env: ManagerBasedRLEnv,
    threshold: float = 0.05,
    object_cfg: SceneEntityCfg = SceneEntityCfg("object"),
    table_name = "Table"
) -> torch.Tensor:

    object: RigidObject = env.scene[object_cfg.name]
    table_view = XFormPrim(prim_paths_expr=f"{env.scene.env_regex_ns}/{table_name}")
    table_pos, _ = table_view.get_local_poses()
    height_above_table = torch.norm(table_pos[:, 2:3] - object.data.root_pos_w[:, 2:3], dim=1)
    return height_above_table > threshold

def object_in_gripper_reach(
    env: ManagerBasedRLEnv,
    threshold: float = 0.06,
    object_cfg: SceneEntityCfg = SceneEntityCfg("object"),
    ee_frame_cfg: SceneEntityCfg = SceneEntityCfg("ee_frame"),
) -> torch.Tensor:

    # extract the used quantities (to enable type-hinting)

    object: RigidObject = env.scene[object_cfg.name]
    tcp: FrameTransformer = env.scene[ee_frame_cfg.name]

    # distance of the gripper to the object: (num_envs,)
    distance = torch.norm(tcp.data.target_pos_w[:, 0, :3] - object.data.root_pos_w[:, :3], dim=1)

    # rewarded if the object is released above the threshold
    return distance < threshold


def object_position_in_robot_root_frame(
    env: ManagerBasedRLEnv,
    robot_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    object_cfg: SceneEntityCfg = SceneEntityCfg("object"),
) -> torch.Tensor:
    """The position of the object in the robot's root frame."""
    robot: RigidObject = env.scene[robot_cfg.name]
    object: RigidObject = env.scene[object_cfg.name]
    object_pos_w = object.data.root_pos_w[:, :3]
    object_pos_b, _ = subtract_frame_transforms(robot.data.root_pos_w, robot.data.root_quat_w, object_pos_w)
    return object_pos_b

def rigid_object_pose_in_env_root_frame(
    env: ManagerBasedRLEnv,
    object_cfg: SceneEntityCfg = SceneEntityCfg("object"),
) -> torch.Tensor:
    """The pose of the object in the environment's root frame."""
    object: RigidObject = env.scene[object_cfg.name]
    object_pos_w = object.data.root_pos_w - env.scene.env_origins
    object_quat_w = object.data.root_quat_w
    return torch.cat([object_pos_w, object_quat_w], dim=-1)


def ee_frame_position(env: ManagerBasedRLEnv, ee_frame_cfg: SceneEntityCfg = SceneEntityCfg("ee_frame")) -> torch.Tensor:
    ee_frame: FrameTransformer = env.scene[ee_frame_cfg.name]
    ee_frame_pos = ee_frame.data.target_pos_w[:, 0, :] - env.scene.env_origins[:, 0:3]
    return ee_frame_pos

def ee_frame_orientation(env: ManagerBasedRLEnv, ee_frame_cfg: SceneEntityCfg = SceneEntityCfg("ee_frame")) -> torch.Tensor:
    ee_frame: FrameTransformer = env.scene[ee_frame_cfg.name]
    ee_frame_quat = ee_frame.data.target_quat_w[:, 0, :]
    return ee_frame_quat

def arm_joint_pos(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """The joint positions of the asset.

    Note: Only the joints configured in :attr:`asset_cfg.joint_ids` will have their positions returned.
    """
    # extract the used quantities (to enable type-hinting)
    asset: Articulation = env.scene[asset_cfg.name]
    joint_ids = [asset.joint_names.index(joint_name) for joint_name in ARM_JOINTS]
    return asset.data.joint_pos[:, joint_ids]

def gripper_joint_pos(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """The joint positions of the asset.

    Note: Only the joints configured in :attr:`asset_cfg.joint_ids` will have their positions returned.
    """
    # extract the used quantities (to enable type-hinting)
    asset: Articulation = env.scene[asset_cfg.name]
    joint_ids = [asset.joint_names.index(joint_name) for joint_name in GRIPPER_JOINTS[:1]]
    return asset.data.joint_pos[:, joint_ids]

def arm_joint_vel(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """The joint velocities of the asset.

    Note: Only the joints configured in :attr:`asset_cfg.joint_ids` will have their velocities returned.
    """
    # extract the used quantities (to enable type-hinting)
    asset: Articulation = env.scene[asset_cfg.name]
    joint_ids = [asset.joint_names.index(joint_name) for joint_name in ARM_JOINTS]
    return asset.data.joint_vel[:, joint_ids]

def gripper_joint_vel(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """The joint velocities of the asset.

    Note: Only the joints configured in :attr:`asset_cfg.joint_ids` will have their velocities returned.
    """
    # extract the used quantities (to enable type-hinting)
    asset: Articulation = env.scene[asset_cfg.name]
    joint_ids = [asset.joint_names.index(joint_name) for joint_name in GRIPPER_JOINTS[:1]]
    return asset.data.joint_vel[:, joint_ids]