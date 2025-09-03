# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from __future__ import annotations

import torch
import math
from typing import TYPE_CHECKING

from isaaclab.assets import RigidObject
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils.math import subtract_frame_transforms
from isaaclab.sensors import FrameTransformer
from isaaclab_tasks.manager_based.tng_ur5.tng_assets.ur5.ur5 import ARM_JOINTS, GRIPPER_JOINTS, MAX_GRIPPER_DISPLACEMENT, GRIPPING_CENTER_OFFSET
from isaaclab.assets import Articulation

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def object_reached_goal(
    env: ManagerBasedRLEnv,
    target_cfg: SceneEntityCfg = SceneEntityCfg("target_object"),
    object_cfg: SceneEntityCfg = SceneEntityCfg("object"),
) -> torch.Tensor:

    object: RigidObject = env.scene[object_cfg.name]
    target: RigidObject = env.scene[target_cfg.name]
    distance = torch.norm(target.data.root_pos_w[:, :3] - object.data.root_pos_w[:, :3], dim=1)

    object_size = object.cfg.spawn.size
    target_size = target.cfg.spawn.size
    target_diag = math.sqrt(target_size[0]**2 + target_size[1]**2)
    object_diag = math.sqrt(object_size[0]**2 + object_size[1]**2)
    max_hor_dist = (target_diag - object_diag) / 2.0
    max_vert_dist = (object_size[2] + target_size[2]) / 2.0
    max_dist = math.sqrt(max_hor_dist**2 + max_vert_dist**2)

    return distance < max_dist

def gripper_opened(
    env: ManagerBasedRLEnv,
    object_cfg: SceneEntityCfg = SceneEntityCfg("object"),
) -> torch.Tensor:
    object: RigidObject = env.scene[object_cfg.name]
    gripper_pos = gripper_joint_pos(env)
    object_width = object.cfg.spawn.size[0]

    return gripper_pos[:, 0] < MAX_GRIPPER_DISPLACEMENT - (object_width / 2.0)*1.1

def gripper_open_on_approach(
    env: ManagerBasedRLEnv,
    object_cfg: SceneEntityCfg = SceneEntityCfg("object"),
) -> torch.Tensor:
    gripper_open = gripper_opened(env, object_cfg)
    gripper_approaching = object_in_gripper_reach(env, object_cfg=object_cfg)
    object_at_goal = object_reached_goal(env, object_cfg=object_cfg)
    return gripper_open & gripper_approaching & (~object_at_goal)

def gripper_open_after_target_reached(
    env: ManagerBasedRLEnv,
    object_cfg: SceneEntityCfg = SceneEntityCfg("object"),
) -> torch.Tensor:
    gripper_open = gripper_opened(env, object_cfg)
    target_reached = object_reached_goal(env, object_cfg=object_cfg)
    return gripper_open & target_reached

def root_height_above_reference(
    env: ManagerBasedRLEnv, threshold: float = 0.01, reference_specific_offset: float = 0.0, object_asset_cfg: SceneEntityCfg = SceneEntityCfg("object"), reference_asset_cfg: SceneEntityCfg = SceneEntityCfg("table"), offset_asset_cfg: SceneEntityCfg | None = None) -> torch.Tensor:
    """Terminate when the asset's root height is below the minimum height.

    Note:
        This is currently only supported for flat terrains, i.e. the minimum height is in the world frame.
    """
    # extract the used quantities (to enable type-hinting)
    object_asset: RigidObject = env.scene[object_asset_cfg.name]
    reference_asset: RigidObject = env.scene[reference_asset_cfg.name]
    object_height = object_asset.cfg.spawn.size[2]
    limit = reference_asset.data.root_pos_w[:, 2] + reference_specific_offset + object_height/2.0 + threshold
    if offset_asset_cfg is not None:
        offset_asset: RigidObject = env.scene[offset_asset_cfg.name]
        limit += offset_asset.cfg.spawn.size[2]
    return object_asset.data.root_pos_w[:, 2] > limit

def object_in_gripper_reach(
    env: ManagerBasedRLEnv,
    threshold: float = 0.04,
    object_cfg: SceneEntityCfg = SceneEntityCfg("object"),
    ee_frame_cfg: SceneEntityCfg = SceneEntityCfg("ee_frame"),
) -> torch.Tensor:

    # extract the used quantities (to enable type-hinting)

    object: RigidObject = env.scene[object_cfg.name]
    tcp: FrameTransformer = env.scene[ee_frame_cfg.name]
    object_height = object.cfg.spawn.size[2]

    # distance of the gripper to the object: (num_envs,)
    distance = torch.norm(tcp.data.target_pos_w[:, 0, :3] - object.data.root_pos_w[:, :3], dim=1)

    # rewarded if the object is released above the threshold
    return distance < GRIPPING_CENTER_OFFSET + (object_height/2.0) + threshold


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