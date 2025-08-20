# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Common functions that can be used to activate certain terminations for the lift task.

The functions can be passed to the :class:`isaaclab.managers.TerminationTermCfg` object to enable
the termination introduced by the function.
"""

from __future__ import annotations

import torch
from typing import TYPE_CHECKING

from isaaclab.assets import RigidObject
from isaaclab.sensors import FrameTransformer
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils.math import combine_frame_transforms

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv

def object_reached_goal(
    env: ManagerBasedRLEnv,
    threshold: float = 0.07,
    target_cfg: SceneEntityCfg = SceneEntityCfg("target_object"),
    object_cfg: SceneEntityCfg = SceneEntityCfg("object"),
) -> torch.Tensor:
    """Termination condition for the object reaching the goal position.

    Args:
        env: The environment.
        command_name: The name of the command that is used to control the object.
        threshold: The threshold for the object to reach the goal position. Defaults to 0.02.
        robot_cfg: The robot configuration. Defaults to SceneEntityCfg("robot").
        object_cfg: The object configuration. Defaults to SceneEntityCfg("object").

    """
    # extract the used quantities (to enable type-hinting)
    object: RigidObject = env.scene[object_cfg.name]
    target: RigidObject = env.scene[target_cfg.name]
    # distance of the end-effector to the object: (num_envs,)
    distance = torch.norm(target.data.root_pos_w[:, :3] - object.data.root_pos_w[:, :3], dim=1)

    # rewarded if the object is lifted above the threshold
    return distance < threshold

def object_released_by_gripper(
    env: ManagerBasedRLEnv,
    threshold: float = 0.05,
    object_cfg: SceneEntityCfg = SceneEntityCfg("object"),
    ee_frame_cfg: SceneEntityCfg = SceneEntityCfg("ee_frame"),
) -> torch.Tensor:

    # extract the used quantities (to enable type-hinting)

    object: RigidObject = env.scene[object_cfg.name]
    tcp: FrameTransformer = env.scene[ee_frame_cfg.name]

    # distance of the gripper to the object: (num_envs,)
    distance = torch.norm(tcp.data.target_pos_w[0, :, :3] - object.data.root_pos_w[:, :3], dim=1)

    # rewarded if the object is released above the threshold
    return distance > threshold

def object_reached_goal_and_last_state_reached(
    env: ManagerBasedRLEnv,
    threshold: float = 0.03,
    target_cfg: SceneEntityCfg = SceneEntityCfg("target_object"),
    object_cfg: SceneEntityCfg = SceneEntityCfg("object"),
) -> torch.Tensor:

    object_at_goal = object_reached_goal(env, threshold, target_cfg, object_cfg)
    last_state_reached = env.extras["state"] == 9
    done = object_at_goal & last_state_reached
    if done.sum() > 0:
        print(f"Object reached goal and last state reached: {done.sum().item()} envs out of {env.scene.num_envs}.")

    return done


def object_reached_goal_and_released_by_gripper(
    env: ManagerBasedRLEnv,
    threshold_goal: float = 0.03,
    threshold_release: float = 0.07,
    target_cfg: SceneEntityCfg = SceneEntityCfg("target_object"),
    object_cfg: SceneEntityCfg = SceneEntityCfg("object"),
    ee_frame_cfg: SceneEntityCfg = SceneEntityCfg("ee_frame"),
) -> torch.Tensor:

    object_at_goal = object_reached_goal(env, threshold_goal, target_cfg, object_cfg)
    released = object_released_by_gripper(env, threshold_release, object_cfg, ee_frame_cfg)
    done = object_at_goal & released
    if done.sum() > 0:
        print(f"Object reached goal and released by gripper: {done.sum().item()} envs out of {env.scene.num_envs}.")

    return done