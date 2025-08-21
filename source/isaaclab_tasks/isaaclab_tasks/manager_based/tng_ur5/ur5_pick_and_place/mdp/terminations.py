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

from isaaclab.managers import SceneEntityCfg
from .observations import object_reached_goal, object_in_gripper_reach

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


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
        print(f"Object reached goal and last state reached: IDs {done.nonzero(as_tuple=False).squeeze(-1).tolist()} envs out of {env.scene.num_envs}.")

    return done


def object_reached_goal_and_released_by_gripper(
    env: ManagerBasedRLEnv,
    target_cfg: SceneEntityCfg = SceneEntityCfg("target_object"),
    object_cfg: SceneEntityCfg = SceneEntityCfg("object"),
    ee_frame_cfg: SceneEntityCfg = SceneEntityCfg("ee_frame"),
) -> torch.Tensor:

    goal_reached = object_reached_goal(env=env, target_cfg=target_cfg, object_cfg=object_cfg)
    object_out_of_reach = (~object_in_gripper_reach(env=env, object_cfg=object_cfg, ee_frame_cfg=ee_frame_cfg))
    done = goal_reached & object_out_of_reach
    if done.sum() > 0:
        print(f"Object reached goal and released by gripper: IDs {done.nonzero(as_tuple=False).squeeze(-1).tolist()}.")

    return done