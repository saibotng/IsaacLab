
from __future__ import annotations

import math
import torch
from typing import TYPE_CHECKING
import isaaclab.utils.math as math_utils
from isaaclab.managers import SceneEntityCfg
import random
import yaml

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedEnv

def set_rigid_object_poses(
        env: ManagerBasedEnv,
        env_id: torch.Tensor,
        asset_cfgs: list[SceneEntityCfg],
        pose_list: list
): 
    for i in range(len(asset_cfgs)):
        asset_cfg = asset_cfgs[i]
        asset = env.scene[asset_cfg.name]
        root_states = asset.data.default_root_state[[env_id]].clone()

        # Write pose to simulation
        pose_tensor = torch.tensor([pose_list[i]], device=env.device)
        positions = pose_tensor[:, 0:3] + env.scene.env_origins[env_id, 0:3] + root_states[0, 0:3]
        orientations = math_utils.quat_from_euler_xyz(pose_tensor[:, 3], pose_tensor[:, 4], pose_tensor[:, 5])
        asset.write_root_pose_to_sim(
            torch.cat([positions, orientations], dim=-1), env_ids=torch.tensor([env_id], device=env.device)
        )
        asset.write_root_velocity_to_sim(
            torch.zeros(1, 6, device=env.device), env_ids=torch.tensor([env_id], device=env.device)
        )



def randomize_object_pose(
    env: ManagerBasedEnv,
    env_ids: torch.Tensor,
    asset_cfgs: list[SceneEntityCfg],
    min_separation: float = 0.0,
    pose_range: dict[str, tuple[float, float]] = {},
    max_sample_tries: int = 5000,
):
    if env_ids is None:
        return

    for cur_env in env_ids.tolist():
        pose_list = sample_object_poses(
            num_objects=len(asset_cfgs),
            min_separation=min_separation,
            pose_range=pose_range,
            max_sample_tries=max_sample_tries,
        )
        set_rigid_object_poses(env, cur_env, asset_cfgs, pose_list)


def sample_object_poses(
    num_objects: int,
    min_separation: float = 0.0,
    pose_range: dict[str, tuple[float, float]] = {},
    max_sample_tries: int = 5000,
):
    range_list = [pose_range.get(key, (0.0, 0.0)) for key in ["x", "y", "z", "roll", "pitch", "yaw"]]
    pose_list = []

    for i in range(num_objects):
        for j in range(max_sample_tries):
            sample = [random.uniform(range[0], range[1]) for range in range_list]

            # Accept pose if it is the first one, or if reached max num tries
            if len(pose_list) == 0 or j == max_sample_tries - 1:
                pose_list.append(sample)
                break

            # Check if pose of object is sufficiently far away from all other objects
            separation_check = [math.dist(sample[:3], pose[:3]) > min_separation for pose in pose_list]
            if False not in separation_check:
                pose_list.append(sample)
                break

    return pose_list