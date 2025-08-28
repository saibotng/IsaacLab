
from __future__ import annotations

import math
import torch
from typing import TYPE_CHECKING
import isaaclab.utils.math as math_utils
from isaaclab.managers import SceneEntityCfg
import random
import yaml
from isaaclab_tasks.manager_based.tng_ur5.env_utils.env_config_scheduler import EnvConfigSchedulerBase
from isaaclab_tasks.manager_based.tng_ur5.tng_assets.ur5.ur5 import reset_joints_by_degree

from isaacsim.core.utils import prims as prim_utils
from isaacsim.core.prims.impl.xform_prim import XFormPrim
import numpy as np

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedEnv

def add_lists(a, b):
    return (np.array(a) + np.array(b)).tolist()

def reset_env_from_scheduler(
        env: ManagerBasedEnv,
        env_ids: torch.Tensor,
        asset_cfgs: list[SceneEntityCfg],
        scheduler: EnvConfigSchedulerBase
):
    if env.extras.get("scheduler") is None:
        scheduler.register_in_env(env)

    for env_id in env_ids.tolist():
        case = scheduler.get_new_case_for_env(env_id, env)

        try:
            obj_pos, obj_rpy = case["object"]["pos"], convert_deg_to_rad(case["object"]["rpy"])
            tgt_pos, tgt_rpy = case["target"]["pos"], convert_deg_to_rad(case["target"]["rpy"])
            table_offset_pos, table_offset_rpy = case["table_offset"]["pos"], convert_deg_to_rad(case["table_offset"]["rpy"])
            object_poses = [add_lists(obj_pos, table_offset_pos) + obj_rpy, add_lists(tgt_pos, table_offset_pos) + tgt_rpy]

            set_rigid_object_poses(env, env_id, asset_cfgs, object_poses)
            #set_table_pose_offset(env, env_id, table_offset_pos, table_offset_rpy)
            #patch_mdps_dependencies()

        except KeyError as e:
            print(f"[WARNING] Missing key in case definition: {e}. {e} will be set to default.")

def set_table_pose_offset(
        env: ManagerBasedEnv,
        env_id: torch.Tensor,
        pos: list[float],
        rpy: list[float]
): 

    default_pos = torch.tensor([env.cfg.scene.table.init_state.pos], device=env.device)
    default_rot = torch.tensor(env.cfg.scene.table.init_state.rot, device=env.device)
    table_path = env.scene.env_prim_paths[env_id] + "/Table"

    table = XFormPrim(prim_paths_expr=table_path)
    quat = math_utils.quat_from_euler_xyz(roll = torch.tensor(rpy[0], device=env.device), pitch = torch.tensor(rpy[1], device=env.device), yaw = torch.tensor(rpy[2], device=env.device))
    new_table_pos = torch.tensor(default_pos, device=env.device) + torch.tensor([pos], device=env.device)
    new_table_rot = math_utils.quat_mul(default_rot, quat).unsqueeze(0)
    table.set_local_poses(new_table_pos, new_table_rot)



def reset_env_random(
    env: ManagerBasedEnv,
    env_ids: torch.Tensor,
    asset_cfgs: list[SceneEntityCfg],
    min_separation: float,
    object_pose_range: dict[str, tuple[float, float]],
    max_sample_tries: int,
    joint_rel_degree_range: tuple[float, float],
    gripper_abs_m_range: tuple[float, float],

):
    if env_ids is None:
        return

    for cur_env in env_ids.tolist():
        pose_list = sample_object_poses(
            num_objects=len(asset_cfgs),
            min_separation=min_separation,
            pose_range=object_pose_range,
            max_sample_tries=max_sample_tries,
        )
        set_rigid_object_poses(env, cur_env, asset_cfgs, pose_list)

    reset_joints_by_degree(env, env_ids, joint_rel_degree_range, gripper_abs_m_range)

def convert_deg_to_rad(deg: list[float]) -> list[float]:
    """Convert a list of angles in degrees to radians."""
    return [math.radians(angle) for angle in deg]

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