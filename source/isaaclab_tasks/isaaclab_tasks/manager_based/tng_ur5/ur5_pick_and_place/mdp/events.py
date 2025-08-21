
from __future__ import annotations

import math
import torch
from typing import TYPE_CHECKING, Literal

import carb
import omni.physics.tensors.impl.api as physx
import omni.usd
from isaacsim.core.utils.extensions import enable_extension
from pxr import Gf, Sdf, UsdGeom, Vt

import isaaclab.sim as sim_utils
import isaaclab.utils.math as math_utils
from isaaclab.actuators import ImplicitActuator
from isaaclab.assets import Articulation, DeformableObject, RigidObject
from isaaclab.managers import EventTermCfg, ManagerTermBase, SceneEntityCfg
from isaaclab.terrains import TerrainImporter
import random

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedEnv

from isaaclab.envs.mdp import reset_root_state_uniform

def reset_root_state_uniform_nonoverlap(
    env: ManagerBasedEnv,
    env_ids: torch.Tensor,
    pose_range: dict,
    velocity_range: dict,
    asset_a: SceneEntityCfg,
    asset_b: SceneEntityCfg,
    min_xy_dist: float = 0.15,
    max_trials: int = 1000,
):
    """
    Vectorised rejection sampler that keeps re-randomising the **root
    states** of `asset_a` (Object) and `asset_b` (Target) until their
    XY separation is ≥ `min_xy_dist`.
    """
    if env_ids is None:
        env_ids = slice(None)
    
    device = env.unwrapped.device
    env_ids = torch.as_tensor(env_ids, device=device)
    
    # Sample asset_a first (this stays fixed)
    reset_root_state_uniform(
        env, env_ids, pose_range, velocity_range, asset_cfg=asset_a
    )
    
    # Now iteratively sample asset_b until no conflicts
    ids_left = env_ids.clone()
    
    for trial in range(max_trials):
        if len(ids_left) == 0:
            break
            
        # Sample asset_b only for environments that still have conflicts
        reset_root_state_uniform(
            env, ids_left, pose_range, velocity_range, asset_cfg=asset_b
        )
        
        # Check distances for the remaining environments
        pos_a = env.scene[asset_a.name].data.root_pos_w[ids_left, :2]
        pos_b = env.scene[asset_b.name].data.root_pos_w[ids_left, :2]
        dist = torch.linalg.norm(pos_a - pos_b, dim=-1)
        
        # Keep only envs that are still too close
        mask_bad = dist < min_xy_dist
        ids_left = ids_left[mask_bad]
    
    # Fallback for any remaining conflicts
    if len(ids_left) > 0:
        env.scene[asset_b.name].data.root_pos_w[ids_left, 0] += min_xy_dist
        print(f"Applied fallback X-shift to {len(ids_left)} environments")


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