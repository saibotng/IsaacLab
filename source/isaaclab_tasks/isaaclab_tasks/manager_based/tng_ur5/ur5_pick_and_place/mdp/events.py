
from __future__ import annotations

import math
import torch
from typing import TYPE_CHECKING
from isaaclab.assets.rigid_object.rigid_object_cfg import RigidObjectCfg
import isaaclab.utils.math as math_utils
from isaaclab.managers import SceneEntityCfg
import random
import yaml
from isaaclab_tasks.manager_based.tng_ur5.env_utils.env_config_scheduler import EnvConfigSchedulerBase
from isaaclab_tasks.manager_based.tng_ur5.tng_assets.ur5.ur5 import reset_joints_by_degree
from isaaclab.sim.schemas.schemas_cfg import RigidBodyPropertiesCfg
from isaacsim.core.utils import prims as prim_utils
from isaacsim.core.prims.impl.xform_prim import XFormPrim
import numpy as np
from isaaclab.envs.mdp.events import reset_root_state_uniform
from isaaclab.assets import Articulation
from pxr import UsdShade, UsdGeom, Sdf, Gf
import isaaclab.sim as sim_utils


if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedEnv

def add_lists(a, b):
    return (np.array(a) + np.array(b)).tolist()



def reset_env_from_scheduler(
        env: ManagerBasedEnv,
        env_ids: torch.Tensor,
        scheduler: EnvConfigSchedulerBase,
        target_asset_cfg: SceneEntityCfg = SceneEntityCfg("target_object", body_names="Target"),
        object_asset_cfg: SceneEntityCfg = SceneEntityCfg("object", body_names="Object"),
        table_asset_cfg: SceneEntityCfg = SceneEntityCfg("table", body_names="Table"),
        robot_asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),

):
    if env.extras.get("scheduler") is None:
        scheduler.register_in_env(env)

    for env_id in env_ids.tolist():
        case = scheduler.get_new_case_for_env(env_id, env)

        if "table_offset" in case:
            table_offset_pos, table_offset_rpy = case["table_offset"]["pos"], convert_deg_to_rad(case["table_offset"]["rpy"])
            set_rigid_object_poses(env, env_id, [table_asset_cfg], [table_offset_pos + table_offset_rpy])
        else:
            table_offset_pos, table_offset_rpy = [0, 0, 0], [0, 0, 0]
            print("[WARNING] Missing key in case definition: 'table_offset'. FALLING BACK TO DEFAULT")

        if "robot_joint_offsets" in case and "gripper_offset" in case:
            #TODO: remove reduncdancy with ur5.reset_joints_by_degree
            robot_joint_offsets = convert_deg_to_rad(case["robot_joint_offsets"])
            gripper_offset = case["gripper_offset"]

            asset: Articulation = env.scene[robot_asset_cfg.name]

            joint_pos_default = asset.data.default_joint_pos[env_id, robot_asset_cfg.joint_ids].clone()
            joint_vel_default = asset.data.default_joint_vel[env_id, robot_asset_cfg.joint_ids].clone()

            joint_pos_offsets = torch.tensor(robot_joint_offsets + gripper_offset * 2, device=env.device).unsqueeze(0)
            joint_pos = joint_pos_default + joint_pos_offsets
            joint_pos_limits = asset.data.soft_joint_pos_limits[env_id, robot_asset_cfg.joint_ids]
            joint_pos = joint_pos.clamp_(joint_pos_limits[..., 0], joint_pos_limits[..., 1])

            asset.write_joint_state_to_sim(
                joint_pos.view(1, -1),
                joint_vel_default.view(1, -1).unsqueeze(0),
                env_ids=torch.tensor([env_id], device=env.device),
                joint_ids=robot_asset_cfg.joint_ids,
    )
        else:
            print("[WARNING] Missing key in case definition: 'robot_joint_offsets' or 'gripper_offset'. FALLING BACK TO DEFAULT")

        if "object_rgb" in case:
            object_rgb = case["object_rgb"]
            force_color_material_on_all_meshes(env, env_id, object_asset_cfg, rgb=object_rgb)
        else:
            print("[WARNING] Missing key in case definition: 'object_rgb'. FALLING BACK TO DEFAULT")

        if "target_rgb" in case:
            target_rgb = case["target_rgb"]
            force_color_material_on_all_meshes(env, env_id, target_asset_cfg, rgb=target_rgb)
        else:
            print("[WARNING] Missing key in case definition: 'target_rgb'. FALLING BACK TO DEFAULT")

        obj_pos, obj_rpy = case["object"]["pos"], convert_deg_to_rad(case["object"]["rpy"])
        tgt_pos, tgt_rpy = case["target"]["pos"], convert_deg_to_rad(case["target"]["rpy"])
        object_pose = add_lists(obj_pos, table_offset_pos) + add_lists(obj_rpy, table_offset_rpy)
        target_pose = add_lists(tgt_pos, table_offset_pos) + add_lists(tgt_rpy, table_offset_rpy)
        set_rigid_object_poses(env, env_id, [object_asset_cfg, target_asset_cfg], [object_pose, target_pose])

def reset_env_random(
    env: ManagerBasedEnv,
    env_ids: torch.Tensor,
    min_separation: float,
    objects_on_table_pose_range: dict[str, tuple[float, float]],
    table_pose_range: dict[str, tuple[float, float]],
    max_sample_tries: int,
    joint_rel_degree_range: tuple[float, float],
    gripper_abs_m_range: tuple[float, float],
    target_asset_cfg: SceneEntityCfg = SceneEntityCfg("target_object", body_names="Target"),
    object_asset_cfg: SceneEntityCfg = SceneEntityCfg("object", body_names="Object"),
    table_asset_cfg: SceneEntityCfg = SceneEntityCfg("table", body_names="Table"),
    robot_asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")

):
    if env_ids is None:
        return
    
    table_range_list = [table_pose_range.get(key, (0.0, 0.0)) for key in ["x", "y", "z", "roll", "pitch", "yaw"]]
    table_ranges = torch.tensor(table_range_list, device=env.device)
    table_rand_samples = math_utils.sample_uniform(table_ranges[:, 0], table_ranges[:, 1], (len(env_ids), 6), device=env.device)

    for i, cur_env in enumerate(env_ids.tolist()):
        assets_on_table = [object_asset_cfg, target_asset_cfg]
        table_offset = table_rand_samples[i].tolist()
        pose_list = sample_object_poses(
            num_objects=len(assets_on_table),
            min_separation=min_separation,
            pose_range=objects_on_table_pose_range,
            max_sample_tries=max_sample_tries,
            offset=table_offset
        )
        assets = assets_on_table + [table_asset_cfg]
        poses = pose_list + [table_offset]
        set_rigid_object_poses(env, cur_env, assets, poses)

    reset_joints_by_degree(env, env_ids, joint_rel_degree_range, gripper_abs_m_range, asset_cfg=robot_asset_cfg)

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
        delta_orientations = math_utils.quat_from_euler_xyz(pose_tensor[:, 3], pose_tensor[:, 4], pose_tensor[:, 5])
        orientations = math_utils.quat_mul(delta_orientations, root_states[0, 3:7].unsqueeze(0))
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
    offset: list[float] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
):
    range_list = [pose_range.get(key, (0.0, 0.0)) for key in ["x", "y", "z", "roll", "pitch", "yaw"]]
    pose_list = []

    for i in range(num_objects):
        for j in range(max_sample_tries):
            sample = [random.uniform(range[0], range[1]) for range in range_list]
            sample = (np.array(sample) + np.array(offset)).tolist()

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

def get_asset_root_prim_path(env: ManagerBasedEnv, asset_cfg: SceneEntityCfg, env_id: int) -> str:
    """Return the concrete prim path for this asset in this env."""
    asset = env.scene[asset_cfg.name]
    # Isaac Lab assets commonly expose per-env prim paths as a list
    if hasattr(asset, "prim_paths"):
        return asset.prim_paths[env_id]
    # Fallback: some expose a single expr or list
    if hasattr(asset, "cfg") and hasattr(asset.cfg, "prim_path"):
        p = asset.cfg.prim_path
        if isinstance(p, (list, tuple)):
            return p[env_id]
        # Replace common wildcard (env_.*) with concrete env id
        return p.replace("env_.*", f"env_{env_id}").replace("env_*", f"env_{env_id}")
    raise RuntimeError(f"Cannot resolve prim path for asset '{asset_cfg.name}'")

def ensure_vec3(rgb_like):
    return (float(rgb_like[0]), float(rgb_like[1]), float(rgb_like[2]))

def define_preview_material(stage, mat_path, rgb):
    shader_path = f"{mat_path}/Shader"
    mat    = UsdShade.Material.Define(stage, mat_path)
    shader = UsdShade.Shader.Define(stage, shader_path)
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
    shader.CreateInput("metallic",     Sdf.ValueTypeNames.Float).Set(0.2)
    shader_out = shader.CreateOutput("surface", Sdf.ValueTypeNames.Token)
    mat_out    = mat.CreateSurfaceOutput()
    mat_out.ConnectToSource(shader_out)
    return mat

def iter_meshes_under(root_prim):
    # DFS over descendants; yield Mesh prims
    stack = list(root_prim.GetChildren())
    while stack:
        p = stack.pop()
        if p.IsA(UsdGeom.Mesh):
            yield p
        stack.extend(p.GetChildren())

def force_color_material_on_all_meshes(env, env_id: int, asset_cfg, rgb,
                                        unbind_existing: bool = True):

    stage = env.scene.stage  
    root_path = get_asset_root_prim_path(env, asset_cfg, env_id)
    root_prim = stage.GetPrimAtPath(root_path)

    mat_name = f"{root_path}/geometry/material"
    mat = define_preview_material(stage, mat_name, ensure_vec3(rgb))

    for mesh in iter_meshes_under(root_prim):
        mb = UsdShade.MaterialBindingAPI(mesh)

        if unbind_existing:
                mb.UnbindAllBindings()

        mb.Bind(mat, bindingStrength=UsdShade.Tokens.strongerThanDescendants)
        gprim = UsdGeom.Gprim(mesh)
        gprim.GetDisplayColorAttr().Set([Gf.Vec3f(*rgb)])
        gprim.GetDisplayOpacityAttr().Set([1.0])