
from __future__ import annotations

import math
import torch
from typing import TYPE_CHECKING
from isaaclab.assets.rigid_object.rigid_object_cfg import RigidObjectCfg
from isaaclab.assets.rigid_object_collection.rigid_object_collection import RigidObjectCollection
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
from pxr import UsdShade, UsdGeom, Sdf, Gf, UsdPhysics
import isaaclab.sim as sim_utils
import isaaclab.sensors.camera as camera_utils
from scipy.spatial.transform import Rotation as R


if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedEnv

def add_lists(a, b):
    return (np.array(a) + np.array(b)).tolist()

def intrinsic_euler_deg_to_quat_wxyz(rpy):
    qx, qy, qz, qw = R.from_euler("XYZ", [rpy[0], rpy[1], rpy[2]], degrees=True).as_quat() 
    return torch.tensor([qw, qx, qy, qz], dtype=torch.float32)

def rearrange_wxyz_to_target_quat(quat_wxyz: torch.Tensor) -> torch.Tensor:
    return torch.tensor([quat_wxyz[1], -quat_wxyz[0], -quat_wxyz[3], quat_wxyz[2]])

def _gf_xform_to_pos_quat(xf: Gf.Matrix4d):
    t = xf.ExtractTranslation()
    q = Gf.Quatf(xf.ExtractRotationQuat())  # cast to float quat
    pos = torch.tensor([t[0], t[1], t[2]], dtype=torch.float32)
    quat_wxyz = torch.tensor([q.GetReal(), q.GetImaginary()[0], q.GetImaginary()[1], q.GetImaginary()[2]],
                             dtype=torch.float32)
    return pos, quat_wxyz

def set_cam_pose_relative_to_parent(cam, env_id: int, camera_pose: dict, device) -> None:
    parent_prim = cam._parent_prims[env_id]
    cache = UsdGeom.XformCache()
    parent_world_xf = cache.GetLocalToWorldTransform(parent_prim)
    parent_pos, parent_quat = _gf_xform_to_pos_quat(parent_world_xf)

    # relative transform (in parent frame)
    rel_pos = torch.tensor(camera_pose["pos"], dtype=torch.float32, device=device)  
    rel_rot = torch.tensor(convert_deg_to_rad(camera_pose["rpy"]), device=device)
    #rel_quat = math_utils.quat_from_euler_xyz(rel_rot[0], rel_rot[1], rel_rot[2])
    rel_quat = intrinsic_euler_deg_to_quat_wxyz(camera_pose["rpy"]).to(device)

    parent_pos = parent_pos.to(device)
    parent_quat = parent_quat.to(device)

    # Compose correctly: world = parent ∘ relative
    world_quat = rearrange_wxyz_to_target_quat(math_utils.quat_mul(parent_quat, rel_quat))                 
    world_pos  = parent_pos + math_utils.quat_apply(parent_quat, rel_pos)    

    cam.set_world_poses(
        positions=world_pos.unsqueeze(0),
        orientations=world_quat.unsqueeze(0),
        env_ids=torch.tensor([env_id], device=device),
    )

def reset_env_from_scheduler(
        env: ManagerBasedEnv,
        env_ids: torch.Tensor,
        scheduler: EnvConfigSchedulerBase,
        target_asset_cfg: SceneEntityCfg = SceneEntityCfg("target_object", body_names="Target"),
        object_asset_cfg: SceneEntityCfg = SceneEntityCfg("object", body_names="Object"),
        table_asset_cfg: SceneEntityCfg = SceneEntityCfg("table", body_names="Table"),
        robot_asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
        camera_global_main_cfg: SceneEntityCfg = SceneEntityCfg("camera_global_main"),
        camera_global_secondary_cfg: SceneEntityCfg = SceneEntityCfg("camera_global_secondary"),
        camera_wrist_cfg: SceneEntityCfg = SceneEntityCfg("camera_wrist"),
        object_distractors_cfg: SceneEntityCfg = SceneEntityCfg("distractor_objects"),
        target_distractors_cfg: SceneEntityCfg = SceneEntityCfg("distractor_targets"),
):
    if env.extras.get("scheduler") is None:
        scheduler.register_in_env(env)

    for env_id in env_ids.tolist():
        case = scheduler.get_new_case_for_env(env_id, env)

        camera_keys = {
            "camera_pose_main": camera_global_main_cfg,
            "camera_pose_secondary": camera_global_secondary_cfg,
            "camera_pose_wrist": camera_wrist_cfg,
        }
        for key, cam_cfg in camera_keys.items():
            if key in case:
                camera_pose = case[key]
                cam: camera_utils.Camera = env.scene[cam_cfg.name]
                set_cam_pose_relative_to_parent(cam, env_id, camera_pose, env.device)
            else:
                print(f"[WARNING] Missing key in case definition: '{key}'. FALLING BACK TO DEFAULT")

        if "table_offset" in case:
            table_offset_pos, table_offset_rpy = case["table_offset"]["pos"], convert_deg_to_rad(case["table_offset"]["rpy"])
            set_rigid_object_poses(env, env_id, [table_asset_cfg], [table_offset_pos + table_offset_rpy])
        else:
            table_offset_pos, table_offset_rpy = [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]
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
            root_path = get_asset_root_prim_path(env, object_asset_cfg, env_id)
            force_color_material_on_all_meshes(env, root_path, rgb=object_rgb)
        else:
            print("[WARNING] Missing key in case definition: 'object_rgb'. FALLING BACK TO DEFAULT")

        if "target_rgb" in case:
            target_rgb = case["target_rgb"]
            root_path = get_asset_root_prim_path(env, target_asset_cfg, env_id)
            force_color_material_on_all_meshes(env, root_path, rgb=target_rgb)

        obj_pos, obj_rpy = case["object"]["pos"], convert_deg_to_rad(case["object"]["rpy"])
        tgt_pos, tgt_rpy = case["target"]["pos"], convert_deg_to_rad(case["target"]["rpy"])
        object_pose = add_lists(obj_pos, table_offset_pos) + add_lists(obj_rpy, table_offset_rpy)
        target_pose = add_lists(tgt_pos, table_offset_pos) + add_lists(tgt_rpy, table_offset_rpy)
        set_rigid_object_poses(env, env_id, [object_asset_cfg, target_asset_cfg], [object_pose, target_pose])

        if "distractors" in case:
            colors_objects, poses_objects = get_distractor_colors_and_poses(case["distractors"]["objects"], table_offset_pos, table_offset_rpy)
            colors_targets, poses_targets = get_distractor_colors_and_poses(case["distractors"]["targets"], table_offset_pos, table_offset_rpy)
            root_asset_object = env.scene[object_asset_cfg.name]
            root_asset_target = env.scene[target_asset_cfg.name]
            root_pose_object = root_asset_object.data.default_root_state[[env_id]].clone()
            root_pose_target = root_asset_target.data.default_root_state[[env_id]].clone()
            set_distractor_poses(env, env_id, object_distractors_cfg, poses_objects, root_pose_object)
            set_distractor_poses(env, env_id, target_distractors_cfg, poses_targets, root_pose_target)
            set_distractor_colors(env, env_id, object_distractors_cfg, colors_objects)
            set_distractor_colors(env, env_id, target_distractors_cfg, colors_targets)
        else:
            print("[INFO] No distractors in case definition. FALLING BACK TO DEFAULT")

        # if env_id == 0:
        #     paths = get_asset_collection_root_prim_paths(env, SceneEntityCfg("distractor_objects"), env_id)
        #     root_asset = env.scene["object"]
        #     root_pose = root_asset.data.default_root_state[[env_id]].clone()
        #     set_distractor_poses(env, env_id, SceneEntityCfg("distractor_objects"), [[0.0,0.0,0.0,0.0,0.0,0.0]], root_pose)

        # if env_id == 1:
        #     paths = get_asset_collection_root_prim_paths(env, SceneEntityCfg("distractor_objects"), env_id)


def set_distractor_colors(
        env: ManagerBasedEnv,
        env_id: torch.Tensor,
        distractor_collection_cfg: SceneEntityCfg,
        colors: list,
):
    distractor_paths = get_asset_collection_root_prim_paths(env, distractor_collection_cfg, env_id)
    for i, color in enumerate(colors):
        prim_path = distractor_paths[i]
        force_color_material_on_all_meshes(env, prim_path, rgb=color)


def get_distractor_colors_and_poses(
    distractor_dict: dict,
    table_offset_pos: list[float],
    table_offset_rpy: list[float] 
):
    colors = []
    poses = []
    for distractor in distractor_dict:
        pos = add_lists(distractor["pos"], table_offset_pos)
        rpy = add_lists(convert_deg_to_rad(distractor["rpy"]), table_offset_rpy)
        poses.append(pos + rpy)
        colors.append(distractor["rgb"])
    return colors, poses



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

def set_distractor_poses(
        env: ManagerBasedEnv,
        env_id: torch.Tensor,
        distractor_collection_asset_cfg: SceneEntityCfg,
        pose_list: list,
        root_pose: torch.Tensor
):
    distractor_asset: RigidObjectCollection = env.scene[distractor_collection_asset_cfg.name]
    for i, pose in enumerate(pose_list):
        pose_tensor = torch.tensor([pose_list[i]], device=env.device)
        positions = pose_tensor[:, 0:3] + env.scene.env_origins[env_id, 0:3] + root_pose[0, 0:3]
        delta_orientations = math_utils.quat_from_euler_xyz(pose_tensor[:, 3], pose_tensor[:, 4], pose_tensor[:, 5])
        orientations = math_utils.quat_mul(delta_orientations, root_pose[0, 3:7].unsqueeze(0))
        distractor_state = torch.cat([positions, orientations, torch.zeros(1, 6, device=env.device)], dim=-1)

        distractor_asset.write_object_state_to_sim(
            distractor_state, env_ids=torch.tensor([env_id], device=env.device), object_ids=torch.tensor([i], device=env.device)
        )

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

def get_asset_collection_root_prim_paths(env: ManagerBasedEnv, asset_cfg: SceneEntityCfg, env_id: int) -> list[str]:
    """Return the concrete prim paths for this asset collection in this env."""
    asset_collection = env.scene[asset_cfg.name]
    # Isaac Lab assets commonly expose per-env prim paths as a list
    env_unspecific_paths = asset_collection._prim_paths
    env_specific_paths = [p.replace("env_.*", f"env_{env_id}").replace("env_*", f"env_{env_id}") for p in env_unspecific_paths]
    return env_specific_paths

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

def force_color_material_on_all_meshes(env, root_path, rgb,
                                        unbind_existing: bool = True):

    stage = env.scene.stage  
    #root_path = get_asset_root_prim_path(env, asset_cfg, env_id)
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


def reset_all_assets_to_default(env: ManagerBasedEnv, env_ids: torch.Tensor):
    """Reset the scene to the default state specified in the scene configuration."""
    # rigid object collections (not supported in built-in IsaacLab function, so we add it here)
    for rigid_object_collection in env.scene.rigid_object_collections.values():
        default_object_state = rigid_object_collection.data.default_object_state[env_ids].clone()
        # Add environment origins to the position (for collections, this needs to be done manually)
        default_object_state[..., :3] += env.scene.env_origins[env_ids].unsqueeze(1)
        rigid_object_collection.write_object_state_to_sim(default_object_state, env_ids=env_ids)

    # rigid objects
    for rigid_object in env.scene.rigid_objects.values():
        # obtain default and deal with the offset for env origins
        default_root_state = rigid_object.data.default_root_state[env_ids].clone()
        default_root_state[:, 0:3] += env.scene.env_origins[env_ids]
        # set into the physics simulation
        rigid_object.write_root_pose_to_sim(default_root_state[:, :7], env_ids=env_ids)
        rigid_object.write_root_velocity_to_sim(default_root_state[:, 7:], env_ids=env_ids)
    # articulations
    for articulation_asset in env.scene.articulations.values():
        # obtain default and deal with the offset for env origins
        default_root_state = articulation_asset.data.default_root_state[env_ids].clone()
        default_root_state[:, 0:3] += env.scene.env_origins[env_ids]
        # set into the physics simulation
        articulation_asset.write_root_pose_to_sim(default_root_state[:, :7], env_ids=env_ids)
        articulation_asset.write_root_velocity_to_sim(default_root_state[:, 7:], env_ids=env_ids)
        # obtain default joint positions
        default_joint_pos = articulation_asset.data.default_joint_pos[env_ids].clone()
        default_joint_vel = articulation_asset.data.default_joint_vel[env_ids].clone()
        # set into the physics simulation
        articulation_asset.set_joint_position_target(default_joint_pos, env_ids=env_ids)
        articulation_asset.set_joint_velocity_target(default_joint_vel, env_ids=env_ids)
        articulation_asset.write_joint_state_to_sim(default_joint_pos, default_joint_vel, env_ids=env_ids)
    # deformable objects
    for deformable_object in env.scene.deformable_objects.values():
        # obtain default and set into the physics simulation
        nodal_state = deformable_object.data.default_nodal_state_w[env_ids].clone()
        deformable_object.write_nodal_state_to_sim(nodal_state, env_ids=env_ids)