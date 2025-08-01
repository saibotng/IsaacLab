# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from dataclasses import MISSING

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg, DeformableObjectCfg, RigidObjectCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import CurriculumTermCfg as CurrTerm
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors.frame_transformer.frame_transformer_cfg import FrameTransformerCfg
from isaaclab.sensors import CameraCfg
from isaaclab.sim.spawners.from_files.from_files_cfg import GroundPlaneCfg, UsdFileCfg
from isaaclab.utils import configclass
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import RecorderTermCfg, RecorderManagerBaseCfg, RecorderTerm, DatasetExportMode
import torch
import datetime
from isaaclab.envs import ManagerBasedEnv


from . import mdp
from isaaclab_tasks.manager_based.tng_ur5.tng_assets.ur5.ur5 import reset_joints_by_degree
import os
from dotenv import load_dotenv

load_dotenv()  # loads variables from .env into os.environ

DATASET_BASE_DIR = os.getenv("DATASET_BASE_DIR")
##
# Scene definition
##


def reset_root_state_uniform_nonoverlap(
    env: ManagerBasedEnv,
    env_ids: torch.Tensor,
    pose_range: dict,
    velocity_range: dict,
    asset_a: SceneEntityCfg,
    asset_b: SceneEntityCfg,
    min_xy_dist: float = 0.15,
    max_trials: int = 20,
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
    mdp.reset_root_state_uniform(
        env, env_ids, pose_range, velocity_range, asset_cfg=asset_a
    )
    
    # Now iteratively sample asset_b until no conflicts
    ids_left = env_ids.clone()
    
    for trial in range(max_trials):
        if len(ids_left) == 0:
            break
            
        # Sample asset_b only for environments that still have conflicts
        mdp.reset_root_state_uniform(
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


@configclass
class ObjectTableSceneCfg(InteractiveSceneCfg):
    """Configuration for the pick and place scene with a robot and a object.
    This is the abstract base implementation, the exact scene is defined in the derived classes
    which need to set the target object, robot and end-effector frames
    """

    # robots: will be populated by agent env cfg
    robot: ArticulationCfg = MISSING
    # end-effector sensor: will be populated by agent env cfg
    ee_frame: FrameTransformerCfg = MISSING
    # target object: will be populated by agent env cfg
    object: RigidObjectCfg | DeformableObjectCfg = MISSING

    target_object: RigidObjectCfg | DeformableObjectCfg = MISSING  # for pick and place

    # Camera
    camera_global: CameraCfg = MISSING

    camera_wrist: CameraCfg = MISSING

    # Stand
    stand = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Stand",
        init_state=AssetBaseCfg.InitialStateCfg(pos=[0, 0, 0], rot=[0.707, 0, 0, 0.707]),
        spawn=UsdFileCfg(usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Mounts/Stand/stand_instanceable.usd", scale=(1.55, 1.55, 1.55)),
    )
    

    # Table
    table = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Table",
        init_state=AssetBaseCfg.InitialStateCfg(pos=[0.5, 0.3, 0], rot=[0.707, 0, 0, -0.707]),
        spawn=UsdFileCfg(usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Mounts/ThorlabsTable/table_instanceable.usd")
    )
    # plane
    plane = AssetBaseCfg(
        prim_path="/World/GroundPlane",
        init_state=AssetBaseCfg.InitialStateCfg(pos=[0, 0, -0.795]),
        spawn=GroundPlaneCfg(),
    )

    # lights
    light = AssetBaseCfg(
        prim_path="/World/light",
        spawn=sim_utils.DomeLightCfg(color=(0.75, 0.75, 0.75), intensity=3000.0),
    )


##
# MDP settings
##




@configclass
class ActionsCfg:
    """Action specifications for the MDP."""

    # will be set by agent env cfg
    arm_action: mdp.JointPositionActionCfg | mdp.DifferentialInverseKinematicsActionCfg = MISSING
    gripper_action: mdp.BinaryJointPositionActionCfg = MISSING


@configclass
class ObservationsCfg:
    """Observation specifications for the MDP."""

    @configclass
    class JointObsCfg(ObsGroup):
        """Observations for policy group."""

        joint_pos = ObsTerm(func=mdp.joint_pos)
        joint_vel = ObsTerm(func=mdp.joint_vel)

        def __post_init__(self):
            self.enable_corruption = True
            self.concatenate_terms = False

    class CameraObsCfg(ObsGroup):
        camera_wrist = ObsTerm(func=mdp.image, params={"sensor_cfg": SceneEntityCfg("camera_wrist"), "normalize": False})
        camera_global = ObsTerm(func=mdp.image, params={"sensor_cfg": SceneEntityCfg("camera_global"), "normalize": False})

    # observation groups
    joints: JointObsCfg = JointObsCfg()
    cameras: CameraObsCfg = CameraObsCfg()


@configclass
class EventCfg:
    """Configuration for events."""

    reset_all = EventTerm(func=mdp.reset_scene_to_default, mode="reset")
    #randomize_actuator_gains
    #randomize_joint_parameters
    #randomize_fixed_tendon_parameters
    #randomize orientations



    reset_objects = EventTerm(
        func=reset_root_state_uniform_nonoverlap,
        mode="reset",
        params={
            "pose_range": {"x": (-0.2, 0.2), "y": (-0.3, 0.18), "z": (0.0, 0.0)},
            "velocity_range": {},
            "asset_a": SceneEntityCfg("object", body_names="Object"),
            "asset_b": SceneEntityCfg("target_object", body_names="Target"),
        },
    )
    reset_joints = EventTerm(
        func=reset_joints_by_degree,
        mode="reset",
    )


@configclass
class RewardsCfg:
    """Reward terms for the MDP."""

    reaching_object = RewTerm(func=mdp.object_ee_distance, params={"std": 0.1}, weight=1.0)

    lifting_object = RewTerm(func=mdp.object_is_lifted, params={"minimal_height": 0.04}, weight=15.0)

    # action penalty
    action_rate = RewTerm(func=mdp.action_rate_l2, weight=-1e-4)

    joint_vel = RewTerm(
        func=mdp.joint_vel_l2,
        weight=-1e-4,
        params={"asset_cfg": SceneEntityCfg("robot")},
    )


@configclass
class TerminationsCfg:
    """Termination terms for the MDP."""

    time_out = DoneTerm(func=mdp.time_out, time_out=True)

    object_dropping = DoneTerm(
        func=mdp.root_height_below_minimum, params={"minimum_height": -0.05, "asset_cfg": SceneEntityCfg("object")}
    )

    success = DoneTerm(
        func=mdp.object_reached_goal_and_last_state_reached,
    )



@configclass
class CurriculumCfg:
    """Curriculum terms for the MDP."""

    action_rate = CurrTerm(
        func=mdp.modify_reward_weight, params={"term_name": "action_rate", "weight": -1e-1, "num_steps": 10000}
    )

    joint_vel = CurrTerm(
        func=mdp.modify_reward_weight, params={"term_name": "joint_vel", "weight": -1e-1, "num_steps": 10000}
    )


##
# Environment configuration
##



class ObservationRecorder(RecorderTerm):
    """
    Dump the *already‑computed* observations that the
    ObservationManager buffered this step.
    """
    def record_pre_step(self):
        # The observation manager stores the last obs in `_obs_buffer` :contentReference[oaicite:0]{index=0}
        obs = self._env.observation_manager.compute()     # 1 μs call, returns dict
        # Move to CPU so HDF5 export happens from host memory
        
        obs_dict = {
            "joints_pos_state": obs["joints"]["joint_pos"][:, :-1],
            "joints_vel_state": obs["joints"]["joint_vel"][:, :-1],
            "camera_global": obs["cameras"][:, :, :, :3],
            "camera_wrist": obs["cameras"][:, :, :, 3:],
        }

        obs_dict_cpu = {k: (
            v.cpu() if torch.is_tensor(v)
            else {kk: vv.cpu() for kk, vv in v.items()}
        ) for k, v in obs_dict.items()}

        return "obs_pre", obs_dict_cpu
    
    def record_post_step(self):
        obs = self._env.observation_manager.compute_group(group_name="joints")  
        obs_dict = {
            "joints_pos_action": obs["joint_pos"][:, :-1],
        }
        obs_dict_cpu = {k: (
            v.cpu() if torch.is_tensor(v)
            else {kk: vv.cpu() for kk, vv in v.items()}
        ) for k, v in obs_dict.items()}
        return "obs_post", obs_dict_cpu

@configclass
class ObservationRecorderCfg(RecorderTermCfg):
    class_type: type[RecorderTerm] = ObservationRecorder


@configclass
class RecorderCfg_SM(RecorderManagerBaseCfg):
    record_observation = ObservationRecorderCfg()
    # where & how to export -------------------------------------------------
    dataset_export_dir_path = f"{DATASET_BASE_DIR}/pick_and_place/record/{datetime.datetime.now().strftime('%Y-%m-%d_%H%M')}"     # default: /tmp/isaaclab/logs
    dataset_filename = "all_obs"
    dataset_export_mode = DatasetExportMode.EXPORT_SUCCEEDED_FAILED_IN_SEPARATE_FILES

@configclass
class RecorderCfg_Inference(RecorderManagerBaseCfg):
    record_observation = ObservationRecorderCfg()
    # where & how to export -------------------------------------------------
    dataset_export_dir_path = f"{DATASET_BASE_DIR}/pick_and_place/inference/{datetime.datetime.now().strftime('%Y-%m-%d_%H%M')}"     # default: /tmp/isaaclab/logs
    dataset_filename = "all_obs"
    dataset_export_mode = DatasetExportMode.EXPORT_SUCCEEDED_FAILED_IN_SEPARATE_FILES

@configclass
class PickAndPlaceEnvCfg(ManagerBasedRLEnvCfg):
    """Configuration for the pick-and-place environment."""

    # Scene settings
    scene: ObjectTableSceneCfg = ObjectTableSceneCfg(num_envs=8, env_spacing=5)
    # Basic settings
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()

    # MDP settings
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    events: EventCfg = EventCfg()
    curriculum: CurriculumCfg = CurriculumCfg()
    recorders: RecorderManagerBaseCfg = RecorderCfg_SM()

    def __post_init__(self):
        """Post initialization."""
        # general settings
        self.decimation = 5
        self.episode_length_s = 10.0
        # simulation settings
        self.sim.dt = 0.01  # 100Hz
        self.sim.render_interval = self.decimation

        self.sim.physx.bounce_threshold_velocity = 0.2
        self.sim.physx.bounce_threshold_velocity = 0.01
        self.sim.physx.gpu_found_lost_aggregate_pairs_capacity = 1024 * 1024 * 4
        self.sim.physx.gpu_total_aggregate_pairs_capacity = 16 * 1024
        self.sim.physx.friction_correlation_distance = 0.00625


