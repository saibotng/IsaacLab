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
TABLE_HEIGHT = 0.0
##
# Scene definition
##


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
    camera_global_front: CameraCfg = MISSING

    camera_global_side: CameraCfg = MISSING

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
        init_state=AssetBaseCfg.InitialStateCfg(pos=[0.5, 0.3, TABLE_HEIGHT], rot=[0.707, 0, 0, -0.707]),
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
        camera_global_front = ObsTerm(func=mdp.image, params={"sensor_cfg": SceneEntityCfg("camera_global_front"), "normalize": False})
        camera_global_side = ObsTerm(func=mdp.image, params={"sensor_cfg": SceneEntityCfg("camera_global_side"), "normalize": False})

    # observation groups
    joints: JointObsCfg = JointObsCfg()
    cameras: CameraObsCfg = CameraObsCfg()


@configclass
class EventCfg_SM:
    """Configuration for events."""

    reset_all = EventTerm(func=mdp.reset_scene_to_default, mode="reset")
    #randomize_actuator_gains
    #randomize_joint_parameters
    #randomize_fixed_tendon_parameters
    #randomize orientations



    reset_objects = EventTerm(
        func= mdp.reset_root_state_uniform_nonoverlap,
        mode="reset",
        params={
            "pose_range": {"x": (-0.2, 0.2), "y": (-0.3, 0.18), "z": (0.0, 0.0), "roll": (0.0, 0.0), "pitch": (3.14, 3.14), "yaw": (0.785, 2.356)},
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
class EventCfg_Inference(EventCfg_SM):
    """Configuration for events."""
    reset_joints = EventTerm(
        func=reset_joints_by_degree,
        mode="reset",
        params={
            "joint_rel_degree_range": (-5.0, 5.0),
            "gripper_abs_m_range": (0.00, 0.01),
        }
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
class TerminationsCfg_SM:
    """Termination terms for the MDP."""

    time_out = DoneTerm(func=mdp.time_out, time_out=True)

    object_dropping = DoneTerm(
        func=mdp.root_height_below_minimum, params={"minimum_height": TABLE_HEIGHT-0.05, "asset_cfg": SceneEntityCfg("object")}
    )

    success = DoneTerm(
        func=mdp.object_reached_goal_and_last_state_reached,
    )

@configclass
class TerminationsCfg_Inference:
    """Termination terms for the MDP."""

    time_out = DoneTerm(func=mdp.time_out, time_out=True)

    object_dropping = DoneTerm(
        func=mdp.root_height_below_minimum, params={"minimum_height": TABLE_HEIGHT-0.05, "asset_cfg": SceneEntityCfg("object")}
    )

    success = DoneTerm(
        func=mdp.object_reached_goal,
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
   
            "camera_global_front": obs["cameras"][:, :, :, 0:3],
            "camera_global_side": obs["cameras"][:, :, :, 3:6],
            "camera_wrist": obs["cameras"][:, :, :, 6:],
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
    scene: ObjectTableSceneCfg = ObjectTableSceneCfg(num_envs=4, env_spacing=8)
    # Basic settings
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()

    # MDP settings
    rewards: RewardsCfg = RewardsCfg()
    terminations = TerminationsCfg_SM()
    events = EventCfg_SM()
    curriculum: CurriculumCfg = CurriculumCfg()
    #recorders: RecorderManagerBaseCfg = RecorderCfg_SM()

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


