# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from dataclasses import MISSING

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg, DeformableObjectCfg, RigidObjectCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
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





from . import mdp
import os
from dotenv import load_dotenv
from math import pi

load_dotenv()  # loads variables from .env into os.environ

DATASET_BASE_DIR = os.getenv("DATASET_BASE_DIR")
TABLE_SCALING_FACTOR = 1.7
TABLE_HEIGHT = 0.4165 * TABLE_SCALING_FACTOR
TABLE_OFFSET = -0.08

DEFAULT_PROMPT = "Pick up the blue cube and place it on the black platform."
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
        spawn=UsdFileCfg(usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Mounts/Stand/stand_instanceable.usd", scale=(1.0, 1.0, 1.0)),
    )

    table = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Table",                      # unique per-env
        init_state=RigidObjectCfg.InitialStateCfg(pos=[0.5, 0.0, TABLE_OFFSET - TABLE_HEIGHT], rot=[0.0 , 0, 0, 1.0]),
        spawn=UsdFileCfg(
            usd_path="/home/innovation-hacking/luebbet/dev/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/manager_based/tng_ur5/tng_assets/Collected_willowbench/willowbench.usd",
            variants={"PhysicsVariant": "RigidBody"},
            scale=(TABLE_SCALING_FACTOR, 1.0, TABLE_SCALING_FACTOR),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=True,
                kinematic_enabled=True,
            ),
            collision_props=sim_utils.CollisionPropertiesCfg(
                collision_enabled=True,
            ),
        ),
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
class EventCfg_SM:
    """Configuration for events."""

    reset_all = EventTerm(func=mdp.reset_scene_to_default, mode="reset")

    reset_env = EventTerm(
        func=mdp.reset_env_random,
        mode="reset",
        params={
            "objects_on_table_pose_range": {"x": (-0.2, 0.2), "y": (-0.3, 0.18), "z": (0.0, 0.0), "roll": (0.0, 0.0), "pitch": (0.0, 0.0), "yaw": (-35 * (pi / 180), 35 * (pi / 180))},
            "min_separation": 0.15,
            "table_pose_range": {"x": (0.0, 0.0), "y": (0.0, 0.0), "z": (-0.08, 0.08), "roll": (0.0, 0.0), "pitch": (0.0, 0.0), "yaw": (0.0, 0.0)},
            "max_sample_tries": 5000,
            "joint_rel_degree_range": (-10.0, 10.0),
            "gripper_abs_m_range": (0.00, 0.04),
        },
    )




@configclass
class EventCfg_Inference:
    """Configuration for events."""
    reset_all = EventTerm(func=mdp.reset_scene_to_default, mode="reset")

    reset_env = EventTerm(
        func=mdp.reset_env_random,
        mode="reset",
        params={
            "objects_on_table_pose_range": {"x": (-0.12, 0.12), "y": (-0.22, 0.1), "z": (0.0, 0.0), "roll": (0.0, 0.0), "pitch": (0.0, 0.0), "yaw": (-20 * (pi / 180), 20 * (pi / 180))},
            "min_separation": 0.15,
            "table_pose_range": {"x": (0.0, 0.0), "y": (0.0, 0.0), "z": (-0.04, 0.04), "roll": (0.0, 0.0), "pitch": (0.0, 0.0), "yaw": (0.0, 0.0)},
            "max_sample_tries": 5000,
            "joint_rel_degree_range": (-5.0, 5.0),
            "gripper_abs_m_range": (0.00, 0.01),
        },
    )



@configclass
class TerminationsCfg_SM:
    """Termination terms for the MDP."""

    time_out = DoneTerm(func=mdp.time_out, time_out=True)

    object_dropping = DoneTerm(
        func=mdp.root_height_below_reference, params={"threshold": 0.1, "reference_specific_offset": TABLE_HEIGHT}
    )

    success = DoneTerm(
        func=mdp.object_reached_goal_and_last_state_reached,
    )

@configclass
class TerminationsCfg_Inference:
    """Termination terms for the MDP."""

    time_out = DoneTerm(func=mdp.time_out, time_out=True)

    object_dropping = DoneTerm(
        func=mdp.root_height_below_reference, params={"threshold": 0.1, "reference_specific_offset": TABLE_HEIGHT}
    )

    success = DoneTerm(
        func=mdp.object_reached_goal_and_released_by_gripper
    )


##
# Environment configuration
##

@configclass
class ObservationsCfg:
    """Observation specifications for the MDP."""

    @configclass
    class ArmJointObsCfg(ObsGroup):
        """Observations for policy group."""

        arm_joint_pos = ObsTerm(func=mdp.arm_joint_pos)
        arm_joint_vel = ObsTerm(func=mdp.arm_joint_vel)

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = False
    @configclass
    class GripperJointObsCfg(ObsGroup):
        gripper_joint_pos = ObsTerm(func=mdp.gripper_joint_pos)
        gripper_joint_vel = ObsTerm(func=mdp.gripper_joint_vel)

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = False
    @configclass
    class CameraObsCfg(ObsGroup):
        camera_wrist = ObsTerm(func=mdp.image, params={"sensor_cfg": SceneEntityCfg("camera_wrist"), "normalize": False})
        camera_global_front = ObsTerm(func=mdp.image, params={"sensor_cfg": SceneEntityCfg("camera_global_front"), "normalize": False})
        camera_global_side = ObsTerm(func=mdp.image, params={"sensor_cfg": SceneEntityCfg("camera_global_side"), "normalize": False})

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = False
    @configclass
    class EndEffectorObsCfg(ObsGroup):
        end_effector_pos = ObsTerm(func=mdp.ee_frame_position, params={"ee_frame_cfg": SceneEntityCfg("ee_frame")})
        end_effector_quat = ObsTerm(func=mdp.ee_frame_orientation, params={"ee_frame_cfg": SceneEntityCfg("ee_frame")})

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = True

    @configclass
    class RigidObjectsObsCfg(ObsGroup):
        object_pose = ObsTerm(func=mdp.rigid_object_pose_in_env_root_frame, params={"object_cfg": SceneEntityCfg("object")})
        target_pose = ObsTerm(func=mdp.rigid_object_pose_in_env_root_frame, params={"object_cfg": SceneEntityCfg("target_object")})

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = False

    @configclass
    class SubtaskObsCfg(ObsGroup):
        object_reached_target = ObsTerm(func=mdp.object_reached_goal)
        object_lifted = ObsTerm(func=mdp.root_height_above_reference, params={"threshold": 0.01, "reference_specific_offset": TABLE_HEIGHT, "offset_asset_cfg": SceneEntityCfg("target_object")})
        object_in_gripper_reach = ObsTerm(func=mdp.object_in_gripper_reach)
        gripper_open_on_approach = ObsTerm(func=mdp.gripper_open_on_approach)
        gripper_open_after_target_reached = ObsTerm(func=mdp.gripper_open_after_target_reached)

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = False


    @configclass
    class EnvLoggingObsCfg(ObsGroup):
        time_alive = ObsTerm(func=mdp.current_time_s)
        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = False

    # observation groups
    arm_joints: ArmJointObsCfg = ArmJointObsCfg()
    gripper_joint: GripperJointObsCfg = GripperJointObsCfg()
    cameras: CameraObsCfg = CameraObsCfg()
    end_effector: EndEffectorObsCfg = EndEffectorObsCfg()
    rigid_objects: RigidObjectsObsCfg = RigidObjectsObsCfg()
    subtasks: SubtaskObsCfg = SubtaskObsCfg()
    env_logging: EnvLoggingObsCfg = EnvLoggingObsCfg()


class ObservationRecorder(RecorderTerm):
    """
    Dump the *already‑computed* observations that the
    ObservationManager buffered this step.
    """
    def record_pre_step(self):
        obs = self._env.obs_buf
        obs_dict = {
            "arm_joints_pos_state": obs["arm_joints"]["arm_joint_pos"],
            "gripper_joint_pos_state": obs["gripper_joint"]["gripper_joint_pos"],
            "tcp_pose_state": obs["end_effector"],

            "camera_global_front": obs["cameras"]["camera_global_front"],
            "camera_global_side": obs["cameras"]["camera_global_side"],
            "camera_wrist": obs["cameras"]["camera_wrist"],
        }

        obs_dict_cpu = {k: (
            v.cpu() if torch.is_tensor(v)
            else {kk: vv.cpu() for kk, vv in v.items()}
        ) for k, v in obs_dict.items()}

        return "obs_pre", obs_dict_cpu

    def record_post_step(self):
        obs = self._env.obs_buf
        obs_dict = {
            "arm_joints_pos_action": obs["arm_joints"]["arm_joint_pos"],
            "gripper_joint_pos_action": obs["gripper_joint"]["gripper_joint_pos"],
            "tcp_pose_action": obs["end_effector"]
        }
        obs_dict_cpu = {k: (
            v.cpu() if torch.is_tensor(v)
            else {kk: vv.cpu() for kk, vv in v.items()}
        ) for k, v in obs_dict.items()}
        return "obs_post", obs_dict_cpu
    
    def record_post_reset(self, env_ids):
        scheduler = self._env.extras.get("scheduler", None)
        prompts = (scheduler.get_prompts(env_ids)
                if scheduler else [DEFAULT_PROMPT] * self._env.num_envs)
        prompt_bytes, prompt_len = self.encode_prompts_uint8(prompts)
        obs_dict = {
            "prompt_bytes": prompt_bytes,   # uint8 [num_envs, max_len]
            "prompt_len": prompt_len,       # int32 [num_envs]
        }
        obs_dict_cpu = {k: (
            v.cpu() if torch.is_tensor(v)
            else {kk: vv.cpu() for kk, vv in v.items()}
        ) for k, v in obs_dict.items()}
        return "obs_post_reset", obs_dict_cpu

    def encode_prompts_uint8(self, prompts):
        # prompts: list[str] length = num_envs
        b = [p.encode("utf-8") for p in prompts]
        max_len = max(len(x) for x in b) if b else 0
        out = torch.zeros((len(b), max_len), dtype=torch.uint8, device=self._env.device)
        lengths = torch.zeros((len(b),), dtype=torch.int32, device=self._env.device)
        for i, bi in enumerate(b):
            n = min(len(bi), max_len)
            if n > 0:
                out[i, :n] = torch.as_tensor(list(bi[:n]), dtype=torch.uint8, device=self._env.device)
            lengths[i] = n
        return out, lengths.unsqueeze(-1)

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
    scene: ObjectTableSceneCfg = ObjectTableSceneCfg(num_envs=10, env_spacing=8)
    # Basic settings
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()

    # MDP settings
    terminations = TerminationsCfg_SM()
    events = EventCfg_SM()
    recorders: RecorderManagerBaseCfg = RecorderCfg_SM()
    curriculum = None
    rewards = None


    def __post_init__(self):
        """Post initialization."""
        # general settings
        self.decimation = 5
        self.episode_length_s = 20.0
        # simulation settings
        self.sim.dt = 0.01  # 100Hz
        self.sim.render_interval = self.decimation

        self.sim.physx.bounce_threshold_velocity = 0.2
        self.sim.physx.bounce_threshold_velocity = 0.01
        self.sim.physx.gpu_found_lost_aggregate_pairs_capacity = 1024 * 1024 * 4
        self.sim.physx.gpu_total_aggregate_pairs_capacity = 16 * 1024
        self.sim.physx.friction_correlation_distance = 0.00625


