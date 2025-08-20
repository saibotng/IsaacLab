import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg

from isaaclab.assets.articulation import ArticulationCfg
import os
import torch
from isaaclab.envs import ManagerBasedEnv
from isaaclab.managers import SceneEntityCfg
from isaaclab.assets import Articulation
import isaaclab.utils.math as math_utils
import math

# Constant for the root/base directory of the project (dynamic)
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

ARM_JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

GRIPPER_JOINTS = [
    "hand_to_left_finger",
    "hand_to_right_finger",
]

UR5_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"{BASE_DIR}/ur5/ur5/ur5.usd",
        activate_contact_sensors=False,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            rigid_body_enabled=True,
            disable_gravity=True,
            max_depenetration_velocity=5.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True, solver_position_iteration_count=8, solver_velocity_iteration_count=0
        ),

    ),
    init_state=ArticulationCfg.InitialStateCfg(
        joint_pos={
            "shoulder_pan_joint": 0.0,
            "shoulder_lift_joint": -1.712,
            "elbow_joint": 1.712,
            "wrist_1_joint": -1.712,
            "wrist_2_joint": -1.571,
            "wrist_3_joint": 0.0,
            "hand_to_.*": 0.0
        },
    ),
    actuators={
        "ur5_shoulder": ImplicitActuatorCfg(
            joint_names_expr=["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint"],
            effort_limit=87.0,
            velocity_limit=0.6,
            velocity_limit_sim=0.6,
            stiffness=400.0,
            damping=40.0,
        ),
        "ur5_wrist": ImplicitActuatorCfg(
            joint_names_expr=["wrist_2_joint", "wrist_3_joint"],
            effort_limit=87.0,
            velocity_limit=2.0,
            velocity_limit_sim=2.0,
            stiffness=400.0,
            damping=40.0,
        ),
        "ur5_hand": ImplicitActuatorCfg(
            joint_names_expr=GRIPPER_JOINTS,
            effort_limit_sim=200.0,
            velocity_limit_sim=0.2,
            stiffness=1500.0,                 # Increased from 2e3 - stronger position hold
            damping=200.0,   
        ),
    },
    soft_joint_pos_limit_factor=1.0,
)

UR5_RECORD_CFG = UR5_CFG.copy()
UR5_RECORD_CFG.spawn.rigid_props.disable_gravity = True
UR5_RECORD_CFG.actuators["ur5_shoulder"].stiffness = 200.0
UR5_RECORD_CFG.actuators["ur5_shoulder"].damping = 80.0
UR5_RECORD_CFG.actuators["ur5_wrist"].stiffness = 200.0
UR5_RECORD_CFG.actuators["ur5_wrist"].damping = 80.0

UR5_INFERENCE_CFG = UR5_CFG.copy()
UR5_INFERENCE_CFG.spawn.rigid_props.disable_gravity = True
UR5_INFERENCE_CFG.actuators["ur5_shoulder"].stiffness = 1500.0
UR5_INFERENCE_CFG.actuators["ur5_shoulder"].damping = 80.0
UR5_INFERENCE_CFG.actuators["ur5_wrist"].stiffness = 1500.0
UR5_INFERENCE_CFG.actuators["ur5_wrist"].damping = 80.0
UR5_INFERENCE_CFG.actuators["ur5_hand"].stiffness = 5000.0



def reset_joints_by_degree(
    env: ManagerBasedEnv,
    env_ids: torch.Tensor,
    joint_rel_degree_range: tuple[float, float] = (-20.0, 20.0),
    gripper_abs_m_range: tuple[float, float] = (0.00, 0.04),
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
):
    # extract the used quantities (to enable type-hinting)
    asset: Articulation = env.scene[asset_cfg.name]
    # get default joint state
    joint_pos = asset.data.default_joint_pos[env_ids, asset_cfg.joint_ids].clone()
    joint_vel = asset.data.default_joint_vel[env_ids, asset_cfg.joint_ids].clone()

    joint_rel_rad_range = tuple(x / 180 * math.pi for x in joint_rel_degree_range)

    # scale these values randomly
    joint_pos += math_utils.sample_uniform(*joint_rel_rad_range, joint_pos.shape, joint_pos.device)
    
    # normalize joint positions to [-pi, +pi]
    joint_pos = (joint_pos + math.pi) % (2 * math.pi) - math.pi

    gripper_pos = math_utils.sample_uniform(
        *gripper_abs_m_range, (len(env_ids)), joint_pos.device
    )
    test = joint_pos[:, -2].clone()
    joint_pos[:, -2] = gripper_pos
    joint_pos[:, -1] = gripper_pos

    # clamp joint pos to limits
    joint_pos_limits = asset.data.soft_joint_pos_limits[env_ids, asset_cfg.joint_ids]
    joint_pos = joint_pos.clamp_(joint_pos_limits[..., 0], joint_pos_limits[..., 1])
    # clamp joint vel to limits

    # set into the physics simulation
    asset.write_joint_state_to_sim(
        joint_pos.view(len(env_ids), -1),
        joint_vel.view(len(env_ids), -1),
        env_ids=env_ids,
        joint_ids=asset_cfg.joint_ids,
    )