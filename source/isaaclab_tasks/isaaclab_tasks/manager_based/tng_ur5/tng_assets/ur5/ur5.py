import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg

from isaaclab.assets.articulation import ArticulationCfg
import os


# Constant for the root/base directory of the project (dynamic)
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
print(f"Base directory: {BASE_DIR}")

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
            disable_gravity=False,
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
            joint_names_expr=["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint"],
            effort_limit=87.0,
            velocity_limit=100.0,
            stiffness=400.0,
            damping=40.0,
        ),
        "ur5_wrist": ImplicitActuatorCfg(
            joint_names_expr=["wrist_1_joint", "wrist_2_joint", "wrist_3_joint"],
            effort_limit=87.0,
            velocity_limit=100.0,
            stiffness=400.0,
            damping=40.0,
        ),
        "ur5_hand": ImplicitActuatorCfg(
            joint_names_expr=["hand_to_.*"],
            effort_limit_sim=200.0,
            velocity_limit_sim=0.2,
            stiffness=5e3,                 # Increased from 2e3 - stronger position hold
            damping=2e2,   
        ),
    },
        soft_joint_pos_limit_factor=1.0,
)

UR5_HIGH_PD_CFG = UR5_CFG.copy()
UR5_HIGH_PD_CFG.spawn.rigid_props.disable_gravity = True
UR5_HIGH_PD_CFG.actuators["ur5_shoulder"].stiffness = 800.0
UR5_HIGH_PD_CFG.actuators["ur5_shoulder"].damping = 80.0
UR5_HIGH_PD_CFG.actuators["ur5_wrist"].stiffness = 800.0
UR5_HIGH_PD_CFG.actuators["ur5_wrist"].damping = 80.0