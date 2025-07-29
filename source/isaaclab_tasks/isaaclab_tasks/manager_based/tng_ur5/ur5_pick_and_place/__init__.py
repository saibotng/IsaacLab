import gymnasium as gym
import os

gym.register(
    id="TNG-Pick-And-Place-Cube-UR5-IK-Abs-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    kwargs={
        "env_cfg_entry_point": f"{__name__}.ik_abs_env_cfg:UR5CubePickAndPlaceEnvCfg",
    },
    disable_env_checker=True,
)