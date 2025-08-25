from isaaclab.managers import SceneEntityCfg
from isaaclab_tasks.manager_based.tng_ur5.env_utils.env_config_scheduler import EnvConfigScheduler
from isaaclab_tasks.manager_based.tng_ur5.ur5_pick_and_place.mdp.events import reset_env_from_scheduler

def patch_env_config_for_configuration_scheduling(env_cfg, yaml_path):
    scheduler = EnvConfigScheduler(yaml_path, env_cfg.scene.num_envs, env_cfg.sim.device)
    env_cfg.events.reset_env.func = reset_env_from_scheduler
    env_cfg.events.reset_env.params = {
        "asset_cfgs": [
            SceneEntityCfg("object", body_names="Object"),
            SceneEntityCfg("target_object", body_names="Target"),
        ],
        "scheduler": scheduler
    }
