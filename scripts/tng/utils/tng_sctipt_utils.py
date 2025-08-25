from isaaclab.managers import SceneEntityCfg
from isaaclab_tasks.manager_based.tng_ur5.env_utils.env_config_scheduler import EnvConfigScheduler

def patch_env_config_for_configuration_scheduling(env_cfg, yaml_path) -> EnvConfigScheduler:
    scheduler = EnvConfigScheduler.from_yaml(yaml_path)
    env_cfg.env_config_scheduler = scheduler
    env_cfg.events.reset_objects.func = scheduler.on_reset
    env_cfg.events.reset_objects.params = {
        "asset_cfgs": [
            SceneEntityCfg("object", body_names="Object"),
            SceneEntityCfg("target_object", body_names="Target"),
        ],
    }
    return scheduler