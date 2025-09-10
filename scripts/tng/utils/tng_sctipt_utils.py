from isaaclab_tasks.manager_based.tng_ur5.env_utils.env_config_scheduler import EnvConfigSchedulerDatagen, EnvConfigSchedulerBenchmark
from isaaclab_tasks.manager_based.tng_ur5.ur5_pick_and_place.mdp.events import reset_env_from_scheduler

def patch_env_config_for_configuration_scheduling(env_cfg, yaml_path, mode):
    if mode == "datagen":
        scheduler = EnvConfigSchedulerDatagen(yaml_path, env_cfg.scene.num_envs, env_cfg.sim.device)
    elif mode == "benchmark":
        scheduler = EnvConfigSchedulerBenchmark(yaml_path, env_cfg.scene.num_envs, env_cfg.sim.device)
    else:
        raise ValueError(f"Unknown mode: {mode}")
    
    env_cfg.events.reset_env.func = reset_env_from_scheduler
    env_cfg.events.reset_env.params = {
        "scheduler": scheduler
    }

def patch_env_recorder_dir_config(env_cfg, recorder_dir):
    env_cfg.recorders.dataset_export_dir_path = recorder_dir
    env_cfg.recorders.dataset_filename = "datagen_recordings"
