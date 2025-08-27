"""
Evaluate a fine‑tuned Robotic Foundation Model (RFM) on the joint‑level
*TNG‑Pick‑And‑Place‑Cube‑UR5‑IK‑Abs‑Play‑v0* environment.

start with:
python scripts/tng/ur5_pick_and_place_eval.py --headless --enable_cameras --blackwell

"""

import torch
import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Evaluate RFM on UR5 pick‑and‑place (joint control)")
parser.add_argument("--chunk_size", type=int, default=16, help="Future horizon K that RFM outputs")
parser.add_argument("--action_horizon", type=int, default=10, help="Action horizon for the RFM")
parser.add_argument("--num_envs", type=int, default=3, help="Number of environments to simulate.")
parser.add_argument("--disable_fabric", action="store_true", help="Disable Fabric (USD I/O fallback)")
parser.add_argument("--from_yaml", type=str, default=None, help="Path to the benchmark YAML file.")
parser.add_argument("--blackwell", action="store_true", help="Enable this when using a RTX 50xx GPU")

temp_args, _ = parser.parse_known_args()

if temp_args.blackwell:
    parser.add_argument(
        "--renderer",
        type=str,
        default="PathTracing",
        choices=["RayTracedLighting", "PathTracing"],
        help="Renderer to use.",
    )
    parser.add_argument(
        "--samples_per_pixel_per_frame",
        type=int,
        default=1,
        help="Number of samples per pixel per frame.",
    )

AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

import gymnasium as gym
from isaaclab_tasks.utils.parse_cfg import parse_env_cfg 
from isaaclab_tasks.manager_based.tng_ur5.ur5_pick_and_place.pick_and_place_env_cfg import PickAndPlaceEnvCfg
from utils.tng_sctipt_utils import patch_env_config_for_configuration_scheduling
from isaaclab_tasks.manager_based.tng_ur5.rfm_utils.gr00t_inference_client import Gr00tInferenceClient
from isaaclab_tasks.manager_based.tng_ur5.rfm_utils.rfm_action_manager import RFMActionManager
from isaaclab_tasks.manager_based.tng_ur5.env_utils.env_config_scheduler import EnvConfigSchedulerBenchmark
from isaaclab_tasks.manager_based.tng_ur5.ur5_pick_and_place.pick_and_place_env_cfg import DEFAULT_PROMPT

def print_verbose_info_for_subtasks(subtasks: list[str], obs) -> None:
    for subtask in subtasks:
        print(f"Envs achieving subtask '{subtask}':{obs['subtasks'][subtask].nonzero(as_tuple=False).squeeze(-1).tolist()}")

def main(argv: list[str] | None = None) -> None:
    env_cfg: PickAndPlaceEnvCfg = parse_env_cfg(
        "TNG-Pick-And-Place-Cube-UR5-IK-Abs-Play-v0",
        device=args.device,
        use_fabric=not args.disable_fabric,
        num_envs=args.num_envs
    )


    if args.from_yaml:
        patch_env_config_for_configuration_scheduling(env_cfg, args.from_yaml, "benchmark")

    gr00t_client: Gr00tInferenceClient = Gr00tInferenceClient(host="localhost", port=5555)
    env: gym.Env = gym.make("TNG-Pick-And-Place-Cube-UR5-IK-Abs-Play-v0", cfg=env_cfg)
    obs, _ = env.reset()

    scheduler: EnvConfigSchedulerBenchmark = env.unwrapped.extras.get("scheduler", None)

    num_envs: int = env.unwrapped.num_envs
    action_dim: int = env.unwrapped.action_space.shape[-1]
    device: torch.device = env.unwrapped.device

    rfm_action_manager = RFMActionManager(num_envs, args.chunk_size, args.action_horizon, action_dim, gr00t_client, device)
    done_counter = 0
    success_counter = 0
    env_ids = torch.arange(num_envs, device=device)
    idle_mask = torch.zeros(num_envs, dtype=torch.bool, device=device)

    try:
        while simulation_app.is_running():

            with torch.inference_mode():

                if scheduler:
                    prompts = scheduler.get_prompts(env_ids)
                    scheduler.update_metrics(obs)
                    idle_mask = scheduler.idle_mask.clone()
                else:
                    prompts = [DEFAULT_PROMPT]*num_envs
                    print_verbose_info_for_subtasks(["object_reached_target", "object_in_gripper_reach", "object_lifted"], obs)

                print(f"Successful terminations: {success_counter} / {done_counter}")


                #idle_mask = scheduler.idle_mask if scheduler else torch.zeros(num_envs, dtype=torch.bool, device=device)
                env_actions = rfm_action_manager.get_targets(obs, prompts, idle_mask)
                obs, _, terminated, truncated, _ = env.step(env_actions)
                done_mask = (terminated | truncated).to(device=device)
                rfm_action_manager.update_target_tracking(obs, done_mask)

                if done_mask.any():
                    relevant_dones = done_mask & (~idle_mask)
                    relevant_successes = env.unwrapped.termination_manager.get_term("success").to(device=device) & (~idle_mask)
                    done_counter += sum(relevant_dones)
                    success_counter += sum(relevant_successes)
                    if scheduler:
                        all_assigned = (scheduler.cursor >= len(scheduler.order))
                        inflight = len([case for case in scheduler.cases_being_processed if case is not None])
                        if all_assigned and inflight == 0:
                            overall_success_rate = success_counter / done_counter if done_counter > 0 else 0.0
                            print(f"Overall success rate: {overall_success_rate*100:.1f}% ({success_counter} / {done_counter})")
                            print("All cases processed, exiting.")
                            scheduler.finalize_and_store_results()
                            break 


    finally:
        try: env.close()
        finally: simulation_app.close()


if __name__ == "__main__":
    main()