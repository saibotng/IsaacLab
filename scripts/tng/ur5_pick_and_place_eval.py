"""
Evaluate a fine‑tuned Robotic Foundation Model (RFM) on the joint‑level
*TNG‑Pick‑And‑Place‑Cube‑UR5‑IK‑Abs‑Play‑v0* environment.

Changes compared to the previous draft
======================================
* **Motion‑completion gating**: the next target joint configuration is
  dispatched **only after** the robot has reached (within a configurable
  tolerance) the current target.  This guarantees the arm settles on each
  waypoint before progressing through the RFM‑predicted chunk.
* Added CLI flag ``--joint_tol`` (default ``0.02`` rad or meters for
  prismatic joints).
* Lightweight helper ``extract_joint_pos`` tries common access patterns
  to read joint positions from the environment.  Adjust as needed for
  your custom task.

Assumptions & integration points are unchanged – replace ``load_rfm``
with your actual model loader.

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
parser.add_argument("--bench_from_yaml", type=str, default=None, help="Path to the benchmark YAML file.")
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
from isaaclab_tasks.manager_based.tng_ur5.env_utils.env_config_scheduler import EnvConfigScheduler

DEFAULT_TASK_DESCRIPTION = "Pick up the blue cube and place it on the black platform"



def main(argv: list[str] | None = None) -> None:
    env_cfg: PickAndPlaceEnvCfg = parse_env_cfg(
        "TNG-Pick-And-Place-Cube-UR5-IK-Abs-Play-v0",
        device=args.device,
        use_fabric=not args.disable_fabric,
        num_envs=args.num_envs
    )
    result_dict = {}
    if args.bench_from_yaml:
        patch_env_config_for_configuration_scheduling(env_cfg, args.bench_from_yaml)

    gr00t_client: Gr00tInferenceClient = Gr00tInferenceClient(host="localhost", port=5555)
    env: gym.Env = gym.make("TNG-Pick-And-Place-Cube-UR5-IK-Abs-Play-v0", cfg=env_cfg)
    obs, _ = env.reset()

    scheduler: EnvConfigScheduler = env.unwrapped.extras.get("scheduler", None)

    num_envs: int = env.unwrapped.num_envs
    action_dim: int = env.unwrapped.action_space.shape[-1]
    device: torch.device = env.unwrapped.device

    rfm_action_manager = RFMActionManager(num_envs, args.chunk_size, args.action_horizon, action_dim, gr00t_client, device)
    done_counter = 0
    success_counter = 0
    env_ids = torch.arange(num_envs, device=device)
    try:
        while simulation_app.is_running():

            with torch.inference_mode():
                #TODO: compute really necessary?
                obs = env.unwrapped.observation_manager.compute()
                prompts = scheduler.get_prompts(env_ids) if scheduler else [DEFAULT_TASK_DESCRIPTION]*num_envs

                env_actions = rfm_action_manager.get_targets(obs, prompts)
                obs, _, terminated, truncated, _ = env.step(env_actions)
                done_mask = (terminated | truncated).to(device=device)
                rfm_action_manager.update_target_tracking(obs, done_mask)

                if done_mask.any():
                    done_counter += sum(done_mask)
                    success_counter += sum(env.unwrapped.termination_manager.get_term("success"))

                if scheduler:
                    all_assigned = (scheduler.cursor >= len(scheduler.order))
                    inflight = len(scheduler.cases_being_processed)
                    if all_assigned and inflight == 0:
                        break

                print(f"Envs reached target: {obs['subtasks']['object_reached_target'].nonzero(as_tuple=False).squeeze(-1).tolist()}")
                print(f"Envs in Gripper Reach: {obs['subtasks']['object_in_gripper_reach'].nonzero(as_tuple=False).squeeze(-1).tolist()}")
                print(f"Envs lifted: {obs['subtasks']['object_lifted'].nonzero(as_tuple=False).squeeze(-1).tolist()}")
                print(f"Successful terminations: {success_counter} / {done_counter}")
                        


    finally:
        try: env.close()
        finally: simulation_app.close()


if __name__ == "__main__":
    main()