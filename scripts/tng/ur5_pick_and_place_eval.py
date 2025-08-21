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
from utils.gr00t_inference_client import RobotInferenceClient
from utils.action_buffer import ActionBuffer
import numpy as np
import torch
from collections import deque
import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Evaluate RFM on UR5 pick‑and‑place (joint control)")
parser.add_argument("--chunk_size", type=int, default=16, help="Future horizon K that RFM outputs")
parser.add_argument("--action_horizon", type=int, default=10, help="Action horizon for the RFM")
parser.add_argument("--joint_tol", type=float, default=0.003, help="Joint convergence tolerance (rad/m)")
parser.add_argument("--gripper_vel_tol", type=float, default=0.01, help="Gripper velocity tolerance (rad/m)")
parser.add_argument("--num_envs", type=int, default=2, help="Number of environments to simulate.")
parser.add_argument("--disable_fabric", action="store_true", help="Disable Fabric (USD I/O fallback)")
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




def main(argv: list[str] | None = None) -> None:
    env_cfg = parse_env_cfg(
        "TNG-Pick-And-Place-Cube-UR5-IK-Abs-Play-v0",
        device=args.device,
        use_fabric=not args.disable_fabric,
        num_envs=args.num_envs
    )

    gr00t_client: RobotInferenceClient = RobotInferenceClient(host="localhost", port=5555)

    env: gym.Env = gym.make("TNG-Pick-And-Place-Cube-UR5-IK-Abs-Play-v0", cfg=env_cfg)
    obs, _ = env.reset()

    num_envs: int = env.unwrapped.num_envs
    action_dim: int = env.unwrapped.action_space.shape[-1]
    device: torch.device = env.unwrapped.device

    buffer = ActionBuffer(num_envs, args.chunk_size, args.action_horizon, action_dim, device)

    err_deque = deque(maxlen=4)
    err_deque.append(torch.zeros(num_envs, device=device)) 

    action_idx_deque = deque(maxlen=4)
    action_idx_deque.append(torch.zeros(num_envs, device=device)) 
    done_counter = 0
    success_counter = 0
    try:
        while simulation_app.is_running():

            with torch.inference_mode():
                buffer.maybe_get_new_actions(env, gr00t_client)

                actions = buffer.actions
                obs, _, terminated, truncated, _ = env.step(actions)
                done_mask = (terminated | truncated).to(device=device)

                q = obs['arm_joints']['arm_joint_pos'] 
                number_of_arm_joints = q.shape[-1]

                err_arm = torch.abs(q - buffer.actions[:,:number_of_arm_joints]).max(dim=-1).values  
                err_deque.append(err_arm)
                stacked_err = torch.stack(list(err_deque), dim=0)
                err_span = stacked_err.max(dim=0).values - stacked_err.min(dim=0).values

                gripper_vel = obs['gripper_joint']['gripper_joint_vel'].squeeze()
                gripper_reached = (gripper_vel.abs() < args.gripper_vel_tol).to(device=device)

                action_idx_deque.append(buffer.ptr.clone())
                stacked_action_idx = torch.stack(list(action_idx_deque), dim=0)
                action_idx_span = stacked_action_idx.max(dim=0).values - stacked_action_idx.min(dim=0).values

                arm_reached = err_arm < args.joint_tol
                stuck = (err_span < 1e-5) & (action_idx_span == 0)
                envs_to_update_targets = ((arm_reached & gripper_reached) | stuck)

                buffer.maybe_update_targets(envs_to_update_targets)
                buffer.maybe_reset_buffer(done_mask)
                


                    

                if done_mask.any():
                    done_counter += sum(done_mask)
                    success_counter += sum(env.unwrapped.termination_manager.get_term("success"))

                print(f"Envs reached target: {obs['subtasks']['object_reached_target'].nonzero(as_tuple=False).squeeze(-1).tolist()}")
                print(f"Envs in Gripper Reach: {obs['subtasks']['object_in_gripper_reach'].nonzero(as_tuple=False).squeeze(-1).tolist()}")
                print(f"Envs lifted: {obs['subtasks']['object_lifted'].nonzero(as_tuple=False).squeeze(-1).tolist()}")
                print(f"Envs Stuck: {stuck.nonzero(as_tuple=False).squeeze(-1).tolist()}")
                print(f"Successful terminations: {success_counter} / {done_counter}")
                    


    finally:
        try: env.close()
        finally: simulation_app.close()


if __name__ == "__main__":
    main()