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
"""
from utils.gr00t_inference_client import RobotInferenceClient
import numpy as np
import torch
from collections import deque
import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Evaluate RFM on UR5 pick‑and‑place (joint control)")
parser.add_argument("--chunk_size", type=int, default=16, help="Future horizon K that RFM outputs")
parser.add_argument("--action_horizon", type=int, default=10, help="Action horizon for the RFM")
parser.add_argument("--joint_tol", type=float, default=0.003, help="Joint convergence tolerance (rad/m)")
parser.add_argument("--num_envs", type=int, default=2, help="Number of environments to simulate.")
parser.add_argument("--disable_fabric", action="store_true", help="Disable Fabric (USD I/O fallback)")
parser.add_argument("--blackwell", action="store_true", help="Enable this when using a RTX 50xx GPU")

# Parse args first to check if renderer should be enabled
temp_args, _ = parser.parse_known_args()

# Conditionally add renderer arguments
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
# AppLauncher CLI
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

# ------------------------------------------------------------------
# Build environment
# ------------------------------------------------------------------



import importlib
from pathlib import Path
from typing import Tuple
import sys

import gymnasium as gym
import torch
from isaaclab_tasks.utils.parse_cfg import parse_env_cfg 
import time




TASK_DESCRIPTION = "Pick up the blue cube and place it on the black platform"
DELTA_ACTIONS = True
SNAP_GRIPPER_ACTIONS = True
GRIPPER_SNAP_THRESHOLD = 0.015
ENFORCE_GRIPPER_DELTA = 0.001


def maybe_snap_gripper_actions(gripper_actions):
    if SNAP_GRIPPER_ACTIONS:
        return [x + ENFORCE_GRIPPER_DELTA if x > GRIPPER_SNAP_THRESHOLD else x for x in gripper_actions]
    return gripper_actions

def convert_raw_abs_action_to_action_chunk(action) -> torch.Tensor:
    gripper_actions = maybe_snap_gripper_actions(action["action.gripper"])
    gripper_arr = np.stack([gripper_actions, gripper_actions], axis=1)
    arm_arr = action["action.robot_arm"]
    action_arr = np.concatenate([arm_arr, gripper_arr], axis=1)
    action_chunk = torch.from_numpy(action_arr).to(device='cuda')
    return action_chunk

def convert_raw_delta_action_to_action_chunk(action, observation) -> torch.Tensor:
    gripper_deltas = action["action.delta_gripper"]
    arm_deltas = action["action.delta_robot_arm"]

    gripper_state = observation["state.gripper"].squeeze()
    arm_state = observation["state.robot_arm"].squeeze()

    gripper_actions = np.cumsum(gripper_deltas, axis=0) + gripper_state
    arm_actions     = np.cumsum(arm_deltas,     axis=0) + arm_state

    gripper_actions = maybe_snap_gripper_actions(gripper_actions)

    gripper_arr = np.stack([gripper_actions, gripper_actions], axis=1)

    action_arr = np.concatenate([arm_actions, gripper_arr], axis=1)
    action_chunk = torch.from_numpy(action_arr).to(device='cuda')
    return action_chunk

def convert_observations_to_gr00t_format(env_obs: dict):
        gr00t_obs = {
            "video.camera_wrist": env_obs["cameras"][:, :, 6:].cpu().unsqueeze(0).numpy(),
            "video.camera_global_side": env_obs["cameras"][:, :, 3:6].cpu().unsqueeze(0).numpy(),
            "video.camera_global_front": env_obs["cameras"][:, :, :3].cpu().unsqueeze(0).numpy(),
            "state.robot_arm": env_obs["joints"][:6].cpu().unsqueeze(0).numpy(),
            "state.gripper": env_obs["joints"][6:7].cpu().unsqueeze(0).numpy(),
            "annotation.human.action.task_description": [TASK_DESCRIPTION],
        }
        return gr00t_obs

def convert_gr00t_action_to_state_action_chunk(gr00t_action, gr00t_obs) -> torch.Tensor:
        for key, value in gr00t_action.items():
            print(f"Action: {key}: {value.shape}")
        if DELTA_ACTIONS:
            action_chunk = convert_raw_delta_action_to_action_chunk(gr00t_action, gr00t_obs)
        else:
            action_chunk = convert_raw_abs_action_to_action_chunk(gr00t_action)
        return action_chunk


class ActionBuffer:

    def __init__(self, num_envs: int, chunk_size: int, action_horizon: int, action_dim: int, device: torch.device):
        self.num_envs = num_envs
        self.chunk_size = chunk_size
        self.action_horizon = action_horizon
        self.action_dim = action_dim
        self.device = device

        self.buffer = torch.zeros(num_envs, chunk_size, action_dim, device=device)
        self.ptr = torch.full((num_envs,), chunk_size, dtype=torch.long, device=device)
        self.current_target = torch.zeros(num_envs, action_dim, device=device)
        self.last_action_reached = torch.ones(num_envs, dtype=torch.bool, device=device)

    def needs_refill(self) -> torch.Tensor:
        return self.last_action_reached
 
    def refill(self, mask: torch.Tensor, new_chunk: torch.Tensor):
        if mask.ndim != 1 or mask.shape[0] != self.num_envs:
            raise ValueError("mask must be shape [num_envs]")

        m = mask.sum().item()
        if m == 0:
            return

        # normalize new_chunk shape
        if new_chunk.shape == (m, self.chunk_size, self.action_dim):
            src = new_chunk
        else:
            raise ValueError(f"new_chunk shape {tuple(new_chunk.shape)} incompatible with mask and buffer")

        # write only masked rows
        self.buffer[mask] = src
        self.ptr[mask] = 0
        self.current_target[mask] = self.buffer[mask, 0, :]
        # If chunk_size==1 we immediately need a refill after executing this target once.
        self.last_action_reached[mask] = (self.chunk_size == 1)


    def update_targets(self, update_mask: torch.Tensor):
        at_last_now = update_mask & (self.ptr == (self.action_horizon - 1))
        if at_last_now.any():
            self.last_action_reached[at_last_now] = True

        can_advance = update_mask & (self.ptr < self.action_horizon - 1)
        if can_advance.any():
            self.ptr[can_advance] += 1
            next_idx = self.ptr[can_advance]
            self.current_target[can_advance] = self.buffer[can_advance, next_idx, :]
        
    def reset(self, done_mask: torch.Tensor):
        if not done_mask.any():
            return
        self.buffer[done_mask] = 0
        self.current_target[done_mask] = 0
        self.ptr[done_mask] = self.chunk_size
        self.last_action_reached[done_mask] = True

    @property
    def actions(self) -> torch.Tensor:
        return self.current_target


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

    while simulation_app.is_running():

        with torch.inference_mode():
            refill_mask = buffer.needs_refill()

            if refill_mask.any():
                full_obs = env.unwrapped.observation_manager.compute()
                env_ids = refill_mask.nonzero(as_tuple=False).squeeze(-1).tolist()
                new_chunk = torch.empty(len(env_ids), args.chunk_size, action_dim, device=device)

                for k, env_idx in enumerate(env_ids):
                    env_obs = {
                        "cameras": full_obs["cameras"][env_idx],
                        "joints":  full_obs["joints"]["joint_pos"][env_idx],
                    }
                    gr00t_obs = convert_observations_to_gr00t_format(env_obs)
                    gr00t_action = gr00t_client.get_action(gr00t_obs)
                    new_chunk[k] = convert_gr00t_action_to_state_action_chunk(gr00t_action, gr00t_obs)

                buffer.refill(refill_mask, new_chunk)

            actions = buffer.actions
            obs, _, terminated, truncated, _ = env.step(actions)
            done_mask = (terminated | truncated).to(device=device)


            q = obs['joints']['joint_pos'] 
            err_arm = torch.abs(q[:,:6] - buffer.actions[:,:6]).max(dim=-1).values  
            arm_reached = err_arm < args.joint_tol
            err_deque.append(err_arm)

            stacked_err = torch.stack(list(err_deque), dim=0)
            err_span = stacked_err.max(dim=0).values - stacked_err.min(dim=0).values

            action_idx_deque.append(buffer.ptr.clone())
            stacked_action_idx = torch.stack(list(action_idx_deque), dim=0)
            action_idx_span = stacked_action_idx.max(dim=0).values - stacked_action_idx.min(dim=0).values

            stuck = (err_span < 1e-5) & (action_idx_span == 0)
            if stuck.any():
                print(f"Stuck envs: {stuck.sum()} / {num_envs}")
            
            envs_to_update_targets = (arm_reached | stuck)
            buffer.update_targets(envs_to_update_targets)

            if done_mask.any():
                buffer.reset(done_mask)

    env.close()
    simulation_app.close()


if __name__ == "__main__":
    main()