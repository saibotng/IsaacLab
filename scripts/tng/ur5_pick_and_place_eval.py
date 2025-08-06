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
from typing import Any, Dict, Optional, Union
from pydantic import BaseModel
from abc import ABC, abstractmethod
import zmq
from io import BytesIO
import numpy as np
import torch

class ModalityConfig(BaseModel):
    """Configuration for a modality."""

    delta_indices: list[int]
    """Delta indices to sample relative to the current index. The returned data will correspond to the original data at a sampled base index + delta indices."""
    modality_keys: list[str]
    """The keys to load for the modality in the dataset."""


class BasePolicy(ABC):
    @abstractmethod
    def get_action(self, observations: Dict[str, Any]) -> Dict[str, Any]:
        """
        Abstract method to get the action for a given state.

        Args:
            observations: The observations from the environment.

        Returns:
            The action to take in the environment in dictionary format.
        """
        raise NotImplementedError

    @abstractmethod
    def get_modality_config(self) -> Dict[str, ModalityConfig]:
        """
        Return the modality config of the policy.
        """
        raise NotImplementedError

class TorchSerializer:
    @staticmethod
    def to_bytes(data: dict) -> bytes:
        buffer = BytesIO()
        torch.save(data, buffer)
        return buffer.getvalue()

    @staticmethod
    def from_bytes(data: bytes) -> dict:
        buffer = BytesIO(data)
        obj = torch.load(buffer, weights_only=False)
        return obj
    
class BaseInferenceClient:
    def __init__(
        self,
        host: str = "localhost",
        port: int = 5555,
        timeout_ms: int = 15000,
        api_token: str = None,
    ):
        self.context = zmq.Context()
        self.host = host
        self.port = port
        self.timeout_ms = timeout_ms
        self.api_token = api_token
        self._init_socket()

    def _init_socket(self):
        """Initialize or reinitialize the socket with current settings"""
        self.socket = self.context.socket(zmq.REQ)
        self.socket.connect(f"tcp://{self.host}:{self.port}")

    def ping(self) -> bool:
        try:
            self.call_endpoint("ping", requires_input=False)
            return True
        except zmq.error.ZMQError:
            self._init_socket()  # Recreate socket for next attempt
            return False

    def kill_server(self):
        """
        Kill the server.
        """
        self.call_endpoint("kill", requires_input=False)

    def call_endpoint(
        self, endpoint: str, data: dict | None = None, requires_input: bool = True
    ) -> dict:
        """
        Call an endpoint on the server.

        Args:
            endpoint: The name of the endpoint.
            data: The input data for the endpoint.
            requires_input: Whether the endpoint requires input data.
        """
        request: dict = {"endpoint": endpoint}
        if requires_input:
            request["data"] = data
        if self.api_token:
            request["api_token"] = self.api_token

        self.socket.send(TorchSerializer.to_bytes(request))
        message = self.socket.recv()
        response = TorchSerializer.from_bytes(message)

        if "error" in response:
            raise RuntimeError(f"Server error: {response['error']}")
        return response

    def __del__(self):
        """Cleanup resources on destruction"""
        self.socket.close()
        self.context.term()

class RobotInferenceClient(BaseInferenceClient, BasePolicy):
    """
    Client for communicating with the RealRobotServer
    """

    def __init__(self, host: str = "localhost", port: int = 5555, api_token: str = None):
        super().__init__(host=host, port=port, api_token=api_token)

    def get_action(self, observations: Dict[str, Any]) -> Dict[str, Any]:
        return self.call_endpoint("get_action", observations)

    def get_modality_config(self) -> Dict[str, ModalityConfig]:
        return self.call_endpoint("get_modality_config", requires_input=False)



import argparse

from isaaclab.app import AppLauncher
parser = argparse.ArgumentParser(description="Evaluate RFM on UR5 pick‑and‑place (joint control)")
#parser.add_argument("--rfm", type=int, default=8, help="Path/module of the RFM to load")
parser.add_argument("--chunk_size", type=int, default=16, help="Future horizon K that RFM outputs")
parser.add_argument("--joint_tol", type=float, default=0.005, help="Joint convergence tolerance (rad/m)")
parser.add_argument("--disable_fabric", action="store_true", help="Disable Fabric (USD I/O fallback)")
parser.add_argument("--num_envs", type=int, default=None, help="Number of parallel environments")

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
from isaaclab_tasks.utils.parse_cfg import parse_env_cfg  # deferred import
import time



TASK_DESCRIPTION = "Pick up the red cube and place it on the green area"

# -----------------------------------------------------------------------------
# Utilities
# -----------------------------------------------------------------------------


def reset_rfm(done_ids: torch.Tensor):
    """Reset the RFM state for the given envs."""
    # This is a no-op in this example, but you can implement
    # RFM-specific reset logic here if needed.
    pass

def request_rfm_server(env_obs: dict) -> None:
        rfm_obs = {
            "video.camera_wrist": env_obs["cameras"][:, :, 3:].cpu().unsqueeze(0).numpy(),
            "video.camera_global": env_obs["cameras"][:, :, :3].cpu().unsqueeze(0).numpy(),
            "state.robot_arm": env_obs["joints"][:6].cpu().unsqueeze(0).numpy(),
            "state.gripper": env_obs["joints"][6:7].cpu().unsqueeze(0).numpy(),
            "annotation.human.action.task_description": [TASK_DESCRIPTION],
        }
        action = zmq_client_call(rfm_obs)

        for key, value in action.items():
            print(f"Action: {key}: {value.shape}")

        gripper_arr = np.stack([action["action.gripper"], action["action.gripper"]], axis=1)
        arm_arr = action["action.robot_arm"]
        action_arr = np.concatenate([arm_arr, gripper_arr], axis=1)
        action_chunk = torch.from_numpy(action_arr).to(device='cuda')
        return action_chunk

def zmq_client_call(obs: dict):
    policy_client = RobotInferenceClient(host="localhost", port=5555, api_token=None)

    print("Available modality config available:")
    modality_configs = policy_client.get_modality_config()
    print(modality_configs.keys())

    time_start = time.time()
    action = policy_client.get_action(obs)
    print(f"Total time taken to get action from server: {time.time() - time_start} seconds")
    return action

class ActionBuffer:
    """Chunked action buffer with *reach‑to‑advance* gating."""

    def __init__(self, num_envs: int, chunk_size: int, action_dim: int, device: torch.device):
        self.num_envs = num_envs
        self.buffer = torch.zeros(num_envs, chunk_size, action_dim, device=device)
        self.ptr = torch.full((num_envs,), chunk_size, dtype=torch.long, device=device)  # forces refill
        self.chunk_size = chunk_size
        self.action_dim = action_dim
        self.current_target = torch.zeros(num_envs, action_dim, device=device)
        self.last_action_reached = torch.ones(num_envs, dtype=torch.bool, device=device)  # initially all need refill

    # ------------------------------------------------------------------
    # High‑level API
    # ------------------------------------------------------------------
    def needs_refill(self) -> torch.Tensor:
        """Boolean mask of envs whose chunk has been fully consumed."""
        return self.last_action_reached
 
    def refill(self, new_chunk: torch.Tensor):
        """Overwrite buffer and reset pointers."""
        if new_chunk.shape != self.buffer.shape:
            raise ValueError(
                f"Chunk shape {new_chunk.shape} does not match buffer shape {self.buffer.shape}."
            )
        self.buffer.copy_(new_chunk)
        self.ptr.zero_()
        # First waypoint becomes the active target
        self.current_target.copy_(new_chunk[:, 0, :])

    def update_targets(self, reached_mask: torch.Tensor):
        """Advance to the **next** waypoint *only* for envs that reached the current one."""
        can_advance = reached_mask & (self.ptr < self.chunk_size - 1)
        self.last_action_reached = reached_mask & (self.ptr == self.chunk_size - 1)
        if can_advance.any():
            self.ptr[can_advance] += 1
            # Gather next target
            next_idx = self.ptr[can_advance]
            self.current_target[can_advance] = self.buffer[can_advance, next_idx, :]

    def mark_done(self, done_ids: torch.Tensor):
        """On env reset, invalidate its chunk so it is refilled next step."""
        self.ptr[done_ids] = self.chunk_size  # will trigger needs_refill()
        
    def reset(self, done_mask: torch.Tensor):
        """Reset the buffer for done environments only."""
        # Reset buffer for done environments
        self.buffer[done_mask] = 0
        # Force refill only for done environments on next step
        self.last_action_reached[done_mask] = True

    @property
    def actions(self) -> torch.Tensor:
        """Return the *currently active* target for all envs."""
        return self.current_target


# -----------------------------------------------------------------------------
# Main evaluation loop
# -----------------------------------------------------------------------------

def main(argv: list[str] | None = None) -> None:
    env_cfg = parse_env_cfg(
        "TNG-Pick-And-Place-Cube-UR5-IK-Abs-Play-v0",
        device=args.device,
        num_envs=args.num_envs,
        use_fabric=not args.disable_fabric,
    )

    env: gym.Env = gym.make("TNG-Pick-And-Place-Cube-UR5-IK-Abs-Play-v0", cfg=env_cfg)
    obs, _ = env.reset()

    num_envs: int = env.unwrapped.num_envs
    action_dim: int = env.unwrapped.action_space.shape[-1]
    device: torch.device = env.unwrapped.device

    # ------------------------------------------------------------------
    # Instantiate helpers
    # ------------------------------------------------------------------
    #rfm = load_rfm(args.rfm, device)
    buffer = ActionBuffer(num_envs, args.chunk_size, action_dim, device)

    # ------------------------------------------------------------------
    # Simulation loop
    # ------------------------------------------------------------------
    while simulation_app.is_running():
        with torch.inference_mode():
            # ------------------------------------------------------------------
            # (1) Rollout new chunks where necessary
            # ------------------------------------------------------------------
            refill_mask = buffer.needs_refill()
            if refill_mask.any():
                # Generate a realistic action chunk for demonstration
                # Starting from current joint positions
                
                # Define a smooth trajectory for 8 steps
                new_chunk = torch.zeros(num_envs, args.chunk_size, action_dim, device=device)
                
                for env_idx in range(num_envs):
                    
                    if refill_mask[env_idx]:
                        env_obs = {
                            "cameras": obs["cameras"][env_idx],
                            "joints": obs["joints"]["joint_pos"][env_idx],
                        }

                        new_chunk[env_idx] = request_rfm_server(env_obs)  # (K, action_dim)

                    else:
                        # For envs that don't need refill, copy current chunk
                        new_chunk[env_idx] = buffer.buffer[env_idx]
                
                if new_chunk.shape != (num_envs, args.chunk_size, action_dim):
                    raise ValueError(
                        "RFM returned shape {} — expected {}".format(
                            new_chunk.shape, (num_envs, args.chunk_size, action_dim)
                        )
                    )



                # Call RFM for the **entire batch** (simpler; slice if expensive)
                # new_chunk = torch.zeros(num_envs, args.chunk_size, action_dim, device=device)#rfm(obs)[..., : args.chunk_size, :]
                # if new_chunk.shape != (num_envs, args.chunk_size, action_dim):
                #     raise ValueError(
                #         "RFM returned shape {} — expected {}".format(
                #             new_chunk.shape, (num_envs, args.chunk_size, action_dim)
                #         )
                #     )
                buffer.refill(new_chunk)

            # ------------------------------------------------------------------
            # (2) Send current target actions to env
            # ------------------------------------------------------------------
            actions = buffer.actions
            obs, _, terminated, truncated, _ = env.step(actions)
            done_mask = terminated | truncated

            # ------------------------------------------------------------------
            # (3) Measure convergence & advance targets
            # ------------------------------------------------------------------
            q = obs['joints']['joint_pos']
            err = torch.abs(q - buffer.actions).max(dim=-1).values  # shape (N,)
            reached = err < args.joint_tol
            buffer.update_targets(reached)

            # ------------------------------------------------------------------
            # (4) Handle resets
            # ------------------------------------------------------------------
            if done_mask.any():
                reset_rfm(done_mask.nonzero(as_tuple=False).squeeze(-1))
                buffer.reset(done_mask)
                env.reset()

    # ------------------------------------------------------------------
    # Clean shutdown
    # ------------------------------------------------------------------
    env.close()
    simulation_app.close()


if __name__ == "__main__":
    main()