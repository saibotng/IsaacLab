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




import argparse

from isaaclab.app import AppLauncher
parser = argparse.ArgumentParser(description="Evaluate RFM on UR5 pick‑and‑place (joint control)")
parser.add_argument("--rfm", type=int, default=8, help="Path/module of the RFM to load")
parser.add_argument("--chunk_size", type=int, default=8, help="Future horizon K that RFM outputs")
parser.add_argument("--joint_tol", type=float, default=0.0001, help="Joint convergence tolerance (rad/m)")
parser.add_argument("--disable_fabric", action="store_true", help="Disable Fabric (USD I/O fallback)")
parser.add_argument("--num_envs", type=int, default=None, help="Number of parallel environments")

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

import gymnasium as gym
import torch
from isaaclab_tasks.utils.parse_cfg import parse_env_cfg  # deferred import

# -----------------------------------------------------------------------------
# Utilities
# -----------------------------------------------------------------------------

def load_rfm(rfm_id: str, device: torch.device) -> "callable":
    """Load the RFM and move it to *device*.

    * Accepts either ``path/to/checkpoint.pt`` (expects a ``dict`` with a
      ``"model"`` key *or* a standalone ``nn.Module``), **or** a dotted
      Python import path optionally followed by ``:attribute``.
    * Must return a **callable** that maps ``(N, *obs_shape) -> (N, K,
      action_dim)`` with contiguous tensors on *device*.
    """
    return None
    module_path, _, attr = rfm_id.partition(":")

    if attr:  # dotted import like "my_pkg.models.rfm:RFMPolicy"
        module = importlib.import_module(module_path)
        rfm = getattr(module, attr)
    else:  # assume checkpoint
        checkpoint = torch.load(rfm_id, map_location=device)
        rfm = checkpoint["model"] if isinstance(checkpoint, dict) else checkpoint

    if not callable(rfm):
        raise TypeError("Loaded RFM is not callable — please wrap it or implement __call__.")

    try:
        rfm.to(device)  # type: ignore[attr-defined]
    except AttributeError:
        pass  # Not an nn.Module → ignore

    return rfm

def reset_rfm(done_ids: torch.Tensor):
    """Reset the RFM state for the given envs."""
    # This is a no-op in this example, but you can implement
    # RFM-specific reset logic here if needed.
    pass


class ActionBuffer:
    """Chunked action buffer with *reach‑to‑advance* gating."""

    def __init__(self, num_envs: int, chunk_size: int, action_dim: int, device: torch.device):
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
    rfm = load_rfm(args.rfm, device)
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
                        start_pos = torch.tensor([0.0, -1.712, 1.712, -1.712, -1.571, 0.0, 0.0, 0.0], device=device)
                        
                        joint_deltas = torch.tensor([
                            [0.15, 0.08, -0.05, 0.06, 0.04, 0.02, 0.01, 0.005],    # Step 1
                            [0.25, 0.12, -0.08, 0.09, 0.06, 0.03, 0.015, 0.008],   # Step 2
                            [0.30, 0.15, -0.10, 0.12, 0.08, 0.04, 0.02, 0.01],     # Step 3
                            [0.32, 0.18, -0.12, 0.15, 0.10, 0.05, 0.025, 0.012],   # Step 4
                            [0.30, 0.20, -0.15, 0.16, 0.15, 0.051, 0.027, 0.013],   # Step 5
                            [0.45, 0.23, -0.20, 0.17, 0.17, 0.055, 0.030, 0.020],  # Step 6
                            [0.48, 0.29, -0.26, 0.20, 0.18, 0.056, 0.031, 0.026],  # Step 7
                            [0.50, 0.30, -0.30, 0.24, 0.20, 0.060, 0.038, 0.030],  # Step 8
                        ], device=device)
                        
                        # Generate cumulative positions
                        for step in range(args.chunk_size):
                            new_chunk[env_idx, step] = start_pos + joint_deltas[step]
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

    # ------------------------------------------------------------------
    # Clean shutdown
    # ------------------------------------------------------------------
    env.close()
    simulation_app.close()


if __name__ == "__main__":
    main()