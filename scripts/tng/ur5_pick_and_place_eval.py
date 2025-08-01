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
parser.add_argument("--joint_tol", type=float, default=0.02, help="Joint convergence tolerance (rad/m)")
parser.add_argument("--disable_fabric", action="store_true", help="Disable Fabric (USD I/O fallback)")
parser.add_argument("--num_envs", type=int, default=None, help="Number of parallel environments")
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

    # ------------------------------------------------------------------
    # High‑level API
    # ------------------------------------------------------------------
    def needs_refill(self) -> torch.Tensor:
        """Boolean mask of envs whose chunk has been fully consumed."""
        return self.ptr >= self.chunk_size

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
                # Call RFM for the **entire batch** (simpler; slice if expensive)
                new_chunk = torch.zeros(num_envs, args.chunk_size, action_dim, device=device)#rfm(obs)[..., : args.chunk_size, :]
                if new_chunk.shape != (num_envs, args.chunk_size, action_dim):
                    raise ValueError(
                        "RFM returned shape {} — expected {}".format(
                            new_chunk.shape, (num_envs, args.chunk_size, action_dim)
                        )
                    )
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