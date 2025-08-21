import torch

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

        if new_chunk.shape == (m, self.chunk_size, self.action_dim):
            src = new_chunk
        else:
            raise ValueError(f"new_chunk shape {tuple(new_chunk.shape)} incompatible with mask and buffer")

        self.buffer[mask] = src
        self.ptr[mask] = 0
        self.current_target[mask] = self.buffer[mask, 0, :]
        self.last_action_reached[mask] = (self.chunk_size == 1)


    def maybe_update_targets(self, update_mask: torch.Tensor):
        if not update_mask.any():
            return
        
        at_last_now = update_mask & (self.ptr == (self.action_horizon - 1))
        if at_last_now.any():
            self.last_action_reached[at_last_now] = True

        can_advance = update_mask & (self.ptr < self.action_horizon - 1)
        if can_advance.any():
            self.ptr[can_advance] += 1
            next_idx = self.ptr[can_advance]
            self.current_target[can_advance] = self.buffer[can_advance, next_idx, :]
        
    def maybe_reset_buffer(self, done_mask: torch.Tensor):
        if not done_mask.any():
            return
        self.buffer[done_mask] = 0
        self.current_target[done_mask] = 0
        self.ptr[done_mask] = self.chunk_size
        self.last_action_reached[done_mask] = True

    @property
    def actions(self) -> torch.Tensor:
        return self.current_target