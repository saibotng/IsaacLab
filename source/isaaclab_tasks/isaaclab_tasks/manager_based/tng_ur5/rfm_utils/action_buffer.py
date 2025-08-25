import torch
from .gr00t_inference_client import RobotInferenceClient
import numpy as np

DELTA_ACTIONS = True
SNAP_GRIPPER_ACTIONS = True
GRIPPER_SNAP_THRESHOLD = 0.015
ENFORCE_GRIPPER_DELTA = 0.0015
TASK_DESCRIPTION = "Pick up the blue cube and place it on the black platform"

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

def extract_gr00t_obs_from_full_obs(full_obs: dict, env_idx):
        gr00t_obs = {
            "video.camera_wrist": full_obs["cameras"]["camera_wrist"][env_idx].cpu().unsqueeze(0).numpy(),
            "video.camera_global_side": full_obs["cameras"]["camera_global_side"][env_idx].cpu().unsqueeze(0).numpy(),
            "video.camera_global_front": full_obs["cameras"]["camera_global_front"][env_idx].cpu().unsqueeze(0).numpy(),
            "state.robot_arm": full_obs["arm_joints"]["arm_joint_pos"][env_idx].cpu().unsqueeze(0).numpy(),
            "state.gripper": full_obs["gripper_joint"]["gripper_joint_pos"][env_idx].cpu().unsqueeze(0).numpy(),
            "annotation.human.action.task_description": [TASK_DESCRIPTION],
        }
        return gr00t_obs

def convert_gr00t_action_to_state_action_chunk(gr00t_action, gr00t_obs) -> torch.Tensor:
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
        self.last_raw_action_dicts = [None for _ in range(num_envs)]


 
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
        # Clear the last raw action dicts for reset environments
        done_indices = done_mask.nonzero(as_tuple=False).squeeze(-1).tolist()
        for idx in done_indices:
            self.last_raw_action_dicts[idx] = None

    def maybe_get_new_actions(self, env, gr00t_client: RobotInferenceClient) -> None:
        if not self.needs_refill.any():
            return

        refill_mask = self.needs_refill.clone()
        full_obs = env.unwrapped.observation_manager.compute()
        env_ids = refill_mask.nonzero(as_tuple=False).squeeze(-1).tolist()
        new_chunk = torch.empty(len(env_ids), self.chunk_size, self.action_dim, device=self.device)
        for k, env_idx in enumerate(env_ids):
            gr00t_obs = extract_gr00t_obs_from_full_obs(full_obs, env_idx)
            gr00t_action = gr00t_client.get_action(gr00t_obs)
            new_chunk[k] = convert_gr00t_action_to_state_action_chunk(gr00t_action, gr00t_obs)
            # Store the raw action dict for this environment
            self.last_raw_action_dicts[env_idx] = gr00t_action

        self.refill(refill_mask, new_chunk)
        return

    @property
    def actions(self) -> torch.Tensor:
        return self.current_target
    
    @property
    def needs_refill(self) -> torch.Tensor:
        return self.last_action_reached