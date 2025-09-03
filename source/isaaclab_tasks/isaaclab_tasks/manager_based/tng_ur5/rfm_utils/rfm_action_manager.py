import torch
from .gr00t_inference_client import Gr00tInferenceClient
import numpy as np
from collections import deque

DELTA_ACTIONS = True
SNAP_GRIPPER_ACTIONS = True
GRIPPER_SNAP_THRESHOLD = 0.015
ENFORCE_GRIPPER_DELTA = 0.0015
ARM_TARGET_TOL = 0.003
GRIPPER_TARGET_TOL = 0.0001


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



def convert_gr00t_action_to_state_action_chunk(gr00t_action, gr00t_obs) -> torch.Tensor:
        if DELTA_ACTIONS:
            action_chunk = convert_raw_delta_action_to_action_chunk(gr00t_action, gr00t_obs)
        else:
            action_chunk = convert_raw_abs_action_to_action_chunk(gr00t_action)
        return action_chunk

def clone_masked_tensor_dict(obj, mask):
    if torch.is_tensor(obj):
        return obj[mask].clone()
    if isinstance(obj, dict):
        # recurse so you’re safe even if you someday add depth > 2
        return {k: clone_masked_tensor_dict(v, mask) for k, v in obj.items()}
    return obj 


class RFMActionManager:
    def __init__(self, 
                 num_envs: int, 
                 chunk_size: int, 
                 action_horizon: int, 
                 action_dim: int, 
                 rfm_client: Gr00tInferenceClient, 
                 device: torch.device):
        
        self.num_envs = num_envs
        self.chunk_size = chunk_size
        self.action_horizon = action_horizon
        self.action_dim = action_dim
        self.device = device
        self.buffer = torch.zeros(num_envs, chunk_size, action_dim, device=device)
        self.ptr = torch.full((num_envs,), 0, dtype=torch.long, device=device)
        self.current_target = torch.zeros(num_envs, action_dim, device=device)
        self.last_action_reached = torch.ones(num_envs, dtype=torch.bool, device=device)
        self.observation_at_last_waypoint = [None for _ in range(num_envs)]
        self.rfm_client = rfm_client

        self.err_deque = deque(maxlen=4)
        self.err_deque.append(torch.zeros(num_envs, device=device))

        self.gripper_pos_deque = deque(maxlen=2)
        self.gripper_pos_deque.append(torch.zeros(num_envs, device=device))

        self.action_idx_deque = deque(maxlen=4)
        self.action_idx_deque.append(torch.zeros(num_envs, device=device))


 
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
        self.last_action_reached[mask] = (self.action_horizon == 1)


    def maybe_update_targets(self, update_mask: torch.Tensor, obs):
        if not update_mask.any():
            return
        #TODO: store latest observations for envs that get updated. Don't forget to clear them on reset. Don't forget to clone(). First check if similar to current approach. If yes, this is awood workflow for delta tcp as well
        at_last_now = update_mask & (self.ptr == (self.action_horizon - 1))
        can_advance = update_mask & (self.ptr < self.action_horizon - 1)

        if at_last_now.any():
            self.last_action_reached[at_last_now] = True
        
        if can_advance.any():
            for env_idx in can_advance.nonzero(as_tuple=False).squeeze(-1).tolist():
                self.observation_at_last_waypoint[env_idx] = clone_masked_tensor_dict(obs, env_idx)
            self.ptr[can_advance] += 1
            next_idx = self.ptr[can_advance]
            self.current_target[can_advance] = self.buffer[can_advance, next_idx, :]
        
    def maybe_reset_buffer(self, done_mask: torch.Tensor):
        if not done_mask.any():
            return
        self.buffer[done_mask] = 0
        self.current_target[done_mask] = 0
        self.last_action_reached[done_mask] = True

        done_indices = done_mask.nonzero(as_tuple=False).squeeze(-1).tolist()
        for idx in done_indices:
            self.observation_at_last_waypoint[idx] = None

    def maybe_get_new_action_chunk_from_rfm(self, obs, prompts, idle_mask) -> None:
        if not self.last_action_reached.any():
            return

        refill_mask = self.last_action_reached.clone()
        idle_ids = idle_mask.nonzero(as_tuple=False).squeeze(-1).tolist()
        env_ids = refill_mask.nonzero(as_tuple=False).squeeze(-1).tolist()
        new_chunk = torch.empty(len(env_ids), self.chunk_size, self.action_dim, device=self.device)
        for k, env_id in enumerate(env_ids):
            if env_id in idle_ids:
                new_chunk[k] = self.construct_action_chunk_from_obs_for_idle_envs(obs, env_id)
            else:
                gr00t_obs = self.construct_gr00t_obs_from_env_obs_and_prompt(obs, env_id, prompts[env_id])
                gr00t_action = self.rfm_client.get_action(gr00t_obs)
                new_chunk[k] = convert_gr00t_action_to_state_action_chunk(gr00t_action, gr00t_obs)

        self.refill(refill_mask, new_chunk)
    
    def get_targets(self, obs, prompts, idle_mask):
        self.maybe_get_new_action_chunk_from_rfm(obs, prompts, idle_mask)
        return self.current_target
    
    def update_target_tracking(self, obs, done_mask):
        q = obs['arm_joints']['arm_joint_pos'] 
        number_of_arm_joints = q.shape[-1]

        err_arm = torch.abs(q - self.current_target[:,:number_of_arm_joints]).max(dim=-1).values  
        self.err_deque.append(err_arm)
        stacked_err = torch.stack(list(self.err_deque), dim=0)
        err_span = stacked_err.max(dim=0).values - stacked_err.min(dim=0).values

        gripper_pos = obs['gripper_joint']['gripper_joint_pos'].squeeze(1)

        self.gripper_pos_deque.append(gripper_pos)
        stacked_gripper_pos = torch.stack(list(self.gripper_pos_deque), dim=0)
        gripper_pos_span = stacked_gripper_pos.max(dim=0).values - stacked_gripper_pos.min(dim=0).values

        gripper_static = (gripper_pos_span < GRIPPER_TARGET_TOL).to(device=self.device)

        self.action_idx_deque.append(self.ptr.clone())
        stacked_action_idx = torch.stack(list(self.action_idx_deque), dim=0)
        action_idx_span = stacked_action_idx.max(dim=0).values - stacked_action_idx.min(dim=0).values

        arm_reached = err_arm < ARM_TARGET_TOL
        can_advance = (arm_reached & gripper_static)
        stuck = (err_span < 1e-4) & (action_idx_span == 0) & (gripper_static) & (~can_advance)
        envs_to_update_targets = (can_advance | stuck)

        self.maybe_update_targets(envs_to_update_targets, obs)
        self.maybe_reset_buffer(done_mask)

        if stuck.any():
            print(f"WARNING: Envs Stuck: {stuck.nonzero(as_tuple=False).squeeze(-1).tolist()}")

    def construct_gr00t_obs_from_env_obs_and_prompt(self, full_obs: dict, env_idx, prompt: str):
            if self.observation_at_last_waypoint[env_idx] is None:
                arm_delta = np.zeros((1, full_obs["arm_joints"]["arm_joint_pos"].shape[-1]), dtype=np.float64)
                gripper_delta = np.zeros((1, full_obs["gripper_joint"]["gripper_joint_pos"].shape[-1]), dtype=np.float64)
            else:
                arm_delta = full_obs["arm_joints"]["arm_joint_pos"][env_idx].cpu().unsqueeze(0).numpy() - self.observation_at_last_waypoint[env_idx]["arm_joints"]["arm_joint_pos"].cpu().unsqueeze(0).numpy().astype(np.float64)
                gripper_delta = full_obs["gripper_joint"]["gripper_joint_pos"][env_idx].cpu().unsqueeze(0).numpy() - self.observation_at_last_waypoint[env_idx]["gripper_joint"]["gripper_joint_pos"].cpu().unsqueeze(0).numpy().astype(np.float64)

            gr00t_obs = {
                "video.camera_wrist": full_obs["cameras"]["camera_wrist"][env_idx].cpu().unsqueeze(0).numpy(),
                "video.camera_global_side": full_obs["cameras"]["camera_global_side"][env_idx].cpu().unsqueeze(0).numpy(),
                "video.camera_global_front": full_obs["cameras"]["camera_global_front"][env_idx].cpu().unsqueeze(0).numpy(),
                "state.robot_arm": full_obs["arm_joints"]["arm_joint_pos"][env_idx].cpu().unsqueeze(0).numpy(),
                "state.gripper": full_obs["gripper_joint"]["gripper_joint_pos"][env_idx].cpu().unsqueeze(0).numpy(),
                "state.delta_robot_arm": arm_delta,
                "state.delta_gripper": gripper_delta,
                "state.tcp_pose": full_obs["end_effector"][env_idx].cpu().unsqueeze(0).numpy(),
                "annotation.human.action.task_description": [prompt],
            }
            return gr00t_obs
    
    def construct_action_chunk_from_obs_for_idle_envs(self, obs: dict, env_idx: int) -> torch.Tensor:
        arm_joints = obs["arm_joints"]["arm_joint_pos"][env_idx]
        gripper_joints = obs["gripper_joint"]["gripper_joint_pos"][env_idx]
        action_chunk = torch.cat([arm_joints, gripper_joints, gripper_joints], dim=0).unsqueeze(0).repeat(self.chunk_size, 1)
        return action_chunk
