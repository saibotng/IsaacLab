import math, random, yaml, torch

from isaaclab.envs.manager_based_env import ManagerBasedEnv
from isaaclab.managers import SceneEntityCfg

from isaaclab_tasks.manager_based.tng_ur5.ur5_pick_and_place.mdp.events import set_rigid_object_poses

def convert_deg_to_rad(deg: list[float]) -> list[float]:
    """Convert a list of angles in degrees to radians."""
    return [math.radians(angle) for angle in deg]

class EnvConfigScheduler:
    def __init__(self, yaml_path: str):
        """Initialize the scheduler from a YAML configuration file.
        
        Args:
            yaml_path: Path to the YAML file containing the cases configuration
            cursor: Starting cursor position (default: 0)
        """
        loaded_yaml = yaml.safe_load(open(yaml_path, "r"))
        self.cases = loaded_yaml["cases"]
        self.order = list(range(len(self.cases)))
        self.cursor = 0
        self.env_to_case: dict[int, int] = {}
        self.idle_mask: torch.Tensor | None = None
        # Add instance ID for debugging
        self._instance_id = id(self)
        print(f"[DEBUG] EnvConfigScheduler created with ID: {self._instance_id}")
    
    def debug_info(self) -> str:
        """Return debug information about this scheduler instance."""
        return f"Scheduler ID: {self._instance_id}, env_to_case: {self.env_to_case}, cursor: {self.cursor}"
    
    def _attach_idle_mask(self, env):
        if self.idle_mask is None:
            self.idle_mask = torch.zeros(env.num_envs, dtype=torch.bool, device=env.device)

    def get_prompts(self, env_ids) -> list[str]:
        prompts = []
        for env_id in env_ids:
            case_id = self.env_to_case.get(env_id, None)
            if case_id is not None:
                case = self.cases[case_id]
                prompt = case["prompt"]
                prompts.append(prompt)
            else:
                prompts.append("")
        return prompts

    # This is what EventTerm will call on reset:
    def on_reset(self,
        env: ManagerBasedEnv,
        env_ids: torch.Tensor,
        asset_cfgs: list[SceneEntityCfg]
    ):  
        self._attach_idle_mask(env)

        remaining = len(self.order) - self.cursor
        n_assign = min(remaining, env_ids.numel())

        assign_ids = env_ids[:n_assign]
        idle_ids   = env_ids[n_assign:]

        if assign_ids.numel() > 0:

            for env_id in assign_ids.tolist():

                idx = self.order[self.cursor]; self.cursor += 1
                self.env_to_case[env_id] = idx
                case = self.cases[idx]

                obj_pos, obj_rpy = case["object"]["pos"], convert_deg_to_rad(case["object"]["rpy"])
                tgt_pos, tgt_rpy = case["target"]["pos"], convert_deg_to_rad(case["target"]["rpy"])
                poses = [obj_pos + obj_rpy, tgt_pos + tgt_rpy]

                set_rigid_object_poses(env, env_id, asset_cfgs, poses)
        print(f"[DEBUG] Accessing scheduler after env generation: {self.debug_info()}")

        if idle_ids.numel() > 0:
            if self.idle_mask is not None:
                self.idle_mask[idle_ids] = True
            return
        
        # purely informative
        if hasattr(env, "extras"):
            env.extras["all_cases_assigned"] = (self.cursor >= len(self.order))
            env.extras["idle_mask"] = self.idle_mask
