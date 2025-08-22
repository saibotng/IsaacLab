from dataclasses import dataclass, field
import math, random, yaml, torch

from isaaclab.envs.manager_based_env import ManagerBasedEnv
from isaaclab.managers import SceneEntityCfg

from isaaclab_tasks.manager_based.tng_ur5.ur5_pick_and_place.mdp.events import set_rigid_object_poses

def convert_deg_to_rad(deg: list[float]) -> list[float]:
    """Convert a list of angles in degrees to radians."""
    return [math.radians(angle) for angle in deg]

@dataclass
class BenchmarkScheduler:
    cases: list
    order: list[int]
    cursor: int = 0
    env_to_case: dict[int, int] = field(default_factory=dict)
    idle_mask: torch.Tensor | None = None

    @classmethod
    def from_yaml(cls, path):
        loaded_yaml = yaml.safe_load(open(path, "r"))
        order = list(range(len(loaded_yaml["cases"])))
        return cls(cases=loaded_yaml["cases"], order=order)
    
    def _attach_idle_mask(self, env):
        if self.idle_mask is None:
            self.idle_mask = torch.zeros(env.num_envs, dtype=torch.bool, device=env.device)

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

        if idle_ids.numel() > 0:
            self.idle_mask[idle_ids] = True
            env.reset_buf[idle_ids] = 0
            return


        # purely informative
        if hasattr(env, "extras"):
            env.extras["all_cases_assigned"] = (self.cursor >= len(self.order))
            env.extras["idle_mask"] = self.idle_mask
