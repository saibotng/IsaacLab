import math, random, yaml, torch

from isaaclab.envs.manager_based_env import ManagerBasedEnv
from isaaclab.managers import SceneEntityCfg

class EnvConfigScheduler:
    def __init__(self, yaml_path: str, num_envs, device):
        """Initialize the scheduler from a YAML configuration file.
        
        Args:
            yaml_path: Path to the YAML file containing the cases configuration
            cursor: Starting cursor position (default: 0)
        """
        loaded_yaml = yaml.safe_load(open(yaml_path, "r"))
        self.cases = loaded_yaml["test_cases"]
        self.idle_case = loaded_yaml["idle_case"]
        self.order = list(range(len(self.cases)))
        self.cursor = 0
        self.cases_being_processed = [None] * num_envs
        self.idle_mask = torch.zeros(num_envs, dtype=torch.bool, device=device)

    def debug_info(self) -> str:
        """Return debug information about this scheduler instance."""
        return f"Scheduler ID: {id(self)}, processed_cases: {self.cases_being_processed}, cursor: {self.cursor}"
    
    def register_in_env(self, env):
        env.extras["scheduler"] = self

    def get_prompts(self, env_ids) -> list[str]:
        prompts = []
        for case_idx in self.cases_being_processed:
            if case_idx is not None:
                case = self.cases[case_idx]
                prompt = case["prompt"]
                prompts.append(prompt)
            else:
                prompts.append("")
        return prompts
    
    def get_new_case_for_env(self, env_id, env):
        if self.cursor >= len(self.order):
            self.idle_mask[env_id] = True
            self.cases_being_processed[env_id] = None
            case =  self.idle_case

        else:
            case_idx = self.order[self.cursor]
            self.cursor += 1
            self.cases_being_processed[env_id] = case_idx
            case = self.cases[case_idx]
        env.extras["all_cases_assigned"] = (self.cursor >= len(self.order))
        env.extras["idle_mask"] = self.idle_mask
        return case


