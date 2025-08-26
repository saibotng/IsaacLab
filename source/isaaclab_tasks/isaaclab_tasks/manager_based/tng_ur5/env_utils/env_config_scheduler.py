import math, random, yaml, torch

from isaaclab.envs.manager_based_env import ManagerBasedEnv
from isaaclab.managers import SceneEntityCfg
from copy import deepcopy

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
        self.required_metrics = loaded_yaml.get("required_metrics", [])
        self.order = list(range(len(self.cases)))
        self.cursor = 0
        self.cases_being_processed = [None] * num_envs
        self.idle_mask = torch.zeros(num_envs, dtype=torch.bool, device=device)
        self.results_dict = self._gen_empty_results_dict()

    def debug_info(self) -> str:
        """Return debug information about this scheduler instance."""
        return f"Scheduler ID: {id(self)}, processed_cases: {self.cases_being_processed}, cursor: {self.cursor}"
    
    def register_in_env(self, env):
        env.extras["scheduler"] = self

    def get_prompts(self, env_ids) -> list[str]:
        prompts = []
        for env_id in env_ids:
            case_idx = self.cases_being_processed[env_id]
            if case_idx is not None:
                case = self.cases[case_idx]
                prompt = case["prompt"]
            else:
                prompt = self.idle_case["prompt"]
            prompts.append(prompt)
        return prompts
    
    def get_new_case_for_env(self, env_id, env):
        success_mask = env.unwrapped.termination_manager.get_term("success")
        if success_mask[env_id]:
            case_idx = self.cases_being_processed[env_id]
            if case_idx is not None:
                self.results_dict["cases"][case_idx]["success"] = True   
        if self.cursor >= len(self.order):
            self.idle_mask[env_id] = True
            self.cases_being_processed[env_id] = None
            case = self.idle_case

        else:
            case_idx = self.order[self.cursor]
            self.cursor += 1
            self.cases_being_processed[env_id] = case_idx
            case = self.cases[case_idx]
        env.extras["all_cases_assigned"] = (self.cursor >= len(self.order))
        env.extras["idle_mask"] = self.idle_mask
        
        return case

    def update_metrics(self, obs):
        metrics_observations = {m: obs['subtasks'][m].nonzero(as_tuple=False).squeeze(-1).tolist() for m in self.required_metrics}
        for metric, envs in metrics_observations.items():
            print(f"Envs satisfying {metric}: {envs}")   
            for env_id in envs:
                case_idx = self.cases_being_processed[env_id]
                if case_idx is not None:
                    self.results_dict["cases"][case_idx]["metrics"][metric] = True 

    def _gen_empty_results_dict(self) -> dict:
        result_dict = {
            "total_cases": len(self.cases),
            "cases": deepcopy(self.cases),
            "success_rate": 0.0
        }
        for case in result_dict["cases"]:
            case["success"] = False
            case["metrics"] = {}
            for metric in self.required_metrics:
                case["metrics"][metric] = False
        return result_dict

    def get_results_dict(self) -> dict:
        """Return the current results dictionary."""
        return self.results_dict
