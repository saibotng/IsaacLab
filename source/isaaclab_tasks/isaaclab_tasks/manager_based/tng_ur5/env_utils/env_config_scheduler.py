import math, random, yaml, torch

from isaaclab.envs.manager_based_env import ManagerBasedEnv
from isaaclab.managers import SceneEntityCfg
import datetime
import os
import json
from copy import deepcopy

class EnvConfigScheduler:
    def __init__(self, yaml_path: str, num_envs, device):
        """Initialize the scheduler from a YAML configuration file.
        
        Args:
            yaml_path: Path to the YAML file containing the cases configuration
            cursor: Starting cursor position (default: 0)
        """
        loaded_yaml = yaml.safe_load(open(yaml_path, "r"))
        self.name = loaded_yaml.get("name", "Unknown")
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
            "metrics": self.required_metrics,
            "cases": deepcopy(self.cases),
            "success_rate": 0.0,
            "metric_successes_rates": {}
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
    
    def finalize_and_store_results(self):
        """Finalize the results dictionary by computing the overall success rate."""
        total_cases = self.results_dict['total_cases']
        total_successes = 0
        metric_successes = {m: 0 for m in self.required_metrics}
        for case in self.results_dict["cases"]:
            if case["success"]:
                total_successes += 1
            for metric, achieved in case["metrics"].items():
                if achieved:
                    metric_successes[metric] += 1

        self.results_dict["success_rate"] = total_successes / total_cases
        self.results_dict["metric_successes_rates"] = {k: v / total_cases for k, v in metric_successes.items()}
        ts = datetime.datetime.now().strftime("%Y-%m-%d_%H%M%S")
        output_path = os.path.join("tng_benchmark_results", self.name, f"results_{ts}.json")
        os.makedirs(os.path.dirname(output_path), exist_ok=True)
        with open(output_path, "w") as f:
            json.dump(self.results_dict, f, ensure_ascii=False, indent=2)
        print_results_summary(self.results_dict)


def _print_results_summary(results_dict: dict):
    """Print a summary of the results."""
    
    print("Metric Success Rates:")
    for metric, success_rate in results_dict["metric_successes_rates"].items():
        print(f" - {metric}: {success_rate:.2%}")

    print(f"Overall Success Rate: {results_dict['success_rate']:.2%}")

def print_results_summary(results_dict: dict):
    """
    Minimal, structured summary that relies on precomputed fields:
      - total_cases
      - success_rate
      - metric_successes_rates
    and only iterates over cases for display (no heavy recomputation).
    """
    # ---- helpers (formatting only) -----------------------------------------
    def pct(x):
        try:
            return f"{float(x) * 100:.1f}%"
        except Exception:
            return "—"

    def tick(b): 
        return "✓" if b else "✗"

    def clip(s, n=60):
        s = "" if s is None else str(s)
        return s if len(s) <= n else s[: n - 1] + "…"

    line = "=" * 78
    subline = "-" * 78

    # ---- pull precomputed fields ------------------------------------------
    total_cases = results_dict.get("total_cases", 0)
    overall_rate = results_dict.get("success_rate", 0.0)
    metric_rates = results_dict.get("metric_successes_rates", {}) or {}
    metrics = results_dict.get("metrics", []) or []
    cases = results_dict.get("cases", []) or []

    # ---- header ------------------------------------------------------------
    print(line)
    print("BENCHMARK SUMMARY".center(78))
    print(line)

    # ---- overview (uses precomputed overall rate) --------------------------
    print("Overview")
    print(subline)
    print(f"Total cases       : {total_cases}")
    print(f"Overall success   : {pct(overall_rate)}")

    # ---- per-metric success rates (uses precomputed metric rates) ----------
    print("\nMetric success rates")
    print(subline)
    if metric_rates:
        for m in sorted(metric_rates):
            print(f" - {m:<24} {pct(metric_rates[m])}")
    else:
        print("No metrics were specified.")

    # ---- per-case compact table (display only; no rate recompute) ----------
    print("\nPer-case results")
    print(subline)
    if not cases:
        print("No cases to display.")
    else:
        # Determine compact header for metric columns (first char of each name)
        metric_header = " ".join((m[:1] or " ") for m in metrics)
        metric_cols_width = max(0, len(metrics) * 2 - (1 if len(metrics) > 0 else 0))
        prompt_width = max(20, 78 - (3 + 2 + 8 + 2 + metric_cols_width + 2))  # keep line ~78 chars

        # Header row
        print(f"{'#':>3}  {'Succ':<4}  {metric_header:<{metric_cols_width}}  Prompt")
        print(subline)

        # Print in the order provided (no sorting -> minimal work)
        for idx, c in enumerate(cases, start=1):
            succ = bool(c.get("success", False))
            cm = c.get("metrics", {}) or {}
            metrics_row = " ".join(tick(cm.get(m, False)) for m in metrics)
            prompt = clip(c.get("prompt", ""), prompt_width)
            print(f"{idx:>3}  {tick(succ):<4}  {metrics_row:<{metric_cols_width}}  {prompt}")

    # ---- legend ------------------------------------------------------------
    print("\nLegend")
    print(subline)
    print("✓ / ✗ : achieved / not achieved")
    if metrics:
        print("Metric columns: first letter of each metric name, in the same order as listed above.")
    print(line)