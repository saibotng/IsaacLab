from yaml_gen_utils import BenchCase, Pose, FIXED_TARGET_XY, grid_centers
import argparse
import math
from dataclasses import asdict
from typing import List, Tuple
import yaml


PROMPT_TEXT = "Pick up the blue cube and place it on the black platform"
RPY_ZERO = [0.0, 0.0, 0.0]
Z_CONST = 0.0



def dist_xy(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    return math.hypot(dx, dy)


def make_pose(x: float, y: float) -> Pose:
    return Pose(pos=[float(x), float(y), Z_CONST], rpy=RPY_ZERO.copy())


def build_cases(dim: float,  resolution: int, threshold: float) -> List[BenchCase]:
    test_cases: List[BenchCase] = []

    centers = grid_centers(dim, resolution)

    for c in centers:
        if dist_xy(c, FIXED_TARGET_XY) < threshold:
            continue
        (ox, oy) = c
        (tx, ty) = FIXED_TARGET_XY
        test_cases.append(BenchCase(
            id="",
            object=make_pose(ox, oy),
            target=make_pose(tx, ty),
            prompt=PROMPT_TEXT
        ))

    # Fill IDs with zero padding
    total = len(test_cases)
    width = max(3, int(math.ceil(math.log10(max(1, total + 1)))))
    for idx, case in enumerate(test_cases, start=1):
        case.id = f"case_{idx:0{width}d}"

    return test_cases


def build_yaml(name: str, dim: float, resolution: int, threshold: float, required_metrics: List[str]):
    # Idle case kept (you can adjust the coordinates here if you prefer)
    doc = {
        "name": name,
        "idle_case": {
            "id": "idle_case",
            "object": {"pos": [0.1, -0.1, 0.0], "rpy": [0.0, 0.0, 0.0]},
            "target": {"pos": [-0.1, 0.05, 0.0], "rpy": [0.0, 0.0, 0.0]},
            "prompt": "Idle",
        },
        "required_metrics": required_metrics,
        "test_cases": []
    }

    cases = build_cases(dim, resolution, threshold)
    for c in cases:
        doc["test_cases"].append({
            "id": c.id,
            "object": asdict(c.object),
            "target": asdict(c.target),
            "prompt": c.prompt
        })
    return doc


def main():
    ap = argparse.ArgumentParser(description="Generate Isaac Sim YAML of pick-and-place benchmark with fixed target.")
    ap.add_argument("--name", type=str, required=True, help="Name of the dataset.")
    ap.add_argument("--dim", type=float, required=True, help="Half-size of square region (x,y in [-dim, dim]).")
    ap.add_argument("--resolution", type=int, default=4, help="Grid resolution r (r×r cells)")
    ap.add_argument("--threshold", type=float, default=0.1, help="Minimal distance between object and target.")
    ap.add_argument("--required-metrics", type=str, nargs="*", default=["object_lifted", "object_reached_target", "object_in_gripper_reach",  "gripper_open_on_approach", "gripper_open_after_target_reached"], help="List of required metrics for the benchmark.")
    ap.add_argument("--out_dir", type=str, required=True, help="Output YAML filepath.")
    args = ap.parse_args()

    doc = build_yaml(args.name, args.dim, args.resolution, args.threshold, args.required_metrics)
    with open(args.out_dir + args.name + ".yaml", "w", encoding="utf-8") as f:
        yaml.safe_dump(doc, f, sort_keys=False, default_flow_style=False)

    # A quick summary on stdout
    total = len(doc["test_cases"])
    print(f"Wrote {args.out_dir + args.name + '.yaml'}")
    print(f"TOTAL episodes:       {total}")


if __name__ == "__main__":
    main()