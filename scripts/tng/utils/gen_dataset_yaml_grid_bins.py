
from yaml_gen_utils import build_pairs, get_corners_with_margin, grid_points, DatasetCase, Pose
import argparse
import math
from dataclasses import dataclass, asdict
from typing import List
import yaml
import numpy as np
import random


PROMPT_TEXT = "Pick up the blue cube and place it on the black platform"
TRAVEL_HEIGHT = 0.12
RPY_ZERO = [0.0, 0.0, 0.0]
Z_CONST = 0.0


def build_cases(dim: float, resolution: int, threshold: float, calibration_episodes: bool) -> List[DatasetCase]:
    test_cases: List[DatasetCase] = []

    if calibration_episodes:
        corners = get_corners_with_margin(dim, relative_margin=0.1)
        for (ox, oy) in corners:
            for (tx, ty) in corners:
                if (ox == tx) and (oy == ty):
                    continue
                test_cases.append(DatasetCase(
                    id="",  # fill later
                    object=Pose(pos=[ox, oy, Z_CONST], rpy=RPY_ZERO.copy()),
                    target=Pose(pos=[tx, ty, Z_CONST], rpy=RPY_ZERO.copy()),
                    travel_height=TRAVEL_HEIGHT,
                    prompt=PROMPT_TEXT
                ))


    # 3) Grid episodes (ordered pairs of distinct centers)
    candidates = grid_points(dim, resolution)
    pairs = build_pairs(np.array(candidates), threshold)  # no threshold for grid bins
    for p in pairs:
        (ox, oy) = p[0]
        (tx, ty) = p[1]
        test_cases.append(DatasetCase(
            id="",
            object=Pose(pos=[ox, oy, Z_CONST], rpy=RPY_ZERO.copy()),
            target=Pose(pos=[tx, ty, Z_CONST], rpy=RPY_ZERO.copy()),
            travel_height=TRAVEL_HEIGHT,
            prompt=PROMPT_TEXT
        ))
    # Fill IDs with zero padding
    total = len(test_cases)
    width = max(3, int(math.ceil(math.log10(max(1, total + 1)))))
    for idx, case in enumerate(test_cases, start=1):
        case.id = f"case_{idx:0{width}d}"

    return test_cases


def build_yaml(name: str, dim: float, resolution: int, threshold: float, calibration_episodes: bool):
    # Idle case kept (you can adjust the coordinates here if you prefer)
    doc = {
        "name": name,
        "idle_case": {
            "id": "idle_case",
            "object": {"pos": [0.1, -0.1, 0.0], "rpy": [0.0, 0.0, 0.0]},
            "target": {"pos": [-0.1, 0.05, 0.0], "rpy": [0.0, 0.0, 0.0]},
            "travel_height": 0.12,
            "prompt": "Idle",
        },
        "test_cases": []
    }

    cases = build_cases(dim, resolution, threshold, calibration_episodes)
    for c in cases:
        doc["test_cases"].append({
            "id": c.id,
            "object": asdict(c.object),
            "target": asdict(c.target),
            "travel_height": c.travel_height,
            "prompt": c.prompt
        })
    return doc


def main():
    ap = argparse.ArgumentParser(description="Generate Isaac Sim YAML of pick-and-place episodes.")
    ap.add_argument("--name", type=str, required=True, help="Name of the dataset.")
    ap.add_argument("--dim", type=float, required=True, help="Half-size of square region (x,y in [-dim, dim]).")
    ap.add_argument("--resolution", type=int, default=0, help="Grid resolution r (r×r cells). 0 disables grid episodes.")
    ap.add_argument("--threshold", type=float, default=0.1, help="Minimal distance between object and target.")
    ap.add_argument("--required-metrics", type=str, nargs="*", default=["object_lifted", "object_reached_target", "object_in_gripper_reach"], help="List of required metrics for the benchmark.")
    ap.add_argument("--calibration_episodes", action="store_true", help="Include calibration episodes (corners + edges).")
    ap.add_argument("--out_dir", type=str, required=True, help="Output YAML filepath.")
    ap.add_argument("--seed", type=int, default=None, help="RNG seed for reproducibility.")
    args = ap.parse_args()

    if args.seed is not None:
        random.seed(args.seed)
        np.random.seed(args.seed)

    doc = build_yaml(args.name, args.dim, args.resolution, args.threshold, args.calibration_episodes)
    with open(args.out_dir + args.name + ".yaml", "w", encoding="utf-8") as f:
        yaml.safe_dump(doc, f, sort_keys=False, default_flow_style=False)

    # A quick summary on stdout
    total = len(doc["test_cases"])
    r = args.resolution
    calib_total = 4 * 3 if args.calibration_episodes else 0
    grid_total = (r * r) if r > 0 else 0
    print(f"Wrote {args.out_dir + args.name + '.yaml'}")
    print(f"Calibration episodes: {calib_total}")
    print(f"Grid episodes:        {grid_total}")
    print(f"TOTAL episodes:       {total}")


if __name__ == "__main__":
    main()