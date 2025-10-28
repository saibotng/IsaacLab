import argparse
import math
from dataclasses import asdict
from typing import List
import yaml
import numpy as np
import random
from yaml_gen_utils import (
    build_pairs, 
    grid_points,
    BenchCase, 
    Pose, 
    RPY_ZERO, 
    CameraPose, 
    maybe_add_distractors, 
    maybe_randomize_camera_poses, 
    maybe_randomize_target_object_colors, 
    maybe_randomize_joints, 
    maybe_randomize_table_height, 
    maybe_randomize_target_object_yaw_angles,
    NoAliasDumper
)
from colors import Color



def build_cases(dim: float,  
                resolution: int, 
                threshold: float, 
                randomize_colors: bool, 
                limit_colors: bool,
                joint_randomization_range: float, 
                table_height_randomization_range: float, 
                objects_yaw_randomization_range: float, 
                camera_main: str, 
                camera_secondary: str, 
                camera_wrist: str, 
                randomize_camera_poses: bool, 
                distractors: bool,
                seed: int) -> List[BenchCase]:

    base_seed = seed if seed is not None else random.randint(0, 2**32-1)
    random.seed(base_seed)
    np.random.seed(base_seed)

    rng_pairs = random.Random(base_seed)
    rng_joints = random.Random(base_seed + 1)
    rng_table = random.Random(base_seed + 2)
    rng_colors = random.Random(base_seed + 3)
    rng_yaw = random.Random(base_seed + 4)
    rng_camera = random.Random(base_seed + 5)
    rng_distractors = random.Random(base_seed + 6)
    rng_distractors_yaw = random.Random(base_seed + 7)

    test_cases: List[BenchCase] = []
    candidates = grid_points(dim, resolution)
    pairs = build_pairs(candidates, threshold, rng=rng_pairs)

    for (o, t) in pairs:
        object_pose = Pose(pos=[o[0], o[1], 0.0], rpy=RPY_ZERO.copy())
        target_pose = Pose(pos=[t[0], t[1], 0.0], rpy=RPY_ZERO.copy())
        case = BenchCase(
            id="",
            object=object_pose,
            target=target_pose,
            camera_pose_main = CameraPose.get_camera_pose_from_string(camera_main),
            camera_pose_secondary = CameraPose.get_camera_pose_from_string(camera_secondary),
            camera_pose_wrist = CameraPose.get_camera_pose_from_string(camera_wrist),
        )
        case = maybe_randomize_table_height(table_height_randomization_range > 0.0, case, table_height_randomization_range, rng_table)
        case = maybe_randomize_target_object_yaw_angles(objects_yaw_randomization_range > 0.0, case, objects_yaw_randomization_range, rng_yaw)
        case = maybe_randomize_joints(joint_randomization_range > 0.0, case, joint_randomization_range, rng_joints)
        case = maybe_randomize_target_object_colors(randomize_colors, limit_colors, case, rng_colors)
        case = maybe_randomize_camera_poses(randomize_camera_poses, case, rng_camera)
        case = maybe_add_distractors(distractors, limit_colors, case, dim, threshold, objects_yaw_randomization_range, rng_distractors, rng_distractors_yaw)
        test_cases.append(case)

    # Fill IDs with zero padding
    total = len(test_cases)
    width = max(3, int(math.ceil(math.log10(max(1, total + 1)))))
    for idx, case in enumerate(test_cases, start=1):
        case.id = f"case_{idx:0{width}d}"

    return test_cases


def build_yaml(name: str, 
               dim: float, 
               resolution: int, 
               threshold: float, 
               required_metrics: List[str], 
               randomize_colors: bool,
               limit_colors: bool, 
               joint_randomization_range: float, 
               table_height_randomization_range: float, 
               objects_yaw_randomization_range: float, 
               camera_main: str, 
               camera_secondary: str, 
               camera_wrist: str, 
               randomize_camera_poses: bool, 
               distractors: bool,
               seed: int,
               metadata: dict):
    doc = {
        "name": name,
        "metadata": metadata,
        "idle_case": {
            "id": "idle_case",
            "object": {"pos": [0.1, -0.1, 0.0], "rpy": [0.0, 0.0, 0.0]},
            "target": {"pos": [-0.1, 0.05, 0.0], "rpy": [0.0, 0.0, 0.0]},
            "prompt": "Idle",
        },
        "required_metrics": required_metrics,
        "test_cases": []
    }

    cases = build_cases(dim, 
                        resolution, 
                        threshold, 
                        randomize_colors, 
                        limit_colors,
                        joint_randomization_range, 
                        table_height_randomization_range, 
                        objects_yaw_randomization_range, 
                        camera_main, 
                        camera_secondary, 
                        camera_wrist, 
                        randomize_camera_poses, 
                        distractors,
                        seed)
    for c in cases:
        case_dict = {
            "id": c.id,
            "object": asdict(c.object),
            "target": asdict(c.target),
            "object_rgb": c.object_rgb,
            "target_rgb": c.target_rgb,
            "table_offset": asdict(c.table_offset),
            "robot_joint_offsets": c.robot_joint_offsets,
            "gripper_offset": c.gripper_offset,
            "prompt": c.prompt,
            "camera_pose_main": asdict(c.camera_pose_main),
            "camera_pose_secondary": asdict(c.camera_pose_secondary),
            "camera_pose_wrist": asdict(c.camera_pose_wrist),
        }
        if c.distractors is not None:
            case_dict["distractors"] = c.distractors
        doc["test_cases"].append(case_dict)

    return doc


def main():
    ap = argparse.ArgumentParser(description="Generate Isaac Sim YAML of pick-and-place benchmark with fixed target.")
    ap.add_argument("--name", type=str, required=True, help="Name of the dataset.")
    ap.add_argument("--dim", type=float, required=True, help="Half-size of square region (x,y in [-dim, dim]).")
    ap.add_argument("--resolution", type=int, default=4, help="Grid resolution r (r×r cells)")
    ap.add_argument("--threshold", type=float, default=0.1, help="Minimal distance between object and target.")
    ap.add_argument("--required-metrics", type=str, nargs="*", default=["object_lifted", "object_reached_target", "object_in_gripper_reach", "gripper_open_on_approach", "gripper_open_after_target_reached"], help="List of required metrics for the benchmark.")
    ap.add_argument("--out_dir", type=str, required=True, help="Output YAML filepath.")
    ap.add_argument("--seed", type=int, default=None, help="RNG seed for reproducibility.")
    ap.add_argument("--randomize_colors", action="store_true", help="If set, randomize object/target colors.")
    ap.add_argument("--limit_colors", action="store_true", help="If set, limit colors to a small set of high-contrast colors.")
    ap.add_argument("--joint_randomization_range", type=float, default=0.0, help="Range (in degrees) for randomizing robot's starting joint positions. If 0, no randomization.")
    ap.add_argument("--table_height_randomization_range", type=float, default=0.0, help="Range (in meters) for randomizing table height. If 0, no randomization.")
    ap.add_argument("--objects_yaw_randomization_range", type=float, default=0.0, help="Range (in degrees) for randomizing object/target yaw. If 0, no randomization.")
    ap.add_argument("--camera_main", type=str, default="CAMERA_FRONT_POSE", help="Camera pose for main view. Options: CAMERA_FRONT_POSE, CAMERA_SIDE_POSE, CAMERA_WRIST_POSE")
    ap.add_argument("--camera_secondary", type=str, default="CAMERA_SIDE_POSE", help="Camera pose for secondary view. Options: CAMERA_FRONT_POSE, CAMERA_SIDE_POSE, CAMERA_WRIST_POSE")
    ap.add_argument("--camera_wrist", type=str, default="CAMERA_WRIST_POSE", help="Camera pose for wrist view. Options: CAMERA_FRONT_POSE, CAMERA_SIDE_POSE, CAMERA_WRIST_POSE")
    ap.add_argument("--randomize_camera_poses", action="store_true", help="If set, randomize camera poses")
    ap.add_argument("--distractors", action="store_true", help="If set, add distractor objects and targets.")
    args = ap.parse_args()

    # Create metadata from all parsed arguments, excluding certain fields
    metadata = vars(args).copy()
    # Remove fields that shouldn't be in metadata
    metadata.pop('required_metrics', None)
    metadata.pop('out_dir', None)

    doc = build_yaml(args.name, 
                     args.dim, 
                     args.resolution, 
                     args.threshold, 
                     args.required_metrics, 
                     args.randomize_colors, 
                     args.limit_colors,
                     args.joint_randomization_range, 
                     args.table_height_randomization_range, 
                     args.objects_yaw_randomization_range, 
                     args.camera_main,
                     args.camera_secondary,
                     args.camera_wrist,
                     args.randomize_camera_poses,
                     args.distractors,
                     args.seed,
                     metadata)
    with open(args.out_dir + args.name + ".yaml", "w", encoding="utf-8") as f:
        yaml.dump(doc, f, sort_keys=False, default_flow_style=False, Dumper=NoAliasDumper)

    # A quick summary on stdout
    total = len(doc["test_cases"])
    print(f"Wrote {args.out_dir + args.name + '.yaml'}")
    print(f"TOTAL episodes:       {total}")


if __name__ == "__main__":
    main()