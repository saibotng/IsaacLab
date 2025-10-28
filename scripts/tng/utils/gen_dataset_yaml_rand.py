
# Standard library imports
import argparse
import math
import random
from dataclasses import asdict
from typing import List
import numpy as np
import yaml
from yaml_gen_utils import (
    get_corners_with_margin, 
    gen_random_pairs, 
    DatasetCase, 
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


CALIBRATION_MARGIN = 0.1  # relative to dim


def build_cases(dim: float, 
                num_random: int, 
                fixed_target: bool, 
                threshold: float, 
                calibration_episodes: bool, 
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
                seed: int) -> List[DatasetCase]:
    
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

    test_cases: List[DatasetCase] = []
    pairs = gen_random_pairs(dim, num_random, fixed_target, threshold, rng=rng_pairs)
    for (o, t) in pairs:
        object_pose = Pose(pos=[o[0], o[1], 0.0], rpy=RPY_ZERO.copy())
        target_pose = Pose(pos=[t[0], t[1], 0.0], rpy=RPY_ZERO.copy())
        case = DatasetCase(
            id="",
            object=object_pose,
            target=target_pose,
            camera_pose_main = CameraPose.get_camera_pose_from_string(camera_main),
            camera_pose_secondary = CameraPose.get_camera_pose_from_string(camera_secondary),
            camera_pose_wrist = CameraPose.get_camera_pose_from_string(camera_wrist),
        )
        case = maybe_randomize_table_height(table_height_randomization_range > 0.0, case, table_height_randomization_range, rng_table)
        test_cases.append(case)


    if calibration_episodes:
        corners = get_corners_with_margin(dim, relative_margin=CALIBRATION_MARGIN)
        if table_height_randomization_range > 0.0:
            calibration_heights = [-table_height_randomization_range*(1+CALIBRATION_MARGIN), table_height_randomization_range*(1+CALIBRATION_MARGIN)]
        else:
            calibration_heights = [0.0]
        for z in calibration_heights:
            for (ox, oy) in corners:
                for (tx, ty) in corners:
                    if (ox == tx) and (oy == ty):
                        continue
                    object_pose = Pose(pos=[ox, oy, 0.0], rpy=RPY_ZERO.copy())
                    target_pose = Pose(pos=[tx, ty, 0.0], rpy=RPY_ZERO.copy())
                    case = DatasetCase(
                        id="",
                        object=object_pose,
                        target=target_pose,
                        camera_pose_main = CameraPose.get_camera_pose_from_string(camera_main),
                        camera_pose_secondary = CameraPose.get_camera_pose_from_string(camera_secondary),
                        camera_pose_wrist = CameraPose.get_camera_pose_from_string(camera_wrist),
                    )
                    case.table_offset = Pose(pos=[0.0, 0.0, z], rpy=RPY_ZERO.copy())
                    test_cases.append(case)


    for case in test_cases:
        case = maybe_randomize_target_object_yaw_angles(objects_yaw_randomization_range > 0.0, case, objects_yaw_randomization_range, rng_yaw)
        case = maybe_randomize_joints(joint_randomization_range > 0.0, case, joint_randomization_range, rng_joints)
        case = maybe_randomize_target_object_colors(randomize_colors, limit_colors, case, rng_colors)
        case = maybe_randomize_camera_poses(randomize_camera_poses, case, rng_camera)
        case = maybe_add_distractors(distractors, limit_colors, case, dim, threshold, objects_yaw_randomization_range, rng_distractors, rng_distractors_yaw)

    # Assign IDs with zero-padding
    total = len(test_cases)
    width = max(3, int(math.ceil(math.log10(max(1, total + 1)))))
    for idx, case in enumerate(test_cases, start=1):
        case.id = f"case_{idx:0{width}d}"

    return test_cases


def build_yaml(name: str, 
               dim: float, 
               num_random: int, 
               fixed_target: bool, 
               threshold: float, 
               calibration_episodes: bool, 
               randomize_colors: bool, 
               limit_colors: bool,
               joint_randomization_range: float, 
               table_height_randomization_range: float, 
               objects_yaw_randomization_range: float, 
               camera_main: str,
               camera_secondary: str,
               camera_wrist: str,
               randomize_camera_poses: bool,
               seed: int,
               distractors: bool,
               metadata: dict) -> dict:
    doc = {
        "name": name,
        "metadata": metadata,
        "idle_case": {
            "id": "idle_case",
            "object": {"pos": [0.1, -0.1, 0.0], "rpy": [0.0, 0.0, 0.0]},
            "target": {"pos": [-0.1, 0.05, 0.0], "rpy": [0.0, 0.0, 0.0]},
            "travel_height": 0.12,
            "prompt": "Idle",
        },
        "test_cases": []
    }

    cases = build_cases(dim, 
                        num_random, 
                        fixed_target, 
                        threshold, 
                        calibration_episodes, 
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
            "travel_height": c.travel_height,
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
    ap = argparse.ArgumentParser(description="Generate YAML with calibration + randomized episodes under a distance threshold.")
    ap.add_argument("--name", type=str, required=True, help="Name of the dataset.")
    ap.add_argument("--dim", type=float, required=True, help="Half-size of square (x,y in [-dim, dim]).")
    ap.add_argument("--num-random-episodes", type=int, required=True, help="Number of additional random episodes to generate.")
    ap.add_argument("--threshold", type=float, required=True, help="Minimum distance between object and target (meters).")
    ap.add_argument("--fixed-target", action="store_true", help="If set, target is always at the center (0,0).")
    ap.add_argument("--calibration_episodes", action="store_true", help="Include calibration episodes (corners + edges).")
    ap.add_argument("--seed", type=int, default=None, help="RNG seed for reproducibility.")
    ap.add_argument("--out_dir", type=str, required=True, help="Output YAML filepath.")
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

    if args.num_random_episodes < 0:
        raise ValueError("--num-random-episodes must be >= 0")

    # Create metadata from all parsed arguments, excluding certain fields
    metadata = vars(args).copy()
    # Remove fields that shouldn't be in metadata
    metadata.pop('out_dir', None)
    metadata.pop('fixed_target', None)
    
    doc = build_yaml(args.name, 
                     args.dim, 
                     args.num_random_episodes, 
                     args.fixed_target, 
                     args.threshold, 
                     args.calibration_episodes, 
                     args.randomize_colors,
                     args.limit_colors, 
                     args.joint_randomization_range, 
                     args.table_height_randomization_range, 
                     args.objects_yaw_randomization_range, 
                     args.camera_main,
                     args.camera_secondary,
                     args.camera_wrist,
                     args.randomize_camera_poses,
                     args.seed,
                     args.distractors,
                     metadata)

    with open(args.out_dir + args.name + ".yaml", "w", encoding="utf-8") as f:
        yaml.dump(doc, f, sort_keys=False, default_flow_style=False, Dumper=NoAliasDumper)

    calib_total = 4 * 3 if args.calibration_episodes else 0
    calib_total *= 2 if args.table_height_randomization_range > 0.0 else 1
    total = calib_total + args.num_random_episodes
    print(f"Wrote {args.out_dir + args.name + '.yaml'}")
    print(f"Calibration episodes: {calib_total}")
    print(f"Random episodes:      {args.num_random_episodes}  (threshold = {args.threshold})")
    print(f"TOTAL episodes:       {total}")



if __name__ == "__main__":
    main()
