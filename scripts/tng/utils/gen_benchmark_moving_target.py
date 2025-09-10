import argparse
import math
from dataclasses import asdict
from typing import List
import yaml
import numpy as np
import random
from yaml_gen_utils import build_pairs, grid_points, BenchCase, Pose, RPY_ZERO, sample_yaw_angles_for_poses, sample_robot_starting_pose_offsets, sample_table_offset, PROMPT_TEMPLATE, CameraPose
from colors import Color

#TODO: a lot of duplicate code. Improve that

class NoAliasDumper(yaml.SafeDumper):
    """YAML dumper that prevents the use of anchors and aliases."""
    def ignore_aliases(self, data):
        return True




def build_cases(dim: float,  
                resolution: int, 
                threshold: float, 
                randomize_colors: bool, 
                joint_randomization_range: float, 
                table_height_randomization_range: float, 
                objects_yaw_randomization_range: float, 
                camera_main: str, 
                camera_secondary: str, 
                camera_wrist: str, 
                randomize_camera_poses: bool, 
                seed: int) -> List[BenchCase]:
    test_cases: List[BenchCase] = []

    candidates = grid_points(dim, resolution)

    pairs = build_pairs(candidates, threshold)

    for (o, t) in pairs:
        object_pose = Pose(pos=[o[0], o[1], 0.0], rpy=RPY_ZERO.copy())
        target_pose = Pose(pos=[t[0], t[1], 0.0], rpy=RPY_ZERO.copy())
        if objects_yaw_randomization_range > 0.0:
            object_pose, target_pose = sample_yaw_angles_for_poses([object_pose, target_pose], objects_yaw_randomization_range)

        case = BenchCase(
            id="",
            object=object_pose,
            target=target_pose,
            camera_pose_main = CameraPose.get_camera_pose_from_string(camera_main),
            camera_pose_secondary = CameraPose.get_camera_pose_from_string(camera_secondary),
            camera_pose_wrist = CameraPose.get_camera_pose_from_string(camera_wrist),
        )

        if joint_randomization_range > 0.0: 
            case.robot_joint_offsets, case.gripper_offset = sample_robot_starting_pose_offsets(joint_randomization_range=joint_randomization_range)

        if table_height_randomization_range > 0.0:
            case.table_offset = sample_table_offset(max_z_offset=table_height_randomization_range)

        if randomize_colors:
            object_color = Color.sample(n=1)[0]
            target_color = Color.sample(n=1, exclude=[object_color])[0]
            case.object_rgb = object_color.rgb
            case.target_rgb = target_color.rgb
            case.prompt = PROMPT_TEMPLATE.format(object_color=object_color.pretty, target_color=target_color.pretty)

        if randomize_camera_poses:
            case.camera_pose_main = CameraPose.apply_camera_pose_randomization(case.camera_pose_main, position_jitter=0.05, rotation_jitter=2.0)
            case.camera_pose_secondary = CameraPose.apply_camera_pose_randomization(case.camera_pose_secondary, position_jitter=0.05, rotation_jitter=2.0)
            case.camera_pose_wrist = CameraPose.apply_camera_pose_randomization(case.camera_pose_wrist, position_jitter=0.01, rotation_jitter=1.0)

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
               joint_randomization_range: float, 
               table_height_randomization_range: float, 
               objects_yaw_randomization_range: float, 
               camera_main: str, 
               camera_secondary: str, 
               camera_wrist: str, 
               randomize_camera_poses: bool, 
               seed: int,
               metadata: dict):
    # Idle case kept (you can adjust the coordinates here if you prefer)
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
                        joint_randomization_range, 
                        table_height_randomization_range, 
                        objects_yaw_randomization_range, 
                        camera_main, 
                        camera_secondary, 
                        camera_wrist, 
                        randomize_camera_poses, 
                        seed)
    for c in cases:
        doc["test_cases"].append({
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
        })
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
    ap.add_argument("--joint_randomization_range", type=float, default=0.0, help="Range (in degrees) for randomizing robot's starting joint positions. If 0, no randomization.")
    ap.add_argument("--table_height_randomization_range", type=float, default=0.0, help="Range (in meters) for randomizing table height. If 0, no randomization.")
    ap.add_argument("--objects_yaw_randomization_range", type=float, default=0.0, help="Range (in degrees) for randomizing object/target yaw. If 0, no randomization.")
    ap.add_argument("--camera_main", type=str, default="CAMERA_FRONT_POSE", help="Camera pose for main view. Options: CAMERA_FRONT_POSE, CAMERA_SIDE_POSE, CAMERA_WRIST_POSE")
    ap.add_argument("--camera_secondary", type=str, default="CAMERA_SIDE_POSE", help="Camera pose for secondary view. Options: CAMERA_FRONT_POSE, CAMERA_SIDE_POSE, CAMERA_WRIST_POSE")
    ap.add_argument("--camera_wrist", type=str, default="CAMERA_WRIST_POSE", help="Camera pose for wrist view. Options: CAMERA_FRONT_POSE, CAMERA_SIDE_POSE, CAMERA_WRIST_POSE")
    ap.add_argument("--randomize_camera_poses", action="store_true", help="If set, randomize camera poses")
    args = ap.parse_args()

    if args.seed is not None:
        random.seed(args.seed)
        np.random.seed(args.seed)

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
                     args.joint_randomization_range, 
                     args.table_height_randomization_range, 
                     args.objects_yaw_randomization_range, 
                     args.camera_main,
                     args.camera_secondary,
                     args.camera_wrist,
                     args.randomize_camera_poses,
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