
from yaml_gen_utils import get_corners_with_margin, gen_random_pairs, DatasetCase, Pose, sample_robot_starting_pose_offsets, sample_table_offset, PROMPT_TEMPLATE, sample_yaw_angles_for_poses, RPY_ZERO, CameraPose, gen_distractor_positions
from colors import Color
import argparse
import math
import random
from dataclasses import asdict
from typing import List, Tuple
import yaml
import numpy as np

#TODO: a lot of duplicate code. Improve that
class NoAliasDumper(yaml.SafeDumper):
    """YAML dumper that prevents the use of anchors and aliases."""
    def ignore_aliases(self, data):
        return True




CALIBRATION_MARGIN = 0.1  # relative to dim


def build_cases(dim: float, 
                num_random: int, 
                fixed_target: bool, 
                threshold: float, 
                calibration_episodes: bool, 
                randomize_colors: bool, 
                joint_randomization_range: float, 
                table_height_randomization_range: float, 
                objects_yaw_randomization_range: float, 
                camera_main: str,
                camera_secondary: str,
                camera_wrist: str,
                randomize_camera_poses: bool,
                distractors: bool,
                seed: int) -> List[DatasetCase]:

    test_cases: List[DatasetCase] = []
    pairs = gen_random_pairs(dim, num_random, fixed_target, threshold)
    for (o, t) in pairs:
        object_pose = Pose(pos=[o[0], o[1], 0.0], rpy=RPY_ZERO.copy())
        target_pose = Pose(pos=[t[0], t[1], 0.0], rpy=RPY_ZERO.copy())
        if objects_yaw_randomization_range > 0.0:
            object_pose, target_pose = sample_yaw_angles_for_poses([object_pose, target_pose], objects_yaw_randomization_range)

        case = DatasetCase(
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

        if distractors:
            num_object_distractors = random.randint(0,3)
            num_target_distractors = random.randint(0,3)
            distractor_positions = gen_distractor_positions(num_object_distractors + num_target_distractors, [case.object, case.target], threshold=threshold, dim=dim)
            distractor_colors = Color.sample(n=num_object_distractors + num_target_distractors, exclude=[Color.from_rgb(case.object_rgb), Color.from_rgb(case.target_rgb)])
            object_distractors = []
            target_distractors = []
            for i in range(num_object_distractors):
                distractor_pose = Pose(pos=[distractor_positions[i][0], distractor_positions[i][1], 0.0], rpy=RPY_ZERO.copy())
                if objects_yaw_randomization_range > 0.0:
                    distractor_pose = sample_yaw_angles_for_poses([distractor_pose], objects_yaw_randomization_range)[0]
                object_distractors.append({"pos": distractor_pose.pos, "rpy": distractor_pose.rpy, "rgb": distractor_colors[i].rgb})

            for i in range(num_target_distractors):
                distractor_pose = Pose(pos=[distractor_positions[i + num_object_distractors][0], distractor_positions[i + num_object_distractors][1], 0.0], rpy=RPY_ZERO.copy())
                if objects_yaw_randomization_range > 0.0:
                    distractor_pose = sample_yaw_angles_for_poses([distractor_pose], objects_yaw_randomization_range)[0]
                target_distractors.append({"pos": distractor_pose.pos, "rpy": distractor_pose.rpy, "rgb": distractor_colors[i + num_object_distractors].rgb})

            case.distractors = {
                "objects": object_distractors,
                "targets": target_distractors
            }

        test_cases.append(case)

    if calibration_episodes:
        corners = get_corners_with_margin(dim, relative_margin=CALIBRATION_MARGIN)
        if table_height_randomization_range > 0.0:
            calibration_heights = [-table_height_randomization_range*CALIBRATION_MARGIN, table_height_randomization_range*CALIBRATION_MARGIN]
        else:
            calibration_heights = [0.0]
        for z in calibration_heights:
            for (ox, oy) in corners:
                for (tx, ty) in corners:
                    if (ox == tx) and (oy == ty):
                        continue

                    object_pose = Pose(pos=[ox, oy, 0.0], rpy=RPY_ZERO.copy())
                    target_pose = Pose(pos=[tx, ty, 0.0], rpy=RPY_ZERO.copy())
                    if objects_yaw_randomization_range > 0.0:
                        object_pose, target_pose = sample_yaw_angles_for_poses([object_pose, target_pose], objects_yaw_randomization_range)
                    case = DatasetCase(
                        id="",  # fill later
                        object=object_pose,
                        target=target_pose,
                        camera_pose_main = CameraPose.get_camera_pose_from_string(camera_main),
                        camera_pose_secondary = CameraPose.get_camera_pose_from_string(camera_secondary),
                        camera_pose_wrist = CameraPose.get_camera_pose_from_string(camera_wrist),
                    )

                    case.table_offset.pos[2] = z

                    if joint_randomization_range > 0.0: 
                        case.robot_joint_offsets, case.gripper_offset = sample_robot_starting_pose_offsets(joint_randomization_range=joint_randomization_range)


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


    # 3) Random episodes with distance threshold
    pairs = gen_random_pairs(dim, num_random, fixed_target, threshold)
    for (o, t) in pairs:
        object_pose = Pose(pos=[o[0], o[1], 0.0], rpy=RPY_ZERO.copy())
        target_pose = Pose(pos=[t[0], t[1], 0.0], rpy=RPY_ZERO.copy())
        if objects_yaw_randomization_range > 0.0:
            object_pose, target_pose = sample_yaw_angles_for_poses([object_pose, target_pose], objects_yaw_randomization_range)

        case = DatasetCase(
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
    ap.add_argument("--joint_randomization_range", type=float, default=0.0, help="Range (in degrees) for randomizing robot's starting joint positions. If 0, no randomization.")
    ap.add_argument("--table_height_randomization_range", type=float, default=0.0, help="Range (in meters) for randomizing table height. If 0, no randomization.")
    ap.add_argument("--objects_yaw_randomization_range", type=float, default=0.0, help="Range (in degrees) for randomizing object/target yaw. If 0, no randomization.")
    ap.add_argument("--camera_main", type=str, default="CAMERA_FRONT_POSE", help="Camera pose for main view. Options: CAMERA_FRONT_POSE, CAMERA_SIDE_POSE, CAMERA_WRIST_POSE")
    ap.add_argument("--camera_secondary", type=str, default="CAMERA_SIDE_POSE", help="Camera pose for secondary view. Options: CAMERA_FRONT_POSE, CAMERA_SIDE_POSE, CAMERA_WRIST_POSE")
    ap.add_argument("--camera_wrist", type=str, default="CAMERA_WRIST_POSE", help="Camera pose for wrist view. Options: CAMERA_FRONT_POSE, CAMERA_SIDE_POSE, CAMERA_WRIST_POSE")
    ap.add_argument("--randomize_camera_poses", action="store_true", help="If set, randomize camera poses")
    ap.add_argument("--distractors", action="store_true", help="If set, add distractor objects and targets.")
    args = ap.parse_args()


    if args.seed is not None:
        random.seed(args.seed)
        np.random.seed(args.seed)

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
