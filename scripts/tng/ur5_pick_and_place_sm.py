# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""
Script to run an environment with a pick and place state machine.

The state machine is implemented in the kernel function `infer_state_machine`.
It uses the `warp` library to run the state machine in parallel on the GPU.
start with:
python scripts/tng/ur5_pick_and_place_sm.py --headless --enable_cameras --blackwell --from_yaml tng_datasets/random_100.yaml

"""

"""Launch Omniverse Toolkit first."""

import argparse

from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Pick and place state machine for pick and place environments.")
parser.add_argument(
    "--disable_fabric", action="store_true", default=False, help="Disable fabric and use USD I/O operations."
)
parser.add_argument("--num_envs", type=int, default=2, help="Number of environments to simulate.")

parser.add_argument("--from_yaml", type=str, default=None, help="Path to the dataset YAML file.")
parser.add_argument("--blackwell", action="store_true", help="Enable this when using a RTX 50xx GPU")

# Parse args first to check if renderer should be enabled
temp_args, _ = parser.parse_known_args()

# Conditionally add renderer arguments
if temp_args.blackwell:
    parser.add_argument(
        "--renderer",
        type=str,
        default="PathTracing",
        choices=["RayTracedLighting", "PathTracing"],
        help="Renderer to use.",
    )
    parser.add_argument(
        "--samples_per_pixel_per_frame",
        type=int,
        default=1,
        help="Number of samples per pixel per frame.",
    )
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything else."""

import gymnasium as gym
import torch
from collections.abc import Sequence

import warp as wp

from isaaclab.assets.rigid_object.rigid_object_data import RigidObjectData

import isaaclab_tasks  # noqa: F401
from isaaclab_tasks.manager_based.tng_ur5.ur5_pick_and_place.pick_and_place_env_cfg import PickAndPlaceEnvCfg
from isaaclab_tasks.utils.parse_cfg import parse_env_cfg
from utils.tng_sctipt_utils import patch_env_config_for_configuration_scheduling
from isaaclab_tasks.manager_based.tng_ur5.env_utils.env_config_scheduler import EnvConfigSchedulerDatagen
from isaaclab_tasks.manager_based.tng_ur5.tng_assets.ur5.ur5 import GRIPPING_CENTER_OFFSET

# initialize warp
wp.init()


class GripperState:
    """States for the gripper."""

    OPEN = wp.constant(1.0)
    CLOSE = wp.constant(-1.0)


class PickPlaceSmState:
    """States for the pick state machine."""

    REST = wp.constant(0)
    APPROACH_ABOVE_OBJECT = wp.constant(1)
    APPROACH_OBJECT = wp.constant(2)
    GRASP_OBJECT = wp.constant(3)
    LIFT_OBJECT = wp.constant(4)
    MOVE_ABOVE_TARGET = wp.constant(5)
    APPROACH_TARGET = wp.constant(6)
    RELEASE_OBJECT = wp.constant(7)
    RETRACT = wp.constant(8)  
    DONE = wp.constant(9)       


class PickPlaceSmWaitTime:
    """Additional wait times (in s) for states for before switching."""

    REST = wp.constant(0.3)
    APPROACH_ABOVE_OBJECT = wp.constant(1.0)
    APPROACH_OBJECT = wp.constant(0.3)
    GRASP_OBJECT = wp.constant(0.4)
    LIFT_OBJECT = wp.constant(0.3)
    MOVE_ABOVE_TARGET = wp.constant(0.2)
    APPROACH_TARGET = wp.constant(0.3)
    RELEASE_OBJECT = wp.constant(0.3)
    RETRACT = wp.constant(0.8)



@wp.func
def distance_below_threshold(current_pos: wp.vec3, desired_pos: wp.vec3, threshold: float) -> bool:
    return wp.length(current_pos - desired_pos) < threshold


@wp.kernel
def infer_state_machine(
    dt: wp.array(dtype=float),
    sm_state: wp.array(dtype=int),
    sm_wait_time: wp.array(dtype=float),
    ee_pose: wp.array(dtype=wp.transform),
    object_pose: wp.array(dtype=wp.transform),
    des_object_pose: wp.array(dtype=wp.transform),
    des_ee_pose: wp.array(dtype=wp.transform),
    gripper_state: wp.array(dtype=float),
    offset_travel: wp.array(dtype=wp.transform),
    position_threshold: float,
    offset_item_drop: wp.array(dtype=wp.transform),
    offset_gripping_center: wp.array(dtype=wp.transform),
    original_object_pose: wp.array(dtype=wp.transform),
):
    # retrieve thread id
    tid = wp.tid()
    # retrieve state machine state
    state = sm_state[tid]
    # decide next state
    if state == PickPlaceSmState.REST:
        des_ee_pose[tid] = ee_pose[tid]
        gripper_state[tid] = GripperState.OPEN
        # wait for a while
        if sm_wait_time[tid] >= PickPlaceSmWaitTime.REST:
            # move to next state and reset wait time
            sm_state[tid] = PickPlaceSmState.APPROACH_ABOVE_OBJECT
            sm_wait_time[tid] = 0.0

    elif state == PickPlaceSmState.APPROACH_ABOVE_OBJECT:
        des_ee_pose[tid] = wp.transform_multiply(offset_travel[tid] + offset_gripping_center[tid], object_pose[tid])
        gripper_state[tid] = GripperState.OPEN
        if distance_below_threshold(
            wp.transform_get_translation(ee_pose[tid]),
            wp.transform_get_translation(des_ee_pose[tid]),
            position_threshold * 0.4,
        ):
            # wait for a while
            if sm_wait_time[tid] >= PickPlaceSmWaitTime.APPROACH_ABOVE_OBJECT:
                # move to next state and reset wait time
                sm_state[tid] = PickPlaceSmState.APPROACH_OBJECT
                sm_wait_time[tid] = 0.0

    elif state == PickPlaceSmState.APPROACH_OBJECT:
        des_ee_pose[tid] = wp.transform_multiply(offset_gripping_center[tid], object_pose[tid])
        gripper_state[tid] = GripperState.OPEN
        if distance_below_threshold(
            wp.transform_get_translation(ee_pose[tid]),
            wp.transform_get_translation(des_ee_pose[tid]),
            position_threshold * 0.2,
        ):
            if sm_wait_time[tid] >= PickPlaceSmWaitTime.APPROACH_OBJECT:
                # move to next state and reset wait time
                sm_state[tid] = PickPlaceSmState.GRASP_OBJECT
                sm_wait_time[tid] = 0.0

    elif state == PickPlaceSmState.GRASP_OBJECT:
        des_ee_pose[tid] = wp.transform_multiply(offset_gripping_center[tid], object_pose[tid])
        gripper_state[tid] = GripperState.CLOSE
        # wait for a while
        if sm_wait_time[tid] >= PickPlaceSmWaitTime.GRASP_OBJECT:
            # move to next state and reset wait time
            sm_state[tid] = PickPlaceSmState.LIFT_OBJECT
            sm_wait_time[tid] = 0.0

    elif state == PickPlaceSmState.LIFT_OBJECT:
        des_ee_pose[tid] = wp.transform_multiply(offset_travel[tid] + offset_gripping_center[tid], original_object_pose[tid])
        gripper_state[tid] = GripperState.CLOSE
        if distance_below_threshold(
            wp.transform_get_translation(ee_pose[tid]),
            wp.transform_get_translation(des_ee_pose[tid]),
            position_threshold,
        ):
            # wait for a while
            if sm_wait_time[tid] >= PickPlaceSmWaitTime.LIFT_OBJECT:
                # move to next state and reset wait time
                sm_state[tid] = PickPlaceSmState.MOVE_ABOVE_TARGET
                sm_wait_time[tid] = 0.0

    elif state == PickPlaceSmState.MOVE_ABOVE_TARGET:
        des_ee_pose[tid] = wp.transform_multiply(offset_travel[tid] + offset_gripping_center[tid], des_object_pose[tid])
        gripper_state[tid] = GripperState.CLOSE  # Keep gripper closed while moving
        if distance_below_threshold(
            wp.transform_get_translation(ee_pose[tid]),
            wp.transform_get_translation(des_ee_pose[tid]),
            position_threshold,
        ):
            # wait for a while
            if sm_wait_time[tid] >= PickPlaceSmWaitTime.MOVE_ABOVE_TARGET:
                # move to next state and reset wait time
                sm_state[tid] = PickPlaceSmState.APPROACH_TARGET
                sm_wait_time[tid] = 0.0

    elif state == PickPlaceSmState.APPROACH_TARGET:
        des_ee_pose[tid] = wp.transform_multiply(offset_item_drop[tid] + offset_gripping_center[tid], des_object_pose[tid])
        gripper_state[tid] = GripperState.CLOSE
        if distance_below_threshold(
            wp.transform_get_translation(ee_pose[tid]),
            wp.transform_get_translation(des_ee_pose[tid]),
            position_threshold * 0.1,
        ):
            # wait for a while
            if sm_wait_time[tid] >= PickPlaceSmWaitTime.APPROACH_TARGET:
                # move to next state and reset wait time
                sm_state[tid] = PickPlaceSmState.RELEASE_OBJECT
                sm_wait_time[tid] = 0.0

    elif state == PickPlaceSmState.RELEASE_OBJECT:
        des_ee_pose[tid] = wp.transform_multiply(offset_item_drop[tid] + offset_gripping_center[tid], des_object_pose[tid])
        gripper_state[tid] = GripperState.OPEN
        # wait for a while
        if sm_wait_time[tid] >= PickPlaceSmWaitTime.RELEASE_OBJECT:
            # move to next state and reset wait time
            sm_state[tid] = PickPlaceSmState.RETRACT
            sm_wait_time[tid] = 0.0

    elif state == PickPlaceSmState.RETRACT:
        des_ee_pose[tid] = wp.transform_multiply(offset_travel[tid] + offset_gripping_center[tid], des_object_pose[tid])
        gripper_state[tid] = GripperState.OPEN
        if distance_below_threshold(
            wp.transform_get_translation(ee_pose[tid]),
            wp.transform_get_translation(des_ee_pose[tid]),
            position_threshold,
        ):
            # wait for a while
            if sm_wait_time[tid] >= PickPlaceSmWaitTime.RETRACT:
                # move to next state and reset wait time
                sm_state[tid] = PickPlaceSmState.DONE
                sm_wait_time[tid] = 0.0

    elif state == PickPlaceSmState.DONE:
        # keep the end-effector in the last position
        des_ee_pose[tid] = ee_pose[tid]
        gripper_state[tid] = GripperState.OPEN
        return

    # increment wait time
    sm_wait_time[tid] = sm_wait_time[tid] + dt[tid]


class PickAndPlaceSm:
    DEFAULT_TRAVEL_HEIGHT = 0.12
    """A simple state machine in a robot's task space to pick and place an object.

    The state machine is implemented as a warp kernel. It takes in the current state of
    the robot's end-effector and the object, and outputs the desired state of the robot's
    end-effector and the gripper. The state machine is implemented as a finite state
    machine with the following states:

    1. REST: The robot is at rest.
    2. APPROACH_ABOVE_OBJECT: The robot moves above the object.
    3. APPROACH_OBJECT: The robot moves to the object.
    4. GRASP_OBJECT: The robot grasps the object.
    5. LIFT_OBJECT: The robot lifts the object to the desired pose. This is the final state.
    """

    def __init__(self, dt: float, num_envs: int, position_threshold: float, device: torch.device | str = "cpu", ):
        """Initialize the state machine.

        Args:
            dt: The environment time step.
            num_envs: The number of environments to simulate.
            device: The device to run the state machine on.
        """
        # save parameters
        self.dt = float(dt)
        self.num_envs = num_envs
        self.device = device
        self.position_threshold = position_threshold
        # initialize state machine
        self.sm_dt = torch.full((self.num_envs,), self.dt, device=self.device)
        self.sm_state = torch.full((self.num_envs,), 0, dtype=torch.int32, device=self.device)
        self.sm_wait_time = torch.zeros((self.num_envs,), device=self.device)

        # desired state
        self.des_ee_pose = torch.zeros((self.num_envs, 7), device=self.device)
        self.des_gripper_state = torch.full((self.num_envs,), 0.0, device=self.device)

        self.travel_height = PickAndPlaceSm.DEFAULT_TRAVEL_HEIGHT
        self.item_drop_height = 0.03

        # approach above object offset
        self.offset_travel = torch.zeros((self.num_envs, 7), device=self.device)
        self.offset_travel[:, 2] = self.travel_height

        # approach above object offset
        self.offset_item_drop = torch.zeros((self.num_envs, 7), device=self.device)
        self.offset_item_drop[:, 2] = self.item_drop_height

        self.offset_gripping_center = torch.zeros((self.num_envs, 7), device=self.device)
        self.offset_gripping_center[:, 2] = GRIPPING_CENTER_OFFSET
        self.offset_gripping_center[:, -1] = 1.0

        # convert to warp
        self.sm_dt_wp = wp.from_torch(self.sm_dt, wp.float32)
        self.sm_state_wp = wp.from_torch(self.sm_state, wp.int32)
        self.sm_wait_time_wp = wp.from_torch(self.sm_wait_time, wp.float32)
        self.des_ee_pose_wp = wp.from_torch(self.des_ee_pose, wp.transform)
        self.des_gripper_state_wp = wp.from_torch(self.des_gripper_state, wp.float32)
        self.offset_travel_wp = wp.from_torch(self.offset_travel, wp.transform)
        self.offset_item_drop_wp = wp.from_torch(self.offset_item_drop, wp.transform)
        self.offset_gripping_center_wp = wp.from_torch(self.offset_gripping_center, wp.transform)
        self.original_object_pose = torch.full(
            (self.num_envs, 7), float('nan'), device=self.device
        ) 
        self.original_object_pose_wp = wp.from_torch(self.original_object_pose, wp.transform)
        self.has_original_object_pose = torch.zeros((self.num_envs,), dtype=torch.bool, device=self.device)

    def reset_idx(self, env_ids: Sequence[int] = None):
        """Reset the state machine."""
        if env_ids is None:
            env_ids = slice(None)

        self.sm_state[env_ids] = 0
        self.sm_wait_time[env_ids] = 0.0
        self.has_original_object_pose[env_ids] = False
        self.original_object_pose[env_ids] = float('nan')

    def compute(self, ee_pose: torch.Tensor, object_pose: torch.Tensor, des_object_pose: torch.Tensor, travel_height: torch.Tensor) -> torch.Tensor:
        """Compute the desired state of the robot's end-effector and the gripper."""
        # convert all transformations from (w, x, y, z) to (x, y, z, w)
        self.offset_travel[:, 2] = travel_height
        ee_pose = ee_pose[:, [0, 1, 2, 4, 5, 6, 3]]
        object_pose = object_pose[:, [0, 1, 2, 4, 5, 6, 3]]
        des_object_pose = des_object_pose[:, [0, 1, 2, 4, 5, 6, 3]]

        needs_init = (self.sm_state == PickPlaceSmState.GRASP_OBJECT) & (~self.has_original_object_pose)
        if needs_init.any():
            self.original_object_pose[needs_init] = object_pose[needs_init].clone()
            self.has_original_object_pose[needs_init] = True
        ee_pose_wp = wp.from_torch(ee_pose.contiguous(), wp.transform)
        object_pose_wp = wp.from_torch(object_pose.contiguous(), wp.transform)
        des_object_pose_wp = wp.from_torch(des_object_pose.contiguous(), wp.transform)

        # run state machine
        wp.launch(
            kernel=infer_state_machine,
            dim=self.num_envs,
            inputs=[
                self.sm_dt_wp,
                self.sm_state_wp,
                self.sm_wait_time_wp,
                ee_pose_wp,
                object_pose_wp,
                des_object_pose_wp,
                self.des_ee_pose_wp,
                self.des_gripper_state_wp,
                self.offset_travel_wp,
                self.position_threshold,
                self.offset_item_drop_wp,
                self.offset_gripping_center_wp,
                self.original_object_pose_wp,
            ],
            device=self.device,
        )

        # convert transformations back to (w, x, y, z)
        des_ee_pose = self.des_ee_pose[:, [0, 1, 2, 6, 3, 4, 5]]
        # convert to torch
        return torch.cat([des_ee_pose, self.des_gripper_state.unsqueeze(-1)], dim=-1)



def main():
    # parse configuration
    env_cfg: PickAndPlaceEnvCfg = parse_env_cfg(
        "TNG-Pick-And-Place-Cube-UR5-IK-Abs-Record-v0",
        device=args_cli.device,
        num_envs=args_cli.num_envs,
        use_fabric=not args_cli.disable_fabric,
    )
    # create environment
    if args_cli.from_yaml:
        patch_env_config_for_configuration_scheduling(env_cfg, args_cli.from_yaml, "datagen")

    env = gym.make("TNG-Pick-And-Place-Cube-UR5-IK-Abs-Record-v0", cfg=env_cfg)
    obs, _ = env.reset()
    scheduler: EnvConfigSchedulerDatagen = env.unwrapped.extras.get("scheduler", None)
    device = env.unwrapped.device
    num_envs = env.unwrapped.num_envs

    home_orientation = torch.zeros((num_envs, 4), device=device)
    home_orientation[:, 0] = 0.0
    home_orientation[:, 1] = 0.70710678118
    home_orientation[:, 2] = -0.70710678118
    home_orientation[:, 3] = 0.0

    # create state machine
    pick_sm = PickAndPlaceSm(
        env_cfg.sim.dt * env_cfg.decimation, num_envs, position_threshold=0.065, device=device
    )
    done_counter = 0
    success_counter = 0
    idle_mask = torch.zeros(num_envs, dtype=torch.bool, device=device)
    idle_action = torch.ones((1, env.unwrapped.action_space.shape[-1]), device=device)*0.5
    try:
        while simulation_app.is_running():
            # run everything in inference mode
            with torch.inference_mode():
                if scheduler:
                    idle_mask = scheduler.idle_mask.clone()

                env.unwrapped.extras["state"] = pick_sm.sm_state
                tcp_pose = obs["end_effector"]
                object_pose = obs["rigid_objects"]["object_pose"]
                target_pose = obs["rigid_objects"]["target_pose"]

                if scheduler:
                    try:
                        travel_height = torch.tensor(scheduler.get_travel_heights_for_envs(torch.arange(num_envs, device=device)), device=device)#
                    except KeyError:
                        travel_height = torch.full((num_envs,), PickAndPlaceSm.DEFAULT_TRAVEL_HEIGHT, device=device)
                        print("WARNING: 'travel_height' not found in case, using default.")

                else:
                    travel_height = torch.full((num_envs,), PickAndPlaceSm.DEFAULT_TRAVEL_HEIGHT, device=device)

                actions = pick_sm.compute(
                    tcp_pose, object_pose, target_pose, travel_height
                )
                actions[idle_mask] = idle_action

                obs, _, terminated, truncated, _ = env.step(actions)
                done_mask = (terminated | truncated).to(device=env.unwrapped.device)

                if done_mask.any():
                    relevant_dones = done_mask & (~idle_mask)
                    relevant_successes = env.unwrapped.termination_manager.get_term("success").to(device=env.unwrapped.device) & (~idle_mask)
                    done_counter += sum(relevant_dones)
                    success_counter += sum(relevant_successes)
                    print(f"Successful terminations: {success_counter} / {done_counter}")
                    pick_sm.reset_idx(done_mask.nonzero(as_tuple=False).squeeze(-1))

                    if scheduler:
                        all_assigned = (scheduler.cursor >= len(scheduler.order))
                        inflight = len([case for case in scheduler.cases_being_processed if case is not None])
                        if all_assigned and inflight == 0:
                            print("All cases processed, exiting.")
                            break
    finally:
        try: env.close()
        finally: simulation_app.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
