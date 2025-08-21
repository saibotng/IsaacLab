# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""
Script to run an environment with a pick and place state machine.

The state machine is implemented in the kernel function `infer_state_machine`.
It uses the `warp` library to run the state machine in parallel on the GPU.


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
    GRASP_OBJECT = wp.constant(0.5)
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
    offset: wp.array(dtype=wp.transform),
    position_threshold: float,
    offset_item_drop: wp.array(dtype=wp.transform),
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
        des_ee_pose[tid] = wp.transform_multiply(offset[tid], object_pose[tid])
        gripper_state[tid] = GripperState.OPEN
        if distance_below_threshold(
            wp.transform_get_translation(ee_pose[tid]),
            wp.transform_get_translation(des_ee_pose[tid]),
            position_threshold,
        ):
            # wait for a while
            if sm_wait_time[tid] >= PickPlaceSmWaitTime.APPROACH_ABOVE_OBJECT:
                # move to next state and reset wait time
                sm_state[tid] = PickPlaceSmState.APPROACH_OBJECT
                sm_wait_time[tid] = 0.0

    elif state == PickPlaceSmState.APPROACH_OBJECT:
        des_ee_pose[tid] = object_pose[tid]
        gripper_state[tid] = GripperState.OPEN
        if distance_below_threshold(
            wp.transform_get_translation(ee_pose[tid]),
            wp.transform_get_translation(des_ee_pose[tid]),
            position_threshold,
        ):
            if sm_wait_time[tid] >= PickPlaceSmWaitTime.APPROACH_OBJECT:
                # move to next state and reset wait time
                sm_state[tid] = PickPlaceSmState.GRASP_OBJECT
                sm_wait_time[tid] = 0.0

    elif state == PickPlaceSmState.GRASP_OBJECT:
        des_ee_pose[tid] = object_pose[tid]
        gripper_state[tid] = GripperState.CLOSE
        # wait for a while
        if sm_wait_time[tid] >= PickPlaceSmWaitTime.GRASP_OBJECT:
            # move to next state and reset wait time
            sm_state[tid] = PickPlaceSmState.LIFT_OBJECT
            sm_wait_time[tid] = 0.0

    elif state == PickPlaceSmState.LIFT_OBJECT:
        des_ee_pose[tid] = wp.transform_multiply(offset[tid], original_object_pose[tid])
        gripper_state[tid] = GripperState.CLOSE
        if distance_below_threshold(
            wp.transform_get_translation(ee_pose[tid]),
            wp.transform_get_translation(des_ee_pose[tid]),
            0.15,
        ):
            # wait for a while
            if sm_wait_time[tid] >= PickPlaceSmWaitTime.LIFT_OBJECT:
                # move to next state and reset wait time
                sm_state[tid] = PickPlaceSmState.MOVE_ABOVE_TARGET
                sm_wait_time[tid] = 0.0

    elif state == PickPlaceSmState.MOVE_ABOVE_TARGET:
        des_ee_pose[tid] = wp.transform_multiply(offset[tid], des_object_pose[tid])
        gripper_state[tid] = GripperState.CLOSE  # Keep gripper closed while moving
        if distance_below_threshold(
            wp.transform_get_translation(ee_pose[tid]),
            wp.transform_get_translation(des_ee_pose[tid]),
            0.1,
        ):
            # wait for a while
            if sm_wait_time[tid] >= PickPlaceSmWaitTime.MOVE_ABOVE_TARGET:
                # move to next state and reset wait time
                sm_state[tid] = PickPlaceSmState.APPROACH_TARGET
                sm_wait_time[tid] = 0.0

    elif state == PickPlaceSmState.APPROACH_TARGET:
        des_ee_pose[tid] = wp.transform_multiply(offset_item_drop[tid], des_object_pose[tid])
        gripper_state[tid] = GripperState.CLOSE
        if distance_below_threshold(
            wp.transform_get_translation(ee_pose[tid]),
            wp.transform_get_translation(des_ee_pose[tid]),
            position_threshold * 0.25,
        ):
            # wait for a while
            if sm_wait_time[tid] >= PickPlaceSmWaitTime.APPROACH_TARGET:
                # move to next state and reset wait time
                sm_state[tid] = PickPlaceSmState.RELEASE_OBJECT
                sm_wait_time[tid] = 0.0

    elif state == PickPlaceSmState.RELEASE_OBJECT:
        des_ee_pose[tid] = wp.transform_multiply(offset_item_drop[tid], des_object_pose[tid])
        gripper_state[tid] = GripperState.OPEN
        # wait for a while
        if sm_wait_time[tid] >= PickPlaceSmWaitTime.RELEASE_OBJECT:
            # move to next state and reset wait time
            sm_state[tid] = PickPlaceSmState.RETRACT
            sm_wait_time[tid] = 0.0

    elif state == PickPlaceSmState.RETRACT:
        des_ee_pose[tid] = wp.transform_multiply(offset[tid], des_object_pose[tid])
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

    def __init__(self, dt: float, num_envs: int, device: torch.device | str = "cpu", position_threshold=0.01):
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

        # approach above object offset
        self.offset = torch.zeros((self.num_envs, 7), device=self.device)
        self.offset[:, 2] = 0.12
        self.offset[:, -1] = 1.0  # warp expects quaternion as (x, y, z, w)

        # approach above object offset
        self.offset_item_drop = torch.zeros((self.num_envs, 7), device=self.device)
        self.offset_item_drop[:, 2] = 0.03
        self.offset_item_drop[:, -1] = 1.0  # warp expects quaternion as (x, y, z, w)

        # convert to warp
        self.sm_dt_wp = wp.from_torch(self.sm_dt, wp.float32)
        self.sm_state_wp = wp.from_torch(self.sm_state, wp.int32)
        self.sm_wait_time_wp = wp.from_torch(self.sm_wait_time, wp.float32)
        self.des_ee_pose_wp = wp.from_torch(self.des_ee_pose, wp.transform)
        self.des_gripper_state_wp = wp.from_torch(self.des_gripper_state, wp.float32)
        self.offset_wp = wp.from_torch(self.offset, wp.transform)
        self.offset_item_drop_wp = wp.from_torch(self.offset_item_drop, wp.transform)
        self.original_object_pose_wp = None

    def reset_idx(self, env_ids: Sequence[int] = None):
        """Reset the state machine."""
        if env_ids is None:
            env_ids = slice(None)
        self.sm_state[env_ids] = 0
        self.sm_wait_time[env_ids] = 0.0
        self.original_object_pose_wp = None

    def compute(self, ee_pose: torch.Tensor, object_pose: torch.Tensor, des_object_pose: torch.Tensor) -> torch.Tensor:
        """Compute the desired state of the robot's end-effector and the gripper."""
        # convert all transformations from (w, x, y, z) to (x, y, z, w)
        ee_pose = ee_pose[:, [0, 1, 2, 4, 5, 6, 3]]
        object_pose = object_pose[:, [0, 1, 2, 4, 5, 6, 3]]
        des_object_pose = des_object_pose[:, [0, 1, 2, 4, 5, 6, 3]]

        # convert to warp
        if self.original_object_pose_wp is None:
            self.original_object_pose_wp = wp.from_torch(object_pose.contiguous(), wp.transform)
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
                self.offset_wp,
                self.position_threshold,
                self.offset_item_drop_wp,
                self.original_object_pose_wp,
            ],
            device=self.device,
        )

        # convert transformations back to (w, x, y, z)
        des_ee_pose = self.des_ee_pose[:, [0, 1, 2, 6, 3, 4, 5]]
        # convert to torch
        return torch.cat([des_ee_pose, self.des_gripper_state.unsqueeze(-1)], dim=-1)

#TODO: remove magic numbers
#TODO: improve and parametrize state machine (and use home orientation)

def main():
    # parse configuration
    env_cfg: PickAndPlaceEnvCfg = parse_env_cfg(
        "TNG-Pick-And-Place-Cube-UR5-IK-Abs-Record-v0",
        device=args_cli.device,
        num_envs=args_cli.num_envs,
        use_fabric=not args_cli.disable_fabric,
    )
    # create environment
    env = gym.make("TNG-Pick-And-Place-Cube-UR5-IK-Abs-Record-v0", cfg=env_cfg)

    obs, _ = env.reset()
    home_orientation = torch.zeros((env.unwrapped.num_envs, 4), device=env.unwrapped.device)
    home_orientation[:, 0] = 0.0
    home_orientation[:, 1] = 0.70710678118
    home_orientation[:, 2] = -0.70710678118
    home_orientation[:, 3] = 0.0

    # create state machine
    pick_sm = PickAndPlaceSm(
        env_cfg.sim.dt * env_cfg.decimation, env.unwrapped.num_envs, env.unwrapped.device, position_threshold=0.01
    )
    done_counter = 0
    try:
        while simulation_app.is_running():
            # run everything in inference mode
            with torch.inference_mode():
                env.unwrapped.extras["state"] = pick_sm.sm_state

                tcp_pose = obs["end_effector"]
                object_pose = obs["rigid_objects"]["object_pose"]
                target_pose = obs["rigid_objects"]["target_pose"]

                actions = pick_sm.compute(
                    tcp_pose, object_pose, target_pose
                )

                obs, _, terminated, truncated, _ = env.step(actions)

                if terminated.any():
                    done_counter += sum(terminated)
                    print(f"Done counter: {done_counter}")
                    pick_sm.reset_idx(terminated.nonzero(as_tuple=False).squeeze(-1))
    finally:
        # close the environment
        try: env.close()
        finally: simulation_app.close()


if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()
