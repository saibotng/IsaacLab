# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import argparse

from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(
    description="This script demonstrates adding a custom robot to an Isaac Lab environment."
)
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments to spawn.")
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import numpy as np
import torch

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import AssetBaseCfg
from isaaclab.assets.articulation import ArticulationCfg
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR
from isaaclab_tasks.manager_based.tng_ur5.tng_assets.ur5.ur5 import UR5_CFG
import os


# Constant for the root/base directory of the project (dynamic)
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
print(f"Base directory: {BASE_DIR}")





class NewRobotsSceneCfg(InteractiveSceneCfg):
    """Designs the scene."""

    # Ground-plane
    ground = AssetBaseCfg(prim_path="/World/defaultGroundPlane", spawn=sim_utils.GroundPlaneCfg())

    # lights
    dome_light = AssetBaseCfg(
        prim_path="/World/Light", spawn=sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75))
    )

    # robot
    UR5 = UR5_CFG.replace(prim_path="{ENV_REGEX_NS}/UR5")


def run_simulator(sim: sim_utils.SimulationContext, scene: InteractiveScene):
    sim_dt = sim.get_physics_dt()
    sim_time = 0.0
    count = 0

    while simulation_app.is_running():
        # reset
        if count % 500 == 0:
            # reset counters
            count = 0
            # reset the scene entities to their initial positions offset by the environment origins
            root_ur5_state = scene["UR5"].data.default_root_state.clone()
            root_ur5_state[:, :3] += scene.env_origins

            # copy the default root state to the sim for the jetbot's orientation and velocity
            scene["UR5"].write_root_pose_to_sim(root_ur5_state[:, :7])
            scene["UR5"].write_root_velocity_to_sim(root_ur5_state[:, 7:])

            # copy the default joint states to the sim
            joint_pos, joint_vel = (
                scene["UR5"].data.default_joint_pos.clone(),
                scene["UR5"].data.default_joint_vel.clone(),
            )
            scene["UR5"].write_joint_state_to_sim(joint_pos, joint_vel)
            # clear internal buffers
            scene.reset()
            print("[INFO]: Resetting UR5 state...")

        # move the UR5
        ur5_action = scene["UR5"].data.default_joint_pos.clone()
        ur5_action[:, 0] = 0.5 * np.sin(2 * np.pi * 0.5 * sim_time)
        ur5_action[:, 1] = 0.5 * np.cos(2 * np.pi * 0.5 * sim_time)
        ur5_action[:, 2] = 0.5 * np.sin(2 * np.pi * 0.5 * sim_time)
        ur5_action[:, 3] = 0.5 * np.cos(2 * np.pi * 0.5 * sim_time)
        ur5_action[:, 4] = 0.5 * np.sin(2 * np.pi * 0.5 * sim_time)
        ur5_action[:, 5] = 0.5 * np.cos(2 * np.pi * 0.5 * sim_time)
        scene["UR5"].set_joint_position_target(ur5_action)  

        scene.write_data_to_sim()
        sim.step()
        sim_time += sim_dt
        count += 1
        scene.update(sim_dt)


def main():
    """Main function."""
    # Initialize the simulation context
    sim_cfg = sim_utils.SimulationCfg(device=args_cli.device)
    sim = sim_utils.SimulationContext(sim_cfg)

    sim.set_camera_view([3.5, 0.0, 3.2], [0.0, 0.0, 0.5])
    # design scene
    scene_cfg = NewRobotsSceneCfg(args_cli.num_envs, env_spacing=2.0)
    scene = InteractiveScene(scene_cfg)
    # Play the simulator
    sim.reset()
    # Now we are ready!
    print("[INFO]: Setup complete...")
    # Run the simulator
    run_simulator(sim, scene)


if __name__ == "__main__":
    main()
    simulation_app.close()
