# Copyright (c) 2022-2023, The ORBIT Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""
This script demonstrates different single-arm manipulators.

.. code-block:: bash

    # Usage
    ./orbit.sh -p source/standalone/work_dir/arms.py

"""

from __future__ import annotations

"""Launch Isaac Sim Simulator first."""


import argparse

from omni.isaac.orbit.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="This script demonstrates different single-arm manipulators.")
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import torch
import traceback

import carb
import omni.isaac.core.utils.prims as prim_utils

import omni.isaac.orbit.sim as sim_utils
from omni.isaac.orbit.assets import Articulation
from omni.isaac.orbit.utils.assets import ISAAC_NUCLEUS_DIR

##
# Pre-defined configs
##
from orbit_assets import KINOVA_CFG  # isort:skip


def design_scene() -> tuple[dict, list[list[float]]]:
    """Designs the scene."""
    # Ground-plane
    cfg = sim_utils.GroundPlaneCfg()
    cfg.func("/World/defaultGroundPlane", cfg)
    # Lights
    cfg = sim_utils.DomeLightCfg(intensity=2000.0, color=(0.75, 0.75, 0.75))
    cfg.func("/World/Light", cfg)

    # Create separate groups called "Origin1", "Origin2"
    # Each group will have a mount and a robot on top of it
    origins = [[0.0, -1.0, 0.0], [0.0, 1.0, 0.0]]

    # Origin 1 with Kinova
    prim_utils.create_prim("/World/Origin1", "Xform", translation=origins[0])
    # -- Table
    cfg = sim_utils.UsdFileCfg(
        usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Mounts/Stand/stand_instanceable.usd", scale=(2.0, 2.0, 2.0)
    )
    cfg.func("/World/Origin1/Table", cfg, translation=(0.0, 0.0, 1.05))
    # -- Robot
    kinova_arm_cfg = KINOVA_CFG.replace(prim_path="/World/Origin1/Robot")
    kinova_arm_cfg.init_state.pos = (0.0, 0.0, 1.05)
    robot_kinova = Articulation(cfg=kinova_arm_cfg)

    # return the scene information
    scene_entities = {"robot_kinova": robot_kinova}
    return scene_entities, origins


import numpy as np


def run_simulator(sim: sim_utils.SimulationContext, entities: dict[str, Articulation], origins: torch.Tensor):
    """Runs the simulation loop."""
    # Define simulation stepping
    sim_dt = sim.get_physics_dt()
    sim_time = 0.0
    count = 0
    robot = entities["robot_kinova"]
    root_state = robot.data.default_root_state.clone()
    root_state[:, :3] += origins[0]
    robot.write_root_state_to_sim(root_state)
    robot.reset()
    joint_pos, joint_vel = robot.data.default_joint_pos.clone(), robot.data.default_joint_vel.clone()

    # Simulate physics
    while simulation_app.is_running():

        if count % 2000 == 0:
            # reset counters
            sim_time = 0.0
            count = 0

            # root state
            root_state = robot.data.default_root_state.clone()
            robot.write_root_state_to_sim(root_state)
            # set joint positions
            joint_pos, joint_vel = robot.data.default_joint_pos.clone(), robot.data.default_joint_vel.clone()
            robot.write_joint_state_to_sim(joint_pos, joint_vel)
            # clear internal buffers
            robot.reset()
            print("[INFO]: Resetting robots state...")
        # apply random actions to the robots
        
        # generate random joint positions
        # joint_pos_target = joint_pos + torch.randn_like(robot.data.joint_pos) * 0.1
        # joint_pos[:, 6:12] = np.sin(sim_time * 2) * 0.5 + 0.5
        # print(f"{joint_pos[:, 6:12]}")
        # joint_pos[:, 5] = torch.randn_like(joint_pos[:, 5]) * 2
        # apply action to the robot
        robot.set_joint_position_target(joint_pos)
        # write data to sim
        robot.write_data_to_sim()
        # perform step
        sim.step()
        # update sim-time
        sim_time += sim_dt
        count += 1
        # update buffers
        for robot in entities.values():
            robot.update(sim_dt)


def main():
    """Main function."""
    # Initialize the simulation context
    sim_cfg = sim_utils.SimulationCfg(use_fabric=False)
    sim = sim_utils.SimulationContext(sim_cfg)
    # Set main camera
    sim.set_camera_view([3.5, 0.0, 3.2], [0.0, 0.0, 0.5])
    # design scene
    scene_entities, scene_origins = design_scene()
    scene_origins = torch.tensor(scene_origins, device=sim.device)
    # Play the simulator
    sim.reset()
    # Now we are ready!
    print("[INFO]: Setup complete...")
    # Run the simulator
    run_simulator(sim, scene_entities, scene_origins)


if __name__ == "__main__":
    try:
        # run the main execution
        main()
    except Exception as err:
        carb.log_error(err)
        carb.log_error(traceback.format_exc())
        raise
    finally:
        # close sim app
        simulation_app.close()
