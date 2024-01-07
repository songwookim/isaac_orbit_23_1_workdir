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
from omni.isaac.orbit.sensors.camera import Camera, CameraCfg
from omni.isaac.orbit.assets import RigidObject, RigidObjectCfg
from omni.isaac.orbit.utils.assets import NVIDIA_NUCLEUS_DIR, ISAAC_NUCLEUS_DIR

##
# Pre-defined configs
##
from orbit_assets import KINOVA_CFG  # isort:skip


def define_sensor() -> Camera:
    """Defines the camera sensor to add to the scene."""
    # Setup camera sensor
    # In contras to the ray-cast camera, we spawn the prim at these locations.
    # This means the camera sensor will be attached to these prims.
    prim_utils.create_prim("/World/Origin_rgb", "Xform")
    prim_utils.create_prim("/World/Origin_dist_to_camera", "Xform")
    # prim_utils.create_prim("/World/Origin_motion_vectors", "Xform")
    # prim_utils.create_prim("/World/Origin_instance_segmentation", "Xform")
    camera_cfg = CameraCfg(
        prim_path="/World/Origin_.*/CameraSensor",
        update_period=0,
        height=480,
        width=640,
        data_types=["rgb", "distance_to_image_plane", "normals"],
        spawn=sim_utils.PinholeCameraCfg(  # or FisheyeCameraCfg
            focal_length=24.0, focus_distance=400.0, horizontal_aperture=20.955, clipping_range=(0.1, 1.0e5)
        ),
        debug_vis=True,
    )
    # Create camera
    camera = Camera(cfg=camera_cfg)

    return camera


def design_scene() -> tuple[dict, list[list[float]]]:
    """Designs the scene."""
    # Ground-plane
    cfg = sim_utils.GroundPlaneCfg()
    cfg.func("/World/defaultGroundPlane", cfg)

    # Lights
    cfg = sim_utils.DomeLightCfg(intensity=2000.0, color=(0.75, 0.75, 0.75))
    cfg.func("/World/Light", cfg)

    # Kinova
    origins = [[0.0, -1.0, 0.0]]
    prim_utils.create_prim("/World/Origin", "Xform", translation=origins[0])

    # -- Table
    cube_cfg = RigidObjectCfg(
        prim_path="/World/Origin1/Table",
        spawn=sim_utils.CuboidCfg(  # from AssetBaseCfg
            size=(2, 2, 1),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(),  # from RigidObjectSpawnerCfg
            mass_props=sim_utils.MassPropertiesCfg(mass=5.0),  # from RigidObjectSpawnerCfg
            collision_props=sim_utils.CollisionPropertiesCfg(),  # from RigidObjectSpawnerCfg
            # visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 1.0, 1.0), metallic=0.5),  # from ShapeCfg
            # http://localhost:34080/omniverse://localhost/NVIDIA/Materials/2023_1/Automotive/Leather/Leather_Pattern_01.mdl
            visual_material=sim_utils.MdlFileCfg(
                mdl_path=f"{NVIDIA_NUCLEUS_DIR}/Materials/Base/Metals/Aluminum_Anodized.mdl",
                # mdl_path=f"{NVIDIA_NUCLEUS_DIR}/Materials/2023_1/Automotive/Leather/Leather_Pattern_02.mdl",
                albedo_brightness=0.1,
                texture_scale=(0.1, 0.1),
            ),  # from ShapeCfg
        ),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(0.0, -1.0, 0.0)),
    )
    RigidObject(cfg=cube_cfg)  # spawn object
    # -- Object
    bottle_cfg = RigidObjectCfg(
        prim_path="/World/Origin1/Object",
        spawn=sim_utils.UsdFileCfg(  # from AssetBaseCfg
            # size=(2, 2, 1),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(),  # from RigidObjectSpawnerCfg
            mass_props=sim_utils.MassPropertiesCfg(mass=5.0),  # from RigidObjectSpawnerCfg
            collision_props=sim_utils.CollisionPropertiesCfg(),  # from RigidObjectSpawnerCfg
            # visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 1.0, 1.0), metallic=0.5),  # from ShapeCfg
            usd_path=f"{ISAAC_NUCLEUS_DIR}/Environments/Office/Props/SM_BottleA.usd"
        ),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(0.0, -1.5, 1.0)),
    )
    RigidObject(cfg=bottle_cfg)  # spawn object

    # -- Robot
    kinova_arm_cfg = KINOVA_CFG.replace(prim_path="/World/Origin1/Robot")
    kinova_arm_cfg.init_state.pos = (0.0, 0.0, 1.05)
    robot_kinova = Articulation(cfg=kinova_arm_cfg)

    # Sensors
    camera = define_sensor()

    # return the scene information
    scene_entities = {"robot_kinova": robot_kinova, "camera": camera}
    return scene_entities, origins


def run_simulator(sim: sim_utils.SimulationContext, entities: dict[str, Articulation], origins: torch.Tensor):
    """Runs the simulation loop."""
    # extract entities for simplified notation
    camera: Camera = entities["camera"]

    eyes = torch.tensor([[0, -1, 1.5], [0, -1, 1.5]], device=sim.device)
    targets = torch.tensor([[0.0, -1.75, 1.0], [0.0, -1.75, 1.0]], device=sim.device)
    camera.set_world_poses_from_view(eyes, targets)
    print(f"{camera._data.pos_w} : camera")

    # Define simulation stepping
    sim_dt = sim.get_physics_dt()
    sim_time = 0.0
    count = 0
    # Simulate physics
    while simulation_app.is_running():
        # reset
        if count % 200 == 0:
            # reset counters
            sim_time = 0.0
            count = 0
            # reset the scene entities
            for index, robot in enumerate(entities.values()):
                if isinstance(robot, Camera) :
                    break
                # root state
                root_state = robot.data.default_root_state.clone()
                root_state[:, :3] += origins[index]
                robot.write_root_state_to_sim(root_state)
                # set joint positions
                joint_pos, joint_vel = robot.data.default_joint_pos.clone(), robot.data.default_joint_vel.clone()
                robot.write_joint_state_to_sim(joint_pos, joint_vel)
                # clear internal buffers
                robot.reset()
            print("[INFO]: Resetting robots state...")
        # apply random actions to the robots
        for robot in entities.values():
            if isinstance(robot, Camera) :
                break
            # generate random joint positions
            joint_pos_target = robot.data.default_joint_pos + torch.randn_like(robot.data.joint_pos) * 0.1

            # apply action to the robot
            robot.set_joint_position_target(joint_pos_target)
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
    sim_cfg = sim_utils.SimulationCfg()
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
