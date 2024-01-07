# Copyright (c) 2022-2023, The ORBIT Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Configuration for the Franka Emika robots.

The following configurations are available:

* :obj:`FRANKA_PANDA_CFG`: Franka Emika Panda robot with Panda hand
* :obj:`FRANKA_PANDA_HIGH_PD_CFG`: Franka Emika Panda robot with Panda hand with stiffer PD control

Reference: https://github.com/frankaemika/franka_ros
"""

import omni.isaac.orbit.sim as sim_utils
from omni.isaac.orbit.actuators import ImplicitActuatorCfg
from omni.isaac.orbit.assets.articulation import ArticulationCfg
from omni.isaac.orbit.utils.assets import ISAAC_ORBIT_NUCLEUS_DIR, ISAAC_NUCLEUS_DIR

##
# Configuration
##
import numpy as np
deg2rad = np.pi / 180

KINOVA_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"{ISAAC_NUCLEUS_DIR}/Robots/Kinova/Jaco2/J2N6S300/j2n6s300_instanceable.usd",
        activate_contact_sensors=False,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            max_depenetration_velocity=5.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True, solver_position_iteration_count=8, solver_velocity_iteration_count=0
        ),
        # collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=0.005, rest_offset=0.0),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        # joint_pos={
        #     # "j2n6s300_joint_0": 180 * deg2rad,
        #     "j2n6s300_joint_1": 210 * deg2rad,
        #     "j2n6s300_joint_2": 0,
        #     "j2n6s300_joint_3": 60 * deg2rad,
        #     "j2n6s300_joint_4": 90 * deg2rad,
        #     "j2n6s300_joint_5": 90 * deg2rad,
        #     "j2n6s300_joint_6": 180 * deg2rad,
        #     # "panda_finger_joint.*": 0.04,
        #     "j2n6s300_joint_finger.*": 0,
        #     # "j2n6s300_link_finger_1",
        #     # "j2n6s300_link_finger_tip_1",
        #     # "j2n6s300_link_finger_2",
        #     # "j2n6s300_link_finger_tip_2",
        #     # "j2n6s300_link_finger_3",
        #     # "j2n6s300_link_finger_tip_3",
        # },
        joint_pos={
            "j2n6s300_joint_1": 0 * deg2rad,
            "j2n6s300_joint_2": 180 * deg2rad,
            "j2n6s300_joint_3": 180 * deg2rad,
            "j2n6s300_joint_4": 180 * deg2rad,
            "j2n6s300_joint_5": 180 * deg2rad,
            "j2n6s300_joint_6": 180 * deg2rad,
            "j2n6s300_joint_finger.*": 1,
        },
    ),
    actuators={
        "arm": ImplicitActuatorCfg(
            joint_names_expr=["j2n6s300_joint_[1-7]"],
            effort_limit=87.0,
            velocity_limit=2.175,
            stiffness=80.0,
            damping=4.0,
        ),
        # "panda_forearm": ImplicitActuatorCfg(
        #     joint_names_expr=["panda_joint[5-7]"],
        #     effort_limit=12.0,
        #     velocity_limit=2.61,
        #     stiffness=80.0,
        #     damping=4.0,
        # ),
        "end_effector": ImplicitActuatorCfg(
            joint_names_expr=["j2n6s300_joint_finger_.*"],
            effort_limit=200.0,
            velocity_limit=0.2,
            stiffness=2e3,
            damping=1e2,
        ),
    },
    soft_joint_pos_limit_factor=1.0,
)
"""Configuration of Franka Emika Panda robot."""


FRANKA_PANDA_HIGH_PD_CFG = KINOVA_CFG.copy()
FRANKA_PANDA_HIGH_PD_CFG.spawn.rigid_props.disable_gravity = True
FRANKA_PANDA_HIGH_PD_CFG.actuators["arm"].stiffness = 400.0
FRANKA_PANDA_HIGH_PD_CFG.actuators["arm"].damping = 80.0
FRANKA_PANDA_HIGH_PD_CFG.actuators["end_effector"].stiffness = 400.0
FRANKA_PANDA_HIGH_PD_CFG.actuators["end_effector"].damping = 80.0
"""Configuration of Franka Emika Panda robot with stiffer PD control.

This configuration is useful for task-space control using differential IK.
"""
