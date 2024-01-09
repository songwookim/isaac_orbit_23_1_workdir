# Copyright (c) 2022-2023, The ORBIT Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Package containing task implementations for various robotic environments."""

import os
import toml

# # Conveniences to other module directories via relative paths
# ORBIT_TASKS_EXT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../"))
# """Path to the extension source directory."""

# ORBIT_TASKS_METADATA = toml.load(os.path.join(ORBIT_TASKS_EXT_DIR, "config", "extension.toml"))
# """Extension metadata dictionary parsed from the extension.toml file."""

# # Configure the module-level variables
# __version__ = ORBIT_TASKS_METADATA["package"]["version"]

##
# Register Gym environments.
##
import os
import toml
import orbit_tasks.manipulation.reach.config.kinova
# from omni.isaac.orbit_tasks.utils import import_packages
# import_packages(__name__)