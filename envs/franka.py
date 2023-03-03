from omni.isaac.kit import SimulationApp

# This line↓ is needed to be executed before importing other modules
app = SimulationApp({"headless": False})

import numpy as np
import omni
from omni import usd
from omni.isaac.core import World
from omni.isaac.core.robots import Robot
from omni.isac.franka import Franka

# FIXED PATH
SERVER = "omniverse://localhost"
PROJECT_DIR = "Projects/Surgym"
