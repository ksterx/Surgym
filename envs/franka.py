from pathlib import Path

from omni.isaac.kit import SimulationApp

# This line↓ is needed to be executed before importing other modules
app = SimulationApp({"headless": False})


import numpy as np
import omni
from omni import usd
from omni.isaac.core import SimulationContext, World
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.stage import add_reference_to_stage, is_stage_loading
from omni.isaac.franka import Franka

# FIXED PATH
SERVER = "omniverse://localhost"
PROJECT_DIR = "Projects/Surgym"
# SCENE_USD = "simple_env.usd"
SCENE_USD = "scene_template.usd"
FRANKA_PATH = "Isaac/Robots/Franka/franka_alt_fingers.usd"

world = World(stage_units_in_meters=1.0)
stage = app.context.get_stage()


# world.scene.add_default_ground_plane()
# franka = Franka(
#     prim_path="/World/Franka",
#     name="franka",
#     position=np.array([0, 0, -1]),
# )
# world.scene.add(franka)
world.reset()
world.render()

# # load stage
usd_path = f"{SERVER}/{PROJECT_DIR}/{SCENE_USD}"
context = SimulationContext()
add_reference_to_stage(usd_path, "/World")
# omni.usd.get_context().open_stage(usd_path)
# stage = app.context.get_stage()

box = stage.GetPrimAtPath("/World/box_01")

print(box.GetPropertyNames())
print(box.GetProperties())

while app.is_running():
    world.step(render=True)
    # print(franka.get_world_pose())
    # print(franka.get_joint_positions())
app.close()
