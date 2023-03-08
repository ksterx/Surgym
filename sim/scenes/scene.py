import math
from abc import ABC, abstractmethod

import torch
from omni.isaac.core import SimulationContext, World
from omni.isaac.core.materials import ParticleMaterial, ParticleMaterialView
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.core.prims import (
    ClothPrim,
    ClothPrimView,
    ParticleSystem,
    ParticleSystemView,
    RigidPrimView,
)
from omni.isaac.core.utils import extensions, nucleus, stage
from omni.isaac.core.utils.prims import (
    get_prim_at_path,
    get_prim_path,
    is_prim_path_valid,
)
from omni.physx.scripts import deformableUtils, particleUtils, physicsUtils
from pxr import Gf, Usd, UsdGeom, UsdLux, UsdPhysics

from surgym.sim.asset import Asset


class BaseScene(ABC):
    STAGE_METERS_PER_UNIT = 0.01  # cm
    STAGE_UP_AXIS = UsdGeom.Tokens.z
    GRAVITY_DIRECTION = Gf.Vec3f(0.0, 0.0, -1.0)
    GRAVITY_MAGNITUDE = 981.0  # cm/s^2

    def __init__(self):
        self.world = World(
            stage_units_in_meter=self.STAGE_METERS_PER_UNIT,
            backend="torch",
            device="cuda",
        )
