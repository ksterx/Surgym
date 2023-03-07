import math
from abc import ABC, abstractmethod

from omni.physx.scripts import physicsUtils
from pxr import Gf, UsdGeom, UsdLux, UsdPhysics

from surgym.sim.asset import Asset


class BaseScene(ABC):
    STAGE_METERS_PER_UNIT = 0.01  # cm
    STAGE_UP_AXIS = UsdGeom.Tokens.z
    GRAVITY_DIRECTION = Gf.Vec3f(0.0, 0.0, -1.0)
    GRAVITY_MAGNITUDE = 981.0  # cm/s^2
