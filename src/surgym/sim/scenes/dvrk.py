import math

from isaacgym import gymapi

from .. import asset
from .scene import Scene


class DVRKScene(Scene):
    def __init__(self, cfg, gym, sim):
        super().__init__(cfg, gym, sim)

        # Load assets (Override)
        self.robot_asset = asset.Asset(cfg, gym, sim, "psm").load()
        # self.soft_asset = asset.Asset(cfg, gym, sim, cfg.run.soft).load()
        # self.table_asset = self.load_asset("table")

        # Define poses of each asset (Override)
        self.robot_position = gymapi.Vec3(0.0, 0.0, -0.7)
        self.robot_rotation = (
            gymapi.Quat(0.0, 0.0, 0.0, 1.0)
            * gymapi.Quat.from_axis_angle(gymapi.Vec3(1, 0, 0), -math.pi / 2)
            * gymapi.Quat.from_axis_angle(gymapi.Vec3(0, 0, 1), math.pi)
            * gymapi.Quat.from_axis_angle(gymapi.Vec3(0, 0, 1), math.pi / 2)
        )
        self.soft_position = gymapi.Vec3(0.0, 0.0, -0.0)
        self.soft_rotation = gymapi.Quat(
            -0.707107, 0.0, 0.0, 0.707107
        ) * gymapi.Quat.from_axis_angle(gymapi.Vec3(0, 0, 1), math.pi / 2)
        self.table_position = gymapi.Vec3(0.0, 0.0, 0.0)
        self.table_rotation = gymapi.Quat(0.0, 0.0, 0.0, 1.0)
