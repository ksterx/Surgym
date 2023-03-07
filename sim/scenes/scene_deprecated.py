import math
from abc import ABC, abstractmethod

import numpy as np
from isaacgym import gymapi

from surgym.sim.asset import Asset


class Scene(ABC):
    def __init__(self, cfg, gym, sim):
        self.cfg = cfg
        self.gym = gym
        self.sim = sim

        # List for later use
        self.envs = []
        self.robots = []
        self.softs = []
        self.tables = []

        # Before creating the env
        self._add_ground()
        cam_props, cam_pos, cam_target = self.set_cam_props()

        # Create viewer when in non-headless mode
        if not self.cfg.run.headless:
            self._create_viewer(cam_props, cam_pos, cam_target)

        # Create cameras when in save_img mode
        if self.cfg.run.save_img:
            self._create_camera(cam_props, cam_pos, cam_target)

    def _create_env(self, spacing=3.0):
        # Cubic environment
        env_lower = gymapi.Vec3(-spacing, 0, -spacing)
        env_upper = gymapi.Vec3(spacing, spacing, spacing)
        num_per_row = int(math.sqrt(self.cfg.run.num_envs))

        env = self.gym.create_env(self.sim, env_lower, env_upper, num_per_row)

        return env

    def _add_ground(self):
        plane_params = gymapi.PlaneParams()
        plane_params.distance = 0.0
        # plane_params.normal = gymapi.Vec3(0, 0, 1)
        self.gym.add_ground(self.sim, plane_params)

    def _create_viewer(self, cam_props, cam_pos, cam_target):
        self.viewer = self.gym.create_viewer(self.sim, cam_props)
        self.gym.viewer_camera_look_at(self.viewer, None, cam_pos, cam_target)
        assert self.viewer is not None, "Failed to create viewer"

    def _create_camera(self, cam_props, cam_pos, cam_target):
        self.cam = self.gym.create_camera_sensor(self.env, cam_props)
        self.gym.set_camera_location(self.cam, self.env, cam_pos, cam_target)

    def set_cam_props(self):
        """This function is used to get camera properties, which are used to create viewer and camera sensor
        return:
            cam_props: gymapi.CameraProperties
            cam_position: gymapi.Vec3
            cam_target: gymapi.Vec3
        """

        cam_props = gymapi.CameraProperties()

        for key, value in self.cfg.cam.items():
            if hasattr(cam_props, key):
                setattr(cam_props, key, value)

        cam_pos = gymapi.Vec3(*self.cfg.cam.position)
        cam_target = gymapi.Vec3(*self.cfg.cam.target)

        return cam_props, cam_pos, cam_target

    def get_robot_dof_info(self):
        dof_names = self.gym.get_asset_dof_names(self.robot_asset.asset)
        num_dofs = len(dof_names)

        print("\n=====Robot information=====")
        for i, name in enumerate(dof_names):
            print(f"{i}: {name}")
        print("===========================\n")

        return num_dofs
