import math
from abc import abstractmethod

import gym
from isaacgym import gymapi
from omegaconf import DictConfig


class BaseEnv(gym.Env):
    def __init__(self, cfg: DictConfig):
        super().__init__()

        self.cfg = cfg

        # Initialize isaacgym
        self.igym = gymapi.acquire_gym()
        self.sim = self._create_sim()

    @abstractmethod
    def step(self, action):
        # TODO: When action happens, change the state of the environment
        pass

    @abstractmethod
    def reset(self):
        pass

    @abstractmethod
    def render(self, mode="human"):
        pass

    def _create_sim(self):

        # Set simulation parameters
        sim_params = gymapi.SimParams()
        sim_params.dt = 1.0 / self.cfg.run.fps
        sim_params.substeps = self.cfg.run.substeps
        sim_params.stress_visualization = self.cfg.run.viz_stress
        sim_params.stress_visualization_min = self.cfg.run.viz_stress_min
        sim_params.stress_visualization_max = self.cfg.run.viz_stress_max
        sim_params.up_axis = gymapi.UP_AXIS_Y
        sim_params.use_gpu_pipeline = self.cfg.run.use_gpu_pipeline
        sim_params.flex.solver_type = self.cfg.flex.solver_type
        sim_params.flex.num_outer_iterations = self.cfg.flex.num_outer_iterations
        sim_params.flex.num_inner_iterations = self.cfg.flex.num_inner_iterations
        sim_params.flex.relaxation = self.cfg.flex.relaxation
        sim_params.flex.shape_collision_margin = self.cfg.flex.shape_collision_margin

        # Set graphics device
        if self.cfg.run.headless and not self.cfg.run.save_img:
            graphics_device = -1  # Disable rendering
        else:
            graphics_device = self.cfg.run.graphics_device

        # Create simulator
        sim = self.igym.create_sim(
            compute_device=self.cfg.run.compute_device,
            graphics_device=graphics_device,
            type=self.sim_type,
            params=sim_params,
        )
        assert sim is not None, "Simulation failed to initialize"
        return sim

    def _add_ground(self):
        plane_params = gymapi.PlaneParams()
        plane_params.distance = 0.0
        # plane_params.normal = gymapi.Vec3(0, 0, 1)
        self.gym.add_ground(self.sim, plane_params)

    def _create_env(self, spacing=3.0):
        # Cubic environment
        env_lower = gymapi.Vec3(-spacing, 0, -spacing)
        env_upper = gymapi.Vec3(spacing, spacing, spacing)
        num_per_row = int(math.sqrt(self.cfg.run.num_envs))

        env = self.gym.create_env(self.sim, env_lower, env_upper, num_per_row)

        return env

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
