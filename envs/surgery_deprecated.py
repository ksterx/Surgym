from surgym.envs.base import BaseEnv
from surgym.sim.asset import RigidAsset, SoftAsset


class SurgeryEnv(BaseEnv):
    def __init__(self, cfg):
        """Action is continuous and oberservation is continuous.

        Args:
            max_episode_steps (_type_, optional): _description_. Defaults to None.
        """
        super().__init__()
        self.action_space = None  # TODO: define action space
        self.observation_space = None  # TODO: define observation space
        self.max_episode_steps = None  # TODO: read from config

        # Add assets to the environment
        robot_asset = RigidAsset(cfg, self.igym, self.sim, "franka")
        soft_asset = SoftAsset(cfg, self.igym, self.sim, cfg.run.soft)

        # Before creating the env
        self._add_ground()
        cam_props, cam_pos, cam_target = self.set_cam_props()

        # Create viewer when in non-headless mode
        if not self.cfg.run.headless:
            self._create_viewer(cam_props, cam_pos, cam_target)

        # Create cameras when in save_img mode
        if self.cfg.run.save_img:
            self._create_camera(cam_props, cam_pos, cam_target)

        # Create actors

    def step(self, action):
        """Step the environment from the action the agent takes and return the observation, reward, done, info.

        Args:
            action (torch/gymtorch): action taken by the agent

        Returns:
            obs: observation of the environment
            reward: reward of the action
            done: whether the episode is done
            info: additional information
        """
        assert self.action_space.contains(action)
        # TODO: When a model predict an action, change the state of the environment

        return obs, reward, done, info

    def reset(self):
        return 0

    def render(self, mode="human"):
        return 0

    def close(self):
        pass

    def seed(self, seed=None):
        pass
