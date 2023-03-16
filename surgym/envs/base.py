# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#


import os

import carb
import gym
from omni.isaac.kit import SimulationApp

STAGE_UNITS_IN_METERS = 1.0
DELTA_TIME = 1.0 / 60.0


class Env(gym.Env):
    """ This class provides a base interface for connecting RL policies with task implementations.
        APIs provided in this interface follow the interface in gym.Env.
        This class also provides utilities for initializing simulation apps, creating the World,
        and registering a task.
    """

    def __init__(self, headless: bool, sim_device: int = 0) -> None:
        """ Initializes RL and task parameters.

        Args:
            headless (bool): Whether to run training headless.
            sim_device (int): GPU device ID for running physics simulation. Defaults to 0.
        """

        experience = ""
        if headless:
            experience = f'{os.environ["EXP_PATH"]}/omni.isaac.sim.python.gym.headless.kit'

        self.app = SimulationApp(
            {"headless": headless, "physics_device": sim_device}, experience=experience
        )
        carb.settings.get_settings().set("/persistent/omnihydra/useSceneGraphInstancing", True)
        self._render = not headless
        self.sim_frame_count = 0

    def set_task(self, task, backend="numpy", sim_params=None, init_sim=True) -> None:
        """ Creates a World object and adds Task to World.
            Initializes and registers task to the environment interface.
            Triggers task start-up.

        Args:
            task (RLTask): The task to register to the env.
            backend (str): Backend to use for task. Can be "numpy" or "torch". Defaults to "numpy".
            sim_params (dict): Simulation parameters for physics settings. Defaults to None.
            init_sim (Optional[bool]): Automatically starts simulation. Defaults to True.
        """

        from omni.isaac.core.utils.stage import get_current_stage
        from omni.isaac.core.world import World

        device = "cpu"
        if sim_params and "use_gpu_pipeline" in sim_params:
            if sim_params["use_gpu_pipeline"]:
                device = "cuda"

        self.world = World(
            stage_units_in_meters=STAGE_UNITS_IN_METERS, rendering_dt=DELTA_TIME, backend=backend, sim_params=sim_params, device=device
        )
        self.stage = self.world.scene.stage()
        self.world.add_task(task)
        self.task = task
        self.num_envs = task.num_envs

        self.observation_space = task.observation_space
        self.action_space = task.action_space

        if sim_params and "enable_viewport" in sim_params:
            if not sim_params["enable_viewport"]:
                import omni

                manager = omni.kit.app.get_app().get_extension_manager()
                manager.set_extension_enabled_immediate("omni.kit.viewport.window", False)

        if init_sim:
            self.world.reset()

    def render(self, mode="human") -> None:
        """ Step the renderer.

        Args:
            mode (str): Select mode of rendering based on OpenAI environments.
        """

        if mode == "human":
            self.world.render()
        else:
            gym.Env.render(self, mode=mode)
        return

    def close(self) -> None:
        """ Closes simulation.
        """

        # bypass USD warnings on stage close
        import omni.usd

        omni.usd.get_context().get_stage().GetRootLayer().Clear()
        self.app.close()
        return

    def seed(self, seed=-1):
        """ Sets a seed. Pass in -1 for a random seed.

        Args:
            seed (int): Seed to set. Defaults to -1.
        Returns:
            seed (int): Seed that was set.
        """

        from omni.isaac.core.utils.torch.maths import set_seed

        return set_seed(seed)

    def step(self, actions):
        """ Basic implementation for stepping simulation.
            Can be overriden by inherited Env classes
            to satisfy requirements of specific RL libraries. This method passes actions to task
            for processing, steps simulation, and computes observations, rewards, and resets.

        Args:
            actions (Union[numpy.ndarray, torch.Tensor]): Actions buffer from policy.
        Returns:
            observations(Union[numpy.ndarray, torch.Tensor]): Buffer of observation data.
            rewards(Union[numpy.ndarray, torch.Tensor]): Buffer of rewards data.
            dones(Union[numpy.ndarray, torch.Tensor]): Buffer of resets/dones data.
            info(dict): Dictionary of extras data.
        """
        self.task.pre_physics_step(actions)
        self.world.step(render=self._render)

        self.sim_frame_count += 1

        observations = self.task.get_observations()
        rewards = self.task.calculate_metrics()
        dones = self.task.is_done()
        info = {}

        return observations, rewards, dones, info

    def reset(self):
        """ Resets the task and updates observations. """
        self.task.reset()
        self.world.step(render=self._render)
        observations = self.task.get_observations()

        return observations

    @property
    def num_envs(self):
        """ Retrieves number of environments.

        Returns:
            num_envs(int): Number of environments.
        """
        return self.num_envs
