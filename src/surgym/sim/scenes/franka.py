import math

from isaacgym import gymapi, gymtorch

from .. import asset
from .scene import Scene


class FrankaScene(Scene):
    def __init__(self, cfg, gym, sim):
        super().__init__(cfg, gym, sim)

        # Load assets (Override)
        soft_asset = asset.SoftAsset(cfg, gym, sim, cfg.run.soft)
        robot_asset = asset.RigidAsset(cfg, gym, sim, "franka")
        table_dims = gymapi.Vec3(0.6, 0.5, 1.0)
        table_options = gymapi.AssetOptions()
        table_options.fix_base_link = True
        table_asset = gym.create_box(
            sim, table_dims.x, table_dims.y, table_dims.z, table_options
        )

        for i in range(self.cfg.run.num_envs):
            env = self._create_env(spacing=3.0)
            self.envs.append(env)

            # Create soft body actor
            soft_pose = gymapi.Transform()
            soft_pose.p = gymapi.Vec3(0.5, 0.6, 0.0)
            soft_pose.r = gymapi.Quat(-0.707107, 0.0, 0.0, 0.707107)

            # Randomize soft body properties
            if i == 0:
                soft_actor = soft_asset.to_actor(env, i, soft_pose, use_cfg=True)
            else:
                soft_actor = soft_asset.to_actor(
                    env,
                    i,
                    soft_pose,
                    use_cfg=False,
                )

            # Create robot actor
            robot_pose = gymapi.Transform()
            robot_pose.p = gymapi.Vec3(0.0, 0.0, 0.0)
            robot_pose.r = gymapi.Quat.from_axis_angle(
                gymapi.Vec3(1, 0, 0), -math.pi / 2
            )  # Set robot upright
            # robot_actor = gym.create_actor(env, robot_asset, robot_pose, "robot", i, 1)
            robot_actor = robot_asset.to_actor(env, i, robot_pose)

            # Create table actor
            table_pose = gymapi.Transform()
            table_pose.p = gymapi.Vec3(table_dims.x, table_dims.y * 0.5, 0)
            table_actor = gym.create_actor(env, table_asset, table_pose, "table", i, 1)

        _jacobian = gym.acquire_jacobian_tensor(sim, "franka")
        jacobian = gymtorch.wrap_tensor(_jacobian)
        print(jacobian.shape)
