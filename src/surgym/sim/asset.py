from hydra.utils import to_absolute_path
from isaacgym import gymapi
from omegaconf import DictConfig, OmegaConf


class Asset:
    def __init__(self, cfg: DictConfig, gym, sim, name):
        self.cfg = cfg
        self.name = name
        self.gym = gym

        # Load asset from cfg
        asset_options = gymapi.AssetOptions()
        new_cfg = OmegaConf.to_container(cfg.object[name]).copy()
        del new_cfg["path"]  # AssetOptions does not have "path" attribute
        for key, value in new_cfg.items():
            setattr(asset_options, key, value)
        self.asset = gym.load_asset(
            sim, to_absolute_path(cfg.dir.asset), cfg.object[name].path, asset_options
        )

    def load(self):
        return self.asset


class SoftAsset(Asset):
    def __init__(self, cfg: DictConfig, gym, sim, name):
        super().__init__(cfg, gym, sim, name)

    def to_actor(
        self, env, group_id: int, pose: gymapi.Transform, use_cfg: bool, **kwargs
    ):

        actor = self.gym.create_actor(env, self.asset, pose, self.name, group_id, 1)
        soft_material = self.set_material(use_cfg=use_cfg, **kwargs)

        self.gym.set_actor_soft_materials(env, actor, [soft_material])
        self.get_info(env, actor)

        return actor

    def set_material(self, use_cfg, **kwargs):
        soft_material = gymapi.SoftMaterial()

        if use_cfg:
            for key, value in self.cfg.soft_material.items():
                if key != "model":
                    setattr(soft_material, key, value)
                elif key == "model" and value == "neo":
                    setattr(
                        soft_material, "model", gymapi.SoftMaterialType.MAT_NEOHOOKEAN
                    )
                elif key == "model" and value == "corot":
                    setattr(
                        soft_material, "model", gymapi.SoftMaterialType.MAT_COROTATIONAL
                    )
                else:
                    raise ValueError("Invalid soft material model")
        else:
            for key, value in kwargs.items():
                setattr(soft_material, key, value)

        return soft_material

    def get_info(self, env, actor):
        asset_soft_body_count = self.gym.get_asset_soft_body_count(self.asset)
        assert asset_soft_body_count > 0, "No soft bodies found in asset"
        actor_soft_body_count = self.gym.get_actor_soft_body_count(env, actor)
        actor_soft_materials = self.gym.get_actor_soft_materials(env, actor)

        print("\n======Soft Material Properties (actor)======")
        for i in range(actor_soft_body_count):
            material = actor_soft_materials[i]
            print(
                f"Body #{i}\nyoungs: {material.youngs}\npoissons: {material.poissons}\ndamping: {material.damping}\nactivation: {material.activation}\nmodel: {material.model}\n"
            )


class RigidAsset(Asset):
    def __init__(self, cfg: DictConfig, gym, sim, name):
        super().__init__(cfg, gym, sim, name)

    def to_actor(self, env, group_id: int, pose: gymapi.Transform):
        actor = self.gym.create_actor(env, self.asset, pose, self.name, group_id, 1)
        self.get_info(env, actor)

        return actor

    def get_info(self, env, actor):
        print("\n======Rigid Body Properties (asset)======")
        print(self.gym.get_asset_rigid_body_dict(self.asset))
        print(self.gym.get_actor_rigid_body_dict(env, actor))
