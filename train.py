<<<<<<< Updated upstream
import hydra
import numpy as np
import ray
from isaacgym import gymapi
from omegaconf import OmegaConf
from ray.rllib.algorithms.dqn import DQNConfig

from surgym import notice
from surgym.envs import SurgeryEnv


@hydra.main(config_path="conf", config_name="config", version_base=None)
def main(cfg):

    ray.init()
    cfg = OmegaConf.to_container(cfg, resolve=True)
    notice.info("Finished")


def train(cfg):
    import torch

    notice.info("Starting training...")
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    if device == torch.device("cpu") or cfg["scale"]["use_gpu"] is False:
        cfg["scale"]["use_gpu"] = False
        notice.info("Using CPU")

    trainer = DQNConfig().environment(env=SurgeryEnv).build()
    trainer.train()


if __name__ == "__main__":
    main()
=======
import hydra
import numpy as np
import ray
import ray.rllib.algorithms
import torch
from isaacgym import gymapi
from omegaconf import OmegaConf

from surgym import notice


@hydra.main(config_path="config", config_name="config", version_base=None)
def main(cfg):
    ray.shutdown()
    ray.init()
    cfg = OmegaConf.to_contaßiner(cfg, resolve=True)


def train(cfg):
    notice.info("Starting training...")
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    if device == torch.device("cpu") or cfg["scale"]["use_gpu"] is False:
        cfg["scale"]["use_gpu"] = False
        notice.info("Using CPU")
>>>>>>> Stashed changes
