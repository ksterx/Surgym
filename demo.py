import hydra
from isaacgym import gymapi
from omegaconf import DictConfig

from surgym.sim import Simulator
from surgym.utils import fix_seed


@hydra.main(config_path="conf", config_name="config", version_base=None)
def main(cfg: DictConfig) -> None:

    fix_seed(cfg.run.seed)
    simulator = Simulator(cfg, gymapi.SIM_FLEX)
    simulator.run()


if __name__ == "__main__":
    main()
    print("\n===========================")
    print("    Simulation Finished    ")
    print("===========================\n")
