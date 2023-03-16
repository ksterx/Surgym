import os

import hydra
import omni


@hydra.main(config_path="config", config_name="config")
def main(cfg):
    env = SurgeryEnv()



if __name__ == "__main__":
    main()
