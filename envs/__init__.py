from surgym.envs.surgery import SurgeryEnv

REGISTERD_ENVS = {
    "surgery": SurgeryEnv,
}


def create_env(env_name):
    env = REGISTERD_ENVS[env_name]
    return env
