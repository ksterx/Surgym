from surgym.envs import Env


def test_env():
    env = Env()
    env.reset()
    env.render()
    env.close()
