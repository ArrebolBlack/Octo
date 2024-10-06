from env import ClutteredTouchPick
import dm_env
from dm_env import specs
import numpy as np

from dm_env._environment import TimeStep


class MyDmEnv(dm_env.Environment):
    def __init__(self, env: ClutteredTouchPick) -> None:
        self.env = env
        self._reset_next_step = True

    def reset(self):
        self._reset_next_step = False
        observation = self.env.reset()
        return dm_env.restart(observation)

    def step(self, action):
        if self._reset_next_step:
            return self.reset()

        observation, reward, done, info = self.env.step(action)
        if done:
            self._reset_next_step = True
            return dm_env.termination(reward, observation)
        else:
            return dm_env.transition(reward, observation)

    def observation_spec(self):
        return {
            'image_primary': specs.BoundedArray(
                shape=(256, 256, 3),
                dtype=np.uint8,
                minimum=0,
                maximum=255,
                name='image_primary'
            ),
            'proprio': specs.BoundedArray(
                shape=(7,),
                dtype=np.float32,
                minimum=-3.14,
                maximum=3.14,
                name='proprio'
            ),
        }

    def action_spec(self):
        return specs.BoundedArray(
            shape=(7,),
            dtype=np.float32,
            minimum=-3.14,
            maximum=3.14,
            name='action'
        )

    def reward_spec(self):
        return specs.BoundedArray(
            shape=(),
            dtype=np.float32,
            minimum=0.0,
            maximum=4.0,
            name='reward'
        )

    def discount_spec(self):
        return specs.BoundedArray(
            shape=(),  # 标量值
            dtype=np.float32,  # 浮点数类型
            minimum=1.0,  # 最小值1.0
            maximum=1.0,  # 最大值1.0
            name='discount'  # 名称
        )

    def close(self):
        self.env.close()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()