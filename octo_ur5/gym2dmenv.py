"""Adapter: Gym env → dm_env interface, used by envlogger for RLDS data recording.

The wrapped env must follow the Gym API (reset/step) and return observations
with 'image_primary' (256x256x3 uint8) and 'proprio' (7 float32).
"""

import dm_env
from dm_env import specs
import numpy as np


class Gym2DmEnv(dm_env.Environment):
    def __init__(self, env) -> None:
        self.env = env
        self._reset_next_step = True

    def reset(self):
        self._reset_next_step = False
        observation = self.env.reset()
        if isinstance(observation, tuple):
            observation = observation[0]
        return dm_env.restart(observation)

    def step(self, action):
        if self._reset_next_step:
            return self.reset()
        observation, reward, done, truncated, info = self.env.step(action)
        if done or truncated:
            self._reset_next_step = True
            return dm_env.termination(reward, observation)
        return dm_env.transition(reward, observation)

    def observation_spec(self):
        return {
            'image_primary': specs.BoundedArray(shape=(256, 256, 3), dtype=np.uint8,
                                                minimum=0, maximum=255, name='image_primary'),
            'proprio': specs.BoundedArray(shape=(7,), dtype=np.float32,
                                         minimum=-3.14, maximum=3.14, name='proprio'),
        }

    def action_spec(self):
        return specs.BoundedArray(shape=(7,), dtype=np.float32, minimum=-3.14, maximum=3.14, name='action')

    def reward_spec(self):
        return specs.BoundedArray(shape=(), dtype=np.float32, minimum=0.0, maximum=4.0, name='reward')

    def discount_spec(self):
        return specs.BoundedArray(shape=(), dtype=np.float32, minimum=1.0, maximum=1.0, name='discount')

    def close(self):
        self.env.close()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()
