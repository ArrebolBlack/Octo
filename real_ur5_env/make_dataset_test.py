import os

import numpy as np
import pybullet as p

from tqdm import tqdm
import time
import math
import envlogger

import tensorflow as tf
import tensorflow_datasets as tfds
from absl import flags
# from absl import logging
from envlogger.backends import tfds_backend_writer
from absl import app

from gym2dmenv import MyDmEnv
import atexit

FLAGS = flags.FLAGS

import logging

log_file = '/home/xiaosa/Newpython/Octo/data_test/simulation.log'

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler(log_file),
        logging.StreamHandler()
    ]
)

flags.DEFINE_string('data_dir', '/home/xiaosa/Newpython/Octo/data_test', 'Directory to save data')
flags.DEFINE_integer('num_episodes', 1, 'Number of episodes to log')


def main(unused_argv):
    logging.info('Creating environment...')

    ycb_models = YCBModels(
        os.path.join('./data/ycb', '**', 'textured-decmp.obj'),
    )
    camera = Camera((1, 1, 1),
                    (0, 0, 0),
                    (0, 0, 1),
                    0.1, 5, (320, 320), 40)

    robot = UR5Robotiq85((0, 0.5, 0), (0, 0, 0))
    env = ClutteredTouchPick(robot, ycb_models, camera, vis=True)
    env = MyDmEnv(env=env)
    logging.info('Done creating environment.')

    def step_fn(unused_timestep, unused_action, unused_env):
        # logging.info('Executing step function...')
        return {'timestamp': time.time(),
                'language_instruction': env.env.language_instruction}

    dataset_config = tfds.rlds.rlds_base.DatasetConfig(
        name='pybullet_ur5_pick_reset_cup_mug',
        observation_info=tfds.features.FeaturesDict(
            {
                'image_primary': tfds.features.Image(
                    shape=(256, 256, 3), encoding_format='jpeg'
                ),
                'proprio': tfds.features.Tensor(
                    shape=(7,), dtype=np.float32, encoding=tfds.features.Encoding.NONE
                ),
            }
        ),
        action_info=tfds.features.Tensor(
            shape=(7,), dtype=np.float32, encoding=tfds.features.Encoding.NONE
        ),
        reward_info=tfds.features.Tensor(
            shape=(), dtype=np.float32, encoding=tfds.features.Encoding.NONE
        ),
        discount_info=tfds.features.Tensor(
            shape=(), dtype=np.float64, encoding=tfds.features.Encoding.NONE
        ),
        step_metadata_info=tfds.features.FeaturesDict(
            {
                'timestamp': tfds.features.Tensor(shape=(), dtype=np.float64, encoding=tfds.features.Encoding.NONE),
                'language_instruction': tfds.features.Text(),
            }
        )
    )

    max_steps_per_episode = 100
    logging.info('Wrapping environment with EnvironmentLogger...')

    with envlogger.EnvLogger(
            env,
            step_fn=step_fn,
            backend=tfds_backend_writer.TFDSBackendWriter(
                data_directory=FLAGS.data_dir,
                split_name='train',
                max_episodes_per_file=FLAGS.num_episodes,
                ds_config=dataset_config
            )
    ) as env:

        logging.info('Done wrapping environment with EnvironmentLogger.')

        logging.info('Starting logging for %r episodes...', FLAGS.num_episodes)

        for i in range(FLAGS.num_episodes):
            logging.info('Starting episode %r...', i)
            # logging.info('Resetting environment...')
            timestep = env.reset()
            # logging.info('Environment reset successfully.')

            while not timestep.last():
                # logging.info('Taking action...')
                action = env.env.read_debug_parameter()
                action = np.array(action, dtype=np.float32)
                # logging.info('Action taken: %r', action)
                timestep = env.step(action)
                # logging.info('Step completed.')
                # print(timestep)

            logging.info('Done logging episode %r.', i)

        logging.info('Done collecting for %r episodes.', FLAGS.num_episodes)

        logging.info('Done logging.')


if __name__ == '__main__':

    try:
        app.run(main)
    except Exception as e:
        logging.error(f"An error occurred: {e}", exc_info=True)

