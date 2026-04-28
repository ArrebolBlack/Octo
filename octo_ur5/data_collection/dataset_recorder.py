"""Record episodes from a PyBullet UR5 environment into RLDS/TFDS format.

Requires the external `pybullet_ur5_robotiq` library for simulation classes
(ClutteredTouchPick, UR5Robotiq85, YCBModels, Camera).

Usage:
    python -m octo_ur5.data_collection.dataset_recorder \
        --data_dir=./data/output --num_episodes=10
"""

import os
import time
import logging

import numpy as np
import envlogger
import tensorflow as tf
import tensorflow_datasets as tfds
from absl import flags, app
from envlogger.backends import tfds_backend_writer

from octo_ur5.gym2dmenv import Gym2DmEnv

# These come from the external pybullet_ur5_robotiq library.
# Install: pip install pybullet_ur5_robotiq (or add to PYTHONPATH)
from env import ClutteredTouchPick
from robot import UR5Robotiq85
from utilities import YCBModels, Camera

FLAGS = flags.FLAGS

flags.DEFINE_string('data_dir', './data/output', 'Directory to save recorded data')
flags.DEFINE_integer('num_episodes', 1, 'Number of episodes to record')


def main(unused_argv):
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
    logging.info('Creating environment...')

    ycb_models = YCBModels(os.path.join('./data/ycb', '**', 'textured-decmp.obj'))
    camera = Camera((1, 1, 1), (0, 0, 0), (0, 0, 1), 0.1, 5, (320, 320), 40)
    robot = UR5Robotiq85((0, 0.5, 0), (0, 0, 0))
    env = ClutteredTouchPick(robot, ycb_models, camera, vis=True)
    env = Gym2DmEnv(env=env)
    logging.info('Environment created.')

    def step_fn(unused_timestep, unused_action, unused_env):
        return {'timestamp': time.time(), 'language_instruction': env.env.language_instruction}

    dataset_config = tfds.rlds.rlds_base.DatasetConfig(
        name='pybullet_ur5_pick_reset_cup_mug',
        observation_info=tfds.features.FeaturesDict({
            'image_primary': tfds.features.Image(shape=(256, 256, 3), encoding_format='jpeg'),
            'proprio': tfds.features.Tensor(shape=(7,), dtype=np.float32, encoding=tfds.features.Encoding.NONE),
        }),
        action_info=tfds.features.Tensor(shape=(7,), dtype=np.float32, encoding=tfds.features.Encoding.NONE),
        reward_info=tfds.features.Tensor(shape=(), dtype=np.float32, encoding=tfds.features.Encoding.NONE),
        discount_info=tfds.features.Tensor(shape=(), dtype=np.float64, encoding=tfds.features.Encoding.NONE),
        step_metadata_info=tfds.features.FeaturesDict({
            'timestamp': tfds.features.Tensor(shape=(), dtype=np.float64, encoding=tfds.features.Encoding.NONE),
            'language_instruction': tfds.features.Text(),
        }),
    )

    logging.info('Wrapping environment with EnvLogger...')
    with envlogger.EnvLogger(
        env,
        step_fn=step_fn,
        backend=tfds_backend_writer.TFDSBackendWriter(
            data_directory=FLAGS.data_dir,
            split_name='train',
            max_episodes_per_file=FLAGS.num_episodes,
            ds_config=dataset_config,
        ),
    ) as env:
        logging.info('Recording %d episode(s)...', FLAGS.num_episodes)
        for i in range(FLAGS.num_episodes):
            logging.info('Episode %d...', i)
            timestep = env.reset()
            while not timestep.last():
                action = env.env.read_debug_parameter()
                action = np.array(action, dtype=np.float32)
                timestep = env.step(action)
            logging.info('Episode %d done.', i)

        logging.info('All episodes recorded.')


if __name__ == '__main__':
    app.run(main)
